#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "can_generator.h"
#include "j1939_frames.h"

static const char *TAG = "can_gen";

QueueHandle_t     g_tx_queue;
volatile op_mode_t g_mode = MODE_IDLE;
SemaphoreHandle_t  g_mode_mutex;
tx_counters_t      g_counters;

static mcp2515_t  *s_dev;

/* ---------- simulation state --------------------------------------------- */

typedef struct {
    float rpm;
    float speed_kmh;
    float coolant_temp;
    float fuel_pct;
    int   rpm_dir;
    int   speed_dir;
    int   coolant_dir;
} sim_state_t;

static sim_state_t s_sim = {
    .rpm          = 1000.0f,
    .speed_kmh    = 60.0f,
    .coolant_temp = 82.0f,
    .fuel_pct     = 80.0f,
    .rpm_dir      = 1,
    .speed_dir    = 1,
    .coolant_dir  = 1,
};

static void update_sim_100ms(sim_state_t *s)
{
    s->rpm += 15.0f * s->rpm_dir;
    if (s->rpm >= 2200.0f) { s->rpm = 2200.0f; s->rpm_dir = -1; }
    if (s->rpm <=  750.0f) { s->rpm =  750.0f; s->rpm_dir =  1; }

    s->speed_kmh += 0.5f * s->speed_dir;
    if (s->speed_kmh >= 120.0f) { s->speed_kmh = 120.0f; s->speed_dir = -1; }
    if (s->speed_kmh <=   0.0f) { s->speed_kmh =   0.0f; s->speed_dir =  1; }
}

static void update_sim_1000ms(sim_state_t *s)
{
    s->coolant_temp += 0.2f * s->coolant_dir;
    if (s->coolant_temp >= 95.0f) { s->coolant_temp = 95.0f; s->coolant_dir = -1; }
    if (s->coolant_temp <= 75.0f) { s->coolant_temp = 75.0f; s->coolant_dir =  1; }

    s->fuel_pct -= 0.05f;
    if (s->fuel_pct < 30.0f) s->fuel_pct = 95.0f;
}

/* ---------- EFLG monitor (esp_timer callback) ----------------------------- */

static void eflg_monitor_cb(void *arg)
{
    uint8_t eflg = mcp2515_read_eflg(s_dev);
    if (eflg == 0) return;

    ESP_LOGW(TAG, "EFLG=0x%02X%s%s%s%s%s", eflg,
        (eflg & MCP_EFLG_TXBO)   ? " TXBO"   : "",
        (eflg & MCP_EFLG_TXEP)   ? " TXEP"   : "",
        (eflg & MCP_EFLG_RXEP)   ? " RXEP"   : "",
        (eflg & MCP_EFLG_RX0OVR) ? " RX0OVR" : "",
        (eflg & MCP_EFLG_RX1OVR) ? " RX1OVR" : "");

    if (eflg & MCP_EFLG_TXBO) {
        mcp2515_recover_bus_off(s_dev);
    }
}

/* ---------- TX drain task ------------------------------------------------- */

static void tx_task(void *arg)
{
    can_frame_t frame;
    for (;;) {
        if (xQueueReceive(g_tx_queue, &frame, portMAX_DELAY) == pdTRUE) {
            mcp2515_send_frame(s_dev, &frame);
        }
    }
}

/* ---------- simulation task ----------------------------------------------- */

static void sim_task(void *arg)
{
    TickType_t last_100ms  = xTaskGetTickCount();
    TickType_t last_1000ms = xTaskGetTickCount();

    for (;;) {
        if (g_mode != MODE_SIMULATING) {
            vTaskDelay(pdMS_TO_TICKS(10));
            last_100ms  = xTaskGetTickCount();
            last_1000ms = xTaskGetTickCount();
            continue;
        }

        vTaskDelayUntil(&last_100ms, pdMS_TO_TICKS(100));

        update_sim_100ms(&s_sim);

        can_frame_t f;
        uint16_t rpm_raw   = (uint16_t)(s_sim.rpm * 8.0f);
        uint16_t speed_raw = (uint16_t)(s_sim.speed_kmh * 256.0f);

        j1939_build_pgn61444(&f, rpm_raw);
        if (xQueueSend(g_tx_queue, &f, 0) == pdTRUE) g_counters.pgn61444_tx++;

        j1939_build_pgn65265(&f, speed_raw);
        if (xQueueSend(g_tx_queue, &f, 0) == pdTRUE) g_counters.pgn65265_tx++;

        if ((xTaskGetTickCount() - last_1000ms) >= pdMS_TO_TICKS(1000)) {
            last_1000ms += pdMS_TO_TICKS(1000);
            update_sim_1000ms(&s_sim);

            uint8_t coolant_raw = (uint8_t)(s_sim.coolant_temp + 40.0f);
            uint8_t fuel_raw    = (uint8_t)(s_sim.fuel_pct / 0.4f);

            j1939_build_pgn65262(&f, coolant_raw);
            if (xQueueSend(g_tx_queue, &f, 0) == pdTRUE) g_counters.pgn65262_tx++;

            j1939_build_pgn65276(&f, fuel_raw);
            if (xQueueSend(g_tx_queue, &f, 0) == pdTRUE) g_counters.pgn65276_tx++;
        }
    }
}

/* ---------- burst task ---------------------------------------------------- */

static const can_frame_t k_burst_frame = {
    .id   = 0x18FF0000UL,
    .dlc  = 8,
    .data = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF},
};

static void burst_task(void *arg)
{
    bool warned = false;
    for (;;) {
        if (g_mode != MODE_BURST) {
            warned = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (xQueueSend(g_tx_queue, &k_burst_frame, pdMS_TO_TICKS(1)) == pdTRUE) {
            g_counters.burst_tx++;
            warned = false;
        } else if (!warned) {
            ESP_LOGW(TAG, "TX queue full during burst mode");
            warned = true;
        }
        taskYIELD();
    }
}

/* ---------- init ---------------------------------------------------------- */

void can_generator_init(mcp2515_t *dev)
{
    s_dev = dev;
    memset(&g_counters, 0, sizeof(g_counters));

    g_tx_queue  = xQueueCreate(32, sizeof(can_frame_t));
    g_mode_mutex = xSemaphoreCreateMutex();

    xTaskCreate(tx_task,    "tx",    2048, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(sim_task,   "sim",   4096, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(burst_task, "burst", 2048, NULL, configMAX_PRIORITIES - 2, NULL);

    esp_timer_handle_t eflg_timer;
    esp_timer_create_args_t ta = {
        .callback = eflg_monitor_cb,
        .name     = "eflg",
    };
    esp_timer_create(&ta, &eflg_timer);
    esp_timer_start_periodic(eflg_timer, 5000000); /* 5 s */
}
