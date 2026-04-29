#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "mcp2515.h"
#include "can_generator.h"
#include "uart_console.h"

#define SPI2_PIN_MOSI  11
#define SPI2_PIN_MISO  13
#define SPI2_PIN_CLK   12
#define SPI2_PIN_CS    10
#define MCP_SPI_HZ     5000000  /* 5 MHz — safe at 3.3V for 8 MHz crystal MCP2515 */

static const char *TAG = "main";

static mcp2515_t s_mcp;

void app_main(void)
{
    /* Initialise SPI2 bus (shared platform resource — owned by app, not driver) */
    spi_bus_config_t buscfg = {
        .mosi_io_num     = SPI2_PIN_MOSI,
        .miso_io_num     = SPI2_PIN_MISO,
        .sclk_io_num     = SPI2_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 16,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = MCP_SPI_HZ,
        .mode           = 0,        /* CPOL=0, CPHA=0 */
        .spics_io_num   = SPI2_PIN_CS,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_mcp.spi));

    /* Initialise MCP2515 — halt on failure (no silent degradation) */
    if (mcp2515_init(&s_mcp) != ESP_OK) {
        ESP_LOGE(TAG, "MCP2515 init failed — halting");
        vTaskSuspend(NULL);
    }

    /* Boot banner — printf goes through console, no driver needed */
    printf("\r\nCAN Traffic Generator — ready\r\n"
           "J1939 250 kbit/s | PGNs: 61444 65262 65265 65276\r\n"
           "Type 'help' for commands.\r\n> ");

    /* Start generator tasks and UART console */
    can_generator_init(&s_mcp);
    uart_console_start(&s_mcp);
}
