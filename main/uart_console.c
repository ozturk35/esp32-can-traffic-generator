#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "uart_console.h"
#include "can_generator.h"
#include "j1939_frames.h"

#define CON_UART     UART_NUM_0
#define CON_BAUD     115200
#define BUF_SIZE     256
#define CON_LINE_MAX 128

static const char *TAG = "console";
static mcp2515_t  *s_dev;

static void con_write(const char *s)
{
    uart_write_bytes(CON_UART, s, strlen(s));
}

static void con_writef(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void con_writef(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_write_bytes(CON_UART, buf, strlen(buf));
}

static const char *mode_str(void)
{
    switch (g_mode) {
        case MODE_SIMULATING: return "SIMULATING";
        case MODE_BURST:      return "BURST";
        default:              return "IDLE";
    }
}

static const char *eflg_desc(uint8_t eflg)
{
    if (eflg == 0) return "no errors";
    static char buf[64];
    buf[0] = '\0';
    if (eflg & MCP_EFLG_TXBO)   strcat(buf, "TXBO ");
    if (eflg & MCP_EFLG_TXEP)   strcat(buf, "TXEP ");
    if (eflg & MCP_EFLG_RXEP)   strcat(buf, "RXEP ");
    if (eflg & MCP_EFLG_RX0OVR) strcat(buf, "RX0OVR ");
    if (eflg & MCP_EFLG_RX1OVR) strcat(buf, "RX1OVR ");
    return buf;
}

static void cmd_help(void)
{
    con_write(
        "Commands:\r\n"
        "  start              -- begin J1939 PGN simulation\r\n"
        "  stop               -- halt all transmission\r\n"
        "  burst on           -- flood bus at max rate\r\n"
        "  burst off          -- stop burst mode\r\n"
        "  inject <id> <data> -- send one frame (id: 8 hex digits, data: 2-16 hex digits)\r\n"
        "  status             -- show mode, counters, EFLG\r\n"
        "  help               -- this message\r\n"
    );
}

static void cmd_status(void)
{
    uint8_t eflg = mcp2515_read_eflg(s_dev);
    con_writef(
        "Mode:         %s\r\n"
        "PGN 61444 TX: %lu frames\r\n"
        "PGN 65262 TX: %lu frames\r\n"
        "PGN 65265 TX: %lu frames\r\n"
        "PGN 65276 TX: %lu frames\r\n"
        "Burst TX:     %lu frames\r\n"
        "MCP2515 EFLG: 0x%02X (%s)\r\n",
        mode_str(),
        (unsigned long)g_counters.pgn61444_tx,
        (unsigned long)g_counters.pgn65262_tx,
        (unsigned long)g_counters.pgn65265_tx,
        (unsigned long)g_counters.pgn65276_tx,
        (unsigned long)g_counters.burst_tx,
        eflg, eflg_desc(eflg)
    );
}

static void cmd_inject(const char *id_str, const char *data_str)
{
    if (!id_str || strlen(id_str) != 8 || !data_str) {
        con_write("ERROR: bad format — use: inject <8-hex-id> <2-16-hex-data>\r\n");
        return;
    }
    size_t dlen = strlen(data_str);
    if (dlen < 2 || dlen > 16 || dlen % 2 != 0) {
        con_write("ERROR: data must be 2-16 hex digits (even count)\r\n");
        return;
    }

    char *end;
    uint32_t id = strtoul(id_str, &end, 16);
    if (*end != '\0') {
        con_write("ERROR: invalid ID hex\r\n");
        return;
    }

    can_frame_t f = {0};
    f.id  = id;
    f.dlc = (uint8_t)(dlen / 2);
    for (int i = 0; i < (int)f.dlc; i++) {
        char pair[3] = {data_str[2*i], data_str[2*i+1], '\0'};
        f.data[i] = (uint8_t)strtoul(pair, NULL, 16);
    }

    if (xQueueSend(g_tx_queue, &f, pdMS_TO_TICKS(50)) == pdTRUE) {
        con_write("OK: frame queued\r\n");
    } else {
        con_write("ERROR: TX queue full\r\n");
    }
}

static void process_line(char *line)
{
    /* strip trailing whitespace */
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' || line[len-1] == ' '))
        line[--len] = '\0';

    if (len == 0) { con_write("> "); return; }

    /* lowercase in place for case-insensitive matching */
    for (int i = 0; line[i]; i++) line[i] = tolower((unsigned char)line[i]);

    if (strcmp(line, "start") == 0) {
        g_mode = MODE_SIMULATING;
        con_write("OK: simulation started\r\n");
    } else if (strcmp(line, "stop") == 0) {
        g_mode = MODE_IDLE;
        con_write("OK: stopped\r\n");
    } else if (strcmp(line, "burst on") == 0) {
        g_mode = MODE_BURST;
        con_write("OK: burst mode ON\r\n");
    } else if (strcmp(line, "burst off") == 0) {
        g_mode = (g_mode == MODE_BURST) ? MODE_IDLE : g_mode;
        con_write("OK: burst mode OFF\r\n");
    } else if (strncmp(line, "inject ", 7) == 0) {
        char *rest    = line + 7;
        char *id_str  = strtok(rest, " ");
        char *data_str = strtok(NULL, " ");
        cmd_inject(id_str, data_str);
    } else if (strcmp(line, "status") == 0) {
        cmd_status();
    } else if (strcmp(line, "help") == 0) {
        cmd_help();
    } else {
        con_writef("ERROR: unknown command '%s' — type 'help'\r\n", line);
    }

    con_write("> ");
}

static void uart_task(void *arg)
{
    char line[CON_LINE_MAX];
    int  pos = 0;
    uint8_t ch;

    for (;;) {
        int n = uart_read_bytes(CON_UART, &ch, 1, pdMS_TO_TICKS(10));
        if (n <= 0) continue;

        if (ch == '\r' || ch == '\n') {
            line[pos] = '\0';
            uart_write_bytes(CON_UART, "\r\n", 2);
            process_line(line);
            pos = 0;
        } else if (ch == 0x7F || ch == 0x08) { /* backspace */
            if (pos > 0) {
                pos--;
                uart_write_bytes(CON_UART, "\b \b", 3);
            }
        } else if (pos < LINE_MAX - 1) {
            line[pos++] = ch;
            uart_write_bytes(CON_UART, &ch, 1); /* echo */
        }
    }
}

void uart_console_start(mcp2515_t *dev)
{
    s_dev = dev;

    uart_config_t cfg = {
        .baud_rate  = CON_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(CON_UART, BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(CON_UART, &cfg);

    xTaskCreate(uart_task, "uart_con", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
}
