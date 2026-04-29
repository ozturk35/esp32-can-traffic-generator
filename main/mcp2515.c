#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "mcp2515.h"

static const char *TAG = "mcp2515";

/* CNF register values for 250 kbit/s with 8 MHz crystal.
 * Bit timing: 1 TQ sync + 2 TQ prop + 6 TQ phseg1 + 6 TQ phseg2 = 16 TQ
 * TQ = 2 * (1/8MHz) = 250 ns  →  bit time = 4 µs  →  250 kbit/s */
#define CNF1_250K  0x00   /* BRP=0, SJW=1 TQ */
#define CNF2_250K  0xB1   /* BTLMODE=1, SAM=0, PHSEG1=6 TQ, PRSEG=2 TQ */
#define CNF3_250K  0x05   /* PHSEG2=6 TQ */

/* ---------- low-level SPI helpers ---------------------------------------- */

uint8_t mcp2515_read_reg(mcp2515_t *dev, uint8_t addr)
{
    uint8_t tx[3] = {MCP_READ, addr, 0x00};
    uint8_t rx[3] = {0};
    spi_transaction_t t = {
        .length    = 24,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(dev->spi, &t);
    return rx[2];
}

void mcp2515_write_reg(mcp2515_t *dev, uint8_t addr, uint8_t value)
{
    uint8_t tx[3] = {MCP_WRITE, addr, value};
    spi_transaction_t t = {
        .length    = 24,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };
    spi_device_polling_transmit(dev->spi, &t);
}

static void mcp2515_reset(mcp2515_t *dev)
{
    uint8_t tx = MCP_RESET;
    spi_transaction_t t = {
        .length    = 8,
        .tx_buffer = &tx,
        .rx_buffer = NULL,
    };
    spi_device_polling_transmit(dev->spi, &t);
}

/* ---------- public API ---------------------------------------------------- */

esp_err_t mcp2515_init(mcp2515_t *dev)
{
    /* Reset: puts MCP2515 into config mode */
    mcp2515_reset(dev);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t canstat = mcp2515_read_reg(dev, MCP_CANSTAT);
    if ((canstat & MCP_MODE_MASK) != MCP_MODE_CONFIG) {
        ESP_LOGE(TAG, "CANSTAT=0x%02X — expected config mode 0x80; check SPI wiring", canstat);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "CANSTAT=0x%02X (config mode OK)", canstat);

    /* Set baud rate */
    mcp2515_write_reg(dev, MCP_CNF1, CNF1_250K);
    mcp2515_write_reg(dev, MCP_CNF2, CNF2_250K);
    mcp2515_write_reg(dev, MCP_CNF3, CNF3_250K);

    uint8_t c1 = mcp2515_read_reg(dev, MCP_CNF1);
    uint8_t c2 = mcp2515_read_reg(dev, MCP_CNF2);
    uint8_t c3 = mcp2515_read_reg(dev, MCP_CNF3);
    if (c1 != CNF1_250K || c2 != CNF2_250K || c3 != CNF3_250K) {
        ESP_LOGE(TAG, "CNF read-back mismatch: CNF1=0x%02X CNF2=0x%02X CNF3=0x%02X", c1, c2, c3);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "CNF1=0x%02X CNF2=0x%02X CNF3=0x%02X — 250 kbit/s OK", c1, c2, c3);

    /* Enter normal mode */
    mcp2515_write_reg(dev, MCP_CANCTRL, MCP_MODE_NORMAL);

    /* Poll for mode transition (MCP2515 needs 11 recessive bits on the bus) */
    int waited = 0;
    while (waited < 50) {
        vTaskDelay(pdMS_TO_TICKS(1));
        uint8_t stat = mcp2515_read_reg(dev, MCP_CANSTAT);
        if ((stat & MCP_MODE_MASK) == MCP_MODE_NORMAL) {
            ESP_LOGI(TAG, "Normal mode confirmed — 250 kbit/s J1939 ready");
            return ESP_OK;
        }
        waited++;
    }

    ESP_LOGE(TAG, "Timeout waiting for normal mode — check CAN bus termination");
    return ESP_FAIL;
}

esp_err_t mcp2515_send_frame(mcp2515_t *dev, const can_frame_t *frame)
{
    /* Poll TXB0CTRL.TXREQ to ensure buffer is free */
    int waited = 0;
    while (mcp2515_read_reg(dev, MCP_TXB0CTRL) & 0x08) {
        if (++waited > 10) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Load TX buffer 0 starting at SIDH in a single SPI transaction.
     * EXIDE (bit 3 of SIDL) must be 1 for 29-bit extended J1939 frames. */
    uint32_t id = frame->id;
    uint8_t  txbuf[14];
    txbuf[0]  = MCP_LOAD_TXB0_SIDH;
    txbuf[1]  = (id >> 21) & 0xFF;                          /* TXB0SIDH */
    txbuf[2]  = ((id >> 13) & 0xE0) | (1 << 3) | ((id >> 16) & 0x03); /* TXB0SIDL + EXIDE */
    txbuf[3]  = (id >>  8) & 0xFF;                          /* TXB0EID8 */
    txbuf[4]  =  id        & 0xFF;                          /* TXB0EID0 */
    txbuf[5]  = frame->dlc & 0x0F;                          /* TXB0DLC  */
    memcpy(&txbuf[6], frame->data, 8);                       /* TXB0D0-D7 */

    spi_transaction_t t = {
        .length    = 14 * 8,
        .tx_buffer = txbuf,
        .rx_buffer = NULL,
    };
    spi_device_polling_transmit(dev->spi, &t);

    /* Request-to-send TXB0 */
    uint8_t rts = MCP_RTS_TXB0;
    spi_transaction_t t2 = {
        .length    = 8,
        .tx_buffer = &rts,
        .rx_buffer = NULL,
    };
    spi_device_polling_transmit(dev->spi, &t2);
    return ESP_OK;
}

uint8_t mcp2515_read_eflg(mcp2515_t *dev)
{
    return mcp2515_read_reg(dev, MCP_EFLG);
}

esp_err_t mcp2515_recover_bus_off(mcp2515_t *dev)
{
    ESP_LOGW(TAG, "Bus-off detected — attempting recovery");
    mcp2515_reset(dev);
    vTaskDelay(pdMS_TO_TICKS(10));
    return mcp2515_init(dev);
}
