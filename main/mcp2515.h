#pragma once
#include "esp_err.h"
#include "driver/spi_master.h"
#include "j1939_frames.h"

/* SPI instruction bytes */
#define MCP_RESET            0xC0
#define MCP_READ             0x03
#define MCP_WRITE            0x02
#define MCP_RTS_TXB0         0x81
#define MCP_LOAD_TXB0_SIDH   0x40

/* Register addresses */
#define MCP_CANSTAT   0x0E
#define MCP_CANCTRL   0x0F
#define MCP_CNF3      0x28
#define MCP_CNF2      0x29
#define MCP_CNF1      0x2A
#define MCP_TXB0CTRL  0x30
#define MCP_TXB0SIDH  0x31
#define MCP_EFLG      0x2D
#define MCP_TEC       0x1C
#define MCP_REC       0x1D

/* CANCTRL REQOP mode values */
#define MCP_MODE_NORMAL      0x00
#define MCP_MODE_CONFIG      0x80
#define MCP_MODE_MASK        0xE0

/* EFLG bits */
#define MCP_EFLG_RX0OVR  (1 << 6)
#define MCP_EFLG_RX1OVR  (1 << 7)
#define MCP_EFLG_TXBO    (1 << 5)
#define MCP_EFLG_TXEP    (1 << 4)
#define MCP_EFLG_RXEP    (1 << 3)

typedef struct {
    spi_device_handle_t spi;
} mcp2515_t;

esp_err_t mcp2515_init(mcp2515_t *dev);
esp_err_t mcp2515_send_frame(mcp2515_t *dev, const can_frame_t *frame);
uint8_t   mcp2515_read_reg(mcp2515_t *dev, uint8_t addr);
void      mcp2515_write_reg(mcp2515_t *dev, uint8_t addr, uint8_t value);
uint8_t   mcp2515_read_eflg(mcp2515_t *dev);
esp_err_t mcp2515_recover_bus_off(mcp2515_t *dev);
