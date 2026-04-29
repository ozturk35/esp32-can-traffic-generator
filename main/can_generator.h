#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "mcp2515.h"

typedef enum {
    MODE_IDLE       = 0,
    MODE_SIMULATING = 1,
    MODE_BURST      = 2,
} op_mode_t;

typedef struct {
    uint32_t pgn61444_tx;
    uint32_t pgn65262_tx;
    uint32_t pgn65265_tx;
    uint32_t pgn65276_tx;
    uint32_t burst_tx;
} tx_counters_t;

extern QueueHandle_t     g_tx_queue;
extern volatile op_mode_t g_mode;
extern SemaphoreHandle_t  g_mode_mutex;
extern tx_counters_t      g_counters;

void can_generator_init(mcp2515_t *dev);
