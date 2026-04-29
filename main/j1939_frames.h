#pragma once
#include <stdint.h>

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

/* Build a 29-bit J1939 CAN ID */
#define J1939_ID(prio, dp, pf, ps, sa) \
    (((uint32_t)(prio) << 26) | \
     ((uint32_t)(dp)   << 24) | \
     ((uint32_t)(pf)   << 16) | \
     ((uint32_t)(ps)   <<  8) | \
     ((uint32_t)(sa)))

/* Pre-computed IDs, SA=0x00 */
#define CAN_ID_PGN_61444  0x0CF00400UL  /* priority=3, EEC1 engine speed      */
#define CAN_ID_PGN_65262  0x18FEEE00UL  /* priority=6, engine temperature 1   */
#define CAN_ID_PGN_65265  0x18FEF100UL  /* priority=6, CCVS vehicle speed     */
#define CAN_ID_PGN_65276  0x18FEFC00UL  /* priority=6, dash display fuel      */

void j1939_build_pgn61444(can_frame_t *f, uint16_t rpm_raw);
void j1939_build_pgn65262(can_frame_t *f, uint8_t  coolant_raw);
void j1939_build_pgn65265(can_frame_t *f, uint16_t speed_raw);
void j1939_build_pgn65276(can_frame_t *f, uint8_t  fuel_raw);
