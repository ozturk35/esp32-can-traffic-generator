#include <string.h>
#include "j1939_frames.h"

void j1939_build_pgn61444(can_frame_t *f, uint16_t rpm_raw)
{
    f->id    = CAN_ID_PGN_61444;
    f->dlc   = 8;
    f->data[0] = 0x00;              /* engine torque mode */
    f->data[1] = 0x7D;              /* driver demand torque */
    f->data[2] = 0x7D;              /* actual engine torque */
    f->data[3] = rpm_raw & 0xFF;    /* SPN 190 LSB — 0.125 RPM/bit */
    f->data[4] = rpm_raw >> 8;      /* SPN 190 MSB */
    f->data[5] = 0xFF;
    f->data[6] = 0x0F;
    f->data[7] = 0x7D;
}

void j1939_build_pgn65262(can_frame_t *f, uint8_t coolant_raw)
{
    f->id    = CAN_ID_PGN_65262;
    f->dlc   = 8;
    f->data[0] = coolant_raw;       /* SPN 110 — 1°C/bit, offset -40°C */
    memset(&f->data[1], 0xFF, 7);
}

void j1939_build_pgn65265(can_frame_t *f, uint16_t speed_raw)
{
    f->id    = CAN_ID_PGN_65265;
    f->dlc   = 8;
    f->data[0] = speed_raw & 0xFF;  /* SPN 84 LSB — 1/256 km/h per bit */
    f->data[1] = speed_raw >> 8;    /* SPN 84 MSB */
    memset(&f->data[2], 0xFF, 6);
}

void j1939_build_pgn65276(can_frame_t *f, uint8_t fuel_raw)
{
    f->id    = CAN_ID_PGN_65276;
    f->dlc   = 8;
    f->data[0] = 0xFF;              /* washer fluid — not available */
    f->data[1] = fuel_raw;          /* SPN 96 — 0.4%/bit */
    memset(&f->data[2], 0xFF, 6);
}
