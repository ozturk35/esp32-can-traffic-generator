# ESP32 CAN Traffic Generator

SAE J1939 CAN bus traffic generator for the ESP32-S3, built with ESP-IDF and FreeRTOS. Designed as the transmitter node in a two-node bench setup alongside the [STM32 CAN Bus Sniffer](https://github.com/ozturk35/stm32-can-bus-sniffer).

**Hardware:** ESP32-S3-WROOM-2-N32R8V + MCP2515 SPI CAN module  
**Bus:** 250 kbit/s, SAE J1939  
**Firmware:** ESP-IDF, FreeRTOS  

## Features

- Continuous J1939 PGN simulation with realistic oscillating values
- Four PGNs transmitted on independent schedules:
  - **PGN 61444** (EEC1 — Engine Speed): every 100 ms, 750–2200 RPM sweep
  - **PGN 65265** (CCVS — Vehicle Speed): every 100 ms, 0–120 km/h sweep
  - **PGN 65262** (Engine Temperature): every 1 s, 75–95 °C oscillation
  - **PGN 65276** (Dash Display — Fuel Level): every 1 s, draining 95% → 30% then reset
- Burst mode: flood bus at maximum rate for stress-testing the receiver
- One-shot frame inject: send arbitrary CAN frames from the terminal
- Per-PGN TX counters and live MCP2515 EFLG monitoring
- UART command interface at 115200 baud

## UART Commands

| Command | Description |
|---|---|
| `start` | Begin J1939 PGN simulation |
| `stop` | Halt all transmission |
| `burst on` | Flood bus at maximum rate |
| `burst off` | Stop burst mode |
| `inject <id> <data>` | Send one frame (`id`: 8 hex digits, `data`: 2–16 hex digits) |
| `status` | Show mode, per-PGN TX counters, MCP2515 EFLG |
| `help` | Print command list |

## Pin Connections (SPI2)

| MCP2515 | ESP32-S3 GPIO | Notes |
|---|---|---|
| VCC | 3.3V | Must be 3.3V |
| GND | GND | |
| SCK | GPIO 12 | SPI2_CLK |
| SI (MOSI) | GPIO 11 | SPI2_MOSI |
| SO (MISO) | GPIO 13 | SPI2_MISO |
| CS | GPIO 10 | Software-driven, active-low |
| INT | — | Not used (generator only) |

## CAN Bus Topology

Both nodes share a two-wire CAN bus (CANH / CANL). Each node must have its own 120 Ω termination resistor across CANH/CANL.

```
ESP32-S3 + MCP2515  ──[120Ω]── CANH/CANL ──[120Ω]── STM32 Nucleo + MCP2515
   (generator)                                             (sniffer)
```

## Build and Flash

```bash
# Set up ESP-IDF environment
source ~/dev/sdks/espressif/v6.0/esp-idf/export.sh

# Configure target
idf.py set-target esp32s3

# Build
idf.py build

# Flash (adjust port as needed)
idf.py -p /dev/ttyUSB0 flash monitor
```

The project targets the **ESP32-S3-WROOM-2-N32R8V** (32 MB Octal flash, 8 MB Octal PSRAM). Settings are in `sdkconfig.defaults`; a matching `partitions-32mb.csv` is included.

## Example Terminal Session

```
CAN Traffic Generator — ready
J1939 250 kbit/s | PGNs: 61444 65262 65265 65276
Type 'help' for commands.
> start
OK: simulation started
> status
Mode:         SIMULATING
PGN 61444 TX: 42 frames
PGN 65262 TX: 4 frames
PGN 65265 TX: 42 frames
PGN 65276 TX: 4 frames
Burst TX:     0 frames
MCP2515 EFLG: 0x00 (no errors)
> inject 18FF0001 DEADBEEF01020304
OK: frame queued
> burst on
OK: burst mode ON
> burst off
OK: burst mode OFF
> stop
OK: stopped
```

## Documentation

See [`Documents/esp32-can-traffic-generator-fsd.md`](Documents/esp32-can-traffic-generator-fsd.md) for the full Functional Specification Document.
