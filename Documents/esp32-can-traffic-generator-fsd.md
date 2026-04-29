# ESP32 CAN Traffic Generator — Functional Specification Document (FSD)

**Version:** 1.0  
**Date:** 2026-04-29  
**Platform:** ESP32-S3-WROOM-2-N32R8V (SLOT1)  
**Status:** Draft

---

## 1. System Overview

### 1.1 Purpose

This document specifies the firmware for a CAN bus traffic generator running on an ESP32-S3 microcontroller. The generator simulates a vehicle ECU network by transmitting realistic SAE J1939 parameter group numbers (PGNs) onto a shared CAN bus at their standard intervals, providing a live traffic source for a companion STM32 Nucleo F446ZE sniffer node.

### 1.2 Problem Statement

Developing and validating a CAN bus sniffer in isolation requires a live CAN traffic source. Connecting to a real vehicle for bench development is impractical. This firmware creates a self-contained, closed CAN network from two boards — a generator (ESP32-S3) and a sniffer (STM32 Nucleo F446ZE) — requiring no external hardware or vehicle access.

### 1.3 Users and Stakeholders

| Stakeholder | Role |
|---|---|
| Firmware developer | Configures and runs the generator; validates sniffer output |
| STM32 sniffer firmware | Consumes CAN frames produced by this generator |

### 1.4 Goals

- Transmit four J1939 PGNs at correct standard intervals with slowly varying, realistic values.
- Provide a burst mode to stress-test the sniffer's ring buffer at maximum frame rate.
- Expose a UART command interface for mode control and ad-hoc frame injection.

### 1.5 Non-Goals

- Full SAE J1939 stack compliance (address claiming, transport protocol, diagnostics).
- Real vehicle data acquisition.
- CAN reception / decoding on the ESP32-S3 side.
- Support for baud rates other than 250 kbit/s.

### 1.6 High-Level System Flow

```
Developer (UART terminal)
        │
        ▼
  UART Command Interface (115200 baud)
        │
        ▼
  PGN Scheduler / Burst Controller
        │
        ▼
  MCP2515 Driver (SPI2, GPIO10-13, INT=GPIO9)
        │
        ▼
  MCP2515 SPI CAN Controller
        │
   CANH / CANL (250 kbit/s, J1939)
        │
        ▼
  STM32 Nucleo F446ZE Sniffer (MCP2515 #1)
```

---

## 2. System Architecture

### 2.1 Logical Architecture

The firmware is structured as three concurrent FreeRTOS tasks:

| Task | Priority | Responsibility |
|---|---|---|
| `sim_task` | Medium | Runs the PGN scheduler; maintains simulated signal values; enqueues CAN frames to the TX queue |
| `burst_task` | Medium | Floods the TX queue at maximum rate when burst mode is active |
| `uart_task` | Low | Reads UART input, parses commands, dispatches mode changes and inject requests |

A shared TX queue decouples producers (`sim_task`, `burst_task`, inject) from the MCP2515 driver. A single driver context (no RTOS abstraction needed for one device) serialises all SPI transactions.

State machine for operating mode:

```
IDLE ──start──► SIMULATING ──burst on──► BURST
                     ▲                      │
                     └────── burst off ─────┘
SIMULATING ──stop──► IDLE
BURST ──stop──► IDLE
```

### 2.2 Hardware / Platform Architecture

#### ESP32-S3 Board

| Parameter | Value |
|---|---|
| Module | ESP32-S3-WROOM-2-N32R8V |
| Flash | 32 MB Octal SPI |
| PSRAM | 8 MB Octal SPI |
| Workbench slot | SLOT1 |
| USB connection | Direct USB (CDC + JTAG) |

#### MCP2515 SPI CAN Controller — Wiring

| MCP2515 Pin | ESP32-S3 GPIO | SPI2 Signal | Notes |
|---|---|---|---|
| VCC | 3.3V | — | Must be 3.3V — not 5V |
| GND | GND | — | |
| SCK | GPIO12 | SPI2_CLK | |
| SI (MOSI) | GPIO11 | SPI2_MOSI | |
| SO (MISO) | GPIO13 | SPI2_MISO | |
| CS | GPIO10 | GPIO output | Active-low chip select |
| INT | GPIO9 | GPIO input | Active-low interrupt (not used in polling mode) |

#### CAN Bus Topology

| Node | Role | Termination |
|---|---|---|
| MCP2515 #2 (ESP32-S3) | Generator | 120 Ω across CANH/CANL |
| MCP2515 #1 (STM32 Nucleo) | Sniffer | 120 Ω across CANH/CANL |

- Bus wiring: CANH–CANH, CANL–CANL between both modules.
- Bus speed: 250 kbit/s (SAE J1939 standard).
- Wire length: short bench cable (< 50 cm); twisted pair preferred.

### 2.3 Software Architecture

#### Build Environment

| Parameter | Value |
|---|---|
| SDK | ESP-IDF v5.x |
| Compiler | xtensa-esp32s3-elf-gcc |
| RTOS | FreeRTOS (included in ESP-IDF) |
| Build system | CMake / idf.py |

#### Module Decomposition

```
main/
  main.c              — App entry, task creation, nvs init
  mcp2515.c/.h        — MCP2515 SPI driver (init, tx, register access)
  can_generator.c/.h  — PGN scheduler, value simulation, burst mode
  uart_console.c/.h   — UART command parser and dispatcher
  j1939_frames.c/.h   — PGN frame construction helpers
```

#### Boot Sequence

1. ESP-IDF app starts; `app_main()` runs on CPU0.
2. SPI2 bus initialised with configured pins.
3. MCP2515 driver initialises: reset → configure CNF registers for 250 kbit/s → set normal mode.
4. If MCP2515 init fails, log error and halt (no silent degradation).
5. TX queue created (depth: 32 frames).
6. `sim_task`, `burst_task`, `uart_task` created.
7. Firmware prints boot banner + prompt to UART.
8. System enters IDLE mode awaiting `start` command.

#### Persistence / Storage

No NVS storage is used. All configuration is held in RAM and reset on power-cycle. (assumed)

---

## 3. Implementation Phases

### 3.1 Phase 1 — Infrastructure Foundation

**Scope**

- SPI2 bus initialisation and MCP2515 low-level driver (reset, register read/write, mode set, baud rate config).
- Single-frame TX path: enqueue one frame, verify it appears on the bus.
- UART console infrastructure: task, line buffer, echo.
- Verify SPI comms with MCP2515 by reading the CANSTAT/CANCTRL registers.

**Deliverables**

- `mcp2515.c/.h`: `mcp2515_init()`, `mcp2515_send_frame()`, `mcp2515_read_reg()`, `mcp2515_write_reg()`.
- UART console task with line input and basic echo.
- Boot banner printed on UART after successful MCP2515 init.
- Single manual `inject` command working (frame visible on STM32 sniffer).

**Exit Criteria**

- TC-1.1, TC-1.2, TC-1.3 pass.
- STM32 sniffer displays at least one injected frame with correct ID and payload.
- No SPI errors or MCP2515 error flags after 60 seconds of idle bus.

**Dependencies**

- MCP2515 module wired to ESP32-S3 per pin table in Section 2.2.
- STM32 sniffer firmware operational and monitoring the bus.

---

### 3.2 Phase 2 — Core Simulation and Command Interface

**Scope**

- PGN scheduler: four PGNs at correct intervals (100 ms / 1000 ms) using `esp_timer` or FreeRTOS `vTaskDelayUntil`.
- Value simulation: slowly varying RPM, speed, coolant temperature, fuel level.
- Burst mode task.
- Full UART command interface: `start`, `stop`, `burst on`, `burst off`, `inject`, `status`, `help`.
- Error detection: MCP2515 EFLG register monitoring.

**Deliverables**

- `can_generator.c/.h`: full PGN scheduler with all four PGNs, value models, burst mode.
- `j1939_frames.c/.h`: frame builder for each PGN.
- `uart_console.c/.h`: complete command parser with all commands implemented.
- `status` command output showing current mode, per-PGN TX counters, and error count.

**Exit Criteria**

- TC-2.1 through TC-2.6 pass.
- TC-3.1, TC-3.2 pass.
- TC-4.1 through TC-4.4 pass.
- TC-5.1 (STM32 sniffer decodes all four PGNs) passes.
- Simulated values confirmed non-static over a 60-second observation window.

**Dependencies**

- Phase 1 complete and exit criteria met.

---

## 4. Functional Requirements

### 4.1 Functional Requirements (FR)

#### FR-1: MCP2515 Initialisation

| ID | Priority | Requirement |
|---|---|---|
| FR-1.1 | Must | The firmware shall initialise the MCP2515 over SPI2 at startup using the pin assignment in Section 2.2. |
| FR-1.2 | Must | The firmware shall configure the MCP2515 for 250 kbit/s operation (CNF1/CNF2/CNF3 for an 8 MHz MCP2515 crystal). |
| FR-1.3 | Must | The firmware shall set the MCP2515 to normal operating mode before any frame is transmitted. |
| FR-1.4 | Must | If MCP2515 initialisation fails (register read-back mismatch), the firmware shall log a fatal error to UART and halt task execution. |
| FR-1.5 | Should | The firmware shall print a boot banner to UART on successful init, including the detected MCP2515 mode. |

#### FR-2: J1939 PGN Simulation

| ID | Priority | Requirement |
|---|---|---|
| FR-2.1 | Must | The firmware shall transmit PGN 61444 (EEC1 — Engine Speed) every 100 ms ± 5 ms while in SIMULATING mode. |
| FR-2.2 | Must | PGN 61444 shall encode engine speed (SPN 190) as a 16-bit value in bytes 3–4 (0.125 RPM/bit), varying between 750 RPM and 2200 RPM. |
| FR-2.3 | Must | The firmware shall transmit PGN 65262 (Engine Temperature 1 — Coolant) every 1000 ms ± 20 ms while in SIMULATING mode. |
| FR-2.4 | Must | PGN 65262 shall encode coolant temperature (SPN 110) in byte 0 (1°C/bit, offset −40°C), varying between 75°C and 95°C. |
| FR-2.5 | Must | The firmware shall transmit PGN 65265 (CCVS — Vehicle Speed) every 100 ms ± 5 ms while in SIMULATING mode. |
| FR-2.6 | Must | PGN 65265 shall encode wheel-based vehicle speed (SPN 84) in bytes 0–1 (1/256 km/h per bit), varying between 0 km/h and 120 km/h. |
| FR-2.7 | Must | The firmware shall transmit PGN 65276 (Dash Display — Fuel Level) every 1000 ms ± 20 ms while in SIMULATING mode. |
| FR-2.8 | Must | PGN 65276 shall encode fuel level (SPN 96) in byte 1 (0.4%/bit), varying between 30% and 95%. |
| FR-2.9 | Must | All simulated signal values shall change incrementally on each update tick — the value after 10 seconds shall differ from the initial value for each signal. |
| FR-2.10 | Must | All PGN frames shall use 29-bit extended CAN IDs in J1939 format: priority (3 bits) + reserved (1 bit) + data page (1 bit) + PF (8 bits) + PS (8 bits) + SA (8 bits). |
| FR-2.11 | Should | The source address (SA) byte of all transmitted frames shall be 0x00 (engine ECU address). |

#### FR-3: Burst Mode

| ID | Priority | Requirement |
|---|---|---|
| FR-3.1 | Must | The firmware shall implement a burst mode that transmits CAN frames at the maximum rate the MCP2515 hardware supports. |
| FR-3.2 | Must | In burst mode, the firmware shall disable the normal PGN scheduler and transmit a fixed 8-byte frame continuously. |
| FR-3.3 | Must | Burst mode shall be activated and deactivated via the UART command interface. |
| FR-3.4 | Should | The firmware shall count the total frames transmitted in burst mode and report the count via the `status` command. |

#### FR-4: UART Command Interface

| ID | Priority | Requirement |
|---|---|---|
| FR-4.1 | Must | The firmware shall provide a UART command interface at 115200 baud, 8N1, on UART0. |
| FR-4.2 | Must | The `start` command shall transition the system from IDLE to SIMULATING mode. |
| FR-4.3 | Must | The `stop` command shall transition the system to IDLE mode from any active mode. |
| FR-4.4 | Must | The `burst on` command shall activate burst mode (disabling normal simulation if active). |
| FR-4.5 | Must | The `burst off` command shall deactivate burst mode and return to SIMULATING mode if simulation was active before burst, or IDLE otherwise. |
| FR-4.6 | Must | The `inject <id_hex> <data_hex>` command shall transmit a single CAN frame with the specified 29-bit ID and up to 8 payload bytes. |
| FR-4.7 | Must | The `status` command shall print: current mode, per-PGN TX frame counters, total burst frames, and MCP2515 error flag register value. |
| FR-4.8 | Should | The `help` command shall print a summary of all available commands. |
| FR-4.9 | Should | On receipt of an unrecognised command, the firmware shall print an error message and a hint to type `help`. |
| FR-4.10 | Should | Each successfully executed command shall receive an acknowledgement line on UART (e.g., `OK: simulation started`). |

#### FR-5: Error Handling

| ID | Priority | Requirement |
|---|---|---|
| FR-5.1 | Should | The firmware shall periodically read the MCP2515 EFLG register and log any error flags (RX0OVR, RX1OVR, TXBO, TXEP, RXEP) to UART. |
| FR-5.2 | Should | On detection of a bus-off condition (TXBO flag), the firmware shall attempt recovery by resetting the MCP2515 and re-entering normal mode. |
| FR-5.3 | May | The firmware shall log a warning to UART if the TX queue fills to capacity during burst mode. |

---

### 4.2 Non-Functional Requirements (NFR)

| ID | Priority | Requirement |
|---|---|---|
| NFR-1.1 | Must | The CAN bus shall operate at 250 kbit/s ± 1% (SAE J1939 standard baud rate). |
| NFR-1.2 | Must | PGN 61444 and PGN 65265 shall be transmitted within ±5 ms of their 100 ms nominal interval under normal load. |
| NFR-1.3 | Must | PGN 65262 and PGN 65276 shall be transmitted within ±20 ms of their 1000 ms nominal interval. |
| NFR-2.1 | Must | Firmware shall be built with ESP-IDF v5.x using the `esp32s3` target. |
| NFR-2.2 | Should | SPI2 clock shall be configured at ≥ 4 MHz to minimise per-frame SPI transfer latency. |
| NFR-2.3 | Should | Burst mode frame rate shall be limited only by MCP2515 hardware capacity, not by software polling delays. |
| NFR-3.1 | Should | UART command response latency shall be < 50 ms from newline receipt. |
| NFR-4.1 | Should | Firmware shall operate continuously for ≥ 24 hours without crash, watchdog reset, or bus-off condition under normal simulation mode. |

---

### 4.3 Constraints

| ID | Constraint |
|---|---|
| C-1 | MCP2515 VCC must be powered from 3.3V to match ESP32-S3 GPIO logic levels. 5V will damage the GPIO pins. |
| C-2 | A 120 Ω termination resistor must be present at each end of the CAN bus (one at the ESP32-S3 module, one at the STM32 module). |
| C-3 | All J1939 PGN frames must use 29-bit extended CAN IDs. The MCP2515 must be configured for extended frame format (EXIDE = 1 in TXBnSIDL). |
| C-4 | Only one client may open the UART console at a time (RFC2217 proxy port 4001 or direct USB CDC). |
| C-5 | The MCP2515 crystal frequency assumed is 8 MHz. CNF register values must be recalculated if a different crystal is fitted. |

---

## 5. Risks, Assumptions and Dependencies

| # | Type | Description | Likelihood | Impact | Mitigation |
|---|---|---|---|---|---|
| R-1 | Technical Risk | MCP2515 CNF register values for 250 kbit/s depend on crystal frequency. An unmarked 16 MHz crystal would cause wrong baud rate. | Medium | High | Verify crystal frequency with oscilloscope or by reading CANSTAT after loopback test. Document CNF values in Appendix. |
| R-2 | Technical Risk | SPI bus conflicts if another peripheral shares SPI2. | Low | Medium | Verify no other SPI device is assigned to GPIO10–13 in sdkconfig. |
| R-3 | Technical Risk | MCP2515 INT pin (GPIO9) not used in initial implementation; polling mode may miss errors under high load. | Low | Low | Phase 2 adds EFLG polling via `status` command. INT-driven error handling may be added later if needed. |
| R-4 | Technical Risk | FreeRTOS task priority inversion if `sim_task` and `burst_task` contend on the TX queue. | Low | Medium | Use a single FreeRTOS queue with proper blocking; only one producer active at a time (burst mode disables sim_task). |
| R-5 | Dependency | STM32 sniffer firmware must be operational to verify end-to-end success criteria. | — | — | Phase 1 exit criteria only require a single injected frame to appear on the sniffer. Full verification deferred to Phase 2. |

**Assumptions**

- MCP2515 module uses an 8 MHz crystal. (assumed)
- MCP2515 INT output is open-drain with pull-up on the module PCB. (assumed)
- ESP-IDF SPI master driver is used (not bit-bang). (assumed)
- UART0 is available as the console UART (no conflict with other CDC interface). (assumed)
- The STM32 sniffer has independent 120 Ω termination fitted on its MCP2515 module. (assumed)

---

## 6. Interface Specifications

### 6.1 External Interfaces

#### CAN Bus (J1939 Physical Layer)

| Parameter | Value |
|---|---|
| Standard | ISO 11898-2 (CAN high-speed) |
| Baud rate | 250 kbit/s |
| Frame format | CAN 2.0B extended (29-bit ID) |
| Termination | 120 Ω at each node |
| Topology | Two-node bus (generator + sniffer) |

#### UART Console

| Parameter | Value |
|---|---|
| Interface | UART0 (USB CDC via SLOT1) |
| Baud rate | 115200 |
| Frame format | 8N1 |
| Line ending | `\r\n` (output) / `\n` or `\r` accepted (input) |

---

### 6.2 Internal Interfaces

| Interface | Type | Description |
|---|---|---|
| TX queue | FreeRTOS queue | Carries `can_frame_t` structs from producers to MCP2515 driver. Depth: 32 frames. |
| Mode flag | Atomic / mutex-protected global | `IDLE`, `SIMULATING`, `BURST` — shared between UART task (writer) and generator/burst tasks (readers). |
| Sim state | Struct in `can_generator.c` | Holds current simulated values for each signal; updated by `sim_task` only. |

---

### 6.3 Data Models

#### `can_frame_t` (internal TX queue element)

```c
typedef struct {
    uint32_t id;       /* 29-bit J1939 CAN ID, bit 31 set = extended */
    uint8_t  dlc;      /* Data length code (0–8) */
    uint8_t  data[8];  /* Payload bytes */
} can_frame_t;
```

#### J1939 29-bit ID Construction

```
Bit 28-26: Priority (3 bits)     — default 3 (0b011) for most PGNs
Bit 25:    Reserved (R)          — 0
Bit 24:    Data Page (DP)        — 0 for PGN < 0x10000
Bit 23-16: PDU Format (PF)       — upper byte of PGN
Bit 15-8:  PDU Specific (PS)     — lower byte of PGN (for PF >= 240, this is group extension)
Bit 7-0:   Source Address (SA)   — 0x00 for engine ECU
```

#### PGN Frame Layouts

**PGN 61444 (0xF004) — EEC1 (Electronic Engine Controller 1)**

| Byte | SPN | Description | Encoding | Simulated Range |
|---|---|---|---|---|
| 0 | 899 | Engine torque mode | 4-bit enum, lower nibble | 0x00 (no request) |
| 1 | 512 | Driver demand engine torque | 1%/bit, offset −125% | 0x7D (0% net) |
| 2 | 513 | Actual engine torque | 1%/bit, offset −125% | 0x7D (0% net) |
| 3–4 | 190 | Engine speed | 16-bit LE, 0.125 RPM/bit | 750–2200 RPM |
| 5 | 1483 | Source address — controlling device | Raw byte | 0xFF (N/A) |
| 6 | 1675 | Engine starter mode | 4-bit nibble | 0x0F (N/A) |
| 7 | 2432 | Engine demand torque | 1%/bit, offset −125% | 0x7D |

CAN ID: `0x0CF00400` (priority=3, DP=0, PF=0xF0, PS=0x04, SA=0x00)

**PGN 65262 (0xFEEE) — Engine Temperature 1**

| Byte | SPN | Description | Encoding | Simulated Range |
|---|---|---|---|---|
| 0 | 110 | Engine coolant temperature | 1°C/bit, offset −40°C → raw = temp + 40 | 75–95°C (raw 115–135) |
| 1–7 | — | Other temperature SPNs | 0xFF (not available) | — |

CAN ID: `0x18FEEE00` (priority=6, DP=0, PF=0xFE, PS=0xEE, SA=0x00)

**PGN 65265 (0xFEF1) — CCVS (Cruise Control/Vehicle Speed)**

| Byte | SPN | Description | Encoding | Simulated Range |
|---|---|---|---|---|
| 0–1 | 84 | Wheel-based vehicle speed | 16-bit LE, 1/256 km/h per bit | 0–120 km/h |
| 2–7 | — | Other CCVS SPNs | 0xFF (not available) | — |

CAN ID: `0x18FEF100` (priority=6, DP=0, PF=0xFE, PS=0xF1, SA=0x00)

**PGN 65276 (0xFEFC) — Dash Display**

| Byte | SPN | Description | Encoding | Simulated Range |
|---|---|---|---|---|
| 0 | 80 | Washer fluid level | 0.4%/bit | 0xFF (not available) |
| 1 | 96 | Fuel level 1 | 0.4%/bit → raw = fuel% / 0.4 | 30–95% (raw 75–237) |
| 2–7 | — | Other dash SPNs | 0xFF (not available) | — |

CAN ID: `0x18FEFC00` (priority=6, DP=0, PF=0xFE, PS=0xFC, SA=0x00)

---

### 6.4 Commands / Opcodes

All commands are ASCII, terminated by `\n` or `\r\n`. The parser shall be case-insensitive for command keywords.

| Command | Syntax | Description | Response on Success |
|---|---|---|---|
| `start` | `start` | Begin periodic PGN simulation (IDLE → SIMULATING) | `OK: simulation started` |
| `stop` | `stop` | Halt all transmission (any mode → IDLE) | `OK: stopped` |
| `burst on` | `burst on` | Activate burst mode | `OK: burst mode ON` |
| `burst off` | `burst off` | Deactivate burst mode | `OK: burst mode OFF` |
| `inject` | `inject <id_hex> <data_hex>` | Transmit one frame. `id_hex`: 8 hex digits (29-bit ID); `data_hex`: 1–16 hex digits (1–8 bytes) | `OK: frame queued` |
| `status` | `status` | Print current mode and counters | See §6.4.1 |
| `help` | `help` | Print command summary | Command list |

#### 6.4.1 `status` Command Output Format

```
Mode:         SIMULATING
PGN 61444 TX: 12340 frames
PGN 65262 TX: 1234 frames
PGN 65265 TX: 12340 frames
PGN 65276 TX: 1234 frames
Burst TX:     0 frames
MCP2515 EFLG: 0x00 (no errors)
```

#### 6.4.2 `inject` Command Examples

```
inject 0CF00400 0000FF1400FF0F7D    # PGN 61444 with engine speed ~500 RPM
inject 18FEEE00 73FFFFFFFFFFFFFF    # PGN 65262 with coolant 75°C (0x73 = 115 raw)
```

---

## 7. Operational Procedures

### 7.1 Hardware Setup

1. Wire MCP2515 module to ESP32-S3 per pin table in Section 2.2. Verify VCC is 3.3V before powering on.
2. Connect CANH and CANL between ESP32-S3 MCP2515 and STM32 MCP2515.
3. Verify 120 Ω termination resistors are present at both ends of the bus.
4. Connect ESP32-S3 SLOT1 USB to the workbench machine.

### 7.2 Build and Flash

```bash
# Set up ESP-IDF environment (adjust path to your IDF install)
source ~/esp/esp-idf/export.sh

# Configure (verify target and flash size)
idf.py set-target esp32s3
idf.py menuconfig    # confirm flash size = 32MB, PSRAM = OPI 8MB

# Build
idf.py build

# Flash (device will enumerate as ttyACM0 or via bench_tab5 symlink)
idf.py -p /dev/ttyACM0 flash

# Monitor UART output
idf.py -p /dev/ttyACM0 monitor
```

> Do not use `hard-reset` or `watchdog-reset` after flashing native USB ESP32-S3. Use `no-reset` then trigger a soft reset via the portal API or the monitor tool.

### 7.3 Normal Operation

1. After boot, the UART console prints the banner and prompt.
2. Type `start` to begin J1939 simulation.
3. Observe the STM32 sniffer terminal — all four PGNs should appear within 1–2 seconds.
4. Type `status` to confirm frame counters are incrementing.
5. Type `stop` to halt transmission.

### 7.4 Burst Mode

1. Ensure simulation is running or the system is in IDLE.
2. Type `burst on`. The STM32 sniffer ring buffer should show a high-rate flood of frames.
3. Observe sniffer for dropped frame counts or overflow indication.
4. Type `burst off` to return to normal operation.

### 7.5 Ad-hoc Frame Injection

```
inject 18FEF100 0000FFFFFFFFFFFF
```

This injects a single PGN 65265 frame with vehicle speed = 0 km/h. The sniffer should display it immediately.

### 7.6 Recovery Procedures

| Scenario | Recovery Action |
|---|---|
| MCP2515 init failure on boot | Check SPI wiring and VCC voltage. Power-cycle the module. Re-flash. |
| Bus-off condition (TXBO set) | Firmware auto-resets MCP2515 (FR-5.2). If bus-off persists, check termination and wiring. |
| UART unresponsive | Power-cycle the ESP32-S3. Use portal reset API: `POST localhost:8080/api/serial/reset` with `{"slot":"SLOT1"}`. |
| No frames seen on sniffer | Run `status` — check MCP2515 EFLG. Verify CAN bus wiring and termination. Try `inject` command and observe sniffer. |

---

## 8. Verification and Validation

### 8.1 Phase 1 Verification

| Test ID | Feature | Procedure | Success Criteria |
|---|---|---|---|
| TC-1.1 | MCP2515 SPI comms | Read CANSTAT register immediately after reset. | Returns 0x80 (configuration mode, no errors). |
| TC-1.2 | 250 kbit/s config | Read CNF1, CNF2, CNF3 registers after init. | Values match computed 250 kbit/s settings for 8 MHz crystal (see Appendix A.2). |
| TC-1.3 | Normal mode entry | Read CANCTRL REQOP bits after init. | REQOP = 0b000 (normal mode). |
| TC-1.4 | Single frame TX | Issue `inject 0CF00400 0000FF1400FF0F7D` via UART. | STM32 sniffer displays frame with ID 0x0CF00400 and matching payload within 50 ms. |
| TC-1.5 | Boot banner | Power-cycle and observe UART. | Banner printed within 2 seconds including MCP2515 mode confirmation. |

---

### 8.2 Phase 2 Verification

| Test ID | Feature | Procedure | Success Criteria |
|---|---|---|---|
| TC-2.1 | PGN 61444 interval | Start simulation; timestamp 100 consecutive PGN 61444 frames on STM32 sniffer. | Inter-frame interval: 100 ms ± 5 ms for all samples. |
| TC-2.2 | PGN 61444 RPM variation | Observe PGN 61444 bytes 3–4 over 30 seconds. | RPM value changes on every frame; stays within 750–2200 RPM. |
| TC-2.3 | PGN 65262 interval | Timestamp 10 consecutive PGN 65262 frames on sniffer. | Inter-frame interval: 1000 ms ± 20 ms. |
| TC-2.4 | PGN 65265 interval | Timestamp 100 consecutive PGN 65265 frames on sniffer. | Inter-frame interval: 100 ms ± 5 ms. |
| TC-2.5 | PGN 65276 interval | Timestamp 10 consecutive PGN 65276 frames on sniffer. | Inter-frame interval: 1000 ms ± 20 ms. |
| TC-2.6 | All PGNs present | Run simulation for 10 seconds; collect sniffer output. | All four PGNs (61444, 65262, 65265, 65276) seen with correct extended IDs. |
| TC-3.1 | Burst mode activation | Issue `burst on`; observe sniffer frame rate. | Sniffer shows continuous frame flood; frame rate > 1000 frames/sec (assuming 250 kbit/s, 8-byte frames). |
| TC-3.2 | Burst mode deactivation | Issue `burst off`; observe sniffer. | Frame flood stops; if simulation was active, PGN frames resume at normal intervals within 200 ms. |
| TC-4.1 | `start`/`stop` commands | Issue `start`, wait 2 s, issue `stop`. Observe UART responses and sniffer. | Correct `OK:` responses; sniffer shows frames during active window only. |
| TC-4.2 | `burst on`/`burst off` | Issue `burst on`, wait 5 s, issue `burst off`. | Correct `OK:` responses; sniffer frame pattern matches burst, then stops or resumes PGNs. |
| TC-4.3 | `inject` command | Issue `inject 18FEEE00 73FFFFFFFFFFFFFF`. | Sniffer displays exactly one PGN 65262 frame with coolant = 75°C (byte 0 = 0x73). |
| TC-4.4 | Unknown command | Type `foo` at the UART prompt. | Firmware responds with error message and hint to type `help`. |
| TC-5.1 | End-to-end sniffer decode | Run simulation for 60 seconds; collect sniffer terminal output. | STM32 sniffer decodes and displays all four PGNs with human-readable values (RPM, km/h, °C, % fuel). Zero frame decode errors logged by sniffer. |

---

### 8.3 Acceptance Tests

| Test ID | Scenario | Procedure | Success Criteria |
|---|---|---|---|
| AT-1 | Full simulation run | Start simulation; observe sniffer for 5 minutes. | All four PGNs received continuously; all simulated values change over the observation window; no bus-off events; no firmware crashes. |
| AT-2 | Burst stress test | Activate burst mode for 60 seconds; observe sniffer ring buffer statistics. | Sniffer buffer never overflows (zero frame drops); firmware remains responsive to UART commands during burst; bus-off not triggered. |
| AT-3 | 24-hour stability | Start simulation and leave running for 24 hours. | No watchdog reset, no bus-off, no memory exhaustion; frame counters still incrementing at end of window. |

---

### 8.4 Traceability Matrix

| Requirement | Priority | Test Case(s) | Status |
|---|---|---|---|
| FR-1.1 | Must | TC-1.1 | Covered |
| FR-1.2 | Must | TC-1.2 | Covered |
| FR-1.3 | Must | TC-1.3 | Covered |
| FR-1.4 | Must | TC-1.5 | Covered |
| FR-1.5 | Should | TC-1.5 | Covered |
| FR-2.1 | Must | TC-2.1, TC-2.6 | Covered |
| FR-2.2 | Must | TC-2.2 | Covered |
| FR-2.3 | Must | TC-2.3, TC-2.6 | Covered |
| FR-2.4 | Must | TC-4.3 (inject verify), AT-1 | Covered |
| FR-2.5 | Must | TC-2.4, TC-2.6 | Covered |
| FR-2.6 | Must | TC-2.4, AT-1 | Covered |
| FR-2.7 | Must | TC-2.5, TC-2.6 | Covered |
| FR-2.8 | Must | TC-2.5, AT-1 | Covered |
| FR-2.9 | Must | TC-2.2, AT-1 | Covered |
| FR-2.10 | Must | TC-1.4, TC-2.6 | Covered |
| FR-2.11 | Should | TC-2.6 (sniffer SA decode) | Covered |
| FR-3.1 | Must | TC-3.1, AT-2 | Covered |
| FR-3.2 | Must | TC-3.1 | Covered |
| FR-3.3 | Must | TC-4.2 | Covered |
| FR-3.4 | Should | TC-4.1 (`status` output) | Covered |
| FR-4.1 | Must | TC-1.5 (boot banner on UART) | Covered |
| FR-4.2 | Must | TC-4.1 | Covered |
| FR-4.3 | Must | TC-4.1 | Covered |
| FR-4.4 | Must | TC-4.2 | Covered |
| FR-4.5 | Must | TC-4.2 | Covered |
| FR-4.6 | Must | TC-4.3 | Covered |
| FR-4.7 | Must | TC-4.1 (status counters) | Covered |
| FR-4.8 | Should | TC-4.4 | Covered |
| FR-4.9 | Should | TC-4.4 | Covered |
| FR-4.10 | Should | TC-4.1, TC-4.2, TC-4.3 | Covered |
| FR-5.1 | Should | AT-1 (status check after run) | Covered |
| FR-5.2 | Should | — | GAP — requires deliberate bus fault to trigger; manual test only |
| FR-5.3 | May | AT-2 | Covered |
| NFR-1.1 | Must | TC-1.2 (CNF registers) | Covered |
| NFR-1.2 | Must | TC-2.1, TC-2.4 | Covered |
| NFR-1.3 | Must | TC-2.3, TC-2.5 | Covered |
| NFR-2.1 | Must | Build system check | Covered |
| NFR-2.2 | Should | — | GAP — SPI clock config not directly tested; verify in sdkconfig |
| NFR-2.3 | Should | TC-3.1 (frame rate measurement) | Covered |
| NFR-3.1 | Should | TC-4.1 (manual timing) | Covered |
| NFR-4.1 | Should | AT-3 | Covered |

---

## 9. Troubleshooting Guide

| Symptom | Likely Cause | Diagnostic Steps | Corrective Action |
|---|---|---|---|
| Boot banner not printed; no UART output | UART0 not enumerated; firmware crashed before UART init | Check `/dev/ttyACM*` devices; try hard reset then monitor | Verify USB cable; check IDF UART console config in menuconfig |
| `MCP2515 init failed` on boot | SPI wiring error; VCC at 5V; CS pin conflict | Scope SPI lines during init; measure VCC with meter | Re-check pin connections; ensure VCC = 3.3V |
| MCP2515 init succeeds but no frames on sniffer | MCP2515 in loopback or listen-only mode; CANCTRL not set to normal | Read CANCTRL via `status`; check EFLG for TXBO | Ensure FR-1.3 code sets REQOP = 0b000; check for bus-off recovery |
| Frames on sniffer with wrong baud rate (garbage) | Crystal mismatch (8 MHz expected, 16 MHz fitted) | Oscilloscope on MCP2515 OSC1 pin; measure clock | Recalculate CNF1/2/3 for actual crystal; update firmware |
| Interval jitter > ±5 ms on 100 ms PGNs | FreeRTOS tick resolution too coarse; task preemption | Check `CONFIG_FREERTOS_HZ` (should be ≥ 1000); profile sim_task timing | Increase FreeRTOS tick rate in menuconfig; use `esp_timer` instead of `vTaskDelay` |
| Bus-off condition (EFLG TXBO set) | Missing termination; short circuit on CAN wires; sniffer not connected | Measure resistance across CANH/CANL (should be ~60 Ω with both terminators) | Add/verify 120 Ω terminators; check wiring continuity |
| Sniffer sees wrong PGN ID (wrong byte order) | 29-bit ID constructed with wrong byte order in MCP2515 TXBSIDH/TXBSIDL/TXBEID8/TXBEID0 | Compare raw CAN frame ID on logic analyser vs expected value | Review MCP2515 extended ID register mapping in datasheet; see Appendix A.3 |
| `inject` command returns `ERROR: bad format` | ID not 8 hex digits; data has spaces in wrong place | Re-read §6.4.2 examples | Correct command syntax |

---

## 10. Appendix

### A.1 Signal Value Simulation Model

Each signal steps by a fixed increment on each update tick. Direction reverses at limits.

| Signal | PGN | Update Interval | Step Size | Min Value | Max Value | Units |
|---|---|---|---|---|---|---|
| Engine speed | 61444 | 100 ms | ±15 | 750 | 2200 | RPM |
| Vehicle speed | 65265 | 100 ms | ±0.5 | 0 | 120 | km/h |
| Coolant temperature | 65262 | 1000 ms | ±0.2 | 75 | 95 | °C |
| Fuel level | 65276 | 1000 ms | −0.05 (drain only; wraps at min back to max) | 30 | 95 | % |

### A.2 MCP2515 Baud Rate Register Values (250 kbit/s, 8 MHz Crystal)

Computed for: Fosc = 8 MHz, TQ = 2 × (1/Fosc) = 250 ns, bit time = 16 TQ.

| Register | Value | Notes |
|---|---|---|
| CNF1 | 0x00 | BRP = 0 (prescaler = 1×2 = 2); SJW = 1 TQ |
| CNF2 | 0xB1 | BTLMODE = 1; SAM = 0; PHSEG1 = 6 TQ; PRSEG = 2 TQ |
| CNF3 | 0x05 | PHSEG2 = 6 TQ |

Bit timing: Sync (1) + PropSeg (2) + PhSeg1 (6) + PhSeg2 (6) = 16 TQ → 250 kbit/s ✓

Sample point: 81.25% (after PhSeg1).

> If the crystal is 16 MHz, set BRP = 1 (CNF1 = 0x01) and keep CNF2/CNF3 unchanged.

### A.3 MCP2515 Extended Frame ID Register Mapping

For a 29-bit CAN ID stored in variable `uint32_t id`:

```c
/* MCP2515 TXBnSIDH / TXBnSIDL / TXBnEID8 / TXBnEID0 */
txbuf[0] = (id >> 21) & 0xFF;              /* SIDH: bits 28–21 */
txbuf[1] = ((id >> 13) & 0xE0) |           /* SIDL: bits 20–18 */
           (1 << 3) |                       /* EXIDE = 1 (extended frame) */
           ((id >> 16) & 0x03);             /* SIDL: bits 17–16 */
txbuf[2] = (id >> 8) & 0xFF;               /* EID8: bits 15–8 */
txbuf[3] = id & 0xFF;                      /* EID0: bits 7–0 */
```

### A.4 J1939 CAN ID Quick Reference

| PGN | Decimal | CAN ID (hex, SA=0x00) | Priority |
|---|---|---|---|
| 61444 (EEC1 — Engine Speed) | 61444 | 0x0CF00400 | 3 |
| 65262 (Engine Temp 1) | 65262 | 0x18FEEE00 | 6 |
| 65265 (CCVS — Vehicle Speed) | 65265 | 0x18FEF100 | 6 |
| 65276 (Dash Display — Fuel) | 65276 | 0x18FEFC00 | 6 |

### A.5 Example UART Session

```
I (523) mcp2515: CANSTAT=0x80, init OK — entering normal mode
I (524) mcp2515: CNF1=0x00 CNF2=0xB1 CNF3=0x05

CAN Traffic Generator — ready
Type 'help' for commands.

> start
OK: simulation started

> status
Mode:         SIMULATING
PGN 61444 TX: 47 frames
PGN 65262 TX: 4 frames
PGN 65265 TX: 47 frames
PGN 65276 TX: 4 frames
Burst TX:     0 frames
MCP2515 EFLG: 0x00 (no errors)

> inject 18FEEE00 73FFFFFFFFFFFFFF
OK: frame queued

> burst on
OK: burst mode ON

> burst off
OK: burst mode OFF

> stop
OK: stopped
```
