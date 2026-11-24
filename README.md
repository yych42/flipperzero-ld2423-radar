# Flipper Zero HLK-LD2423 Radar App

> **Warning — Hardware Testing Pending:** This application was built against Hi-Link's official HLK-LD2423 V1.00 datasheet and serial protocol specification, compiled cleanly for Flipper Zero (SDK 1.5.1), and verified with protocol unit tests, **but has not yet been tested on physical HLK-LD2423 hardware**. Verify your wiring carefully and expect minor adjustments once bench-tested.

Turn your Flipper Zero into a **portable radar display and remote control** for the **Hi-Link HLK-LD2423** 24GHz millimeter-wave human presence sensor.

## Overview

Plug in an HLK-LD2423 module and this app gives you four screens you can flip through with Left/Right:

- **Live Radar Scope:** Plots up to 3 people at once out to the sensor's full **8-meter (~26 ft)** range. You can zoom in or out (`2m`–`8m`), watch dotted movement trails, and tell moving people (solid dots) apart from people sitting or standing still (hollow rings) at a glance.
- **Live Measurements:** Lists each tracked person's distance, direction angle, and walking speed (or `HOLD` when stationary), with a one-button toggle to Left/Right + Forward (X/Y) coordinates.
- **On-Device Sensor Control:** Adjust the radar's max range, detection width, and single- vs. multi-person mode right from your Flipper—no PC tool needed. Also includes Metric/Imperial units, X-axis mirroring for inverted mounts, optional LED/vibration arrival alerts, and restart/factory reset.
- **Status & Pinout:** Shows firmware version, active detection limits, live connection health, and a built-in wiring cheat sheet.

The app also handles Flipper Zero housekeeping automatically—it turns on the 5V GPIO rail on launch (and restores it on exit), frees the serial port from background expansion settings, and keeps the screen awake while monitoring.

## Technical Features

- **Native HLK-LD2423 V1.00 Protocol Support**:
  - Defaults to **115200 baud (8N1)**, with live switching to **256000** for sensors configured to a higher baud rate.
  - Full state-machine stream parser for both **30-byte target report frames** (`AA FF 03 00 ... 55 CC`) and **bidirectional command ACKs** (`FD FC FB FA ... 04 03 02 01`) with clean resynchronization.
- **Full 8 m Range & True-Scale Polar Radar Scope**:
  - Clipped fan radar arcs matching the sensor's configured detection angle (up to **±60°** configurable, 130° antenna FOV) and full **8.0 m (~26 ft)** range.
  - Instant **Up/Down zoom** (`2m` / `4m` / `6m` / `8m`, or feet in Imperial mode), with outer-rim markers for targets beyond the current zoom scale.
  - **Static vs. Moving target distinction** (highlighting LD2423's stationary human presence detection): hollow ring for static/stationary targets (`|speed| ≤ 5 cm/s`), solid disc with approach/recede velocity tick for moving targets.
  - **Live trajectory trails** (per-target breadcrumb history) toggled with `OK`.
- **On-Device HLK-LD2423 Hardware Control**:
  - Configure **Max Detection Distance** (`1.0m`–`8.0m`, cmd `0x09`) and **Max Angle** (`±15°`–`±60°`, cmd `0x09`).
  - Toggle **Multi-Target (3)** vs. **Single-Target (1)** tracking mode (cmd `0x0B`).
  - Automatically queries **Firmware Version** (cmd `0x06`) and **Active Testing Scope** (cmd `0x0A`) on startup.
  - Trigger **Radar Restart** (cmd `0x0C`) or **Factory Reset** (cmd `0x0D`) directly from the Flipper.
- **Flipper Zero Conveniences**:
  - **Auto 5V GPIO (OTG)** power enable on launch and clean restore on exit.
  - Optional **Arrival Alerts** (LED blink or short vibration pulse when target count goes `0 → ≥1`).
  - **Polar (`Dist / Angle / Speed`)** and **Cartesian (`X / Y / Speed`)** telemetry views, Metric/Imperial toggle, and `Mirror X` for flipped mounting.

---

## Hardware Wiring

The HLK-LD2423 uses a **ZH1.5-4P** connector on the back of the PCB (numbered **top to bottom** per the official datasheet). Always verify pin order before powering on:

| HLK-LD2423 Pin (ZH1.5-4P) | Signal | Flipper Zero GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **Pin 1** | `GND` | **GND** (Pin 8) | Any Flipper GND pin |
| **Pin 2** | `TX` (0–3.3V) | **RX** (Pin 14) | Flipper USART RX |
| **Pin 3** | `RX` (0–3.3V) | **TX** (Pin 13) | Flipper USART TX (needed for Config/Query commands) |
| **Pin 4** | `5V` (4.5–5.5V) | **5V** (Pin 1) | Avg ≤127 mA; app auto-enables 5V OTG rail |

> **Mounting Note:** Per the HLK-LD2423 datasheet, the module is directional and must be oriented **horizontally** (wide side horizontal); vertical mounting is not supported.

---

## Controls & Screens

Use **Left / Right** at any time to cycle through all 4 screens, and **Back** to exit.

1. **Radar** *(default)*
   - `Up` / `Down`: Zoom range (`2m` / `4m` / `6m` / `8m`)
   - `OK` (short): Toggle trajectory trails on/off
   - `OK` (long): Clear current trails
2. **Targets** *(live telemetry for T1–T3)*
   - `OK`: Toggle between **Polar** (`Dist`, `Angle`, `Speed` / `HOLD`) and **Cartesian** (`X`, `Y`, `Speed`)
3. **Config** *(LD2423 & UI settings)*
   - `Up` / `Down`: Navigate settings
   - `OK`: Adjust setting or send command (`Max Dist`, `Max Angle`, `Track Mode`, `Units`, `Trails`, `Mirror X`, `Arrival`, `UART Baud`, `Query Radar`, `Restart Radar`, `Factory Reset`)
4. **Info** *(diagnostics & pinout reference)*
   - Displays firmware version, active scope, range resolution (`740 mm`), packet/error counters, Hz, baud rate, 5V OTG state, and ZH1.5-4P pinout
   - `OK`: Re-query firmware version and testing scope

---

## Building & Running

### Build / Launch with uFBT

```bash
ufbt
# Or compile and launch directly on a USB-connected Flipper Zero:
ufbt launch
```

The compiled `.fap` is written to `dist/ld2423_radar.fap` (category: **GPIO**).

### Run Protocol Unit Tests (Host)

```bash
cc -std=c11 -Wall -Wextra -Werror ld2423_protocol.c tests/test_ld2423_protocol.c -lm -o /tmp/test_ld2423 && /tmp/test_ld2423
```

---

## Using with an ESP32-S2 (Standalone)

`ld2423_protocol.h` and `ld2423_protocol.c` are pure standard C with **zero Flipper Zero dependencies**, so you can drop them straight into an **ESP32-S2** project (Arduino IDE, PlatformIO, or ESP-IDF) to log data over USB or stream targets over Wi-Fi.

### ESP32-S2 Wiring

| HLK-LD2423 Pin (ZH1.5-4P) | Signal | ESP32-S2 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Pin 1** | `GND` | **GND** | Common ground |
| **Pin 2** | `TX` (0–3.3V) | Any free GPIO (e.g. **GPIO 18**) | Configured as UART `RX` (safe for 3.3V logic directly) |
| **Pin 3** | `RX` (0–3.3V) | Any free GPIO (e.g. **GPIO 17**) | *Optional* — configured as UART `TX` (only needed to send config commands) |
| **Pin 4** | `5V` (4.5–5.5V) | **5V / VBUS** | Do **not** power from `3.3V`—the LD2423 requires 4.5–5.5V (~127 mA avg) |

> **3-Wire Receive-Only Mode:** If you only have 3 wires available (`5V`, `GND`, and `LD2423 TX → ESP32-S2 RX`), leave Pin 3 (`RX`) disconnected and pass `-1` for the TX pin in `Serial1.begin(...)`. You will still receive live multi-target tracking data automatically on power-up.

### Quick-Start Example (Arduino / PlatformIO)

Copy `ld2423_protocol.h` and `ld2423_protocol.c` into your sketch or `src/` folder, then parse incoming bytes from `Serial1`:

```cpp
#include <Arduino.h>
#include "ld2423_protocol.h"

#define RADAR_RX_PIN 18 // Connect to LD2423 TX (Pin 2)
#define RADAR_TX_PIN 17 // Connect to LD2423 RX (Pin 3), or -1 for 3-wire RX-only

LD2423Parser parser;

void setup() {
    Serial.begin(115200);
    Serial1.begin(LD2423_DEFAULT_BAUDRATE, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    ld2423_parser_init(&parser);

    // Optional (4-wire mode): query firmware version & detection scope
    if (RADAR_TX_PIN >= 0) {
        uint8_t cmd[LD2423_CMD_MAX_FRAME_SIZE];
        size_t len = ld2423_build_query_version(cmd);
        Serial1.write(cmd, len);
    }
}

void loop() {
    while (Serial1.available()) {
        LD2423Report rep;
        LD2423Ack ack;
        LD2423ParseResult res = ld2423_parser_push_byte(&parser, Serial1.read(), &rep, &ack);

        if (res == LD2423ParseReport) {
            for (int i = 0; i < LD2423_MAX_TARGETS; i++) {
                const LD2423Target& t = rep.targets[i];
                if (!t.valid) continue;
                Serial.printf(
                    "T%d [%s] dist=%.2fm ang=%+d deg (X=%dmm Y=%dmm spd=%dcm/s)\n",
                    i + 1,
                    t.is_static ? "STATIC" : "MOVING",
                    t.distance_mm / 1000.0f,
                    t.angle_deg,
                    t.x_mm,
                    t.y_mm,
                    t.speed_cm_s);
            }
        } else if (res == LD2423ParseAck && ack.func_code == LD2423CmdQueryVersion) {
            Serial.printf("LD2423 FW: %u.%02u.%02u%02u%02u\n",
                ack.fw_major, ack.fw_minor, ack.fw_year, ack.fw_month, ack.fw_day);
        }
    }
}
```

*(Note: Avoid strapping/USB pins such as `GPIO 0`, `GPIO 19`, and `GPIO 20` on the ESP32-S2 when picking UART GPIOs.)*
