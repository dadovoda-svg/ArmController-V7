# RP2040 UART0 ↔ USB CDC Transparent Passthrough (Waveshare RP2040-Zero)

Firmware for **Waveshare RP2040-Zero** (RP2040) built with **PlatformIO + Arduino (arduino-pico / Earle Philhower core)**.  
The device acts as a **transparent, bidirectional repeater** between:

- **UART0 (Serial1)** on **GP0 (TX)** / **GP1 (RX)**  ↔  **USB CDC (Serial)** to the host PC

In practice, everything coming from the *node* on UART is forwarded to the PC over USB, and everything coming from the PC over USB is forwarded to the *node* over UART.

---

## Features

- **Bidirectional** and **transparent** bridge (byte-for-byte forwarding, suitable for binary protocols too)
- UART0 via `Serial1`
- USB CDC via `Serial`
- Chunked forwarding buffer (default 256 bytes) to handle continuous traffic

---

## Hardware

- Board: **Waveshare RP2040-Zero**
- UART0 wiring:
  - **GP0 = TX (RP2040 → node RX)**
  - **GP1 = RX (RP2040 ← node TX)**
  - Common **GND** between RP2040 and the node

> ⚠️ Make sure voltage levels are compatible (typically 3.3V).  
> If your node is 5V, use a proper level shifter.

---

## Build & Upload (PlatformIO)

1. Open the project with **VS Code + PlatformIO**.
2. Select the environment (e.g. `rp2040zero`) and build/upload.

Useful CLI commands:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

> Note: the monitor baud rate on USB CDC is often “nominal” (USB is not actually using that baud), but PlatformIO still requires a value.

---

## Configuration

In `src/main.cpp` you can configure:

- `UART_BAUD` for the node UART speed
- `BUF_SZ` for the forwarding chunk size

Example (defaults):

```cpp
static constexpr uint32_t UART_BAUD = 115200;
static constexpr size_t   BUF_SZ    = 256;
```

---

## USB CDC behavior (DTR / terminal not open)

On some systems, USB CDC output may depend on the **DTR** signal (typically asserted when you open a serial terminal on the PC).  
This firmware can force USB output even when the terminal is not open by enabling:

```cpp
Serial.ignoreFlowControl(true);
```

If you prefer a more “classic” behavior (only write when the host terminal is open), remove that line or gate UART→USB forwarding behind a DTR check.

---

## Known limitations

- This bridge does not implement framing, escaping, or checksums: it is intentionally a “virtual cable”.
- At very high data rates, USB/serial buffers can overflow if the host does not read fast enough.

---

## Disclaimer (AI-assisted content)

Parts of this project (including code and/or documentation) were **partially generated with the assistance of artificial intelligence tools**.  
Please review and test the firmware in your own environment before relying on it in production or safety-critical applications.

---

## License

MIT
