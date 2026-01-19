# AS5600Cal — Initial Magnetic Encoder Calibration (AS5600 + Heltec ESP32)

This Arduino sketch is a small calibration/commissioning tool used during the **initial setup of a magnetic encoder (AS5600)** on a rotary axis.
It helps you verify **magnet alignment, distance, and signal quality** while you slowly rotate the shaft with a step/dir driver, showing live data on an OLED and streaming CSV data over Serial.

---

## What it does

* Reads key AS5600 diagnostics via I²C:

  * **STATUS** bits: `MD`, `ML`, `MH`
  * **AGC** (automatic gain control)
  * **MAGNITUDE** (magnetic field strength proxy)
  * **RAW_ANGLE** and computed **ANGLE (deg)**
* Shows live values on a **128×64 SSD1306 OLED**
* Generates a simple step pulse (square wave) using `esp_timer`
* Provides a **two-button control** to rotate the axis in both directions:

  * *Press & hold* = rotate while held
  * *Press again* = continuous rotation
  * *Other button* = stop

---

## Target hardware

### MCU / Board

* **Heltec WiFi Kit 32 (ESP32)** (as per pin mapping in the sketch)

### Sensor

* **AS5600** magnetic encoder (I²C address `0x36`)

### Display

* **SSD1306 128×64 OLED** (I²C address `0x3C`)

### Motor driver (Step/Dir)

* Any stepper driver accepting **STEP / DIR / EN** signals.

---

## Pin mapping (as in the sketch)

### I²C + OLED (Heltec)

| Function              | GPIO |
| --------------------- | ---: |
| I2C SDA               |    4 |
| I2C SCL               |   15 |
| OLED RESET            |   16 |
| VEXT control (Heltec) |   21 |

> Note: On many Heltec boards, `VEXT` must be driven **LOW = ON** to power external peripherals (OLED/sensors). The sketch enables it.

### Buttons

| Button       | GPIO | Mode           |
| ------------ | ---: | -------------- |
| Lower button |   12 | `INPUT_PULLUP` |
| Upper button |   14 | `INPUT_PULLUP` |

Buttons are assumed **active-low** (pressed = `0`).

### Step/Dir/Enable

| Signal       | GPIO |
| ------------ | ---: |
| EN           |   23 |
| DIR          |   19 |
| STEP (pulse) |   22 |

In this sketch:

* `STEP_EN = HIGH` enables the driver
* `STEP_EN = LOW` disables the driver
  If your driver uses inverted logic, adjust accordingly.

---

## Dependencies (Arduino libraries)

Install these via Arduino Library Manager:

* **Adafruit GFX Library**
* **Adafruit SSD1306**

Also uses:

* `Wire` (built-in)
* `esp_timer.h` (ESP32 core; comment in code notes core 3.x)

---

## Build & upload

1. Open `AS5600Cal.ino` in Arduino IDE (or PlatformIO with Arduino framework).
2. Select the correct **ESP32 / Heltec** board profile.
3. Connect AS5600 + OLED to the I²C bus (GPIO 4/15).
4. Upload the sketch.
5. Open Serial Monitor at **115200** baud.

---

## Serial output (CSV)

The sketch prints one CSV line approximately every 50 ms:

**Header**

```
timestamp_ms,MD,ML,MH,AGC,MAGNITUDE,RAW_ANGLE,ANGLE_deg
```

**Fields**

* `timestamp_ms`: `millis()`
* `MD`: Magnet detected (1/0)
* `ML`: Magnetic field too low (1/0)
* `MH`: Magnetic field too high (1/0)
* `AGC`: automatic gain control value (0–255)
* `MAGNITUDE`: 12-bit-ish magnitude reading (0–4095 range used in the UI scaling)
* `RAW_ANGLE`: raw 12-bit angle (0–4095)
* `ANGLE_deg`: converted angle in degrees

This is convenient for logging/plotting (e.g., Serial Plotter, Python, Excel).

---

## OLED UI

Displayed information:

* Angle in degrees
* `MAGNITUDE` and `AGC`
* `MD/ML/MH` status bits
* A **magnitude bar** with reference marks around **25%** and **75%**

---

## Button control logic (how to rotate the axis)

The sketch implements a simple state machine:

### Temporary rotation (press & hold)

* From idle:

  * Hold **GPIO12**: rotate direction A while held
  * Hold **GPIO14**: rotate direction B while held
* Releasing the button stops the motor.

### Continuous rotation

* After a hold-and-release in a direction:

  * Press the **same** button again → motor runs continuously
  * Press the **other** button → stop (returns to idle)

This lets you keep one hand free while watching the OLED / serial logs.

---

## Calibration workflow (recommended)

1. **Mount the magnet** on the shaft (centered as well as possible).
2. Power the system and confirm:

   * OLED shows values updating
   * Serial CSV is streaming
3. Without rotating yet, check:

   * `MD` should be **1** when the magnet is properly detected.
4. Rotate slowly (temporary mode) and observe:

   * `ML` should stay **0** (not too weak)
   * `MH` should stay **0** (not too strong)
   * `MAGNITUDE` should be stable and not near extremes
5. Adjust **magnet distance / alignment** until you get:

   * `MD = 1`
   * `ML = 0`
   * `MH = 0`
   * A comfortable magnitude bar (often mid-range is preferable)
6. Once stable, you can run continuous mode and check for:

   * Smooth `ANGLE_deg` progression over 0–360
   * No sudden jumps/dropouts in `RAW_ANGLE`
   * No unexpected toggling of `MD/ML/MH`

---

## Tuning the speed (step rate)

Step pulses are produced by `esp_timer_start_periodic(toggle_timer, 8000);`

Important detail: the ISR **toggles** the STEP pin each callback, so:

* Callback period = `T`
* STEP square-wave period = `2T`
* STEP frequency = `1 / (2T)`

With `T = 8000 µs`:

* STEP frequency ≈ `1 / (2 * 0.008 s)` ≈ **62.5 Hz**
  (approximately 62.5 steps per second, depending on how your driver counts edges)

To rotate faster/slower, change `8000` (microseconds).

---

## Troubleshooting

* **OLED init failed**

  * Check OLED address (default here: `0x3C`)
  * Verify `VEXT_PIN` behavior on your Heltec revision
  * Confirm I²C wiring and pullups (many modules already include pullups)

* **MD = 0 always**

  * Magnet missing, too far, off-center, or wrong pole orientation
  * Check sensor orientation vs magnet placement

* **ML = 1 frequently**

  * Field too weak: magnet too far or not centered

* **MH = 1 frequently**

  * Field too strong: magnet too close or too strong magnet

* **Motor does not rotate**

  * Verify EN polarity for your driver (this sketch assumes HIGH=enable)
  * Check STEP/DIR wiring and driver power stage supply

---

## Notes

* The sketch is intentionally minimal and aimed at **commissioning/calibration**, not production control.
* If you move to a different ESP32 board, review:

  * I²C pins
  * OLED reset/power scheme
  * available GPIOs for STEP/DIR/EN and buttons

---
## Licenza

MIT
