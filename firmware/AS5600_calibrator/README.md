# AS5600Cal — Initial Magnetic Encoder Calibration (AS5600 + Heltec ESP32)

This Arduino sketch is a small calibration and commissioning tool for the **initial setup of an AS5600 magnetic encoder** on a rotary axis.
It helps you verify **magnet alignment, distance, signal quality, and angle continuity across multiple turns** while rotating the shaft with a step/dir driver, showing live data on an OLED and streaming CSV data over Serial.

---

## What it does

* Reads key AS5600 diagnostics over I2C:
  * **STATUS** bits: `MD`, `ML`, `MH`
  * **AGC** (automatic gain control)
  * **MAGNITUDE** (magnetic field strength proxy)
  * **RAW_ANGLE**
  * **ANGLE_deg**: single-turn angle in degrees
  * **ANGLE_MULTI_deg**: continuous multi-turn angle in degrees
* Reads `RAW_ANGLE` in the main loop as fast as possible
* Updates the OLED and serial CSV output at a slower configurable rate
* Scans the two buttons through a dedicated key state machine at its own configurable interval
* Generates a step pulse square wave using `esp_timer`
* Lets you rotate the axis in both directions with a two-button interface:
  * *Press and hold* = rotate while held
  * *Press the same button again* = continuous rotation
  * *Press the other button* = stop

---

## Target hardware

### MCU / Board

* **Heltec WiFi Kit 32 (ESP32)**

### Sensor

* **AS5600** magnetic encoder, I2C address `0x36`

### Display

* **SSD1306 128x64 OLED**, I2C address `0x3C`

### Motor driver

* Any stepper driver accepting **STEP / DIR / EN** signals

---

## Pin mapping

### I2C + OLED

| Function              | GPIO |
| --------------------- | ---: |
| I2C SDA               |    4 |
| I2C SCL               |   15 |
| OLED RESET            |   16 |
| VEXT control (Heltec) |   21 |

On many Heltec boards, `VEXT` must be driven **LOW = ON** to power external peripherals. The sketch enables it at startup.

### Buttons

| Button       | GPIO | Mode           |
| ------------ | ---: | -------------- |
| Lower button |   12 | `INPUT_PULLUP` |
| Upper button |   14 | `INPUT_PULLUP` |

Buttons are active-low, so a pressed button reads `0`.

### Step / Dir / Enable

| Signal | GPIO |
| ------ | ---: |
| EN     |   23 |
| DIR    |   19 |
| STEP   |   22 |

In the current sketch:

* `HIGH` on `STEP_EN_PIN` enables the driver
* `LOW` on `STEP_EN_PIN` disables the driver

If your driver uses inverted enable logic, adjust that helper.

---

## Dependencies

Install these with the Arduino Library Manager:

* **Adafruit GFX Library**
* **Adafruit SSD1306**

Also used:

* `Wire`
* `esp_timer.h`

---

## Build & upload

1. Open `src/AS5600Cal.ino` in Arduino IDE or PlatformIO.
2. Select the correct ESP32 / Heltec board.
3. Connect the AS5600 and OLED to the I2C bus on GPIO `4` and `15`.
4. Upload the sketch.
5. Open the Serial Monitor at **115200 baud**.

---

## Timing model

The sketch separates fast and slow work:

* **Fast loop**: reads `RAW_ANGLE` continuously and updates the multi-turn angle tracking
* **Key scan loop**: scans buttons and updates the key state machine every `KEY_SCAN_MS`
* **Display/log loop**: updates OLED and serial CSV every `DISPLAY_UPDATE_MS`

Current code defaults:

* `DISPLAY_UPDATE_MS = 100`
* `KEY_SCAN_MS = 20`

---

## Serial output (CSV)

The sketch prints one CSV line every `DISPLAY_UPDATE_MS` milliseconds, so with the current default that is about every **100 ms**.

**Header**

```text
timestamp_ms,MD,ML,MH,AGC,MAGNITUDE,RAW_ANGLE,ANGLE_deg,ANGLE_MULTI_deg
```

**Fields**

* `timestamp_ms`: current `millis()`
* `MD`: magnet detected (`1` or `0`)
* `ML`: magnetic field too low (`1` or `0`)
* `MH`: magnetic field too high (`1` or `0`)
* `AGC`: automatic gain control value
* `MAGNITUDE`: AS5600 magnetic magnitude reading
* `RAW_ANGLE`: raw 12-bit angle, `0..4095`
* `ANGLE_deg`: single-turn angle in degrees, `0..360`
* `ANGLE_MULTI_deg`: continuous signed angle in degrees across multiple turns

This is convenient for logging and plotting with tools such as Serial Plotter, Python, or Excel.

---

## OLED UI

Displayed information:

* Single-turn angle in degrees
* Continuous multi-turn position in degrees
* `MAGNITUDE` and `AGC`
* `MD / ML / MH` status bits
* A magnitude bar with reference marks near **25%** and **75%**
* Raw button levels plus the current key-state-machine state on the bottom line

---

## Button control logic

The button handling is implemented in a separate helper with these logical states:

* `KEY_IDLE`
* `KEY_LOWER_HELD`
* `KEY_UPPER_HELD`
* `KEY_LOWER_RELEASED`
* `KEY_UPPER_RELEASED`
* `KEY_LOWER_CONTINUOUS`
* `KEY_UPPER_CONTINUOUS`

User-visible behavior:

### Temporary rotation

* From idle, hold **GPIO12** to rotate in one direction while held
* From idle, hold **GPIO14** to rotate in the other direction while held
* Releasing the button stops the motor

### Continuous rotation

* After a hold-and-release in one direction, press the **same** button again to start continuous rotation
* Press the **other** button to stop and return to idle

This makes it easier to watch the OLED and serial data while the axis keeps turning.

---

## Multi-turn angle tracking

The AS5600 raw angle wraps every revolution, so the sketch unwraps the `0..4095` raw value into a continuous signed angle.

That means:

* clockwise rotation can continue past `360`, `720`, `1080`, ...
* counterclockwise rotation can go to negative angles such as `-360`, `-720`, ...

This is useful when checking encoder continuity through more than one full revolution.

---

## Calibration workflow

1. Mount the magnet on the shaft as centered as possible.
2. Power the system and confirm that:
   * the OLED is updating
   * serial CSV is streaming
3. Without rotating yet, check that `MD` becomes `1`.
4. Rotate slowly and observe:
   * `ML` should stay `0`
   * `MH` should stay `0`
   * `MAGNITUDE` should remain reasonably stable
5. Adjust magnet distance and alignment until you get:
   * `MD = 1`
   * `ML = 0`
   * `MH = 0`
   * a comfortable mid-range magnitude bar
6. Then check:
   * smooth `ANGLE_deg` progression over `0..360`
   * smooth `ANGLE_MULTI_deg` progression across multiple turns
   * no sudden dropouts in `RAW_ANGLE`
   * no unexpected toggling of `MD`, `ML`, or `MH`

---

## Tuning the speed

Step pulses are generated by:

```cpp
esp_timer_start_periodic(toggle_timer, 1000);
```

Important detail: the timer callback **toggles** the STEP pin each time, so:

* callback period = `T`
* STEP square-wave period = `2T`
* STEP frequency = `1 / (2T)`

With `T = 1000 us`:

* STEP frequency is about **500 Hz**

If you want to rotate faster or slower, change that period in microseconds.

---

## Troubleshooting

* **OLED init failed**
  * Check OLED address `0x3C`
  * Verify `VEXT_PIN` behavior on your Heltec board
  * Confirm I2C wiring and pullups

* **`MD = 0` all the time**
  * Magnet missing, too far away, badly centered, or wrong orientation

* **`ML = 1` often**
  * Magnetic field is too weak

* **`MH = 1` often**
  * Magnetic field is too strong

* **Motor does not rotate**
  * Check enable polarity for your driver
  * Verify STEP/DIR wiring and the motor power stage

---

## Notes

* This sketch is intentionally simple and aimed at **commissioning/calibration**, not production motion control.
* If you move to a different ESP32 board, review:
  * I2C pins
  * OLED reset and power scheme
  * available GPIOs for STEP, DIR, EN, and buttons

---

## Licenza

MIT
