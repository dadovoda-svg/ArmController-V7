#include <Arduino.h>
#include "esp_timer.h"          //Include richiesto solo in core 3.x
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ========== Heltec WiFi Kit 32 pin ==========
constexpr uint8_t I2C_SDA_PIN = 4;
constexpr uint8_t I2C_SCL_PIN = 15;
constexpr uint8_t OLED_RES_PIN = 16;
constexpr uint8_t VEXT_PIN = 21;    // Vext control: LOW=ON, HIGH=OFF (commenta se non serve)
constexpr uint8_t LED_PIN = 25;     // Led bianco

// ========== I2C addresses ==========
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr uint8_t AS5600_ADDR = 0x36;

// ========== AS5600 registers (MSB at lower addr) ==========
constexpr uint8_t REG_STATUS_H = 0x0B;
constexpr uint8_t REG_AGC = 0x1A;
constexpr uint8_t REG_MAGNITUDE = 0x1B;  // MSB; LSB=0x1C
constexpr uint8_t REG_RAW_ANGLE = 0x0C;  // MSB; LSB=0x0D

// ========== OLED setup ==========
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RES_PIN);

// ========== Stepper ==========
// BLU    B-
// ROSSO  B+
// VERDE  A-
// NERO   A+
constexpr uint8_t STEP_EN_PIN = 23;
constexpr uint8_t STEP_DIR_PIN = 19;
constexpr uint8_t STEP_PULSE_PIN = 22;

// ========== Buttons ==========
constexpr uint8_t BTN_LOWER_PIN = 12;
constexpr uint8_t BTN_UPPER_PIN = 14;

// ========== Timing ==========
constexpr uint32_t DISPLAY_UPDATE_MS = 100;
constexpr uint32_t KEY_SCAN_MS = 20;

// ========== Interrupt Timer ==========
esp_timer_handle_t toggle_timer;

enum KeyState : uint8_t {
  KEY_IDLE = 0,
  KEY_LOWER_HELD,
  KEY_UPPER_HELD,
  KEY_LOWER_RELEASED,
  KEY_UPPER_RELEASED,
  KEY_LOWER_CONTINUOUS,
  KEY_UPPER_CONTINUOUS
};

static inline bool isPressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

static inline void setMotorEnabled(bool enabled) {
  digitalWrite(STEP_EN_PIN, enabled ? HIGH : LOW);
}

static inline void setMotorDirection(bool forward) {
  digitalWrite(STEP_DIR_PIN, forward ? HIGH : LOW);
}

// ---------- ISR Timer ----------
void onTimer(void* arg) {
  static bool led_state = false;
  led_state = !led_state;
  digitalWrite(STEP_PULSE_PIN, led_state);
  //digitalWrite(LED_PIN, led_state);
}

// ---------- I2C helpers ----------
uint8_t readByte(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

uint16_t readWord(uint8_t regMSB) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(regMSB);              // MSB first (addr lower)
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t)msb << 8) | lsb;
}

static inline int clamp(int v, int lo, int hi){ return v < lo ? lo : (v > hi ? hi : v); }

// Track the AS5600 angle across multiple turns by unwrapping the 0..4095 value.
float computeContinuousAngleDeg(uint16_t rawAngle) {
  static bool initialized = false;
  static int32_t turnCount = 0;
  static uint16_t prevRawAngle = 0;

  const int16_t wrapThreshold = 2048;  // Half a turn in AS5600 counts.

  if (!initialized) {
    prevRawAngle = rawAngle;
    initialized = true;
  } else {
    int16_t delta = (int16_t)rawAngle - (int16_t)prevRawAngle;

    if (delta > wrapThreshold) {
      turnCount--;
    } else if (delta < -wrapThreshold) {
      turnCount++;
    }

    prevRawAngle = rawAngle;
  }

  int32_t continuousCounts = (turnCount * 4096L) + rawAngle;
  return (continuousCounts * 360.0f) / 4096.0f;
}

KeyState updateKeyStateMachine(bool lowerPressed, bool upperPressed) {
  static KeyState state = KEY_IDLE;

  switch (state) {
    case KEY_IDLE:     //stato iniziale. se premo un tasto avvio il motore nella direzione desiderata
      if (lowerPressed) {
        setMotorEnabled(true);
        setMotorDirection(true);
        state = KEY_LOWER_HELD;
      }
      else if (upperPressed) {
        setMotorEnabled(true);
        setMotorDirection(false);
        state = KEY_UPPER_HELD;
      }
      break;

    case KEY_LOWER_HELD:   //è stato premuto o rilasciato il tasto 12, finchè è premuto resto qui
      if (!lowerPressed) {
        setMotorEnabled(false);
        state = KEY_LOWER_RELEASED;
      }
      break;
    case KEY_UPPER_HELD:   //è stato premuto o rilasciato il tasto 14, finchè è premuto resto qui
      if (!upperPressed) {
        setMotorEnabled(false);
        state = KEY_UPPER_RELEASED;
      }
      break;

    case KEY_LOWER_RELEASED: //è stato rilasciato il tasto 12, la prossima pressione dello stesso avvia il motore indefinitamente
            // se il tasto è cambiato, cambia direzione e entra in modalità temporanea per quella direzione
      if (lowerPressed) {
        setMotorEnabled(true);
        setMotorDirection(true);
        state = KEY_LOWER_CONTINUOUS;
      }
      else if (upperPressed) {
        setMotorEnabled(true);
        setMotorDirection(false);
        state = KEY_UPPER_HELD;
      }
      break;
    case KEY_UPPER_RELEASED: //è stato rilasciato il tasto 14, la prossima pressione dello stesso avvia il motore indefinitamente
            // se il tasto è cambiato, cambia direzione e entra in modalità temporanea per quella direzione
      if (lowerPressed) {
        setMotorEnabled(true);
        setMotorDirection(true);
        state = KEY_LOWER_HELD;
      }
      else if (upperPressed) {
        setMotorEnabled(true);
        setMotorDirection(false);
        state = KEY_UPPER_CONTINUOUS;
      }
      break;

    case KEY_LOWER_CONTINUOUS: //i cambi di stato del tasto 12 non modificano lo stao: il motore continua a girare, il 14 ferma tutto
      if (upperPressed) {
        setMotorEnabled(false);
        state = KEY_IDLE;
      }
      break;
    case KEY_UPPER_CONTINUOUS: //i cambi di stato del tasto 14 non modificano lo stao: il motore continua a girare, il 12 ferma tutto
      if (lowerPressed) {
        setMotorEnabled(false);
        state = KEY_IDLE;
      }
      break;

    default:
      setMotorEnabled(false);
      state = KEY_IDLE;
  }

  return state;
}

// ---------- Setup ----------
void setup() {
  // (opzionale) abilita Vext per alimentare OLED/sensori
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);     // Vext ON
  // led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);      // Led off
  
  pinMode(BTN_LOWER_PIN, INPUT_PULLUP);
  pinMode(BTN_UPPER_PIN, INPUT_PULLUP);
  digitalRead(BTN_LOWER_PIN);       // Pulsante inferiore
  digitalRead(BTN_UPPER_PIN);       // Pulsante superiore

  pinMode(STEP_EN_PIN, OUTPUT);
  digitalWrite(STEP_EN_PIN, LOW);    //23 verde ENABLE high = 0
  pinMode(STEP_DIR_PIN, OUTPUT);
  digitalWrite(STEP_DIR_PIN, HIGH);  //19 giallo DIR  high = 0
  pinMode(STEP_PULSE_PIN, OUTPUT);
  digitalWrite(STEP_PULSE_PIN, HIGH); //22 arancio  STEP  high = 0
  
  delay(20);

  // Crea il timer periodico
  const esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,  // default
    .name = "toggle_timer"
  };
  esp_timer_create(&timer_args, &toggle_timer);
  // Avvia il timer ogni 500000 µs = 500 ms
  esp_timer_start_periodic(toggle_timer, 1000);

  Serial.begin(115200);

  // I2C su pin Heltec
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (1) { delay(1000); }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Heltec AS5600");
  display.display();
  delay(600);

  // Header CSV
  Serial.println("timestamp_ms,MD,ML,MH,AGC,MAGNITUDE,RAW_ANGLE,ANGLE_deg,ANGLE_MULTI_deg");
}

// ---------- Loop ----------
void loop() {
  uint32_t t = millis();
  static uint32_t lastDisplayUpdateMs = 0;
  static uint32_t lastKeyScanMs = 0;
  static KeyState keyState = KEY_IDLE;
  bool lowerPressed = isPressed(BTN_LOWER_PIN);
  bool upperPressed = isPressed(BTN_UPPER_PIN);

  uint16_t rawAngle  = readWord(REG_RAW_ANGLE);

  float angle_deg = (rawAngle * 360.0f) / 4096.0f;
  float angle_multi_deg = computeContinuousAngleDeg(rawAngle);

  if ((uint32_t)(t - lastKeyScanMs) >= KEY_SCAN_MS) {
    lastKeyScanMs = t;
    keyState = updateKeyStateMachine(lowerPressed, upperPressed);
  }

  if ((uint32_t)(t - lastDisplayUpdateMs) >= DISPLAY_UPDATE_MS) {
    lastDisplayUpdateMs = t;

    uint8_t status     = readByte(REG_STATUS_H);
    uint8_t agc        = readByte(REG_AGC);
    uint16_t magnitude = readWord(REG_MAGNITUDE);
    
    uint8_t MD = (status & 0x20) ? 1 : 0;
    uint8_t ML = (status & 0x10) ? 1 : 0;
    uint8_t MH = (status & 0x08) ? 1 : 0;

  // ---- Serial CSV ----
    Serial.print(t);            Serial.print(',');
    Serial.print(MD);           Serial.print(',');
    Serial.print(ML);           Serial.print(',');
    Serial.print(MH);           Serial.print(',');
    Serial.print(agc);          Serial.print(',');
    Serial.print(magnitude);    Serial.print(',');
    Serial.print(rawAngle);     Serial.print(',');
    Serial.print(angle_deg, 2); Serial.print(',');
    Serial.println(angle_multi_deg, 2);

    // ---- OLED ----
    display.clearDisplay();
    display.setCursor(0,0);
    //display.println("AS5600 @Heltec");

    display.print("Ang: ");
    display.print(angle_deg, 1);
    display.println(" deg");

    display.print("Pos: ");
    display.print(angle_multi_deg, 1);
    display.println(" deg");

    display.print("MAG:");
    display.print(magnitude);
    display.print("  AGC:");
    display.println(agc);

    display.print("MD=");
    display.print(MD); display.print(" ML=");
    display.print(ML); display.print(" MH=");
    display.println(MH);

    // === Barra Magnitude ===
    const int x0 = 4, y0 = 35, W = 120, H = 12;
    display.drawRect(x0, y0, W, H, SSD1306_WHITE);      // bordo
    int x25 = x0 + (W * 25) / 100;
    int x75 = x0 + (W * 75) / 100;
    display.drawFastVLine(x25, y0-2, H+4, SSD1306_WHITE);
    display.drawFastVLine(x75, y0-2, H+4, SSD1306_WHITE);

    int fillW = (int)((uint32_t)magnitude * W / 4095U);
    fillW = clamp(fillW, 0, W-2);
    if (fillW > 0) display.fillRect(x0+1, y0+1, fillW, H-2, SSD1306_WHITE);

    int perc = (int)((uint32_t)magnitude * 100 / 4095U);
    display.setCursor(x0 + W + 2, y0 + 8);
    display.setTextSize(1);
    display.print(" MAG:"); display.print(perc); display.print('%');

    display.setCursor(100, 56);
    display.setTextSize(1);
    display.print(lowerPressed ? 0 : 1);
    display.print(upperPressed ? 0 : 1);
    display.print(" "); display.print(keyState);

    display.display();
  }

  /*
  esp_timer_stop(toggle_timer);                     //arresta il timer
  esp_timer_start_periodic(toggle_timer, 1000000);  // cambia il periodo a 1s
  */
}
