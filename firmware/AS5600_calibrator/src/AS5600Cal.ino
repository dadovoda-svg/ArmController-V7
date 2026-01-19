#include <Arduino.h>
#include "esp_timer.h"          //Include richiesto solo in core 3.x
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ========== Heltec WiFi Kit 32 pin ==========
#define I2C_SDA   4
#define I2C_SCL   15
#define OLED_RES  16
#define VEXT_PIN  21    // Vext control: LOW=ON, HIGH=OFF (commenta se non serve)
#define LED_PIN   25    //Led bianco

// ========== I2C addresses ==========
#define OLED_ADDR    0x3C
#define AS5600_ADDR  0x36

// ========== AS5600 registers (MSB at lower addr) ==========
#define REG_STATUS_H   0x0B
#define REG_AGC        0x1A
#define REG_MAGNITUDE  0x1B  // MSB; LSB=0x1C
#define REG_RAW_ANGLE  0x0C  // MSB; LSB=0x0D

// ========== OLED setup ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RES);

// ========== Stepper ==========
// BLU    B-
// ROSSO  B+
// VERDE  A-
// NERO   A+
#define STEP_EN     23
#define STEP_DIR    19
#define STEP_PULSE  22

// ========== Interrupt Timer ==========
esp_timer_handle_t toggle_timer;

// ---------- ISR Timer ----------
void onTimer(void* arg) {
  static bool led_state = false;
  led_state = !led_state;
  digitalWrite(STEP_PULSE, led_state);
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

// ---------- Setup ----------
void setup() {
  // (opzionale) abilita Vext per alimentare OLED/sensori
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);     // Vext ON
  // led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);      // Led off
  
  pinMode(12, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  digitalRead(12);                  // Pulsante inferiore
  digitalRead(14);                  // Pulsante superiore

  pinMode(STEP_EN, OUTPUT);
  digitalWrite(STEP_EN, LOW);    //23 verde ENABLE high = 0
  pinMode(STEP_DIR, OUTPUT);
  digitalWrite(STEP_DIR, HIGH);   //19 giallo DIR  high = 0
  pinMode(STEP_PULSE, OUTPUT);
  digitalWrite(STEP_PULSE, HIGH); //22 arancio  STEP  high = 0
  
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
  esp_timer_start_periodic(toggle_timer, 8000);

  Serial.begin(115200);

  // I2C su pin Heltec
  Wire.begin(I2C_SDA, I2C_SCL);

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
  Serial.println("timestamp_ms,MD,ML,MH,AGC,MAGNITUDE,RAW_ANGLE,ANGLE_deg");
}

// ---------- Loop ----------
void loop() {
  uint32_t t = millis();

  uint8_t status     = readByte(REG_STATUS_H);
  uint8_t agc        = readByte(REG_AGC);
  uint16_t magnitude = readWord(REG_MAGNITUDE);
  uint16_t rawAngle  = readWord(REG_RAW_ANGLE);

  uint8_t MD = (status & 0x20) ? 1 : 0;
  uint8_t ML = (status & 0x10) ? 1 : 0;
  uint8_t MH = (status & 0x08) ? 1 : 0;

  float angle_deg = (rawAngle * 360.0f) / 4096.0f;

  // ---- Serial CSV ----
  Serial.print(t);            Serial.print(',');
  Serial.print(MD);           Serial.print(',');
  Serial.print(ML);           Serial.print(',');
  Serial.print(MH);           Serial.print(',');
  Serial.print(agc);          Serial.print(',');
  Serial.print(magnitude);    Serial.print(',');
  Serial.print(rawAngle);     Serial.print(',');
  Serial.println(angle_deg, 2);

  // ---- OLED ----
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("AS5600 @Heltec");

  display.print("Ang: ");
  display.print(angle_deg, 1);
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


  static uint8_t state=0;
  display.setCursor(100, 56);
  display.setTextSize(1);
  //display.print("    "); 
  display.print(digitalRead(12)); 
  display.print(digitalRead(14)); 
  display.print(" "); display.print(state);
  
  switch (state) {
    case 0:     //stato iniziale. se premo un tasto avvio il motore nella direzione desiderata
      if (!digitalRead(12)) {
        digitalWrite(STEP_EN, HIGH);
        digitalWrite(STEP_DIR, HIGH);      
        state = 1;
      }
      else if (!digitalRead(14)) {
        digitalWrite(STEP_EN, HIGH);
        digitalWrite(STEP_DIR, LOW);      
        state = 2;
      }
      break;

    case 1:   //è stato premuto o rilasciato il tasto 12, finchè è premuto resto qui
      if (digitalRead(12)) {
        digitalWrite(STEP_EN, LOW);
        state = 3;
      }
      break;
    case 2:   //è stato premuto o rilasciato il tasto 14, finchè è premuto resto qui
      if (digitalRead(14)) {
        digitalWrite(STEP_EN, LOW);
        state = 4;
      }
      break;
    
    case 3: //è stato rilasciato il tasto 12, la prossima pressione dello stesso avvia il motore indefinitamente
            // se il tasto è cambiato, cambia direzione e entra in modalità temporanea per quella direzione
      if (!digitalRead(12)) {
        digitalWrite(STEP_EN, HIGH);
        digitalWrite(STEP_DIR, HIGH);
        state = 5;      
      }
      else if (!digitalRead(14)) {
        digitalWrite(STEP_EN, HIGH);
        digitalWrite(STEP_DIR, LOW); 
        state = 2;     
      }
      break;     
    case 4: //è stato rilasciato il tasto 14, la prossima pressione dello stesso avvia il motore indefinitamente
            // se il tasto è cambiato, cambia direzione e entra in modalità temporanea per quella direzione
      if (!digitalRead(12)) {
        digitalWrite(STEP_EN, HIGH);
        digitalWrite(STEP_DIR, HIGH);
        state = 1;      
      }
      else if (!digitalRead(14)) {
        digitalWrite(STEP_EN, HIGH);
        digitalWrite(STEP_DIR, LOW); 
        state = 6;     
      }
      break;

    case 5: //i cambi di stato del tasto 12 non modificano lo stao: il motore continua a girare, il 14 ferma tutto
      if (!digitalRead(14)) {
        digitalWrite(STEP_EN, LOW);
        state = 0;
      }
      break;
    case 6: //i cambi di stato del tasto 14 non modificano lo stao: il motore continua a girare, il 12 ferma tutto
      if (!digitalRead(12)) {
        digitalWrite(STEP_EN, LOW);
        state = 0;
      }
      break;

    default:
        digitalWrite(STEP_EN, LOW);
        state = 0;
  }

  // if (!digitalRead(12)) {
  //   digitalWrite(STEP_EN, HIGH);
  //   digitalWrite(STEP_DIR, HIGH);      
  // }
  // else if (!digitalRead(14)) {
  //   digitalWrite(STEP_EN, HIGH);
  //   digitalWrite(STEP_DIR, LOW);      
  // }
  // else
  //   digitalWrite(STEP_EN, LOW);

  display.display();

  /*
  esp_timer_stop(toggle_timer);                     //arresta il timer
  esp_timer_start_periodic(toggle_timer, 1000000);  // cambia il periodo a 1s
  */
  delay(50);
}
