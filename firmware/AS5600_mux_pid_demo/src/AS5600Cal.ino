#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AS5600Mux.h"

// ========== Heltec WiFi Kit 32 pin ==========
#define I2C_SDA   4
#define I2C_SCL   15
#define OLED_RES  16
#define VEXT_PIN  21    // Vext control: LOW=ON, HIGH=OFF
#define LED_PIN   25    // Led bianco
#define BTN_UPPER 14    // Tasto superiore
#define BTN_LOWER 12    // Tasto inferiore

// ========== I2C addresses ==========
#define OLED_ADDR    0x3C
#define AS5600_ADDR  0x36
#define I2C_MUX      0x70

// ========== AS5600 registers ==========
#define AS5600_RAW_ANGLE 0x0C   
AS5600Mux encoders(Wire, I2C_MUX, AS5600_ADDR);

// ========== OLED setup ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RES);

// ========== Stepper pins ==========
#define ENABLE_PIN  23
#define DIR_PIN     19
#define STEP_PIN    22

// Motor configuration
#define STEPS_PER_REV 3200  // 200 * 16 microstepping
#define DEGREES_PER_STEP (360.0 / STEPS_PER_REV)
//#define DIR_INVERTED
float angleOffset = 181.0;
//float angleOffset = 182.7;

// Control loop timing
#define CONTROL_FREQ 100     // Hz
#define CONTROL_PERIOD_MS (1000 / CONTROL_FREQ)
#define DISPLAY_UPDATE_MS 100  // Update display 10Hz

// Motion profile parameters - variables for runtime tuning
float MAX_SPEED = 3200.0;        // steps/sec
float ACCELERATION = 9600.0;    // steps/sec^2
float MIN_SPEED = 20.0;         // minimum speed for smooth motion

// PID parameters - variables for runtime tuning
float KP = 70.0;
float KI = 20.0;
float KD = 0.1;
float PID_THRESHOLD = 5.0;    // degrees - when to switch to PID mode

// Global variables
volatile float targetPosition = 0;     // target in degrees
float currentPosition = 0;             // current in degrees
float lastPosition = 0;
bool motorEnabled = true;

// Motion profile variables
float profileVelocity = 0;    // current velocity in steps/sec
bool inPIDMode = false;       // Track control mode

// PID variables
float errorSum = 0;
float lastError = 0;

// Button handling
unsigned long btn_upper_press_time = 0;
unsigned long btn_lower_press_time = 0;
bool btn_upper_was_pressed = false;
bool btn_lower_was_pressed = false;
uint8_t displayMode = 0;  // Display mode (for future expansion)

// Step generation task
TaskHandle_t stepTaskHandle = NULL;
volatile float currentStepFrequency = 0;  // steps/sec
volatile bool currentDirection = true;
SemaphoreHandle_t stepMutex = NULL;

// ========== Step generation task - runs on core 0 ==========
void stepGenerationTask(void *parameter) {
  static uint32_t yieldDelay = 0;
  uint32_t lastStepMicros = 0;
  uint32_t stepIntervalMicros = 0;
  
  while (true) {
    // Get current step parameters safely
    if (xSemaphoreTake(stepMutex, 0) == pdTRUE) {
      float freq = currentStepFrequency;
      bool dir = currentDirection;
      bool enabled = motorEnabled;
      xSemaphoreGive(stepMutex);
      
      if (enabled && freq > 0) {
        stepIntervalMicros = 1000000.0 / freq;

        #ifndef DIR_INVERTED
        digitalWrite(DIR_PIN, dir ? LOW : HIGH);    //direct
        #else
        digitalWrite(DIR_PIN, dir ? HIGH : LOW);      //inverted
        #endif

        uint32_t now = micros();
        if (now - lastStepMicros >= stepIntervalMicros) {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(5);
          digitalWrite(STEP_PIN, LOW);
          lastStepMicros = now;
        }
      }
    }
    
    // Small delay to prevent watchdog issues
    if (!yieldDelay) {
      vTaskDelay(1); // 1tick = 1ms
      yieldDelay = 100000;
    }
    else  
      --yieldDelay;
  }
}

void controlLoop() {
  // Read current position from encoder
  currentPosition = readAS5600();
  
  // Calculate position error
  float error = normalizeAngle(targetPosition - currentPosition);
  
  // Motion profile for large movements
  if (abs(error) > PID_THRESHOLD) {
    inPIDMode = false;
    motionProfile(error);
  } 
  // PID control for precise positioning
  else {
    inPIDMode = true;
    pidControl(error);
  }
  
  lastPosition = currentPosition;
}

void motionProfile(float error) {
  float dt = 1.0 / CONTROL_FREQ;
  
  // Calculate distance to decelerate
  float decelDistance = (profileVelocity * profileVelocity) / (2.0 * ACCELERATION);
  float distanceToGo = abs(error) * STEPS_PER_REV / 360.0;
  
  // Determine if we should accelerate, cruise, or decelerate
  if (distanceToGo > decelDistance && abs(profileVelocity) < MAX_SPEED) {
    // Accelerate
    profileVelocity += (error > 0 ? 1 : -1) * ACCELERATION * dt;
    profileVelocity = constrain(profileVelocity, -MAX_SPEED, MAX_SPEED);
  } 
  else if (distanceToGo <= decelDistance) {
    // Decelerate
    float targetVel = sqrt(2.0 * ACCELERATION * distanceToGo);
    if (abs(profileVelocity) > targetVel) {
      profileVelocity -= (profileVelocity > 0 ? 1 : -1) * ACCELERATION * dt;
    }
  }
  
  // Ensure minimum speed for smooth motion
  if (abs(profileVelocity) > 0 && abs(profileVelocity) < MIN_SPEED) {
    profileVelocity = (profileVelocity > 0 ? 1 : -1) * MIN_SPEED;
  }
  
  // Set step generation parameters
  setStepSpeed(abs(profileVelocity), profileVelocity > 0);
  
  // Debug output (throttled)
  // static unsigned long lastDebug = 0;
  // if (millis() - lastDebug > 2000) {
  //   lastDebug = millis();
  //   Serial.printf("PROFILE - Pos: %.1f° Target: %.1f° Err: %.1f° Vel: %.0f st/s\n", 
  //                 currentPosition, targetPosition, error, profileVelocity);
  // }
}

void pidControl(float error) {
  float dt = 1.0 / CONTROL_FREQ;
  
  // PID calculation
  errorSum += error * dt;
  errorSum = constrain(errorSum, -50, 50); // Anti-windup
  
  float errorDiff = (error - lastError) / dt;
  lastError = error;
  
  float output = KP * error + KI * errorSum + KD * errorDiff;
  
  // Convert to steps/sec
  float stepsPerSec = output * STEPS_PER_REV / 360.0;
  stepsPerSec = constrain(stepsPerSec, -MAX_SPEED, MAX_SPEED);
  
  // Apply deadband to prevent jitter
  if (abs(error) < 0.1) {
    stepsPerSec = 0;
    profileVelocity = 0;
    errorSum = 0; // Reset integrator when at target
    
  //   static unsigned long lastDebug = 0;
  //   if (millis() - lastDebug > 200) {
  //     lastDebug = millis();
  //     Serial.printf("AT TARGET - Position: %.2f°\n", currentPosition);
  //   }
  // } else {
  //   static unsigned long lastDebug = 0;
  //   if (millis() - lastDebug > 1000) {
  //     lastDebug = millis();
  //     Serial.printf("PID - Pos: %.2f° Target: %.2f° Err: %.2f°\n", 
  //                   currentPosition, targetPosition, error);
  //   }
  }
  
  setStepSpeed(abs(stepsPerSec), stepsPerSec > 0);
}

void setStepSpeed(float stepsPerSec, bool direction) {
  if (xSemaphoreTake(stepMutex, portMAX_DELAY) == pdTRUE) {
    if (stepsPerSec < 1 || !motorEnabled) {
      currentStepFrequency = 0;
    } else {
      currentStepFrequency = stepsPerSec;
      currentDirection = direction;
    }
    xSemaphoreGive(stepMutex);
  }
}

// float readAS5600() {
//   Wire.beginTransmission(AS5600_ADDR);
//   Wire.write(AS5600_RAW_ANGLE);
//   if (Wire.endTransmission(false) != 0) {
//     return currentPosition; // Return last value on error
//   }
  
//   Wire.requestFrom(AS5600_ADDR, 2);
//   if (Wire.available() == 2) {
//     uint16_t raw = (Wire.read() << 8) | Wire.read();
//     raw &= 0x0FFF; // 12-bit value
//     return ((raw * 360.0) / 4096.0) - angleOffset;  // center offset
//   }
//   return currentPosition; // Return last value on error
// }

float readAS5600() {
  float degrees;

  if (encoders.readAngleDegrees(0, degrees)) {
    return degrees - angleOffset;   // center offset
  }
  return currentPosition;   // Return last value on error
}

float normalizeAngle(float angle) {
  // Normalize to -180 to +180
  // while (angle > 180) angle -= 360;
  // while (angle < -180) angle += 360;
  return angle;
}

// ========== Button handling ==========
void handleButtons() {
  bool btn_upper = !digitalRead(BTN_UPPER);  // Active LOW
  bool btn_lower = !digitalRead(BTN_LOWER);  // Active LOW
  
  // Upper button - Cycle display modes (for future)
  if (btn_upper && !btn_upper_was_pressed) {
    btn_upper_press_time = millis();
    btn_upper_was_pressed = true;
  }
  else if (!btn_upper && btn_upper_was_pressed) {
    unsigned long press_duration = millis() - btn_upper_press_time;
    if (press_duration < 50) {
      // Ignore bounce
    } else if (press_duration < 1000) {
      // Short press - cycle display mode
      displayMode = (displayMode + 1) % 5;  // Only 1 mode for now
      Serial.println("Display mode cycled");
    }
    btn_upper_was_pressed = false;
  }
  
  // Lower button - Homing and motor enable/disable
  if (btn_lower && !btn_lower_was_pressed) {
    btn_lower_press_time = millis();
    btn_lower_was_pressed = true;
    digitalWrite(LED_PIN, HIGH); // LED on while pressed
  }
  else if (!btn_lower && btn_lower_was_pressed) {
    digitalWrite(LED_PIN, LOW); // LED off
    unsigned long press_duration = millis() - btn_lower_press_time;
    
    if (press_duration < 50) {
      // Ignore bounce
    } else if (press_duration < 300) {
      // Short press - Homing to 0°
      targetPosition = 0;
      Serial.println("Homing to 0°");
    } else {
      // Long press - Toggle motor enable
      motorEnabled = !motorEnabled;
      digitalWrite(ENABLE_PIN, motorEnabled ? HIGH : LOW);
      if (!motorEnabled) {
        // Reset velocities when disabling
        profileVelocity = 0;
        currentStepFrequency = 0;
        errorSum = 0;
      }
      Serial.printf("Motor %s\n", motorEnabled ? "ENABLED" : "DISABLED");
    }
    btn_lower_was_pressed = false;
  }
}

// ========== Display update - Compact layout ==========
void updateDisplay() {
  display.clearDisplay();
  
  float error = normalizeAngle(targetPosition - currentPosition);
  
  // Line 1: Title + Mode + Enable status
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("SERVO");
  
  // Show control mode
  display.setCursor(50, 0);
  if (!motorEnabled) {
    display.print("[OFF]");
  } else if (inPIDMode) {
    display.print("[PID]");
  } else {
    display.print("[PROF]");
  }
  
  // Show direction arrow
  display.setCursor(105, 0);
  if (abs(error) < 0.5) {
    display.print("[ ]");  // At target
  } else if (error > 0) {
    display.print("[>]");  // Moving CW
  } else {
    display.print("[<]");  // Moving CCW
  }
  
  // Line 2: Empty for spacing
  
  // Line 3-4: Large current and target position
  display.setTextSize(2);
  display.setCursor(0, 18);
  
  // Format positions compactly
  char posStr[20];
  if (abs(currentPosition) < 100) {
    snprintf(posStr, sizeof(posStr), " %.1f", currentPosition);
  } else {
    snprintf(posStr, sizeof(posStr), " %.0f", currentPosition);
  }
  display.print(posStr);
  
  display.setTextSize(1);  
  display.print((char)26);  // Arrow character →
  if (abs(targetPosition) < 100) {
    snprintf(posStr, sizeof(posStr), " %.1f", targetPosition);
  } else {
    snprintf(posStr, sizeof(posStr), " %.0f", targetPosition);
  }
  display.print(posStr);
  
  // Line 5: Progress bar
  int barY = 38;
  int barWidth = 120;
  int barHeight = 8;
  
  // Draw bar outline
  display.drawRect(4, barY, barWidth, barHeight, SSD1306_WHITE);
  
  // Fill bar based on progress (error relative to starting error)
  float progress = 0;
  if (abs(error) < 0.2) {
    progress = 1.0; // At target
  } else {
    // Estimate progress - this is approximate
    float maxExpectedError = 10.0; // Max possible error
    progress = 1.0 - (abs(error) / maxExpectedError);
    progress = constrain(progress, 0, 1);
  }
  
  int fillWidth = (barWidth - 4) * progress;
  if (fillWidth > 0) {
    display.fillRect(6, barY + 2, fillWidth, barHeight - 4, SSD1306_WHITE);
  }
  
  // Line 6: Error and Velocity
  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print("Err:");
  display.print(abs(error), 1);
  display.print((char)248); // Degree symbol
  
  display.setCursor(60, 52);
  display.print("V:");
  if (motorEnabled) {
    display.print((int)abs(currentStepFrequency));
  } else {
    display.print("---");
  }
  
  display.display();
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  Serial.setTimeout (10000);
  
  // Enable Vext for OLED/sensors
  pinMode(VEXT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);     // Vext ON
  digitalWrite(LED_PIN, LOW);      // Led off
  
  // Buttons
  pinMode(BTN_UPPER, INPUT_PULLUP);
  pinMode(BTN_LOWER, INPUT_PULLUP);
  
  // Initialize stepper pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENABLE_PIN, HIGH); // Enable driver (active LOW)
  
  delay(20);

  // I2C on Heltec pins
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // Fast I2C

  // Create mutex for step parameters
  stepMutex = xSemaphoreCreateMutex();
  
  // Create step generation task on core 0
  xTaskCreatePinnedToCore(
    stepGenerationTask,
    "StepTask",
    2048,
    NULL,
    2,  // High priority
    &stepTaskHandle,
    0   // Core 0
  );
  
  delay(100);
  
  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (1) { delay(1000); }
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println(" Stepper Servo");
  display.println("  Controller");
  display.println();
  display.println("  Initializing...");
  display.display();
  delay(1000);
  
  // Read initial position
  currentPosition = readAS5600();
  targetPosition = currentPosition;
  
  // Print startup info
  Serial.println("\n=== Stepper Servo Controller ===");
  Serial.println("Heltec WiFi Kit 32 + AS5600");
  Serial.println("\nButtons:");
  Serial.println("  Upper: Cycle display modes");
  Serial.println("  Lower: Short=Home, Long=Enable/Disable");
  Serial.println("\nSerial Commands:");
  Serial.println("  Txxxx     - Set target angle (e.g., T90.5)");
  Serial.println("  P         - Print current position");
  Serial.println("  KP:x.x    - Set Kp parameter");
  Serial.println("  KI:x.x    - Set Ki parameter");
  Serial.println("  KD:x.x    - Set Kd parameter");
  Serial.println("  PIDTH:x.x - Set PID threshold");
  Serial.println("  MAXV:xxxx - Set max speed");
  Serial.println("  ACCEL:xxx - Set acceleration");
  Serial.println("  MINV:xx   - Set min speed");
  Serial.println("  SHOW      - Show all parameters");
  Serial.printf("\nInitial position: %.2f°\n", currentPosition);
  Serial.println("================================\n");
}

// ========== Loop ==========
void loop() {
  static unsigned long lastControlTime = 0;
  static unsigned long lastDisplayTime = 0;
  unsigned long now = millis();
  
  // Control loop at fixed frequency
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    lastControlTime = now;
    controlLoop();
  }
  
  // Update display at lower frequency
  if (now - lastDisplayTime >= DISPLAY_UPDATE_MS) {
    lastDisplayTime = now;
    updateDisplay();
  }
  
  // Handle button presses
  handleButtons();
  
  // Handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd.startsWith("T")) {
      float newTarget = cmd.substring(1).toFloat();
      targetPosition = newTarget;
      Serial.printf("Target set to: %.2f°\n", targetPosition);
    }
    else if (cmd.startsWith("KP:")) {
      KP = cmd.substring(3).toFloat();
      Serial.printf("Kp set to: %.3f\n", KP);
      errorSum = 0;
    }
    else if (cmd.startsWith("KI:")) {
      KI = cmd.substring(3).toFloat();
      Serial.printf("Ki set to: %.3f\n", KI);
      errorSum = 0;
    }
    else if (cmd.startsWith("KD:")) {
      KD = cmd.substring(3).toFloat();
      Serial.printf("Kd set to: %.3f\n", KD);
    }
    else if (cmd.startsWith("PIDTH:")) {
      PID_THRESHOLD = cmd.substring(6).toFloat();
      Serial.printf("PID Threshold set to: %.2f°\n", PID_THRESHOLD);
    }
    else if (cmd.startsWith("MAXV:")) {
      MAX_SPEED = cmd.substring(5).toFloat();
      Serial.printf("Max Speed set to: %.0f steps/sec\n", MAX_SPEED);
    }
    else if (cmd.startsWith("ACCEL:")) {
      ACCELERATION = cmd.substring(6).toFloat();
      Serial.printf("Acceleration set to: %.0f steps/sec²\n", ACCELERATION);
    }
    else if (cmd.startsWith("MINV:")) {
      MIN_SPEED = cmd.substring(5).toFloat();
      Serial.printf("Min Speed set to: %.0f steps/sec\n", MIN_SPEED);
    }
    else if (cmd.startsWith("P")) {
      Serial.printf("Current: %.2f° Target: %.2f° Error: %.2f°\n", 
                    currentPosition, targetPosition, 
                    normalizeAngle(targetPosition - currentPosition));
    }
    else if (cmd.startsWith("SHOW")) {
      Serial.println("\n=== Current Parameters ===");
      Serial.printf("PID:  Kp=%.3f  Ki=%.3f  Kd=%.3f  Threshold=%.2f°\n", 
                    KP, KI, KD, PID_THRESHOLD);
      Serial.printf("Motion:  MaxSpeed=%.0f  Accel=%.0f  MinSpeed=%.0f steps/sec\n", 
                    MAX_SPEED, ACCELERATION, MIN_SPEED);
      Serial.printf("Position:  Current=%.2f°  Target=%.2f°  Error=%.2f°\n", 
                    currentPosition, targetPosition, 
                    normalizeAngle(targetPosition - currentPosition));
      Serial.printf("Motor: %s\n", motorEnabled ? "ENABLED" : "DISABLED");
      Serial.println("========================\n");
    }
    else if (cmd.startsWith("EN")) {
      motorEnabled = true;
      digitalWrite(ENABLE_PIN, HIGH);
      Serial.println("Motor ENABLED");
    }
    else if (cmd.startsWith("DIS")) {
      motorEnabled = false;
      digitalWrite(ENABLE_PIN, LOW);
      profileVelocity = 0;
      currentStepFrequency = 0;
      errorSum = 0;
      Serial.println("Motor DISABLED");
    }
    else {
      Serial.println("Unknown command. Type SHOW for help.");
    }
  }
  
  switch (displayMode) {
    case 1:
      targetPosition = 110.0 * sin( millis() * 0.0001 );
      break;
    case 2:
      targetPosition = 110.0 * sin( millis() * 0.0005 );
      break;
    case 3:
      targetPosition = 110.0 * sin( millis() * 0.001 );
      break;  
    case 4:
      targetPosition = -100.0 + 25.0 * int((millis()%20000)/2000.0);
      break;
  }

  delay(1);
}