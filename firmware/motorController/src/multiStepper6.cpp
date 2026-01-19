 
#include "MultiStepper6.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"

#ifdef CONFIG_ESP_TIMER_TASK_AFFINITY
#ifdef CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0
#define TimerSuCPU0
#endif
#endif

// Mutex per proteggere l'accesso allo stato dei motori
static portMUX_TYPE s_timerMux = portMUX_INITIALIZER_UNLOCKED;

// Abilita/disabilita l'uso di gpio_set_level() al posto di digitalWrite()
#define USE_FAST_GPIO 1

// Helper per scrivere velocemente sui pin
static inline void fastWritePin(uint8_t pin, bool level)
{
#if USE_FAST_GPIO
  // Versione con API ESP-IDF (più veloce e con meno overhead di digitalWrite)
  gpio_set_level((gpio_num_t)pin, level ? 1 : 0);
#else
  // Fallback: usa la API Arduino standard
  digitalWrite(pin, level ? HIGH : LOW);
#endif
}

MultiStepper6::MultiStepper6(const uint8_t stepPins[NUM_MOTORS],
                             const uint8_t dirPins[NUM_MOTORS],
                             float   defaultPulseWidthUs,
                             uint32_t tPeriodUs)
: pulseWidthUs(defaultPulseWidthUs),
  timerPeriodUs(tPeriodUs),
  timerHandle(nullptr)
{
  int64_t now = esp_timer_get_time();

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    motors[i].stepPin          = stepPins[i];
    motors[i].dirPin           = dirPins[i];
    motors[i].speedStepsPerSec = 0.0f;
    motors[i].direction        = 0;
    motors[i].stepIntervalUs   = 0;
    motors[i].lastStepTimeUs   = now;
    motors[i].dirPolarity      = +1;    // polarità “normale” di default

    // default nessuna configurazione (0 = non valido)
    stepsPerRev[i]             = 0.0f;
  }
}

void MultiStepper6::begin()
{
  // Configura i pin GPIO come output
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    pinMode(motors[i].stepPin, OUTPUT);
    pinMode(motors[i].dirPin,  OUTPUT);
    fastWritePin(motors[i].stepPin, false);
    fastWritePin(motors[i].dirPin,  false);
  }

  // Crea il timer ESP (gira nel task timer su core 0)
  esp_timer_create_args_t args = {};
  args.callback        = &MultiStepper6::timerCallback;
  args.arg             = this;
  args.dispatch_method = ESP_TIMER_TASK;  // callback eseguito dal task timer (core 0)
  args.name            = "MultiStep6";

  if (esp_timer_create(&args, &timerHandle) != ESP_OK) {
    Serial.println("MultiStepper6: esp_timer_create FAILED");
    timerHandle = nullptr;
    return;
  }

  if (esp_timer_start_periodic(timerHandle, timerPeriodUs) != ESP_OK) {
    Serial.println("MultiStepper6: esp_timer_start_periodic FAILED");
    esp_timer_delete(timerHandle);
    timerHandle = nullptr;
  }
}

void MultiStepper6::end()
{
  if (timerHandle) {
    esp_timer_stop(timerHandle);
    esp_timer_delete(timerHandle);
    timerHandle = nullptr;
  }
}

void MultiStepper6::computeStepInterval(MotorState &m)
{
  float absSpeed = fabs(m.speedStepsPerSec);

  if (absSpeed <= 0.0f) {
    m.stepIntervalUs = 0; // motore fermo
  } else {
    // Periodo [us] = 1e6 / (step/sec)
    float periodUs = 1000000.0f / absSpeed;

    // Evita periodi troppo piccoli rispetto alla pulse width
    if (periodUs < pulseWidthUs * 2.0f) {
      periodUs = pulseWidthUs * 2.0f;
    }

    m.stepIntervalUs = (uint32_t)periodUs;
  }
}

void MultiStepper6::applyDirection(MotorState &m) {
  // direzione effettiva = direzione logica * polarità
  int8_t effectiveDir = m.direction * m.dirPolarity;

  if (effectiveDir > 0) {
    fastWritePin(m.dirPin, true);
  } else if (effectiveDir < 0) {
    fastWritePin(m.dirPin, false);
  }
  // effectiveDir == 0 -> non cambiamo lo stato del pin DIR
}

void MultiStepper6::setSpeed(uint8_t motorIndex, float speedStepsPerSec) {
  if (motorIndex >= NUM_MOTORS) return;

  portENTER_CRITICAL(&s_timerMux);

  MotorState &m = motors[motorIndex];

  m.speedStepsPerSec = speedStepsPerSec;

  if (speedStepsPerSec > 0.0f) {
    m.direction = +1;
  } else if (speedStepsPerSec < 0.0f) {
    m.direction = -1;
  } else {
    m.direction = 0;
  }

  // Aggiorna DIR sul pin
  applyDirection(m);

  // Aggiorna intervallo tra step
  computeStepInterval(m);

  portEXIT_CRITICAL(&s_timerMux);
}

float MultiStepper6::getSpeed(uint8_t motorIndex) const
{
  if (motorIndex >= NUM_MOTORS) return 0.0f;
  return motors[motorIndex].speedStepsPerSec;
}

void MultiStepper6::stopMotor(uint8_t motorIndex)
{
  if (motorIndex >= NUM_MOTORS) return;

  portENTER_CRITICAL(&s_timerMux);

  motors[motorIndex].speedStepsPerSec = 0.0f;
  motors[motorIndex].direction        = 0;
  motors[motorIndex].stepIntervalUs   = 0;

  portEXIT_CRITICAL(&s_timerMux);
}

void MultiStepper6::stopAll()
{
  portENTER_CRITICAL(&s_timerMux);
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    motors[i].speedStepsPerSec = 0.0f;
    motors[i].direction        = 0;
    motors[i].stepIntervalUs   = 0;
  }
  portEXIT_CRITICAL(&s_timerMux);
}

// static
void MultiStepper6::timerCallback(void *arg)
{
  MultiStepper6 *obj = static_cast<MultiStepper6*>(arg);
  if (obj) {
    obj->onTimer();
  }
}

void MultiStepper6::onTimer()
{
  // Questo callback gira nel task del timer su core 0
  int64_t now = esp_timer_get_time(); // microsecondi dal boot

  portENTER_CRITICAL(&s_timerMux);

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    MotorState &m = motors[i];

    if (m.stepIntervalUs == 0 || m.direction == 0) {
      continue; // motore fermo
    }

    int64_t elapsed = now - m.lastStepTimeUs;

    if (elapsed >= (int64_t)m.stepIntervalUs) {
      m.lastStepTimeUs = now;

      // Genera impulso sul pin STEP
      fastWritePin(m.stepPin, true);
      fastWritePin(m.stepPin, false);
    }
  }

  portEXIT_CRITICAL(&s_timerMux);
}

// gestione gradi/sec 

void MultiStepper6::setStepsPerRevolution(uint8_t motorIndex, float stepsPerRevValue)
{
  if (motorIndex >= NUM_MOTORS) return;
  stepsPerRev[motorIndex] = stepsPerRevValue;
}

float MultiStepper6::degPerSecToStepsPerSec(uint8_t motorIndex, float degPerSec) const
{
  if (motorIndex >= NUM_MOTORS) return 0.0f;

  float spr = stepsPerRev[motorIndex];
  if (spr <= 0.0f) {
    // non configurato: meglio non generare movimento
    return 0.0f;
  }

  // steps/sec = deg/sec * (steps/rev) / 360
  return (degPerSec * spr) / 360.0f;
}

void MultiStepper6::setSpeedDegPerSec(uint8_t motorIndex, float degPerSec)
{
  float stepsPerSec = degPerSecToStepsPerSec(motorIndex, degPerSec);
  setSpeed(motorIndex, stepsPerSec);
}

void MultiStepper6::setDirectionPolarity(uint8_t motorIndex, bool invert)
{
  if (motorIndex >= NUM_MOTORS) return;

  portENTER_CRITICAL(&s_timerMux);

  MotorState &m = motors[motorIndex];
  m.dirPolarity = invert ? -1 : +1;

  // riapplica la direzione corrente al pin DIR con la nuova polarità
  applyDirection(m);

  portEXIT_CRITICAL(&s_timerMux);
}

bool MultiStepper6::getDirectionPolarity(uint8_t motorIndex) const
{
  if (motorIndex >= NUM_MOTORS) return false;

  // true se invertito, false se normale
  return (motors[motorIndex].dirPolarity < 0);
}
