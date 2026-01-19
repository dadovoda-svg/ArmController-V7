#pragma once
#include <Arduino.h>
#include "esp_timer.h"

class MultiStepper6
{
  public:
    static const uint8_t NUM_MOTORS = 6;

    // timerPeriodUs: periodo del callback esp_timer in microsecondi
    MultiStepper6(const uint8_t stepPins[NUM_MOTORS],
                  const uint8_t dirPins[NUM_MOTORS],
                  float   defaultPulseWidthUs = 3.0f,
                  uint32_t timerPeriodUs      = 100);

    // Configura i pin e avvia il timer su core 0
    void begin();

    // Ferma e distrugge il timer (opzionale)
    void end();

    // Imposta la velocità in step/sec per un motore (indice 0..5)
    // speedStepsPerSec > 0  -> dir "positiva"
    // speedStepsPerSec < 0  -> dir "negativa"
    // speedStepsPerSec == 0 -> motore fermo
    void setSpeed(uint8_t motorIndex, float speedStepsPerSec);

    float getSpeed(uint8_t motorIndex) const;

    void stopMotor(uint8_t motorIndex);
    void stopAll();

    // Imposta il numero di step per 360° per un motore (considerando anche il microstepping).
    // Esempio: NEMA 200 step/rev con microstepping 1/16 -> 3200 step/rev.
    void setStepsPerRevolution(uint8_t motorIndex, float stepsPerRev);

    // Converte una velocita' in gradi/sec nella corrispondente in step/sec,
    // usando stepsPerRev configurato per quel motore.
    // Se stepsPerRev <= 0 per quel motore, ritorna 0.
    float degPerSecToStepsPerSec(uint8_t motorIndex, float degPerSec) const;

    // Convenience: imposta direttamente la velocita' in gradi/sec.
    // Internamente usa degPerSecToStepsPerSec + setSpeed.
    void setSpeedDegPerSec(uint8_t motorIndex, float degPerSec);

    // invert = false -> polarità normale
    // invert = true  -> direzione invertita (cambia il verso per velocità > 0)
    void setDirectionPolarity(uint8_t motorIndex, bool invert);

    // opzionale: per sapere come è configurato
    bool getDirectionPolarity(uint8_t motorIndex) const;

  private:
    struct MotorState {
      uint8_t stepPin;
      uint8_t dirPin;

      float    speedStepsPerSec;   // velocità richiesta
      int8_t   direction;          // -1, 0, +1
      uint32_t stepIntervalUs;     // intervallo tra due step (us)
      int64_t  lastStepTimeUs;     // timestamp ultimo step (us, da esp_timer_get_time)
      int8_t   dirPolarity;        // +1 = normale, -1 = invertito
    };

    MotorState  motors[NUM_MOTORS];
    float       pulseWidthUs;
    uint32_t    timerPeriodUs;
    esp_timer_handle_t timerHandle;

        // == steps per revolution per ciascun motore ===
    float       stepsPerRev[NUM_MOTORS];

    // --- metodi di supporto ---
    static void timerCallback(void *arg);
    void onTimer();                            // chiamato dal callback del timer

    void computeStepInterval(MotorState &m);
    void applyDirection(MotorState &m);
};
