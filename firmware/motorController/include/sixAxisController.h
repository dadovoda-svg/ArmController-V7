#pragma once
#include <Arduino.h>
#include "SCurvePosVelController.h"
#include "MultiStepper6.h"
#include "ConfigStore.h"

extern bool encoderEnabled[6];      //definito in main.c

class sixAxisController {
public:
    static const uint8_t NUM_JOINTS = 6;

    sixAxisController(MultiStepper6 &stp, AS5600Mux &enc)
        : _steppers(stp),
          _encoders(enc)
    {}

    // ---------- Configuration ----------
    // Ricarica tutti i paramteri dei pid dalla configurazione
    void loadParams (ConfigStore &configParams, float joints[sixAxisController::NUM_JOINTS]) {
        char szParam[4][8];
        for (int i = 0; i < sixAxisController::NUM_JOINTS; ++i) {

            // limiti meccanici
            strcpy (szParam[0], "lmin0");
            szParam[0][4] = '0' + i;
            strcpy (szParam[1], "lmax0");
            szParam[1][4] = '0' + i;
        
            _ctrl[i].setPositionLimits(configParams.get(szParam[0]), configParams.get(szParam[1]), 0.5f); // margin 0.5°

            // parametri S-curve + jerk limit
            strcpy (szParam[0], "sm0");
            szParam[0][2] = '0' + i;
            strcpy (szParam[1], "am0");
            szParam[1][2] = '0' + i;
            strcpy (szParam[2], "jm0");
            szParam[2][2] = '0' + i;
            _ctrl[i].setLimits(configParams.get(szParam[0]), configParams.get(szParam[1]));      //vMax, aMax
            _ctrl[i].setSCurveTime(configParams.get(szParam[2]));

            // PID iniziale conservativo
            strcpy (szParam[0], "kp0");
            szParam[0][2] = '0' + i;
            strcpy (szParam[1], "ki0");
            szParam[1][2] = '0' + i;
            strcpy (szParam[2], "kd0");
            szParam[2][2] = '0' + i;
            strcpy (szParam[3], "ff0");
            szParam[3][2] = '0' + i;
            _ctrl[i].setGains(configParams.get(szParam[0]), configParams.get(szParam[1]), configParams.get(szParam[2]), configParams.get(szParam[3]));
            
            strcpy (szParam[0], "il0");
            szParam[0][2] = '0' + i;
            strcpy (szParam[1], "pt0");
            szParam[1][2] = '0' + i;        
            strcpy (szParam[2], "vt0");
            szParam[2][2] = '0' + i;        
            _ctrl[i].setIntegratorLimit(configParams.get(szParam[0]));
            _ctrl[i].setTolerances(configParams.get(szParam[1]), configParams.get(szParam[2]));
            
            _ctrl[i].setOutputMax(0.0f);
            _ctrl[i].setDeadband(0.10f, 0.15f, 0.8f);

            // le posizioni attuali corrispondono al setpoint
            _ctrl[i].reset(joints[i]);
            _lastPos[i] = joints[i];
            }
    }
        //ricarica tutti i parametri dei PID
        void reloadParams (ConfigStore &configParams) {
        char szParam[4][8];
        for (int i = 0; i < sixAxisController::NUM_JOINTS; ++i) {
            // limiti meccanici
            strcpy (szParam[0], "lmin0");
            szParam[0][4] = '0' + i;
            strcpy (szParam[1], "lmax0");
            szParam[1][4] = '0' + i;
            
            _ctrl[i].setPositionLimits(configParams.get(szParam[0]), configParams.get(szParam[1]), 0.5f); // margin 0.5°

            // parametri S-curve + jerk limit
            strcpy (szParam[0], "sm0");
            szParam[0][2] = '0' + i;
            strcpy (szParam[1], "am0");
            szParam[1][2] = '0' + i;
            strcpy (szParam[2], "jm0");
            szParam[2][2] = '0' + i;
            _ctrl[i].setLimits(configParams.get(szParam[0]), configParams.get(szParam[1]));      //vMax, aMax
            _ctrl[i].setSCurveTime(configParams.get(szParam[2]));

            // PID iniziale conservativo
            strcpy (szParam[0], "kp0");
            szParam[0][2] = '0' + i;
            strcpy (szParam[1], "ki0");
            szParam[1][2] = '0' + i;
            strcpy (szParam[2], "kd0");
            szParam[2][2] = '0' + i;
            strcpy (szParam[3], "ff0");
            szParam[3][2] = '0' + i;
            _ctrl[i].setGains(configParams.get(szParam[0]), configParams.get(szParam[1]), configParams.get(szParam[2]), configParams.get(szParam[3]));
            
            strcpy (szParam[0], "il0");
            szParam[0][2] = '0' + i;
            strcpy (szParam[1], "pt0");
            szParam[1][2] = '0' + i;        
            strcpy (szParam[2], "vt0");
            szParam[2][2] = '0' + i;        
            _ctrl[i].setIntegratorLimit(configParams.get(szParam[0]));
            _ctrl[i].setTolerances(configParams.get(szParam[1]), configParams.get(szParam[2]));

            _ctrl[i].setOutputMax(0.0f);
            _ctrl[i].setDeadband(0.10f, 0.15f, 0.8f);
        }
    }
    // ---------- Lifecycle ----------
    // Call once after you know your initial measured position.
    void reset(void) {
    }

    // ---------- Control Update ----------
    bool update(void) {
        bool fault = false;

        for (int i = 0; i < sixAxisController::NUM_JOINTS; ++i) {
            float pos;
            if (encoderEnabled[i]) {
                if (_encoders.readAngleZeroedDegreesSigned(i, pos)) {
                    _lastPos[i] = pos;
                }
                else {
                    //se perdo la lettura dell'encoder fisso il target ugiale all'ultima posizione letta 
                    //in modo da fermare il movimento
                    pos = _lastPos[i];
                    _ctrl[i].setTarget(pos);
                }
            }
            else {
                pos = 0.0f;
                _ctrl[i].setTarget(pos);
                _lastPos[i] = pos;
            }
            float vcmd = _ctrl[i].update(pos);      // dt da micros()
            _lastVel[i] = vcmd;
            _steppers.setSpeedDegPerSec (i, vcmd);

            //TODO: gestire correttamente un eventaule fault. per ora mi limito a ripulirlo e a segnalarlo sopra
            if (_ctrl[i].fault()) {
                fault = true;
                _ctrl[i].clearFault();
            }
        }
        return (fault);
    }

    void setTarget (float joints[sixAxisController::NUM_JOINTS]) {
        for (int i = 0; i < sixAxisController::NUM_JOINTS; ++i) {
            _ctrl[i].setTarget(joints[i]);
        }
    }

    void setTarget (uint8_t joint, float pos) {
        _ctrl[joint].setTarget(pos);
    }

    float getTarget (uint8_t joint) {
        return _ctrl[joint].target();
    }

    float getLastMeas (uint8_t joint) {
        return _lastPos[joint];
    }

  // Optional debug access
  float refPos(uint8_t i) const { return _ctrl[i].refPos(); }
  float refVel(int8_t i) const { return _ctrl[i].refVel(); }
  float refAcc(int8_t i) const { return _ctrl[i].refAcc(); }
  float lastCmd(int8_t i) const { return _lastVel[i]; }

private:
    MultiStepper6   &_steppers;
    AS5600Mux       &_encoders;
    SCurvePosVelController _ctrl[sixAxisController::NUM_JOINTS];
    float _lastPos[sixAxisController::NUM_JOINTS] = {0.0f};
    float _lastVel[sixAxisController::NUM_JOINTS] = {0.0f};

};
