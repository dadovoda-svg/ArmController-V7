#pragma once
#include <Arduino.h>
#include <math.h>

class SCurvePosVelController {
public:
  struct Limits {
    float v_max = 0.0f;     // [deg/s]
    float a_max = 0.0f;     // [deg/s^2]
    float j_max = 0.0f;     // [deg/s^3]
    float out_max = 0.0f;   // [deg/s] clamp finale (default = v_max)
  };

  struct Gains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;        // usato come D su misura (consigliato)
    float ff_vel = 1.0f;
  };

  struct Tolerances {
    float pos = 0.0f;       // [deg]
    float vel = 0.0f;       // [deg/s]
  };

  enum class FaultCode : uint8_t {
    None = 0,
    PositionLimitExceeded = 1,
    BadLimits = 2
  };

  SCurvePosVelController() = default;

  // ---------- Configuration ----------
  void setLimits(float v_max, float a_max) {
    _lim.v_max = fabsf(v_max);
    _lim.a_max = fabsf(a_max);
    if (_lim.out_max <= 0.0f) _lim.out_max = _lim.v_max;
  }

  // S-curve smoothing time (seconds): accel ramps 0->Amax in t_jerk
  void setSCurveTime(float t_jerk) {
    const float tj = fmaxf(t_jerk, 1e-6f);
    _lim.j_max = (_lim.a_max > 0.0f) ? (_lim.a_max / tj) : 0.0f;
  }

  void setJerkMax(float j_max) { _lim.j_max = fabsf(j_max); }

  void setOutputMax(float out_max) { _lim.out_max = fabsf(out_max); }

  void setGains(float kp, float ki, float kd, float ff_vel = 1.0f) {
    _g.kp = kp; _g.ki = ki; _g.kd = kd; _g.ff_vel = ff_vel;
  }

  void setIntegratorLimit(float i_abs_max) { _i_abs_max = fabsf(i_abs_max); }

  void setTolerances(float pos_tol, float vel_tol) {
    _tol.pos = fabsf(pos_tol);
    _tol.vel = fabsf(vel_tol);
  }

  // Hard mechanical limits: must NEVER be exceeded.
  // stop_margin_deg: zona di guardia interna dove blocchiamo i comandi che spingono "verso fuori"
  void setPositionLimits(float pos_min_deg, float pos_max_deg, float stop_margin_deg = 0.5f) {
    _pos_min = pos_min_deg;
    _pos_max = pos_max_deg;
    _stop_margin = fabsf(stop_margin_deg);
    _limits_enabled = true;

    if (_pos_max <= _pos_min) {
      _fault = FaultCode::BadLimits;
      _fault_latched = true;
    }
  }

  void disablePositionLimits() { _limits_enabled = false; }

  // ---------- Safety / Fault ----------
  bool fault() const { return _fault_latched; }
  FaultCode faultCode() const { return _fault; }

  // Clear fault latch (does NOT move anything; output remains 0 until next update computes it)
  void clearFault() {
    _fault_latched = false;
    _fault = FaultCode::None;
    // reset integrator for safety
    _i_term = 0.0f;
    _ref_vel = 0.0f;
    _ref_acc = 0.0f;
  }

  // ---------- Lifecycle ----------
  // Call once after you know your initial measured position.
  void reset(float measured_pos) {
    if (_limits_enabled) measured_pos = clampf(measured_pos, _pos_min, _pos_max);

    _target = measured_pos;
    _ref_pos = measured_pos;
    _ref_vel = 0.0f;
    _ref_acc = 0.0f;

    _i_term = 0.0f;
    _prev_meas = measured_pos;
    _has_prev = false;

    _fault = FaultCode::None;
    _fault_latched = false;

    _meas_vel_f = 0.0f;
    _last_meas_vel = 0.0f;
    _last_meas_pos = measured_pos;
    _target_gen = 0;
    _meas_gen   = 0;

    _last_us = micros();
  }

  // Target can be updated continuously.
  // Target is clamped inside hard limits (prevention).
  void setTarget(float target_pos) {
    if (_limits_enabled) {
      _target = clampf(target_pos, _pos_min, _pos_max);
    } else {
      _target = target_pos;
    }

    _target_gen++;          // NEW: target changed
    _db_active = false;     // consigliato: esci dalla deadband quando cambia target
  }

  float target() const { return _target; }

  // Optional debug access
  float refPos() const { return _ref_pos; }
  float refVel() const { return _ref_vel; }
  float refAcc() const { return _ref_acc; }

  // ---------- Control Update ----------
  float update(float measured_pos) {
    const uint32_t now = micros();
    float dt = (now - _last_us) * 1e-6f;
    _last_us = now;

    if (dt <= 0.0f) dt = 1e-6f;
    if (dt > 0.2f)  dt = 0.2f;

    return update(measured_pos, dt);
  }

  float update(float measured_pos, float dt_s) {
    const float dt = fmaxf(dt_s, 1e-6f);

    // 0) HARD SAFETY CHECK: measured position must be inside limits.
    if (_limits_enabled) {
      if (measured_pos < _pos_min || measured_pos > _pos_max) {
        // Immediate stop + latch fault
        _fault = FaultCode::PositionLimitExceeded;
        _fault_latched = true;

        // Freeze internal states (prevents integrator windup and odd profile states)
        _i_term = 0.0f;
        _ref_vel = 0.0f;
        _ref_acc = 0.0f;
        _ref_pos = clampf(measured_pos, _pos_min, _pos_max);
        _target = _ref_pos;

        _prev_meas = _ref_pos;
        _has_prev = false;

        _meas_vel_f = 0.0f;
        _last_meas_pos = _ref_pos;
        _last_meas_vel = 0.0f;

        return 0.0f; // command velocity = 0 NOW
      }
    }

    // If fault latched, stay stopped
    if (_fault_latched) {
      return 0.0f;
    }
    
    // ---- velocity estimate (needed for deadband and D term) ----
    float meas_vel = 0.0f;
    if (_has_prev) {
      meas_vel = (measured_pos - _prev_meas) / dt; // [deg/s]
    }

    // ---- Low-pass filter on measured velocity ----
    // tau = 0 => no filtering
    float meas_vel_used = meas_vel;
    if (_vel_f_tau > 0.0f) {
      const float alpha = dt / (_vel_f_tau + dt);
      _meas_vel_f += alpha * (meas_vel - _meas_vel_f);
      meas_vel_used = _meas_vel_f;
    } else {
      _meas_vel_f = meas_vel;
      meas_vel_used = meas_vel;
    }

    // Save last measurement for isSettled()
    _last_meas_pos = measured_pos;
    _last_meas_vel = meas_vel_used;
    _meas_gen = _target_gen;      // NEW: we have a measurement for current target generation

    // ---- Deadband check (uses target vs measured) ----
    if (_db_enabled && !_fault_latched) {
      float tgt = _target;
      if (_limits_enabled) tgt = clampf(tgt, _pos_min, _pos_max);

      const float err_tgt = tgt - measured_pos;
      const float aerr = fabsf(err_tgt);
      //const float avel = fabsf(meas_vel);
      const float avel = fabsf(meas_vel_used);

      if (_db_active) {
        // remain in deadband until we exceed exit threshold or we are moving too fast
        if (aerr <= _db_exit && avel <= _db_vel) {
          // freeze all internal dynamics and stop output
          _ref_pos = tgt;
          _ref_vel = 0.0f;
          _ref_acc = 0.0f;
          _i_term  = 0.0f;     // evita micro-creep da integrale
          _prev_meas = measured_pos;
          _has_prev = true;
          // Save last measurement for isSettled()
          _last_meas_pos = measured_pos;
          //_last_meas_vel = meas_vel;
          _last_meas_vel = meas_vel_used;
          return 0.0f;
        } else {
          _db_active = false; // exit deadband -> resume control
        }
      } else {
        // enter deadband only when close enough AND slow enough
        if (aerr <= _db_enter && avel <= _db_vel) {
          _db_active = true;
          _ref_pos = tgt;
          _ref_vel = 0.0f;
          _ref_acc = 0.0f;
          _i_term  = 0.0f;
          _prev_meas = measured_pos;
          _has_prev = true;
          // Save last measurement for isSettled()
          _last_meas_pos = measured_pos;
          //_last_meas_vel = meas_vel;
          _last_meas_vel = meas_vel_used;
          return 0.0f;
        }
      }
    }

    // 1) Trajectory generator (jerk-limited), always confined within limits
    stepTrajectory(dt);

    // 2) Position error between reference position and measured position
    const float err = _ref_pos - measured_pos;

    // 3) Derivative on measurement (recommended with moving setpoint)
    //const float d = -_g.kd * meas_vel;
    const float d = -_g.kd * meas_vel_used;

    // e solo qui aggiorni lo stato "prev"
    _prev_meas = measured_pos;
    _has_prev = true;

    // 4) Integrator with clamp
    _i_term += _g.ki * err * dt;
    if (_i_abs_max > 0.0f) {
      _i_term = clampf(_i_term, -_i_abs_max, +_i_abs_max);
    }

    const float p = _g.kp * err;
    float v_cmd = (_g.ff_vel * _ref_vel) + p + _i_term + d;

    // 5) Output clamp
    const float out_lim = (_lim.out_max > 0.0f) ? _lim.out_max : _lim.v_max;
    if (out_lim > 0.0f) {
      const float v_sat = clampf(v_cmd, -out_lim, +out_lim);
      if (v_sat != v_cmd && _g.ki != 0.0f) {
        // undo last integration step if saturating
        _i_term -= _g.ki * err * dt;
      }
      v_cmd = v_sat;
    }

    // 6) Soft-stop near hard limits: forbid commands that push outward.
    // This is prevention; the "must not exceed" is enforced by the HARD CHECK above.
    if (_limits_enabled && _stop_margin > 0.0f) {
      const float lo = _pos_min + _stop_margin;
      const float hi = _pos_max - _stop_margin;

      if (measured_pos <= lo && v_cmd < 0.0f) v_cmd = 0.0f;
      if (measured_pos >= hi && v_cmd > 0.0f) v_cmd = 0.0f;
    }

    return v_cmd;
  }

  // Deadband (position) with hysteresis:
  // - enter_deg: quando |target - measured| <= enter_deg e la velocità è bassa, fermo e latched
  // - exit_deg : soglia più alta per uscire (se 0 => exit = enter)
  // - vel_deg_s: condizione aggiuntiva: serve che la velocità misurata sia bassa per entrare/restare in deadband
  void setDeadband(float enter_deg, float exit_deg = 0.0f, float vel_deg_s = 0.5f) {
    _db_enabled = (enter_deg > 0.0f);
    _db_enter = fabsf(enter_deg);
    _db_exit  = (exit_deg > 0.0f) ? fabsf(exit_deg) : _db_enter;
    if (_db_exit < _db_enter) _db_exit = _db_enter; // safety
    _db_vel   = fabsf(vel_deg_s);
    _db_active = false;
  }

  void setVelocityFilterTau(float tau_s) {
    _vel_f_tau = fmaxf(tau_s, 0.0f);   // 0 disables filtering
  }

  void disableDeadband() {
    _db_enabled = false;
    _db_active = false;
  }

  bool inDeadband() const { return _db_active; }

  float getLastMeasuredVel() const { return _last_meas_vel; }

  bool isSettled() const {
    if (_fault_latched) return false;

    if (_meas_gen != _target_gen) return false;   // NEW

    // Se la deadband è attiva, per definizione siamo "fermi a target"
    if (_db_enabled && _db_active) return true;

    // Serve una definizione di "settled": usiamo le tolleranze
    if (_tol.pos <= 0.0f || _tol.vel <= 0.0f) return false;

    float tgt = _target;
    if (_limits_enabled) tgt = clampf(tgt, _pos_min, _pos_max);

    const float pos_err = fabsf(tgt - _last_meas_pos);
    const float vel_abs = fabsf(_last_meas_vel);

    return (pos_err <= _tol.pos) && (vel_abs <= _tol.vel);
  }

private:
  Limits _lim{};
  Gains _g{};
  Tolerances _tol{};

  float _target = 0.0f;

  // reference state
  float _ref_pos = 0.0f;
  float _ref_vel = 0.0f;
  float _ref_acc = 0.0f;

  // PID state
  float _i_term = 0.0f;
  float _i_abs_max = 0.0f;

  // derivative state
  float _prev_meas = 0.0f;
  bool  _has_prev = false;

  // safety limits
  bool  _limits_enabled = false;
  float _pos_min = -INFINITY;
  float _pos_max = +INFINITY;
  float _stop_margin = 0.5f;

  // fault state
  FaultCode _fault = FaultCode::None;
  bool _fault_latched = false;

  // timekeeping
  uint32_t _last_us = 0;

  // Deadband state
  bool  _db_enabled = false;
  bool  _db_active  = false;
  float _db_enter   = 0.0f;   // [deg]
  float _db_exit    = 0.0f;   // [deg]
  float _db_vel     = 0.5f;   // [deg/s]

  // last measurement (for isSettled)
  float _last_meas_pos = 0.0f;
  float _last_meas_vel = 0.0f;
  uint32_t _target_gen = 0;
  uint32_t _meas_gen   = 0;


  // filtered measured velocity (for D, deadband, isSettled)
  float _meas_vel_f = 0.0f;
  float _vel_f_tau  = 0.05f;   // [s] default 50ms (good @200Hz)


  static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }

  static float signf(float x) { return (x > 0.0f) - (x < 0.0f); }

  void stepTrajectory(float dt) {
    if (_lim.v_max <= 0.0f || _lim.a_max <= 0.0f) {
      _ref_pos = _target;
      _ref_vel = 0.0f;
      _ref_acc = 0.0f;
      if (_limits_enabled) _ref_pos = clampf(_ref_pos, _pos_min, _pos_max);
      return;
    }

    // Clamp target within limits (prevention)
    float tgt = _target;
    if (_limits_enabled) tgt = clampf(tgt, _pos_min, _pos_max);

    const float d = tgt - _ref_pos;
    const float ad = fabsf(d);

    // Snap if close enough
    if (_tol.pos > 0.0f && _tol.vel > 0.0f) {
      if (ad <= _tol.pos && fabsf(_ref_vel) <= _tol.vel) {
        _ref_pos = tgt;
        _ref_vel = 0.0f;
        _ref_acc = 0.0f;
        // Serial.print ("rpos ");
        // Serial.println (_ref_pos);
        return;
      }
    }

    // braking distance with a_max: v^2/(2a)
    const float v = _ref_vel;
    const float v_abs = fabsf(v);
    const float d_brake = (v_abs * v_abs) / (2.0f * _lim.a_max);

    float acc_cmd = 0.0f;
    if (ad > d_brake) {
      acc_cmd = signf(d) * _lim.a_max;   // accelerate toward target
    } else {
      const float s = (v_abs > 1e-6f) ? signf(v) : signf(d);
      acc_cmd = -s * _lim.a_max;         // decelerate to stop
    }

    // jerk limit
    if (_lim.j_max > 0.0f) {
      const float da_max = _lim.j_max * dt;
      const float da = clampf(acc_cmd - _ref_acc, -da_max, +da_max);
      _ref_acc += da;
    } else {
      _ref_acc = acc_cmd;
    }

    // integrate velocity
    _ref_vel += _ref_acc * dt;
    _ref_vel = clampf(_ref_vel, -_lim.v_max, +_lim.v_max);

    // integrate position
    _ref_pos += _ref_vel * dt;

    // confine reference to limits (prevention)
    if (_limits_enabled) {
      if (_ref_pos <= _pos_min) { _ref_pos = _pos_min; _ref_vel = 0.0f; _ref_acc = 0.0f; }
      if (_ref_pos >= _pos_max) { _ref_pos = _pos_max; _ref_vel = 0.0f; _ref_acc = 0.0f; }
      tgt = clampf(tgt, _pos_min, _pos_max);
    }

    // overshoot guard relative to target
    const float d2 = tgt - _ref_pos;
    if (signf(d) != 0.0f && signf(d2) != 0.0f && signf(d) != signf(d2)) {
      _ref_pos = tgt;
      _ref_vel = 0.0f;
      _ref_acc = 0.0f;
    }
  }
};

