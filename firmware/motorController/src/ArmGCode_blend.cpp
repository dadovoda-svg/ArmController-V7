#include "ArmGCode_blend.h"
#include <math.h>   // isfinite, fabsf

// ----------------------
// PlannerCoordinator
// ----------------------

static inline float f_abs(float x) { return x < 0 ? -x : x; }
static inline float f_max(float a, float b) { return (a > b) ? a : b; }

void PlannerCoordinator::begin(const PlannerConfig& cfg, const ArmGCodeHooks& hooks) {
  _cfg = cfg;
  _hooks = hooks;

  if (_cfg.queue_max == 0) _cfg.queue_max = 1;

  if (_q) { delete[] _q; _q = nullptr; }
  _q = new Move[_cfg.queue_max];
  _qHead = _qTail = _qCount = 0;

  _executing = false;
  _moving_mask = 0;
}

bool PlannerCoordinator::enqueue(const Move& m) {
  return queuePush(m);
}

void PlannerCoordinator::tick() {
  // Need at least these hooks to operate meaningfully
  if (!_hooks.set_target_all || !_hooks.get_target_one || !_hooks.is_settled_one) return;

  if (_executing) {
    // Optional: segment blending (avoid stop-and-go on micro-segments)
    if (_blend_enabled && _hooks.get_ref_pos_one && _hooks.get_ref_vel_one && _hooks.set_limits_one) {
      Move next;
      if (queuePeek(next)) {
        // Blend only between same move type (G1->G1, G0->G0)
        if (next.type == _cur.type) {
          if (shouldBlendToNext(next)) {
            // consume next and immediately retarget while keeping motion continuous
            Move m2;
            if (queuePop(m2)) startMove(m2);
            return;
          }
        }
      }
    }

    if (moveSettled()) {
      _executing = false;
      _moving_mask = 0;
    }
    return;
  }

  Move m;
  if (queuePop(m)) {
    startMove(m);
  }
}

void PlannerCoordinator::stopImmediate() {
  _executing = false;
  _moving_mask = 0;
  _qHead = _qTail = _qCount = 0;
}

void PlannerCoordinator::estop() {
  stopImmediate();
}

bool PlannerCoordinator::queuePush(const Move& m) {
  if (_qCount >= _cfg.queue_max) return false;
  _q[_qTail] = m;
  _qTail = (_qTail + 1) % _cfg.queue_max;
  _qCount++;
  return true;
}

bool PlannerCoordinator::queuePop(Move& out) {
  if (_qCount == 0) return false;
  out = _q[_qHead];
  _qHead = (_qHead + 1) % _cfg.queue_max;
  _qCount--;
  return true;
}

bool PlannerCoordinator::queuePeek(Move& out) const {
  if (_qCount == 0) return false;
  out = _q[_qHead];
  return true;
}


void PlannerCoordinator::startMove(const Move& m) {
  _cur = m;

  // Compute deltas using a dynamic start:
  // - if we are already executing (blending), use current reference position
  // - otherwise use current TARGET (safe when starting from idle)
  float delta[6];
  float Dmax = 0.0f;
  for (int i = 0; i < 6; i++) {
    float start = _hooks.get_ref_pos_one ? _hooks.get_ref_pos_one(i)
                                        : _hooks.get_target_one(i);
//    float start = _hooks.get_target_one((uint8_t)i);
    if (_executing && _hooks.get_ref_pos_one) start = _hooks.get_ref_pos_one((uint8_t)i);
    delta[i] = _cur.target_deg[i] - start;
    float d = fabsf(delta[i]);
    if (d > Dmax) Dmax = d;

    // Serial1.printf ("## J%d old %4.1f new %4.1f D %4.1f", i+1, start, _cur.target_deg[i], delta[i]);
    // Serial1.println (" - ");
  }

  if (Dmax < _cfg.min_delta_deg) {
    // nothing to do
    _executing = false;
    _moving_mask = 0;
    return;
  }

  const PlannerProfile& prof = (_cur.type == MoveType::RapidG0) ? _cfg.prof_G0 : _cfg.prof_G1;

  const float Vbase = f_max(_cfg.min_v_deg_s, _cur.v_deg_s);
  const float Abase = f_max(_cfg.min_a_deg_s2, prof.a_max_deg_s2);
  const float Tj    = f_max(1e-6f, prof.t_jerk_s);

  // Apply per-joint limits scaled by relative delta (ri = |Δi|/Dmax)
  _moving_mask = 0;

  if (_hooks.set_limits_one && _hooks.set_scurve_time_one) {
    for (int i = 0; i < 6; i++) {
      float d = fabsf(delta[i]);
      if (d < _cfg.min_delta_deg) continue;

      float r  = d / Dmax; // 0..1
      float vi = f_max(_cfg.min_v_deg_s,  Vbase * r);
      float ai = f_max(_cfg.min_a_deg_s2, Abase * r);

      Serial1.printf ("## J%d Vm %4.1f Am %4.1f", i+1, vi, ai);
      Serial1.println (" - ");

      _vmax_deg_s[i]  = vi;
      _amax_deg_s2[i] = ai;
      _hooks.set_limits_one((uint8_t)i, vi, ai);
      _hooks.set_scurve_time_one((uint8_t)i, Tj);
      _moving_mask |= (1u << i);
    }
  } else {
    // If limits APIs are missing, we still can run, just uncoordinated.
    // Consider this acceptable; internal controllers will use their last configured limits.
    for (int i = 0; i < 6; i++) {
      if (fabsf(delta[i]) >= _cfg.min_delta_deg) {
        _vmax_deg_s[i]  = 0.0f;
        _amax_deg_s2[i] = 0.0f;
        _moving_mask |= (1u << i);
      }
    }
  }

  // Start move: single target update
  _hooks.set_target_all(_cur.target_deg);

  _executing = true;
}

bool PlannerCoordinator::moveSettled() const {
  // Only require settled for joints that actually moved
  for (int i = 0; i < 6; i++) {
    if (!(_moving_mask & (1u << i))) continue;
    if (!_hooks.is_settled_one((uint8_t)i)) return false;
  }
  return true;
}


bool PlannerCoordinator::shouldBlendToNext(const Move& next) const {
  // Preconditions: ref hooks + cached limits must be available
  // Heuristic: switch to next target when remaining distance is within a multiple
  // of the estimated braking distance. This prevents the controller from starting
  // to decelerate hard toward intermediate waypoints.
  float worst_margin = 0.0f;

  for (int i = 0; i < 6; i++) {
    // Consider joints that are moving in current segment OR will move in next
    const bool curMoves  = (_moving_mask & (1u << i)) != 0;
    const bool nextMoves = (next.changed_mask & (1u << i)) != 0;
    if (!curMoves && !nextMoves) continue;

    const float pos = _hooks.get_ref_pos_one((uint8_t)i);
    const float vel = fabsf(_hooks.get_ref_vel_one((uint8_t)i));
    const float tgt = _cur.target_deg[i];

    const float rem = fabsf(tgt - pos);

    // Need a meaningful accel limit for this joint to estimate braking distance
    const float a = _amax_deg_s2[i];
    if (a <= 1e-6f) return false;

    const float d_brake = (vel * vel) / (2.0f * a); // deg
    const float window  = f_max(_blend_min_rem, _blend_k_brake * d_brake);

    // If any joint still has too much remaining, do not blend yet
    if (rem > window) return false;

    // Track margin for potential tuning/telemetry (optional)
    const float margin = window - rem;
    if (margin > worst_margin) worst_margin = margin;
  }

  // Extra safety: if current segment is extremely tiny, avoid thrashing
  // (still allows blending if ref is already very close)
  return true;
}

// ----------------------
// ArmGCode
// ----------------------

ArmGCode::ArmGCode(const ArmGCodeConfig& cfg, const PlannerConfig& pcfg)
: _cfg(cfg), _pcfg(pcfg) {}

void ArmGCode::begin(Stream& io, const ArmGCodeHooks& hooks) {
  _io = &io;
  _hooks = hooks;

  if (_cfg.line_max > sizeof(_lineBuf) - 1) _cfg.line_max = sizeof(_lineBuf) - 1;
  if (_cfg.queue_max == 0) _cfg.queue_max = 1;

  // planner uses same queue depth
  _pcfg.queue_max = _cfg.queue_max;
  _planner.begin(_pcfg, _hooks);

  _lineLen = 0;
  _estop = false;

  // Seed current_target from controller targets if possible
  if (_hooks.get_target_one) {
    for (int i=0;i<6;i++) _current_target_deg[i] = _hooks.get_target_one((uint8_t)i);
  }
}

void ArmGCode::poll() {
  if (!_io) return;
  while (readLine()) {
    handleLine(_lineBuf);
  }
}

bool _sawLineEnd = false;  // true se abbiamo appena chiuso riga (CR/LF)
char _lastEol = 0;         // ultimo terminatore visto: '\r' o '\n'

bool ArmGCode::readLine() {
  while (_io->available() > 0) {
    int c = _io->read();
    if (c < 0) break;

    //_io->write(c);
    char ch = (char)c;

    // Se abbiamo appena chiuso una riga, assorbi l'eventuale "secondo" terminatore
    // (gestisce CRLF e LFCR senza generare una riga vuota)
    if (_sawLineEnd) {
      _sawLineEnd = false;
      if ((ch == '\n' && _lastEol == '\r') || (ch == '\r' && _lastEol == '\n')) {
        // swallow
        continue;
      }
      // altrimenti il char è parte della prossima riga e va processato normalmente
    }

    // Fine riga: CR oppure LF
    if (ch == '\n' || ch == '\r') {
      _lineBuf[_lineLen] = '\0';
      _lineLen = 0;

      _sawLineEnd = true;
      _lastEol = ch;

      return true;
    }

    // Accumula caratteri
    if (_lineLen < _cfg.line_max) {
      _lineBuf[_lineLen++] = ch;
    } else {
      // overflow: continua a consumare fino al terminatore, poi segnala in handleLine()
      // saturiamo per far scattare il check "line_too_long"
      _lineLen = _cfg.line_max;
    }
  }
  return false;
}

void ArmGCode::stripComment(char* s) {
  for (char* p = s; *p; ++p) {
    if (*p == ';') { *p = '\0'; return; }
  }
}

void ArmGCode::trimInPlace(char* s) {
  char* p = s;
  while (*p == ' ' || *p == '\t') p++;
  if (p != s) memmove(s, p, strlen(p) + 1);

  size_t n = strlen(s);
  while (n > 0 && (s[n-1] == ' ' || s[n-1] == '\t')) { s[n-1] = '\0'; n--; }
}

bool ArmGCode::parseFloatStrict(const char* s, float& out) {
  char* endp = nullptr;
  out = strtof(s, &endp);
  if (endp == s || *endp != '\0') return false;
  if (!isfinite(out)) return false;
  return true;
}

ArmGCode::Tokens ArmGCode::splitTokens(char* s) {
  Tokens tk;
  char* p = s;
  while (*p) {
    while (*p == ' ' || *p == '\t') p++;
    if (!*p) break;

    if (tk.n < (sizeof(tk.t)/sizeof(tk.t[0]))) tk.t[tk.n++] = p;

    while (*p && *p != ' ' && *p != '\t') p++;
    if (*p) { *p = '\0'; p++; }
  }
  return tk;
}

ArmGCode::Cmd ArmGCode::parseCmd(const char* s) {
  Cmd c;
  if (!s || !s[0]) return c;

  char t = s[0];
  if (t >= 'a' && t <= 'z') t -= 32;
  if (t != 'G' && t != 'M') return c;

  int v = 0;
  const char* p = s + 1;
  if (*p == '\0') return c;
  while (*p) {
    if (*p < '0' || *p > '9') return c;
    v = (v * 10) + (*p - '0');
    p++;
  }
  c.type = t;
  c.code = v;
  return c;
}

bool ArmGCode::parseParams(const Tokens& tk, uint8_t start, Params& p) {
  for (uint8_t i = start; i < tk.n; i++) {
    const char* token = tk.t[i];
    const char* eq = strchr(token, '=');
    if (!eq) { sendErr("bad_param", "format"); return false; }

    int nameLen = (int)(eq - token);
    if (nameLen <= 0) { sendErr("bad_param", "format"); return false; }
    if (nameLen >= 8) { sendErr("bad_param", "name"); return false; }

    char name[8];
    memcpy(name, token, nameLen);
    name[nameLen] = '\0';

    const char* valueStr = eq + 1;
    if (!valueStr[0]) { sendErr("bad_param", "empty"); return false; }

    // V
    if ((name[0]=='V' || name[0]=='v') && name[1]=='\0') {
      float v;
      if (!parseFloatStrict(valueStr, v)) { sendErr("bad_param", "V"); return false; }
      p.hasV = true;
      p.V = v;
      continue;
    }

    // J1..J6
    if ((name[0]=='J' || name[0]=='j') && name[2]=='\0' && name[1]>='1' && name[1]<='6') {
      int idx = name[1] - '1';
      float j;
      if (!parseFloatStrict(valueStr, j)) { sendErr("bad_param", name); return false; }
      p.hasJ[idx] = true;
      p.J[idx] = j;
      continue;
    }

    sendErr("bad_param", name);
    return false;
  }
  return true;
}

void ArmGCode::handleLine(char* line) {
  if (strlen(line) >= _cfg.line_max) { sendErr("line_too_long"); return; }

  stripComment(line);
  trimInPlace(line);
  if (!line[0]) return;

  Tokens tk = splitTokens(line);
  if (tk.n == 0) return;

  Cmd cmd = parseCmd(tk.t[0]);
  if (!cmd.valid()) { sendErr("unknown_command"); return; }

  Params p;
  if (!parseParams(tk, 1, p)) return;

  dispatch(cmd, p);
}

void ArmGCode::sendOk() {
  _io->println("ok");
}

void ArmGCode::sendErr(const char* code, const char* detail) {
  _io->print("error:");
  _io->print(code);
  if (detail) {
    _io->print(" ");
    _io->print(detail);
  }
  _io->println();
}

void ArmGCode::dispatch(const Cmd& cmd, const Params& p) {
  // E-stop gate
  if (_estop) {
    if (cmd.isMovement() || cmd.isHoming() || cmd.isEnableMotors()) {
      sendErr("estop");
      return;
    }
  }

    // M400 pending gate: non bloccare il firmware, ma blocca i comandi "che cambiano stato"
  if (_pending == PendingReply::M400) {
    // Consenti query (debug) e comandi di emergenza/stop
    if (cmd.isM(114) || cmd.isM(115) || cmd.isM(18) || cmd.isM(112)) {
      // allowed
    } else if (cmd.isM(400)) {
      // già in attesa di un M400: rispondi busy (o puoi ignorare)
      sendErr("busy");
      return;
    } else {
      sendErr("busy");
      return;
    }
  }

  if (cmd.isM(115)) { handle_M115(); return; }
  if (cmd.isM(114)) { handle_M114(); return; }
  if (cmd.isM(17))  { handle_M17();  return; }
  if (cmd.isM(18))  { handle_M18();  return; }
  if (cmd.isM(400)) { handle_M400(); return; }
  // if (cmd.isM(400)) { handle_M400_blocking(); return; }
  if (cmd.isM(112)) { handle_M112(); return; }

  if (cmd.isG(28))  { handle_G28(); return; }
  if (cmd.isG(0) || cmd.isG(1)) { handle_G0_G1(cmd, p); return; }

  sendErr("unknown_command");
}

void ArmGCode::handle_M115() {
  _io->println("FIRMWARE_NAME:ArmFW PROTOCOL:AGC1 AXES:6 UNITS:deg,deg_s");
  sendOk();
}

void ArmGCode::handle_M114() {
  float pos[6];

  if (_hooks.get_joint_position_deg) {
    if (!_hooks.get_joint_position_deg(pos)) { sendErr("internal", "pos_fail"); return; }
  } else if (_hooks.get_target_one) {
    for (int i=0;i<6;i++) pos[i] = _hooks.get_target_one((uint8_t)i);
  } else {
    for (int i=0;i<6;i++) pos[i] = _current_target_deg[i];
  }

  char buf[128];
  snprintf(buf, sizeof(buf),
           "J:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
           pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
  _io->println(buf);
  sendOk();
}

void ArmGCode::handle_M17() {
  if (_hooks.set_motors_enabled) _hooks.set_motors_enabled(true);
  sendOk();
}

void ArmGCode::handle_M18() {
  _pending = PendingReply::None;   // cancella eventuale M400 in attesa
  _planner.stopImmediate();
  if (_hooks.set_motors_enabled) _hooks.set_motors_enabled(false);
  sendOk();
}

void ArmGCode::handle_M400() {
  // Se già in attesa di un M400, o lo ignori o rispondi busy:
  if (_pending == PendingReply::M400) {
    sendErr("busy");
    return;
  }

  // Se siamo già idle, possiamo rispondere subito
  if (_planner.isIdle()) {   
    sendOk();
    return;
  }

  // Altrimenti: risposta differita. NON inviare ok qui.
  _pending = PendingReply::M400;
}

// void ArmGCode::handle_M400_blocking() {
//   // Wait until planner idle (queue empty + current move settled)
//   // Keep calling planner.tick() here to ensure progress even if user doesn't call tickPlanner().
//   uint32_t lastKick = millis();
//   while (!_planner.isIdle()) {
//     _planner.tick();
//     delay(1); // ESP32 WDT-friendly

//     // optional: keep the loop from running too tight if something goes wrong
//     // (no time estimates, just a simple watchdog kick)
//     if (millis() - lastKick > 5000) {
//       lastKick = millis();
//       // you may emit a debug line:
//       // _io->println("## M400 waiting");
//     }
//   }
//   sendOk();
// }

void ArmGCode::handle_M112() {
  _pending = PendingReply::None;   // cancella eventuale M400 in attesa
  if (_hooks.estop_now) _hooks.estop_now();
  _planner.estop();
  if (_hooks.set_motors_enabled) _hooks.set_motors_enabled(false);
  _estop = true;
  sendErr("estop");
}

void ArmGCode::handle_G28() {
  // policy v1: refuse if planner is busy
  if (!_planner.isIdle()) { sendErr("busy"); return; }

  if (_hooks.get_motors_enabled && !_hooks.get_motors_enabled()) { sendErr("motors_disabled"); return; }
  if (!_hooks.do_homing_all) { sendErr("internal", "no_home_cb"); return; }

  if (!_hooks.do_homing_all()) { sendErr("internal", "homing_fail"); return; }

  // update local target cache from controller targets (post homing)
  if (_hooks.get_target_one) {
    for (int i=0;i<6;i++) _current_target_deg[i] = _hooks.get_target_one((uint8_t)i);
  } else {
    for (int i=0;i<6;i++) _current_target_deg[i] = 0.0f;
  }

  sendOk();
}

void ArmGCode::handle_G0_G1(const Cmd& cmd, const Params& p) {
  if (_hooks.get_motors_enabled && !_hooks.get_motors_enabled()) { sendErr("motors_disabled"); return; }
  if (_hooks.get_is_homed && !_hooks.get_is_homed()) { sendErr("not_homed"); return; }

  // Require at least one Jk
  bool anyJ = false;
  for (int i=0;i<6;i++) if (p.hasJ[i]) { anyJ = true; break; }
  if (!anyJ) { sendErr("missing_joint_param"); return; }

  // Build absolute targets (partial moves supported)
  Move m;
  m.type = cmd.isG(0) ? MoveType::RapidG0 : MoveType::LinearG1;
  m.changed_mask = 0;

  for (int i=0;i<6;i++) {
    float t = _current_target_deg[i];
    if (p.hasJ[i]) { t = p.J[i]; m.changed_mask |= (1u<<i); }
    m.target_deg[i] = t;
  }

  if (p.hasV) m.v_deg_s = p.V;
  else m.v_deg_s = cmd.isG(0) ? _cfg.v_default_rapid : _cfg.v_default;

  // enqueue
  if (!_planner.enqueue(m)) { sendErr("busy"); return; }

  // update current_target cache for subsequent partial moves
  for (int i=0;i<6;i++) _current_target_deg[i] = m.target_deg[i];

  sendOk();
}
