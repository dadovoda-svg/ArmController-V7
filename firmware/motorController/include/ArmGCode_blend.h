#pragma once
#include <Arduino.h>

// =======================
// Arm GCode Protocol AGC1
// Full parser + queue planner integration (ESP32 Arduino / PlatformIO)
// =======================

enum class MoveType : uint8_t { RapidG0, LinearG1 };

struct Move {
  MoveType type;
  float target_deg[6];   // absolute targets J1..J6 (deg)
  float v_deg_s;         // feedrate-like speed (deg/s)
  uint8_t changed_mask;  // bit0=J1 ... bit5=J6
};

struct ArmGCodeConfig {
  uint16_t line_max = 128;
  uint8_t queue_max = 16;

  // Defaults if V= is omitted
  float v_default = 30.0f;        // G1
  float v_default_rapid = 60.0f;  // G0
};

struct PlannerProfile {
  float a_max_deg_s2 = 120.0f;  // base accel for the joint with max delta
  float t_jerk_s     = 0.050f;  // S-curve time (sec): j_max = a_max / t_jerk
};

struct PlannerConfig {
  uint8_t queue_max = 16;

  // PlannerProfile prof_G1{120.0f, 0.050f};
  // PlannerProfile prof_G0{240.0f, 0.030f};
  PlannerProfile prof_G1{36.0f, 0.050f};
  PlannerProfile prof_G0{72.0f, 0.030f};

  float min_delta_deg = 0.001f; // joints below this delta are treated as not moving

  // tiny minima to avoid zero limits (not mechanical range checks)
  float min_v_deg_s   = 0.05f;
  float min_a_deg_s2  = 0.10f;
};

// Hooks: adapt these to your existing controller
struct ArmGCodeHooks {
  // Motors / homing state
  bool (*get_motors_enabled)() = nullptr;
  void (*set_motors_enabled)(bool en) = nullptr;

  bool (*get_is_homed)() = nullptr;
  bool (*do_homing_all)() = nullptr;

  // E-stop (optional, but recommended)
  void (*estop_now)() = nullptr;

  // Target API (you already have)
  void (*set_target_all)(const float joints_deg[6]) = nullptr;
  void (*set_target_one)(uint8_t joint, float pos_deg) = nullptr; // optional alternative
  float (*get_target_one)(uint8_t joint) = nullptr;

  // Optional: reference state (preferred for smooth segment blending)
  float (*get_ref_pos_one)(uint8_t joint) = nullptr;
  float (*get_ref_vel_one)(uint8_t joint) = nullptr;


  // Per-joint limits for coordinated moves
  void (*set_limits_one)(uint8_t joint, float v_max_deg_s, float a_max_deg_s2) = nullptr;
  void (*set_scurve_time_one)(uint8_t joint, float t_jerk_s) = nullptr;

  // Completion check (you will provide this)
  bool (*is_settled_one)(uint8_t joint) = nullptr;

  // Optional: real joint position readback for M114; if missing, M114 reports target
  bool (*get_joint_position_deg)(float pos_deg[6]) = nullptr;
};

class PlannerCoordinator {
public:
  PlannerCoordinator() = default;

  void begin(const PlannerConfig& cfg, const ArmGCodeHooks& hooks);

  bool enqueue(const Move& m);  // returns false if queue full
  void tick();                  // call frequently from loop()

  bool isExecuting() const { return _executing; }
  bool isIdle() const { return (!_executing && _qCount == 0); }

  void stopImmediate();         // stops current move + clears queue (M18)
  void estop();                 // stops + clears (M112)

private:
  PlannerConfig _cfg{};
  ArmGCodeHooks _hooks{};

  Move* _q = nullptr;
  uint8_t _qHead = 0, _qTail = 0, _qCount = 0;

  bool _executing = false;
  Move _cur{};
  uint8_t _moving_mask = 0;

  // Cached per-joint limits from last coordinated setup (used for blending heuristics)
  float _vmax_deg_s[6]  = {0,0,0,0,0,0};
  float _amax_deg_s2[6] = {0,0,0,0,0,0};

  // Blending control
  bool  _blend_enabled = true;
  float _blend_k_brake = 1.6f;     // how early we switch vs estimated braking distance
  float _blend_min_rem = 0.25f;    // deg: minimum remaining window to allow switch


  bool queuePush(const Move& m);
  bool queuePop(Move& out);
  bool queuePeek(Move& out) const;

  void startMove(const Move& m);
  bool moveSettled() const;
  bool shouldBlendToNext(const Move& next) const;
};

class ArmGCode {
public:
  ArmGCode(const ArmGCodeConfig& cfg = ArmGCodeConfig(),
           const PlannerConfig& pcfg = PlannerConfig());

  void begin(Stream& io, const ArmGCodeHooks& hooks);

  // Process incoming lines (call often)
  void poll();

  // Advance planner (call often, after poll())
  //void tickPlanner() { _planner.tick(); }
  void tickPlanner() {
    // 1) avanza planner/queue come fai già oggi
    _planner.tick();

    // 2) gestisci eventuale risposta differita
    if (_pending == PendingReply::M400) {
      if (_planner.isIdle()) {
        _pending = PendingReply::None;
        sendOk();
      }
    }
  }

private:
  Stream* _io = nullptr;
  ArmGCodeHooks _hooks{};
  ArmGCodeConfig _cfg{};
  PlannerConfig _pcfg{};
  PlannerCoordinator _planner;

  // Line buffer
  char _lineBuf[256];
  uint16_t _lineLen = 0;

  // Internal state
  bool _estop = false;
  float _current_target_deg[6] = {0,0,0,0,0,0}; // for partial moves

  // Parsing utilities
  bool readLine();
  void handleLine(char* line);

  static void stripComment(char* s);
  static void trimInPlace(char* s);
  static bool parseFloatStrict(const char* s, float& out);

  // Deferred replies (non-blocking M400)
  enum class PendingReply : uint8_t { None, M400 };
  PendingReply _pending = PendingReply::None;

  struct Tokens {
    const char* t[24];
    uint8_t n = 0;
  };
  static Tokens splitTokens(char* s);

  struct Cmd {
    char type = 0; // 'G' or 'M'
    int code = -1;
    bool valid() const { return (type=='G' || type=='M') && code >= 0; }
    bool isG(int c) const { return type=='G' && code==c; }
    bool isM(int c) const { return type=='M' && code==c; }
    bool isMovement() const { return type=='G' && (code==0 || code==1); }
    bool isHoming() const { return type=='G' && code==28; }
    bool isEnableMotors() const { return type=='M' && code==17; }
  };
  static Cmd parseCmd(const char* s);

  struct Params {
    bool hasJ[6] = {false,false,false,false,false,false};
    float J[6]   = {0,0,0,0,0,0};
    bool hasV = false;
    float V = 0;
  };
  bool parseParams(const Tokens& tk, uint8_t start, Params& p);

  // Dispatch
  void dispatch(const Cmd& cmd, const Params& p);

  // Output
  void sendOk();
  void sendErr(const char* code, const char* detail = nullptr);

  // Handlers
  void handle_M115();
  void handle_M114();
  void handle_M17();
  void handle_M18();
  void handle_M400();
  // void handle_M400_blocking();
  void handle_M112();

  void handle_G28();
  void handle_G0_G1(const Cmd& cmd, const Params& p);
};
