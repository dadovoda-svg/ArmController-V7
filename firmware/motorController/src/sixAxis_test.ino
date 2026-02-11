#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "MultiStepper6.h"
#include "AS5600Mux.h"
#include "ConfigStore.h"
#include "ConfigConsole.h"
#include "sixAxisController.h"
// #include "ArmGCode.h"
#include "ArmGCode_blend.h"

// =========================
// Configurazione i2c
// =========================
#define I2C_SDA   21
#define I2C_SCL   22

// =========================
// Configurazione OLED
// =========================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1        // nessun pin di reset dedicato
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 400000UL, 400000UL);

// =========================
// Pin motori (adatta ai tuoi)
// =========================
const uint8_t STEP_PINS[MultiStepper6::NUM_MOTORS] = { 4, 13, 16, 18, 26, 32 };
const uint8_t DIR_PINS[MultiStepper6::NUM_MOTORS]  = { 5, 14, 15, 19, 27, 33 };

MultiStepper6 steppers(STEP_PINS, DIR_PINS, 3.0f, 100); // pulseWidth=3µs, timer=100µs

// =========================
// Encoder AS5600 via TCA9548A
// =========================
#define TCA_ADDRESS   0x70
#define AS5600_ADDR   0x36
// se definito forza la lettura di un solo encoder. MODIFICARE ANCHE NEGLI ALTRI SORGENTI
bool encoderEnabled[6] = {true, true, true, true, true, true};

AS5600Mux encoders(Wire, TCA_ADDRESS, AS5600_ADDR);

sixAxisController joints(steppers, encoders);   // multicontroller PID per i motori dei giunti
const uint32_t  PID_PERIOD_uS = 5000;           //  5000us  200Hz
                                                //  3333us  300Hz  
                                                //  2500us  400Hz  <-- velocità massima reale?
                                                //  2000us  500Hz


const uint32_t  TICK_PERIOD_uS = 20000;  

// Definizione parametri
ConfigParam configParams[] = {        //corrente 0.67A
    { "spr0",  6400.00f,   0.0f },    //J1 - 200 s/giro, trasmissione 1:1, microstepping 1/32, polarità pos. -->  6400.00
    { "kp0",      10.0f,   0.0f },    //pid Propotional
    { "ki0",       0.9f,   0.0f },    //pid Integral
    { "kd0",       0.0f,   0.0f },    //pid Differential
    { "sm0",      90.0f,   0.0f },    //velocità massima in gradi al secondo
    { "am0",     180.0f,   0.0f },    //accelerazione massima in gradi al secondo quadrato
    { "jm0",      0.10f,   0.0f },    //t_jerk (S-curve time): 0,10 s → jerk = a_max / t_jerk = 3000 deg/s³
    { "ff0",      1.00f,   0.0f },    //feed-forward multiplier for profile velocity (usually 1.0)
    { "pt0",      1.80f,   0.0f },    //position tolerance [unit]
    { "vt0",      2.00f,   0.0f },    //velocity tolerance [unit]
    { "il0",     200.0f,   0.0f },    //integrator limit
    { "lmin0",  -175.0f,   0.0f },    //position limit min
    { "lmax0",   175.0f,   0.0f },    //position limit max
    { "p0",        0.0f,   0.0f },    //imposta la posizione del motore unico /solo per scopi di test
    { "zoff0",  181.50f,   0.0f },    //offset dello zero per gli encoder

    { "spr1", -14171.43f,   0.0f },    //J2 - 200 s/giro, trasmissione 28:62, microstepping 1/32, polarità neg. -->  -14171.43
    { "kp1",       15.0f,   0.0f },    //corrente 0.67A
    { "ki1",        0.0f,   0.0f },
    { "kd1",        0.0f,   0.0f },
    { "sm1",      90.0f,   0.0f },
    { "am1",      180.0f,   0.0f },
    { "jm1",       0.10f,   0.0f },    
    { "ff1",       1.00f,   0.0f },    
    { "pt1",       1.80f,   0.0f },    
    { "vt1",       2.00f,   0.0f },    
    { "il1",      200.0f,   0.0f },    
    { "lmin1",    -110.0f,   0.0f },
    { "lmax1",     110.0f,   0.0f },
    { "p1",         0.0f,   0.0f },
    { "zoff1",      184.8f,   0.0f },    

    { "spr2", -12800.00f,   0.0f },    //J3 - 200 s/giro, trasmissione 19:38, microstepping 1/32, polarità neg. -->  -12800.00    
    { "kp2",        15.0f,   0.0f },    //corrente 1.0A

    { "ki2",        1.0f,   0.0f },
    { "kd2",        0.0f,   0.0f },
    { "sm2",      90.0f,   0.0f },
    { "am2",      180.0f,   0.0f },
    { "jm2",       0.10f,   0.0f },    
    { "ff2",       1.00f,   0.0f },    
    { "pt2",       1.80f,   0.0f },    
    { "vt2",       2.00f,   0.0f },    
    { "il2",      200.0f,   0.0f },    
    { "lmin2",    -130.0f,   0.0f },
    { "lmax2",     130.0f,   0.0f },
    { "p2",         0.0f,   0.0f },
    { "zoff2",      188.50f,   0.0f },    

    { "spr3", -20800.00f,   0.0f },    //J4 - 200 s/giro, trasmissione 1:13, microstepping 1/8, polarità neg. -->  -10400.00  
    { "kp3",        3.0f,   0.0f },    //corrente 1.5A
    { "ki3",        0.5f,   0.0f },
    { "kd3",        0.0f,   0.0f },
    { "sm3",      90.0f,   0.0f },
    { "am3",      180.0f,   0.0f },
    { "jm3",       0.10f,   0.0f },    
    { "ff3",       1.00f,   0.0f },    
    { "pt3",       1.80f,   0.0f },    
    { "vt3",       2.00f,   0.0f },    
    { "il3",      200.0f,   0.0f },    
    { "lmin3",    -92.0f,   0.0f },
    { "lmax3",    100.0f,   0.0f },
    { "p3",         0.0f,   0.0f },
    { "zoff3",      180.80f,   0.0f },    

    { "spr4",  33600.00f,   0.0f },    //J5 - 200 s/giro, trasmissione 1:21, microstepping 1/8, polarità os. -->  33600.00 
    { "kp4",        3.0f,   0.0f },    //corrente 2.0A
    { "ki4",        1.0f,   0.0f },
    { "kd4",        0.0f,   0.0f },
    { "sm4",      40.0f,   0.0f },
    { "am4",      40.0f,   0.0f },
    { "jm4",       0.10f,   0.0f },    
    { "ff4",       1.00f,   0.0f },    
    { "pt4",       1.80f,   0.0f },    
    { "vt4",       2.00f,   0.0f },    
    { "il4",      200.0f,   0.0f },    
    { "lmin4",    -91.0f,   0.0f },
    { "lmax4",     91.0f,   0.0f },
    { "p4",         0.0f,   0.0f },
    { "zoff4",      178.0f,   0.0f },    

    { "spr5", -20266.66f,   0.0f },    //J6 - 200 s/giro, trasmissione 24:76, microstepping 1/32, polarità neg. -->  -20266.67    
    { "kp5",        5.0f,   0.0f },    //corrente 1.3A
    { "ki5",        1.0f,   0.0f },
    { "kd5",        0.0f,   0.0f },
    { "sm5",      40.0f,   0.0f },
    { "am5",      40.0f,   0.0f },
    { "jm5",       0.10f,   0.0f },    
    { "ff5",       1.00f,   0.0f },    
    { "pt5",       1.8f,   0.0f },    
    { "vt5",       2.0f,   0.0f },    
    { "il5",      200.0f,   0.0f },    
    { "lmin5",    -91.0f,   0.0f },
    { "lmax5",     91.0f,   0.0f },
    { "p5",         0.0f,   0.0f },
    { "zoff5",      183.0f,   0.0f },    

    { "chd",         0.0f,   0.0f },    //canale visualizzato sul display
    { "cht",         0.0f,   0.0f },    //canale visualizzato esportato in trace
  };

constexpr uint32_t CONFIG_VERSION = 7;

ConfigStore   config("myapp_cfg",
                     configParams,
                     sizeof(configParams) / sizeof(configParams[0]),
                     CONFIG_VERSION);

ConfigConsole cfgConsole(config, Serial);

// Flag che segnala che la config è stata modificata
volatile bool configChanged = false;
int  currentEncoderChannelDisplay  = 0;     // canale encoder visualizzato (0..5)
int  currentEncoderChannelTrace  = 0;       // canale encoder tracciato (0..5)
bool traceEnabled = false;
bool displayEnabled = false;
uint8_t demoType = 0;

static bool g_motors_enabled = false;
static bool g_is_homed = false;
// --------------------
// Config globali già valorizzati
// --------------------
const ArmGCodeConfig gcfg = []{
  ArmGCodeConfig c;
  c.line_max = 128;
  c.queue_max = 16;
  c.v_default = 30.0f;
  c.v_default_rapid = 60.0f;
  return c;
}();

const PlannerConfig pcfg = []{
  PlannerConfig p;
  p.queue_max = 16;

  // PROFILI (quelli che vuoi davvero)
  p.prof_G1 = PlannerProfile{30.0f, 0.050f}; // a_max, t_jerk
  p.prof_G0 = PlannerProfile{60.0f, 0.030f};

  // soglia delta
  p.min_delta_deg = 0.001f;

  // minimi anti-zero
  p.min_v_deg_s   = 0.05f;
  p.min_a_deg_s2  = 0.10f;

  return p;
}();

ArmGCode gcode(gcfg, pcfg);

// --- Required by planner ---
static void control_loop (void) {
  //esegue le funzioni minime per mantenere attivo il controllo
}

static void set_target_all(const float joints_deg[6]) {
  joints.setTarget ((float*)joints_deg);
}

static float get_target_one(uint8_t joint) {
  return joints.getTarget (joint);
}

static float get_ref_pos_one(uint8_t joint) {
  return joints.refPos (joint);
}

static float get_ref_vel_one(uint8_t joint) {
  return joints.refVel (joint);
}

static void set_limits_one(uint8_t joint, float v_max, float a_max) {
  joints.setLimits (joint, v_max, a_max);      //vMax, aMax
}

static void set_scurve_time_one(uint8_t joint, float t_jerk_s) {
  joints.setSCurveTime(joint, t_jerk_s);
}

static bool is_settled_one(uint8_t joint) {
  return joints.isSettled(joint);
}

// --- Motors/homing wrappers ---  TODO: da completare l'implementazione
static bool get_motors_enabled() { return g_motors_enabled; }
static void set_motors_enabled(bool en) {
  g_motors_enabled = en;
  // arm.enableMotors(en);
}

static bool get_is_homed() { return g_is_homed; }

static bool do_homing_all() {
  // run your homing routine (blocking in this example)
  // if successful:
  g_is_homed = true;
  return true;
}

static void estop_now() {
  // immediately stop outputs / power stage
}

// Callback chiamata quando un parametro cambia
void onConfigChanged(void* userData, const char* key, float oldVal, float newVal)
{
  // userData è opzionale; qui lo ignoriamo
  (void)userData;
  
  Serial.print("[CONFIG] ");
  Serial.print(key);
  Serial.print(" cambiato da ");
  Serial.print(oldVal, 6);
  Serial.print(" a ");
  Serial.println(newVal, 6);

  // Imposta un flag globale, che il loop() potrà usare
  configChanged = true;

  if (strcmp(key, "chd") == 0) //i canali vanno da 1 a 6
  {
    if (newVal == 0) {    //se ch = 0 disabilito anche il display
      displayEnabled = false;
    }
    else {  //se maggiore di zero abilito il trave
      displayEnabled = true;
      currentEncoderChannelDisplay = ((int) newVal) - 1;
    }
  }
  else if (strcmp(key, "cht") == 0) //i canali vanno da 1 a 6
  {
    if (newVal == 0) {    //se ch = 0 disabilito anche il display
      traceEnabled = false;
    }
    else {  //se maggiore di zero abilito il trave
      traceEnabled = true;
      currentEncoderChannelTrace = ((int) newVal) - 1;
    }
  }
  else if (   strcmp(key, "p0") == 0
      || strcmp(key, "p1") == 0
      || strcmp(key, "p2") == 0
      || strcmp(key, "p3") == 0
      || strcmp(key, "p4") == 0
      || strcmp(key, "p5") == 0 ) {
    uint8_t j = key[1] - '0';
    joints.setTarget (j, newVal);
    Serial.print("Joint ");
    Serial.print(j);
    Serial.print(" target ");
    Serial.println(joints.getTarget(j));
  }
  else {  
    joints.reloadParams(config);
    char szName[] = "zoff0";
    for (uint8_t i = 0; i < AS5600Mux::NUM_ENCODERS; ++i) {
      szName[4] = '0' + i;
      encoders.setZeroOffsetDegrees (i, config.get(szName));
    }
    strcpy (szName, "spr0");
    for (uint8_t i=0; i < sixAxisController::NUM_JOINTS; ++i) {
      szName[3] = '0' + i;
      float stepxRev = config.get(szName);
      if (stepxRev > 0.0f)
      {
        steppers.setDirectionPolarity (i, false);
        steppers.setStepsPerRevolution (i, stepxRev);
      }
      else {
        steppers.setDirectionPolarity (i, true);
        steppers.setStepsPerRevolution (i, -stepxRev);
      }
    }
  }
}

// =========================
// Pulsanti
// =========================
const int PIN_BTN_START_STOP = 35;   // start/stop motori
const int PIN_BTN_CHANNEL    = 36;   // cicla canale encoder mostrato

bool motorsRunning          = false;

// Debounce BTN_START_STOP
bool     lastBtn1State       = HIGH;
bool     lastBtn1Stable      = HIGH;
uint32_t lastDebounce1Ms     = 0;

// Debounce BTN_CHANNEL
bool     lastBtn2State       = HIGH;
bool     lastBtn2Stable      = HIGH;
uint32_t lastDebounce2Ms     = 0;

const uint32_t DEBOUNCE_TIME_MS = 50;

// =========================
// Prototipi
// =========================
void initDisplay();
void showWelcomeScreen();
void updateDisplay();
void handleButtons();
void handleButtonStartStop(bool reading);
void handleButtonChannel(bool reading);
void startMotors();
void stopMotors();

void setup()
{
  char    szName[12];
  float   resetJoints[sixAxisController::NUM_JOINTS] = {0.0f};

  Serial.begin(115200);
  //UART1 su GPIO34 (RX) e GPIO23 (TX)
  Serial1.begin(115200, SERIAL_8N1, 34, 23);

  delay(1000);

  // --- Pulsanti ---
  pinMode(PIN_BTN_START_STOP, INPUT);   //vanno aggiunti i pullup esterni
  pinMode(PIN_BTN_CHANNEL,    INPUT);

  pinMode(2, OUTPUT);
  digitalWrite (2, false);
  pinMode(0, OUTPUT);
  digitalWrite (0, false);
  // --- I2C ---
  Wire.begin(I2C_SDA, I2C_SCL, 400000UL);
  
  // --- Display OLED ---
  initDisplay();
  showWelcomeScreen();

  delay(1500);

  if (!config.begin(false)) {
      Serial.println("Errore: impossibile aprire NVS");
      while (true) { delay(1000); }
  }

  // Registra la callback
  config.setChangeCallback(onConfigChanged, nullptr);

  cfgConsole.begin();

  // --- Encoder AS5600 via MUX ---
  encoders.begin(false, 400000UL); // false: Wire già inizializzata sopra
  // inizializza gli offset dai parametri
  strcpy (szName, "zoff0");
  for (uint8_t i = 0; i < AS5600Mux::NUM_ENCODERS; ++i) {
    szName[4] = '0' + i;
    encoders.setZeroOffsetDegrees (i, config.get(szName));
  }
  // Cattura lo zero per tutti i canali encoder (se collegati)
  for (uint8_t i = 0; i < AS5600Mux::NUM_ENCODERS; ++i) {
    if (encoderEnabled[i]) {
      if (encoders.readAngleZeroedDegreesSigned(i, resetJoints[i])) {
        Serial.print("Encoder ");
        Serial.print(i);
        Serial.println(" catturato");
        ;
      } else {
        Serial.print("Errore cattura encoder ");
        Serial.println(i);
      }
    }
    else {
        resetJoints[i] = 0.0f;
        Serial.print("Encoder ");
        Serial.print(i);
        Serial.println(" disabilitato");      
    }
  }

  //inizializza i parametri e fissa la posizione attuale come setpoint
  //in modo che i motori non si muovano
  joints.loadParams(config, resetJoints);

  float newVal = config.get("chd");
  if (newVal == 0) {    //se ch = 0 disabilito anche il display
    displayEnabled = false;
  }
  else {  //se maggiore di zero abilito il trave
    displayEnabled = true;
    currentEncoderChannelDisplay = ((int) newVal) - 1;
  }

  newVal = config.get("cht");
  if (newVal == 0) {    //se ch = 0 disabilito anche il display
    traceEnabled = false;
  }
  else {  //se maggiore di zero abilito il trave
    traceEnabled = true;
    currentEncoderChannelTrace = ((int) newVal) - 1;
  }

  // --- Motori ---
  steppers.begin();
  strcpy (szName, "spr0");
  for (uint8_t i=0; i < sixAxisController::NUM_JOINTS; ++i) {
    szName[3] = '0' + i;
    float stepxRev = config.get(szName);
    if (stepxRev > 0.0f)
    {
      steppers.setDirectionPolarity (i, false);
      steppers.setStepsPerRevolution (i, stepxRev);
    }
    else {
      steppers.setDirectionPolarity (i, true);
      steppers.setStepsPerRevolution (i, -stepxRev);
    }
  }
  stopMotors();  // inizialmente fermi



  ArmGCodeHooks hooks;
  hooks.get_motors_enabled = get_motors_enabled;
  hooks.set_motors_enabled = set_motors_enabled;
  hooks.get_is_homed = get_is_homed;
  hooks.do_homing_all = do_homing_all;
  hooks.estop_now = estop_now;

  hooks.set_target_all = set_target_all;
  hooks.get_target_one = get_target_one;
  hooks.get_ref_pos_one = get_ref_pos_one;
  hooks.get_ref_vel_one = get_ref_vel_one;

  hooks.set_limits_one = set_limits_one;
  hooks.set_scurve_time_one = set_scurve_time_one;

  hooks.is_settled_one = is_settled_one;

  // Optional: if you can read real position, hook it for M114
  // hooks.get_joint_position_deg = ...

  gcode.begin(Serial1, hooks);

  Serial1.println("## ArmFW AGC1 ready");
}

// =========================
// *****   MAIN LOOP   *****
// =========================
void loop()
{
  uint64_t        timeNow = esp_timer_get_time();  //miscrosecondi
  static uint64_t nextPID = 0;
  static uint64_t nextTick = 0;

  cfgConsole.update();
  // 1) parse and enqueue commands
  gcode.poll();

  handleButtons();

  // la generazione degli step avviene nel callback dell'esp_timer su core 0

  //steppers.setSpeedDegPerSec (3, 20.0);

  //uint8_t chd = (uint8_t)currentEncoderChannelDisplay;
  uint8_t cht = (uint8_t)currentEncoderChannelTrace;
  //va richiamato con cadenza regolare
  if (timeNow >= nextPID) {
    nextPID = timeNow + (uint64_t)PID_PERIOD_uS;
    gcode.tickPlanner();
    joints.update();
  }

  if (timeNow >= nextTick) {
    nextTick = timeNow + (uint64_t)TICK_PERIOD_uS;
    
    if (displayEnabled) {
      updateDisplay();
    }

    if (traceEnabled) {
      //Serial1.print ("## *T:");
      Serial1.print ("@ ");
      Serial1.print (joints.getTarget(cht));
      Serial1.print (",");
      //Serial1.print (" M:");
      Serial1.print (joints.getLastMeas(cht));
      Serial1.print (",");
      //Serial1.print (" C:");
      Serial1.print (joints.lastCmd(cht));
      Serial1.print (",");
      //Serial1.print (" V:");
      Serial1.println (joints.lastVel(cht));
    }
  }
}

// =========================
// Inizializzazione Display
// =========================
void initDisplay()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for (;;) {
      delay(1000);
    }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.display();
}

void showWelcomeScreen()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.println("Six-axis Stepper Test");
  display.println("ESP32 + AS5600 + OLED");
  display.println("");
  display.println("BTN36: start/stop");
  display.println("BTN39: encoder ch");
  display.display();
}

// =========================
// Gestione pulsanti
// =========================
void handleButtons()
{
  bool reading1 = digitalRead(PIN_BTN_START_STOP);
  bool reading2 = digitalRead(PIN_BTN_CHANNEL);

  handleButtonStartStop(reading1);
  handleButtonChannel(reading2);
}

void handleButtonStartStop(bool reading)
{
  if (reading != lastBtn1State) {
    lastDebounce1Ms = millis();
  }

  if ((millis() - lastDebounce1Ms) > DEBOUNCE_TIME_MS) {
    if (reading != lastBtn1Stable) {
      lastBtn1Stable = reading;

      // fronte di discesa: pulsante premuto (INPUT_PULLUP)
      if (lastBtn1Stable == LOW) {
        motorsRunning = !motorsRunning;
        if (motorsRunning) {
          ++demoType;
          if(demoType>3) demoType=0;
          startMotors();
          //abilito i motori
        } else {
          stopMotors();
          //disabilito i motori
        }
      }
    }
  }

  lastBtn1State = reading;
}

void handleButtonChannel(bool reading)
{
  if (reading != lastBtn2State) {
    lastDebounce2Ms = millis();
  }

  if ((millis() - lastDebounce2Ms) > DEBOUNCE_TIME_MS) {
    if (reading != lastBtn2Stable) {
      lastBtn2Stable = reading;

      // fronte di discesa: pulsante premuto (INPUT_PULLUP)
      if (lastBtn2Stable == LOW) {
      }
    }
  }

  lastBtn2State = reading;
}

// =========================
// Comandi motori
// =========================
void startMotors()
{
  Serial.println("Motori: RUN");
}

void stopMotors()
{
  Serial.println("Motori: STOP");
}

// =========================
// Aggiornamento Display
// =========================
void updateDisplay()
{
  static uint32_t lastUpdateMs = 0;
  const uint32_t UPDATE_PERIOD_MS = 200;

  bool okRaw, okAngle, okStat, okSigned;

  if (millis() - lastUpdateMs < UPDATE_PERIOD_MS) {
    return;
  }
  lastUpdateMs = millis();

  uint8_t ch = (uint8_t)currentEncoderChannelDisplay;

  // Lettura registri fondamentali AS5600 per canale corrente
  uint16_t rawAngle = 0;
  uint16_t angle    = 0;
  uint8_t  status   = 0;
  float    degSigned = 0.0f;
  
  if (encoderEnabled[ch]) {
    okRaw    = encoders.readRawAngle(ch, rawAngle);
    okAngle  = encoders.readAngle(ch, angle);
    okStat   = encoders.readRegisters(ch, 0x0B, &status, 1); // STATUS reg = 0x0B
    okSigned = encoders.readAngleZeroedDegreesSigned(ch, degSigned);
  }
  else {
    okRaw    = false;
    okAngle  = false;
    okStat   = false; // STATUS reg = 0x0B
    okSigned = false;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print("Six-axis Test  Ch:");
  display.println(ch+1);

  // Stato motori
  display.print("Motors: ");
  display.print(motorsRunning ? "RUN     " : "STOP    ");
  display.println(demoType);

  display.println("--------------------");

  // RAW_ANGLE
  display.print("RAW  : ");
  if (okRaw) {
    display.print(rawAngle);
  } else {
    display.print("ERR");
  }

  // ANGLE
  display.setCursor(0, 32);
  display.print("Rvel : ");
  if (okAngle) {
    display.print(joints.lastCmd(ch),3);
  } else {
    display.print("ERR");
  }

  // STATUS
  display.setCursor(0, 40);
  display.print("Mspd : ");
  if (okStat) {
    display.print(steppers.getSpeed(ch),3);
  } else {
    display.print("ERR");
  }

  // Angolo signed [-180, 180]
  display.setCursor(0, 48);
  display.print("Deg signed: ");
  if (okSigned) {
    display.print(degSigned, 2);
  } else {
    display.print("ERR");
  }

  display.display();
}

