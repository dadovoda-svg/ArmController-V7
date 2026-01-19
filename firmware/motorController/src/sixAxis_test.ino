#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "MultiStepper6.h"
#include "AS5600Mux.h"
#include "ConfigStore.h"
#include "ConfigConsole.h"
#include "sixAxisController.h"

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

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
bool encoderEnabled[6] = {false, false, true, false, false, false};

AS5600Mux encoders(Wire, TCA_ADDRESS, AS5600_ADDR);

sixAxisController joints(steppers, encoders);   // multicontroller PID per i motori dei giunti
const uint32_t  PID_PERIOD_uS = 100;            //  500us   200Hz
                                                //  200us   500Hz
                                                //  100us  1000Hz

const uint32_t  TICK_PERIOD_uS = 20000;  

// Definizione parametri
ConfigParam configParams[] = {
    { "spr0",   3200.0f,   0.0f },    //step per revolution - microstepping 16x (un valore negativo inverte la direzione)
    { "kp0",       2.0f,   0.0f },    //pid Propotional
    { "ki0",       0.1f,   0.0f },    //pid Integral
    { "kd0",       0.0f,   0.0f },    //pid Differential
    { "sm0",     360.0f,   0.0f },    //velocità massima in gradi al secondo
    { "am0",     720.0f,   0.0f },    //accelerazione massima in gradi al secondo quadrato
    { "jm0",      0.10f,   0.0f },    //t_jerk (S-curve time): 0,10 s → jerk = a_max / t_jerk = 3000 deg/s³
    { "ff0",      1.00f,   0.0f },    //feed-forward multiplier for profile velocity (usually 1.0)
    { "pt0",      0.10f,   0.0f },    //position tolerance [unit]
    { "vt0",      0.50f,   0.0f },    //velocity tolerance [unit]
    { "il0",     200.0f,   0.0f },    //integrator limit
    { "lmin0",   -90.0f,   0.0f },    //position limit min
    { "lmax0",    90.0f,   0.0f },    //position limit max
    { "p0",         0.0f,  0.0f },    //imposta la posizione del motore unico /solo per scopi di test
    { "zoff0",      0.0f,  0.0f },    //offset dello zero per gli encoder

    { "spr1",   3200.0f,   0.0f },
    { "kp1",       1.0f,   0.0f },
    { "ki1",       0.0f,   0.0f },
    { "kd1",       0.0f,   0.0f },
    { "sm1",     120.0f,   0.0f },
    { "am1",     300.0f,   0.0f },
    { "jm1",      0.10f,   0.0f },    
    { "ff1",      1.00f,   0.0f },    
    { "pt1",      0.10f,   0.0f },    
    { "vt1",      0.50f,   0.0f },    
    { "il1",     200.0f,   0.0f },    
    { "lmin1",   -90.0f,   0.0f },
    { "lmax1",    90.0f,   0.0f },
    { "p1",         0.0f,  0.0f },
    { "zoff1",      0.0f,  0.0f },    

    { "spr2",   3200.0f,   0.0f },    
    { "kp2",       1.0f,   0.0f },
    { "ki2",       0.0f,   0.0f },
    { "kd2",       0.0f,   0.0f },
    { "sm2",     120.0f,   0.0f },
    { "am2",     300.0f,   0.0f },
    { "jm2",      0.10f,   0.0f },    
    { "ff2",      1.00f,   0.0f },    
    { "pt2",      0.10f,   0.0f },    
    { "vt2",      0.50f,   0.0f },    
    { "il2",     200.0f,   0.0f },    
    { "lmin2",   -90.0f,   0.0f },
    { "lmax2",    90.0f,   0.0f },
    { "p2",         0.0f,  0.0f },
    { "zoff2",      0.0f,  0.0f },    

    { "spr3",   3200.0f,   0.0f },
    { "kp3",       1.0f,   0.0f },
    { "ki3",       0.0f,   0.0f },
    { "kd3",       0.0f,   0.0f },
    { "sm3",     120.0f,   0.0f },
    { "am3",     300.0f,   0.0f },
    { "jm3",      0.10f,   0.0f },    
    { "ff3",      1.00f,   0.0f },    
    { "pt3",      0.10f,   0.0f },    
    { "vt3",      0.50f,   0.0f },    
    { "il3",     200.0f,   0.0f },    
    { "lmin3",   -90.0f,   0.0f },
    { "lmax3",    90.0f,   0.0f },
    { "p3",         0.0f,  0.0f },
    { "zoff3",      0.0f,  0.0f },    

    { "spr4",  -3200.0f,   0.0f },    
    { "kp4",       1.0f,   0.0f },
    { "ki4",       0.0f,   0.0f },
    { "kd4",       0.0f,   0.0f },
    { "sm4",     120.0f,   0.0f },
    { "am4",     300.0f,   0.0f },
    { "jm4",      0.10f,   0.0f },    
    { "ff4",      1.00f,   0.0f },    
    { "pt4",      0.10f,   0.0f },    
    { "vt4",      0.50f,   0.0f },    
    { "il4",     200.0f,   0.0f },    
    { "lmin4",   -90.0f,   0.0f },
    { "lmax4",    90.0f,   0.0f },
    { "p4",         0.0f,  0.0f },
    { "zoff4",      0.0f,  0.0f },    

    { "spr5",   3200.0f,   0.0f },    
    { "kp5",       1.0f,   0.0f },
    { "ki5",       0.0f,   0.0f },
    { "kd5",       0.0f,   0.0f },
    { "sm5",     120.0f,   0.0f },
    { "am5",     300.0f,   0.0f },
    { "jm5",      0.10f,   0.0f },    
    { "ff5",      1.00f,   0.0f },    
    { "pt5",      0.10f,   0.0f },    
    { "vt5",      0.50f,   0.0f },    
    { "il5",     200.0f,   0.0f },    
    { "lmin5",   -90.0f,   0.0f },
    { "lmax5",    90.0f,   0.0f },
    { "p5",         0.0f,  0.0f },
    { "zoff5",      0.0f,  0.0f },    

    { "ch",         0.0f,  0.0f },    //canale visualizzato/esportato
  };

constexpr uint32_t CONFIG_VERSION = 4;

ConfigStore   config("myapp_cfg",
                     configParams,
                     sizeof(configParams) / sizeof(configParams[0]),
                     CONFIG_VERSION);

ConfigConsole cfgConsole(config, Serial);

// Flag che segnala che la config è stata modificata
volatile bool configChanged = false;
int  currentEncoderChannel  = 0;     // canale encoder visualizzato (0..5)
bool traceEnabled = false;
bool displayEnabled = false;
uint8_t demoType = 0;

// Callback chiamata quando un parametro cambia
void onConfigChanged(void* userData, const char* key, float oldVal, float newVal)
{
  // userData è opzionale; qui lo ignoriamo
  (void)userData;

  Serial.print("[CONFIG] ");uint8_t ch = (uint8_t)currentEncoderChannel;
  Serial.print(key);
  Serial.print(" cambiato da ");
  Serial.print(oldVal, 6);
  Serial.print(" a ");
  Serial.println(newVal, 6);

  // Imposta un flag globale, che il loop() potrà usare
  configChanged = true;


  if (strcmp(key, "ch") == 0) //i canali vanno da 1 a 6
  {
    if (newVal == 0) {    //se ch = 0 disabilito anche il display
      traceEnabled = false;
      displayEnabled = false;
    }
    else if (newVal < 0) {  //se minore di zero disabilito il trace e abilito il display
      traceEnabled = false;      
      displayEnabled = true;
      currentEncoderChannel = ((int) (-newVal)) - 1;
    }
    else {  //se maggiore di zero abilito il trave
      traceEnabled = true;
      currentEncoderChannel = ((int) newVal) - 1;
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

  // --- I2C ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000UL);    // 400 kHz

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
    Serial1.println ("target, meas");
  }

  //inizializza i parametri e fissa la posizione attuale come setpoint
  //in modo che i motori non si muovano
  joints.loadParams(config, resetJoints);

  float newVal = config.get("ch");
  if (newVal == 0) {    //se ch = 0 disabilito anche il display
    traceEnabled = false;
    displayEnabled = false;
  }
  else if (newVal < 0) {  //se minore di zero disabilito il trace e abilito il display
    traceEnabled = false;
    displayEnabled = true;
    currentEncoderChannel = ((int) (-newVal)) - 1;
  }
  else {  //se maggiore di zero abilito il trave
    traceEnabled = true;
    currentEncoderChannel = ((int) newVal) - 1;
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

  //solo per test velocità
  steppers.setStepsPerRevolution(3, 3200);
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

  handleButtons();

  // la generazione degli step avviene nel callback dell'esp_timer su core 0

  steppers.setSpeedDegPerSec (3, 90.0);

  uint8_t ch = (uint8_t)currentEncoderChannel;
  // va richiamato con cadenza regolare
  // if (timeNow >= nextPID) {
  //   nextPID = timeNow + (uint64_t)PID_PERIOD_uS;
  //   joints.update();
  // }
  // if (timeNow >= nextTick) {
  //   nextTick = timeNow + (uint64_t)PID_PERIOD_uS;

  //   if (motorsRunning) {
  //     switch (demoType) {
  //       case 0:
  //         joints.setTarget (ch, 110.0 * sin( millis() * 0.0001 ));
  //         break;
  //       case 1:
  //         joints.setTarget (ch, 110.0 * sin( millis() * 0.00025 ));
  //         break;
  //       case 2:
  //         joints.setTarget (ch, 110.0 * sin( millis() * 0.0005 ));
  //         break;
  //       case 3:
  //         joints.setTarget (ch, -100.0 + 10.0 * int((millis()%60000)/3000.0));
  //         break;
  //     }
  //   }
    
  //   if (displayEnabled) {
  //     updateDisplay();
  //   }
  //   if (traceEnabled) {
  //     //Serial1.print ("T:");
  //     Serial1.print (joints.getTarget(ch));
  //     Serial1.print (",");
  //     //Serial1.print (" M:");
  //     Serial1.print (joints.getLastMeas(ch));
  //     Serial1.print (",");
  //     //Serial1.print (" C:");
  //     Serial1.println (joints.lastCmd(ch));
  //   }
  // }
  
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
        //currentEncoderChannel++;
        if (currentEncoderChannel >= AS5600Mux::NUM_ENCODERS) {
          currentEncoderChannel = 0;
        }
        Serial.print("Canale encoder selezionato: ");
        Serial.println(currentEncoderChannel);
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

  uint8_t ch = (uint8_t)currentEncoderChannel;

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
  display.println(ch);

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
    display.print(joints.lastCmd(0),3);
  } else {
    display.print("ERR");
  }

  // STATUS
  display.setCursor(0, 40);
  display.print("Mspd : ");
  if (okStat) {
    display.print(steppers.getSpeed(0),3);
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

/* ******************************************
# configurazione j6 - stepper 0:
spr0=3200.000000        ingranaggio diretto
kp0=30.000000
ki0=5.000000
kd0=0.000000
sm0=720.000000
am0=2500.000000
jm0=0.100000
ff0=1.000000
pt0=2.000000
vt0=1.000000
il0=200.000000
lmin0=-90.000000
lmax0=171.000000
zoff0=180.000000
#configurazione j5 - stepper 1
spr0=-7085.71420        riduzione 28:62
kp0=30.000000
ki0=1.000000
kd0=0.000000
sm0=720.000000
am0=2500.000000
jm0=0.100000
ff0=1.000000
pt0=2.000000
vt0=1.000000
il0=200.000000
lmin0=-112.000000
lmax0=112.000000
zoff0=183.000000
#configurazione j4 - stepper 2 -      riduzione 32:32
spr2=-3200.000000  
kp2=20.000000
ki2=1.000000
kd2=0.000000
sm2=360.000000
am2=950.000000
jm2=0.100000
ff2=1.000000
pt2=2.000000
vt2=1.000000
il2=200.000000
lmin2=-130.000000
lmax2=130.000000
p2=5.000000
zoff2=188.000000

******************************************/