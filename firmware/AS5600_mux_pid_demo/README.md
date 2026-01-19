# Stepper Servo PID Demo (ESP32 + AS5600 + OLED)

Questa è una **versione demo** di un *controller “servo” per motore stepper* con **feedback da encoder magnetico AS5600**.
Lo scopo è validare rapidamente la catena **motore + driver step/dir + encoder** e fare tuning di base dei parametri, con:

* **Controllo ibrido**: *profilo di moto* per movimenti ampi + **PID** per l’aggancio fine vicino al target
* **Generazione STEP** in task dedicato (ESP32 FreeRTOS, core 0)
* **UI su OLED** (stato, posizione, errore, velocità)
* **Console seriale** per impostare target e parametri a runtime

---

## Funzionalità principali

* Lettura posizione angolare dall’encoder **AS5600** (12 bit, 0–360°) tramite I²C
* Target di posizione in **gradi**
* Modalità di controllo:

  * **PROF** (motion profile) quando l’errore è maggiore di una soglia (`PID_THRESHOLD`)
  * **PID** quando l’errore è entro la soglia, per ottenere precisione e stabilità
* **Deadband**: sotto ~0.1° l’uscita viene azzerata per ridurre jitter/ronzio
* Comandi seriali per modificare:

  * `KP`, `KI`, `KD`
  * soglia di commutazione PID `PID_THRESHOLD`
  * limiti di velocità/accelerazione del profilo (`MAX_SPEED`, `ACCELERATION`, `MIN_SPEED`)

---

## Hardware previsto

### MCU / Board

* ESP32 (nel codice: **Heltec WiFi Kit 32**)

### Encoder

* **AS5600** su bus I²C
* Nel progetto è usata una classe `AS5600Mux` (file `AS5600Mux.h`) e un **I²C MUX** a indirizzo `0x70` (tipicamente TCA9548A o compatibile).

  * Se non usi il mux: dovrai adattare la parte di lettura (nel codice c’è anche una versione “diretta” commentata).

### Display

* OLED **SSD1306 128×64** su I²C (`0x3C`)

### Driver stepper

* Qualsiasi driver con segnali **STEP / DIR / EN**

---

## Pinout (come nel codice)

### I²C + OLED (Heltec)

| Funzione     | GPIO |
| ------------ | ---: |
| SDA          |    4 |
| SCL          |   15 |
| OLED RESET   |   16 |
| VEXT (power) |   21 |
| LED          |   25 |

> Nota Heltec: spesso `VEXT` va portato **LOW per accendere** l’alimentazione esterna. Il codice lo fa in `setup()`.

### Pulsanti

| Pulsante | GPIO | Note                       |
| -------- | ---: | -------------------------- |
| Upper    |   14 | `INPUT_PULLUP`, attivo LOW |
| Lower    |   12 | `INPUT_PULLUP`, attivo LOW |

### Stepper

| Segnale | GPIO |
| ------- | ---: |
| EN      |   23 |
| DIR     |   19 |
| STEP    |   22 |

> Importante: la polarità di `EN` varia tra driver. Nel codice l’abilitazione è gestita con `digitalWrite(ENABLE_PIN, motorEnabled ? HIGH : LOW);`. Verifica sul tuo driver se **HIGH=enable** o **LOW=enable** e modifica di conseguenza.

---

## Dipendenze software

* Arduino framework per ESP32 (core 3.x o compatibile)
* Librerie Arduino (Library Manager):

  * **Adafruit GFX Library**
  * **Adafruit SSD1306**
* File locale:

  * `AS5600Mux.h` (e relativi sorgenti, se separati)

---

## Parametri principali (da codice)

* `STEPS_PER_REV = 3200` (200 step * microstepping 1/16)
* `CONTROL_FREQ = 100 Hz` (loop di controllo ogni 10 ms)
* Profilo di moto:

  * `MAX_SPEED` (steps/s)
  * `ACCELERATION` (steps/s²)
  * `MIN_SPEED` (steps/s)
* PID:

  * `KP`, `KI`, `KD`
  * `PID_THRESHOLD` (gradi): oltre questa soglia usa PROF, sotto usa PID
* `angleOffset`: offset in gradi applicato alla lettura encoder (commissioning)

---

## Come usare (workflow consigliato)

1. **Collega** encoder AS5600 (e MUX se presente) + OLED + driver stepper.
2. Compila e carica lo sketch su ESP32.
3. Apri il Serial Monitor a **115200**.
4. Alla partenza:

   * viene letta la posizione iniziale (`currentPosition`)
   * il target viene impostato a quella posizione (`targetPosition = currentPosition`)
5. Imposta un target da seriale, ad esempio:

   * `T90` oppure `T-45.5`
6. Tuning:

   * aumenta `KP` finché il sistema risponde bene ma non oscilla
   * aggiungi `KD` se serve smorzare
   * aggiungi `KI` con cautela per eliminare errore statico (attenzione a windup)
   * regola `PID_THRESHOLD` per decidere quanto presto passare al PID
   * se i grandi spostamenti sono troppo aggressivi o lenti, lavora su `MAXV`/`ACCEL`

---

## Comandi seriali

* Imposta target:

  * `T<gradi>`
    Esempio: `T90.5`

* Stampa posizione:

  * `P`

* Parametri PID:

  * `KP:<val>`  (es. `KP:70`)
  * `KI:<val>`  (es. `KI:20`)
  * `KD:<val>`  (es. `KD:0.1`)
  * `PIDTH:<gradi>` (es. `PIDTH:5`)

* Profilo di moto:

  * `MAXV:<steps/s>` (es. `MAXV:3200`)
  * `ACCEL:<steps/s2>` (es. `ACCEL:9600`)
  * `MINV:<steps/s>` (es. `MINV:20`)

* Stato completo parametri:

  * `SHOW`

* Enable/Disable motore:

  * `EN`
  * `DIS`

---

## Controllo da pulsanti

* **Upper (GPIO14)**: cicla `displayMode` (attualmente usato anche per alcune demo di traiettoria in `loop()`).
* **Lower (GPIO12)**:

  * pressione breve: “homing” a `0°` (imposta `targetPosition = 0`)
  * pressione lunga: abilita/disabilita motore

---

## Cosa mostra l’OLED

* Titolo “SERVO”
* Stato:

  * `[OFF]` motore disabilitato
  * `[PID]` controllo PID attivo
  * `[PROF]` profilo di moto attivo
* Direzione (freccia) in base al segno dell’errore
* Posizione corrente → target (in gradi)
* Barra di “progresso” (indicativa, basata sull’errore)
* Errore assoluto e velocità (step frequency)

---

## Note tecniche (architettura)

* **Generazione step**: task `stepGenerationTask()` su **core 0**.

  * Legge `currentStepFrequency` e `currentDirection` protetti da `stepMutex`.
  * Genera impulsi STEP con `micros()` e `delayMicroseconds(5)` per la larghezza impulso.
* **Loop di controllo**: `controlLoop()` a 100 Hz su `loop()`.

  * legge encoder
  * calcola errore
  * sceglie `motionProfile()` o `pidControl()`
  * aggiorna la frequenza step tramite `setStepSpeed()`

---

## Troubleshooting rapido

* Motore non si muove:

  * verifica alimentazione driver e polarità `EN`
  * verifica cablaggio STEP/DIR
  * controlla che `motorEnabled` sia true (`EN` da seriale o long press)

* Oscillazioni / instabilità vicino al target:

  * riduci `KP`, aumenta leggermente `KD`
  * riduci `KI` o limita meglio l’integratore
  * aumenta deadband (nel codice è ~0.1°)

* Movimento “duro” o troppo aggressivo nei grandi spostamenti:

  * riduci `MAX_SPEED`
  * riduci `ACCELERATION`
  * aumenta `PID_THRESHOLD` per usare più a lungo il profilo prima del PID

* Angolo non centrato / zero errato:

  * regola `angleOffset` oppure implementa una routine di calibrazione iniziale

---

## Licenza

MIT
