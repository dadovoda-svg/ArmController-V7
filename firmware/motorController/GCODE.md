# Arm GCode Protocol (AGC1) — Specifica v1

Protocollo testuale riga-per-riga per controllo braccio in **joint-space** (6 giunti).
Obiettivi v1: move giunti, enable/disable, homing, report posizione, sync fine coda.
Il canale di output può contenere righe non-protocollo prefissate da `## ` che l'host deve ignorare.

## 0) Glossario rapido
- **Host**: PC/UI che invia comandi (seriale/USB-seriale)
- **Device/Firmware**: braccio che riceve comandi e risponde
- **Queue/Planner**: coda di movimenti accodati ed eseguiti dal firmware

---

## 1) Trasporto e framing

### 1.1 Terminazione riga
- Ogni comando è una riga terminata da `\n` (LF).
- `\r\n` (CRLF) è accettato: il firmware ignora `\r`.

### 1.2 Lunghezza massima
- Max line length (consigliato): **128 char** (esclusi CR/LF).
- Se superato: rispondere `error:line_too_long`.

### 1.3 Spazi e case
- Token separati da 1+ spazi.
- Comandi/parametri case-insensitive (es. `g0` = `G0`, `j1` = `J1`).

### 1.4 Commenti
- `;` commenta fino a fine riga. Il firmware ignora la parte commentata.

---

## 2) Regola Out-of-Band: righe `## `
Il canale di output può contenere righe extra (log/diagnostica) che iniziano con:

- prefisso ESATTO: `## `

### 2.1 Requisito lato host
Durante l’attesa della risposta a un comando, l’host deve:
- ignorare ogni riga che inizi con `## `
- considerare **terminata** la risposta quando riceve:
  - `ok` oppure
  - `error:<code> ...`

> Nota: anche le risposte con dati terminano sempre con `ok` (vedi §4.2).

---

## 3) Sintassi dei comandi (v1)

### 3.1 Forma generale
<command> [<param> ...] [; comment]

- `<command>`: `G<number>` o `M<number>`
- `<param>`: sempre in forma `NAME=VALUE` (senza spazi attorno a `=`)

### 3.2 Parametri supportati
- `J1..J6` (float): target giunti in **gradi**
- `V` (float): velocità in **gradi/secondo** (deg/s)

Esempi:
- `G0 J1=10.0 J2=-20.5 V=60`
- `G1 J3=45 J4=0.0`

### 3.3 Regole parametri movimento
- `G0/G1` richiede **almeno un** parametro `Jk`.
- I giunti non specificati mantengono il target corrente (movimento parziale).
- Se `V` assente:
  - `G0` usa `V_default_rapid`
  - `G1` usa `V_default`

Parametri sconosciuti: `error:bad_param <name>`

---

## 4) Risposte del device

### 4.1 Risposte standard
- Successo: `ok`
- Errore: `error:<code> [detail]`

### 4.2 Risposte con dati
Alcuni comandi producono 1+ righe dati **prima** della riga finale `ok`.

Esempio `M114`:
J:10.000,-20.000,30.000,0.000,0.000,0.000
ok

L’host deve ignorare eventuali righe `## ...` in mezzo.

### 4.3 Codici errore (set minimo v1)
- `unknown_command`
- `line_too_long`
- `bad_param <name>`
- `missing_param <name>`
- `missing_joint_param`
- `motors_disabled`
- `not_homed`
- `busy`
- `internal`
- `estop` (se M112 implementato)

---

## 5) Comandi (v1)

### 5.1 `G0` — Joint move (rapid)
**Sintassi**
- `G0 Jk=<deg> ... [V=<deg_s>]`

**Risposte**
- `ok`
- `error:missing_joint_param`
- `error:motors_disabled`
- `error:not_homed`
- `error:busy`
- `error:bad_param <name>`

---

### 5.2 `G1` — Joint move (normal)
Come `G0` ma usa `V_default` se `V` assente.

---

### 5.3 `G28` — Homing (all joints)
**Sintassi**
- `G28`

**Policy v1 (sicura)**
- Se planner occupato (queue non vuota o in esecuzione): `error:busy`
- Richiede motori abilitati: se no `error:motors_disabled`
- Risponde `ok` quando homing completato

---

### 5.4 `M17` — Enable motors
**Sintassi** `M17`  
**Risposta** `ok`

---

### 5.5 `M18` — Disable motors
**Sintassi** `M18`  
**Effetto consigliato v1**
- stop planner
- svuota coda movimenti
- disabilita motori
**Risposta** `ok`

---

### 5.6 `M400` — Wait until queue empty
**Sintassi** `M400`  
**Comportamento**
- attende fine di tutti i movimenti (queue empty + planner idle)
**Risposta** `ok` a completamento

---

### 5.7 `M114` — Report joint position
**Sintassi** `M114`  
**Output**
- `J:<j1>,<j2>,<j3>,<j4>,<j5>,<j6>`
- `ok`
Formato numerico consigliato: 3 decimali, `.`

---

### 5.8 `M115` — Firmware info
**Sintassi** `M115`  
**Output minimo consigliato**
- `FIRMWARE_NAME:ArmFW PROTOCOL:AGC1 AXES:6 UNITS:deg,deg_s`
- `ok`

---

### 5.9 `M112` — Emergency stop (consigliato)
**Sintassi** `M112`  
**Effetto**
- stop immediato
- svuota coda
- disabilita motori
- entra in stato FAULT (estop)
**Risposta** opzionale (`error:estop` o `ok`), poi rifiuta movimenti finché reset

---

## 6) Semantica di coda (planner) — regole v1

### 6.1 Queue
- capacità: `QUEUE_MAX` (es. 16 o 32)
- `G0/G1` accodano un "Move"
- se coda piena: `error:busy` (non bloccare l'input)

### 6.2 Significato di `ok` su G0/G1
- `ok` = comando validato e **accodato** (o avviato).
- Non implica che il movimento sia completato.
- Per attendere completamento usare `M400`.

### 6.3 Homing `G28`
- non accoda
- se planner occupato: `error:busy`
- esegue e risponde `ok` solo a fine homing

### 6.4 Policy homed
- Se `is_homed` è falso:
  - `G0/G1` -> `error:not_homed`
- (eccezioni/override rimandati a v1.1)

### 6.5 Enable/disable
- `M18` svuota coda e ferma esecuzione (sicurezza).
- Dopo `M18`, `G0/G1/G28` -> `error:motors_disabled` (finché `M17`).

### 6.6 E-stop
- Dopo `M112`, movimenti rifiutati con `error:estop` finché reset.

---

## 7) Esempio sessione

Host -> Device:
M115
M17
G28
G0 J1=0 J2=0 J3=0 J4=0 J5=0 J6=0 V=60
M400
M114

Device -> Host (con log extra):
FIRMWARE_NAME:ArmFW PROTOCOL:AGC1 AXES:6 UNITS:deg,deg_s
ok
ok

homing started
homing done

ok
ok
ok
J:0.000,0.000,0.000,0.000,0.000,0.000
ok
