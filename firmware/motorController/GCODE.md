# Arm GCode Protocol (AGC1) — Specifica v1 (Joint-Space + Planner Queue)

Protocollo testuale riga-per-riga per controllo di un braccio a 6 giunti in **joint-space**.
Questa v1 include: parsing G-code, coda mosse (planner), sincronizzazione con `M400`, enable/disable, homing, report.

> Nota: il canale di output può contenere righe extra (log/rumore) che iniziano con `## ` e che l’host deve ignorare.

---

## 0) Glossario
- **Host**: PC/UI che invia comandi (seriale/USB-seriale).
- **Device/Firmware**: braccio che riceve comandi e risponde.
- **Move**: una richiesta di movimento verso target joint-space.
- **Queue/Planner**: coda FIFO di Move + sequenziamento esecuzione.
- **Settled**: condizione “arrivato e stabilizzato” di un giunto (via `isSettled(joint)`).

---

## 1) Trasporto e framing

### 1.1 Terminazione riga
- Ogni comando è una riga terminata da `\n` (LF).
- `\r\n` (CRLF) è accettato: il firmware ignora `\r`.

### 1.2 Lunghezza massima
- Lunghezza massima riga consigliata: **128 caratteri** (esclusi CR/LF).
- Se la riga supera il limite: `error:line_too_long`.

### 1.3 Spazi e maiuscole/minuscole
- Token separati da 1+ spazi.
- Comandi e nomi parametri case-insensitive (`g0` = `G0`, `j1` = `J1`).

### 1.4 Commenti
- `;` commenta fino a fine riga (stile RepRap). Il firmware ignora la parte commentata.

Esempio:
- `G1 J1=10 ; commento`

---

## 2) Regola Out-of-Band: righe `## ` oppure `@ ` (solo output)

Il canale di output può includere righe non appartenenti al protocollo che iniziano con:

- prefisso ESATTO: `## ` (due `#` + spazio)
- prefisso ESATTO: `@ ` (un `@` + spazio)

### 2.1 Requisito lato host
Quando l’host attende la risposta a un comando deve:
- ignorare tutte le righe che iniziano con `## ` oppure con `@ ` 
- considerare terminata la risposta quando riceve:
  - `ok` oppure
  - `error:<code> ...`

Le righe dati (es. `J:...`) possono apparire prima di `ok`.

---

## 3) Sintassi dei comandi

### 3.1 Forma generale
```
<command> [<param> ...] [; comment]
```

- `<command>`: `G<number>` o `M<number>`
- `<param>`: token `NAME=VALUE` senza spazi attorno a `=`

### 3.2 Parametri supportati (v1)
- `J1..J6` (float): target giunti in **gradi**
- `V` (float): velocità in **gradi/secondo** (deg/s)

Esempi:
- `G0 J1=10.0 J2=-20.5 V=60`
- `G1 J3=45`

### 3.3 Regole parametri movimento
- `G0/G1` richiedono **almeno un** parametro `Jk`.
- I giunti non specificati mantengono il target corrente (movimento parziale).
- Se `V` è assente:
  - `G0` usa `V_default_rapid`
  - `G1` usa `V_default`

Parametri sconosciuti: `error:bad_param <name>`.

---

## 4) Risposte del device

### 4.1 Risposte standard
- Successo: `ok`
- Errore: `error:<code> [detail]`

### 4.2 Risposte con dati (prima di `ok`)
Alcuni comandi emettono una o più righe dati **prima** di `ok`.

Esempio `M114`:
```
J:10.000,-20.000,30.000,0.000,0.000,0.000
ok
```

L’host deve ignorare eventuali righe `## ...` in mezzo.

### 4.3 Formato numerico output
Per `M114` (e output analoghi):
- **3 decimali fissi**
- separatore decimale `.` (punto)
- separatore tra giunti `,` (virgola)
- nessuno spazio

Esempio:
- `J:10.000,-20.500,30.000,0.000,0.000,0.000`

> (Opzionale consigliato) Normalizzazione `-0.000` → `0.000`.

### 4.4 Codici errore (set minimo v1)
- `unknown_command`
- `line_too_long`
- `bad_param <name>`
- `missing_param <name>`
- `missing_joint_param`
- `motors_disabled`
- `not_homed`
- `busy`
- `internal`
- `estop`

---

## 5) Comandi supportati

### 5.1 `G0` — Joint move (rapid)
Accoda un Move “rapido” verso i target indicati.

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
Come `G0` ma con profilo “normale” e `V_default` se `V` non presente.

---

### 5.3 `G28` — Homing (tutti i giunti)
**Sintassi**
- `G28`

**Policy v1 (sicura)**
- Se planner occupato (move in esecuzione o queue non vuota): `error:busy`
- Se motori disabilitati: `error:motors_disabled`
- Risponde `ok` solo a homing completato

---

### 5.4 `M17` — Enable motors
**Sintassi**
- `M17`

**Risposta**
- `ok`

---

### 5.5 `M18` — Disable motors
**Sintassi**
- `M18`

**Effetto v1 (sicuro)**
- ferma subito l’esecuzione corrente
- svuota la queue
- disabilita motori

**Risposta**
- `ok`

---

### 5.6 `M400` — Wait until queue empty + settled
**Sintassi**
- `M400`

**Comportamento**
- blocca finché:
  - la queue è vuota, e
  - non c’è un move in esecuzione, e
  - i giunti del move corrente risultano **settled** (se applicabile)

**Risposta**
- `ok` quando completato
- `error:internal ...` solo in caso di errore grave (opzionale)

---

### 5.7 `M114` — Report joint state (posizione)
**Sintassi**
- `M114`

**Output**
- `J:<j1>,<j2>,<j3>,<j4>,<j5>,<j6>`
- `ok`

I valori devono essere in gradi, 3 decimali fissi.

---

### 5.8 `M115` — Firmware info
**Sintassi**
- `M115`

**Output minimo consigliato**
- `FIRMWARE_NAME:ArmFW PROTOCOL:AGC1 AXES:6 UNITS:deg,deg_s`
- `ok`

---

### 5.9 `M112` — Emergency stop
**Sintassi**
- `M112`

**Effetto**
- stop immediato
- svuota queue
- disabilita motori
- entra in stato E-STOP (fault latched)

**Risposta**
- `error:estop` (consigliato)

Dopo `M112`, tutti i comandi di movimento e homing devono rispondere `error:estop` finché reset.

---

## 6) Semantica Planner / Queue (v1)

### 6.1 Accodamento e capacità
- `G0/G1` accodano un Move.
- Capacità: `QUEUE_MAX` (es. 16/32).
- Se la queue è piena: `error:busy` (non bloccare).

### 6.2 Significato di `ok` su `G0/G1`
- `ok` = comando validato e Move **accodato**.
- Non implica completamento movimento.
- Per attendere completamento usare `M400`.

### 6.3 Avvio del Move
Il planner avvia un Move quando:
- non c’è un Move corrente in esecuzione, e
- la queue non è vuota.

All’avvio del Move il firmware deve:
1) determinare gli spostamenti per giunto `Δi`
2) impostare i limiti per-giunto usando:
   - `setLimits(v_i_max, a_i_max)`
   - `setSCurveTime(t_jerk)`
3) impostare i target finali con **una sola** `setTarget(all)`.

### 6.4 Coordinamento (arrivo simultaneo) tramite scaling
Definisci:
- `Dmax = max_i |Δi|`
- `ri = |Δi| / Dmax` (0..1)

Dato `V` del comando (deg/s) interpretato come:
- velocità max del giunto con `|Δ| = Dmax`

Il firmware imposta per ciascun giunto i (se `|Δi|` > soglia minima):
- `v_i_max = V_base * ri`
- `a_i_max = A_base * ri`
- `t_jerk_i = Tj_base` (uguale per tutti)
- quindi `j_i_max = a_i_max / t_jerk_i` (derivato internamente)

Questo consente ai giunti di eseguire profili S-curve simili e arrivare insieme.

### 6.5 Condizione “Move completato”
Un Move è completato quando tutti i giunti coinvolti risultano **settled**:
- via `isSettled(joint)`.

### 6.6 Homing `G28`
- `G28` non viene accodato.
- Se planner è occupato (queue non vuota o move in esecuzione): `error:busy`.
- `ok` solo quando homing completato e stato homed aggiornato.

### 6.7 Enable/Disable ed E-stop
- Dopo `M18`:
  - motori disabilitati
  - queue svuotata
  - movimenti rifiutati con `error:motors_disabled` finché `M17`
- Dopo `M112`:
  - movimenti e homing rifiutati con `error:estop` finché reset.

---

## 7) Nessun range check nel parser
Il parser/dispatcher non effettua controlli di range sui valori `Jk` e `V`.
I valori vengono accettati se la sintassi è valida e lo stato lo consente.
Eventuali limiti meccanici/sicurezza sono gestiti a livello di controllo/safety (fuori scope AGC1 v1).

---

## 8) Esempio sessione

Host → Device:
```
M115
M17
G28
G0 J1=0 J2=0 J3=0 J4=0 J5=0 J6=0 V=60
G1 J2=-20.0 J3=30.0 V=30
M400
M114
```

Device → Host (con righe extra):
```
FIRMWARE_NAME:ArmFW PROTOCOL:AGC1 AXES:6 UNITS:deg,deg_s
ok
ok
## homing started
## homing done
ok
ok
ok
ok
J:0.000,-20.000,30.000,0.000,0.000,0.000
ok
```

---

## 9) Comportamento host consigliato (minimo)
- Invia una riga comando.
- Legge righe finché:
  - ignora `## ...` e `@ ...`
  - se `ok` → successo
  - se `error:` → errore
  - altrimenti → considera la riga “dati” e continua fino a `ok/error`.
