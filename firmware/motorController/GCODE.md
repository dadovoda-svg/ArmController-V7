# Arm GCode Protocol AGC1 — Command Reference

Documentazione del parser G-code e del planner queue/blending implementati in:

- `include/ArmGCode_blend.h`
- `src/ArmGCode_blend.cpp`
- configurazione applicativa in `src/sixAxis_test.ino`

Il protocollo controlla un braccio a 6 giunti in **joint-space**. Le unità usate dai comandi sono:

- posizione giunti: **gradi**
- velocità: **gradi/secondo**
- accelerazione planner: **gradi/secondo²**
- tempo jerk S-curve: **secondi**

---

## 1. Architettura generale

Il modulo è diviso in due parti principali:

1. **`ArmGCode`**
   - legge righe da uno `Stream` Arduino;
   - rimuove commenti;
   - tokenizza la riga;
   - interpreta comando e parametri;
   - invia risposte `ok` / `error:<code>`;
   - accoda movimenti nel planner.

2. **`PlannerCoordinator`**
   - mantiene una coda FIFO di movimenti;
   - avvia il movimento successivo quando il precedente è completato;
   - coordina i 6 giunti scalando velocità e accelerazione in base allo spostamento relativo;
   - opzionalmente effettua blending fra segmenti compatibili;
   - usa callback/hook verso il controller reale.

Il parser non pilota direttamente i motori. Tutto il collegamento con l'hardware passa attraverso `ArmGCodeHooks`.

---

## 2. Configurazione corrente

### 2.1 Configurazione parser

Nel file applicativo `sixAxis_test.ino` la configurazione corrente è:

```cpp
c.line_max = 128;
c.queue_max = 16;
c.v_default = 30.0f;
c.v_default_rapid = 60.0f;
```

Significato:

| Campo | Valore corrente | Descrizione |
|---|---:|---|
| `line_max` | `128` | lunghezza massima della riga comando, escluso terminatore |
| `queue_max` | `16` | numero massimo di movimenti accodabili |
| `v_default` | `30.0` | velocità default per `G1` se manca `V` |
| `v_default_rapid` | `60.0` | velocità default per `G0` se manca `V` |

### 2.2 Configurazione planner

Nel file applicativo `sixAxis_test.ino` la configurazione corrente del planner è:

```cpp
p.queue_max = 16;
p.prof_G1 = PlannerProfile{10.0f, 0.050f};
p.prof_G0 = PlannerProfile{20.0f, 0.030f};
p.min_delta_deg = 0.001f;
p.min_v_deg_s = 0.05f;
p.min_a_deg_s2 = 0.10f;
```

Significato:

| Campo | Valore corrente | Descrizione |
|---|---:|---|
| `prof_G1.a_max_deg_s2` | `10.0` | accelerazione base per `G1` |
| `prof_G1.t_jerk_s` | `0.050` | tempo jerk S-curve per `G1` |
| `prof_G0.a_max_deg_s2` | `20.0` | accelerazione base per `G0` |
| `prof_G0.t_jerk_s` | `0.030` | tempo jerk S-curve per `G0` |
| `min_delta_deg` | `0.001` | spostamenti inferiori sono considerati nulli |
| `min_v_deg_s` | `0.05` | limite minimo anti-zero per velocità |
| `min_a_deg_s2` | `0.10` | limite minimo anti-zero per accelerazione |

Nota: in `ArmGCode_blend.h` esistono default diversi nella struct, ma l'istanza effettivamente usata da `sixAxis_test.ino` sovrascrive questi valori.

---

## 3. Trasporto seriale e formato riga

### 3.1 Terminatori accettati

Il parser accetta come fine riga:

- `\n`
- `\r`
- `\r\n`
- `\n\r`

Le coppie CR/LF o LF/CR vengono assorbite senza generare una riga vuota aggiuntiva.

### 3.2 Forma generale

```text
<COMANDO> [PARAMETRO=VALORE ...] [; commento]
```

Esempi:

```text
M115
M17
G28
G0 J1=0 J2=15 V=60
G1 J2=-20.5 J3=30 V=25 ; movimento coordinato
M400
M114
```

### 3.3 Spazi e tab

I token sono separati da spazi o tab.

Valido:

```text
G1 J1=10 J2=-20 V=30
```

Non valido, perché il parser vuole `NAME=VALUE` nello stesso token:

```text
G1 J1 = 10
```

### 3.4 Commenti

Il carattere `;` apre un commento fino a fine riga.

```text
G1 J1=10 V=20 ; questo testo viene ignorato
```

Righe vuote o composte solo da commento vengono ignorate e non generano risposta.

### 3.5 Maiuscole/minuscole

Il tipo comando `G`/`M` e i nomi parametro supportati sono case-insensitive.

Esempi equivalenti:

```text
G1 J1=10 V=30
g1 j1=10 v=30
```

### 3.6 Comando

Il comando deve essere il primo token della riga.

Forma accettata:

```text
G<number>
M<number>
```

Il numero deve contenere solo cifre. Quindi `G1`, `G01`, `M400` sono accettati; `G1.0` non lo è.

Se il comando non è riconosciuto:

```text
error:unknown_command
```

---

## 4. Parametri supportati

Il parser supporta solo i seguenti parametri:

| Parametro | Tipo | Unità | Descrizione |
|---|---|---|---|
| `J1` | float | gradi | target assoluto giunto 1 |
| `J2` | float | gradi | target assoluto giunto 2 |
| `J3` | float | gradi | target assoluto giunto 3 |
| `J4` | float | gradi | target assoluto giunto 4 |
| `J5` | float | gradi | target assoluto giunto 5 |
| `J6` | float | gradi | target assoluto giunto 6 |
| `V` | float | gradi/s | velocità richiesta del movimento |

I valori devono essere numeri `float` validi e finiti. Valori non numerici o non finiti vengono rifiutati.

Esempi validi:

```text
G1 J1=10
G1 J1=-10.5 J2=0 V=30
G0 J6=90 V=60
```

Esempi non validi:

```text
G1 J7=10       ; parametro sconosciuto
G1 J1=         ; valore vuoto
G1 J1=abc      ; valore non numerico
G1 SPEED=30    ; parametro sconosciuto
```

Risposte tipiche:

```text
error:bad_param J7
error:bad_param empty
error:bad_param J1
error:bad_param SPEED
```

Nota importante: i parametri vengono parsati prima del dispatch del comando. Di conseguenza un comando come `M115 V=1` è sintatticamente valido, anche se `V` viene poi ignorato da `M115`. Un parametro sconosciuto invece produce comunque errore, anche su comandi che non usano parametri.

---

## 5. Risposte del firmware

### 5.1 Successo

```text
ok
```

### 5.2 Errore

```text
error:<code> [detail]
```

Esempi:

```text
error:unknown_command
error:bad_param J7
error:motors_disabled
error:not_homed
error:busy
```

### 5.3 Risposte con dati

Alcuni comandi producono una riga dati prima di `ok`.

Esempio `M114`:

```text
J:10.000,-20.000,30.000,0.000,0.000,0.000
ok
```

### 5.4 Righe diagnostiche fuori protocollo

Il firmware applicativo può produrre righe diagnostiche/log, per esempio su `Serial1`, con prefisso `## `. Un host robusto dovrebbe ignorare le righe diagnostiche note mentre attende `ok` o `error:`.

Nel documento precedente erano citati anche prefissi `@ `. Il parser G-code non li genera direttamente, ma l'host può comunque trattarli come righe fuori protocollo se usati da altri moduli di trace.

---

## 6. Comandi implementati

## 6.1 `G0` — movimento rapido joint-space

Accoda un movimento rapido verso uno o più target assoluti di giunto.

### Sintassi

```text
G0 Jk=<deg> [Jk=<deg> ...] [V=<deg_s>]
```

Esempi:

```text
G0 J1=0 J2=0 J3=0 J4=0 J5=0 J6=0 V=60
G0 J6=90
```

### Semantica

- richiede almeno un parametro `J1..J6`;
- i giunti non indicati mantengono il target corrente memorizzato dal parser;
- il target è assoluto, non incrementale;
- se `V` è assente usa `v_default_rapid`, attualmente `60.0 deg/s`;
- il movimento viene accodato nella FIFO del planner;
- `ok` significa: comando valido e movimento accodato;
- `ok` non significa movimento completato.

### Stato richiesto

Se i relativi hook sono presenti:

- i motori devono essere abilitati;
- il sistema deve essere in stato homed;
- il sistema non deve essere in E-stop.

### Risposte

Successo:

```text
ok
```

Errori possibili:

```text
error:missing_joint_param
error:motors_disabled
error:not_homed
error:estop
error:busy
error:bad_param <detail>
```

`error:busy` viene restituito se la coda è piena.

---

## 6.2 `G1` — movimento lineare/coordinato joint-space

Accoda un movimento normale verso uno o più target assoluti di giunto.

### Sintassi

```text
G1 Jk=<deg> [Jk=<deg> ...] [V=<deg_s>]
```

Esempi:

```text
G1 J1=10 J2=-20 V=30
G1 J3=45
G1 J2=-10 J3=30 J5=5 V=15
```

### Semantica

`G1` è analogo a `G0`, ma usa il profilo planner normale:

- accelerazione base corrente: `10.0 deg/s²`;
- tempo jerk corrente: `0.050 s`;
- velocità default se manca `V`: `30.0 deg/s`.

Anche per `G1`:

- i target sono assoluti;
- i giunti omessi mantengono il target corrente memorizzato;
- `ok` significa movimento accodato, non movimento concluso.

### Risposte

Successo:

```text
ok
```

Errori possibili:

```text
error:missing_joint_param
error:motors_disabled
error:not_homed
error:estop
error:busy
error:bad_param <detail>
```

---

## 6.3 `G28` — homing globale

Esegue la procedura di homing di tutti i giunti attraverso l'hook `do_homing_all()`.

### Sintassi

```text
G28
```

### Semantica

- non viene accodato nel planner;
- viene eseguito immediatamente dal dispatcher;
- è rifiutato se il planner non è idle;
- richiede motori abilitati, se l'hook `get_motors_enabled()` è presente;
- usa l'hook `do_homing_all()`;
- dopo homing aggiorna la cache interna dei target leggendo `get_target_one()`, se disponibile;
- se `get_target_one()` non è disponibile, imposta la cache interna a zero.

Nell'applicazione corrente il wrapper `do_homing_all()` implementa un soft-homing: legge le posizioni encoder reali e allinea il target corrente a tali posizioni, poi imposta `g_is_homed = true`.

### Risposte

Successo:

```text
ok
```

Errori possibili:

```text
error:busy
error:motors_disabled
error:estop
error:internal no_home_cb
error:internal homing_fail
```

---

## 6.4 `M17` — abilita motori

Abilita i motori attraverso l'hook `set_motors_enabled(true)`.

### Sintassi

```text
M17
```

### Semantica

- chiama `set_motors_enabled(true)` se l'hook è disponibile;
- nell'applicazione corrente, prima di abilitare i motori legge la posizione corrente e allinea il target per evitare scatti al ri-enable;
- se il sistema è in E-stop latched, `M17` viene rifiutato.

### Risposte

Successo:

```text
ok
```

Errore possibile:

```text
error:estop
```

---

## 6.5 `M18` — disabilita motori e stop immediato

Ferma il planner, svuota la coda e disabilita i motori.

### Sintassi

```text
M18
```

### Semantica

- annulla un eventuale `M400` in attesa;
- chiama `_planner.stopImmediate()`;
- azzera lo stato di esecuzione corrente del planner;
- svuota la queue;
- chiama `set_motors_enabled(false)` se l'hook è disponibile;
- nell'applicazione corrente ferma tutti gli stepper e disabilita l'enable dei motori.

`M18` è consentito anche durante un `M400` pendente.

### Risposte

Successo:

```text
ok
```

---

## 6.6 `M400` — wait non bloccante fino a planner idle

Attende il completamento di tutti i movimenti già accodati.

### Sintassi

```text
M400
```

### Semantica implementata

`M400` nella versione attuale è **non bloccante** rispetto al parser.

Comportamento:

1. se il planner è già idle, risponde subito:

   ```text
   ok
   ```

2. se il planner non è idle:
   - non invia subito risposta;
   - imposta uno stato interno `PendingReply::M400`;
   - la risposta `ok` verrà inviata successivamente da `tickPlanner()`, quando `_planner.isIdle()` diventa vero.

Durante un `M400` pendente:

- sono consentiti:
  - `M114`
  - `M115`
  - `M18`
  - `M112`
- un secondo `M400` risponde:

  ```text
  error:busy
  ```

- altri comandi che cambierebbero stato rispondono:

  ```text
  error:busy
  ```

### Condizione di completamento

Il planner è idle quando:

- non sta eseguendo un movimento;
- la coda è vuota.

Un movimento in esecuzione termina quando tutti i giunti coinvolti risultano `settled` tramite `is_settled_one(joint)`.

### Risposte

Successo immediato o differito:

```text
ok
```

Errore possibile:

```text
error:busy
```

---

## 6.7 `M114` — report posizione giunti

Restituisce la posizione dei sei giunti.

### Sintassi

```text
M114
```

### Semantica

La posizione viene ottenuta con questa priorità:

1. se disponibile, usa `get_joint_position_deg(pos)`;
2. altrimenti, se disponibile, usa `get_target_one(joint)`;
3. altrimenti usa la cache interna `_current_target_deg`.

Nell'applicazione corrente `get_joint_position_deg` legge gli encoder AS5600 attraverso il mux e restituisce posizioni in gradi.

### Output

```text
J:<j1>,<j2>,<j3>,<j4>,<j5>,<j6>
ok
```

Formato:

- 6 valori;
- ordine `J1..J6`;
- 3 decimali fissi;
- separatore `,`;
- nessuno spazio.

Esempio:

```text
J:10.000,-20.500,30.000,0.000,0.000,90.000
ok
```

### Errori possibili

```text
error:internal pos_fail
error:bad_param <detail>
```

`pos_fail` viene restituito se `get_joint_position_deg()` è presente ma fallisce.

---

## 6.8 `M115` — informazioni firmware/protocollo

Restituisce una riga identificativa del firmware e del protocollo.

### Sintassi

```text
M115
```

### Output attuale

```text
FIRMWARE_NAME:ArmFW PROTOCOL:AGC1 AXES:6 UNITS:deg,deg_s
ok
```

---

## 6.9 `M112` — Emergency Stop

Esegue uno stop di emergenza latched.

### Sintassi

```text
M112
```

### Semantica

- annulla un eventuale `M400` in attesa;
- chiama `estop_now()` se disponibile;
- chiama `_planner.estop()`;
- svuota la coda planner;
- disabilita i motori tramite `set_motors_enabled(false)`, se disponibile;
- imposta `_estop = true`.

Nell'applicazione corrente `estop_now()`:

- ferma tutti gli stepper;
- disabilita i motori;
- imposta `g_motors_enabled = false`;
- imposta `g_is_homed = false`;
- ferma lo stato `motorsRunning`.

### Risposta

Il comando risponde intenzionalmente con errore:

```text
error:estop
```

Dopo `M112`, i seguenti comandi vengono rifiutati con `error:estop`:

- `G0`
- `G1`
- `G28`
- `M17`

Non è implementato un comando G-code di reset dell'E-stop. Per uscire dallo stato latched serve un reset firmware o una gestione esterna non presente nel parser.

---

## 7. Planner queue

### 7.1 Accodamento

`G0` e `G1` creano una struct `Move`:

```cpp
struct Move {
  MoveType type;
  float target_deg[6];
  float v_deg_s;
  uint8_t changed_mask;
};
```

Dove:

- `type` è `RapidG0` oppure `LinearG1`;
- `target_deg[6]` contiene i target assoluti finali;
- `v_deg_s` contiene la velocità richiesta o default;
- `changed_mask` indica quali giunti erano esplicitamente presenti nel comando.

La coda è FIFO e ha capacità corrente `16`.

Se la coda è piena:

```text
error:busy
```

### 7.2 Significato della cache target

Il parser mantiene `_current_target_deg[6]` per supportare i movimenti parziali.

Esempio:

```text
G1 J1=10 J2=20
G1 J3=30
```

Il secondo comando mantiene internamente `J1=10` e `J2=20`, quindi il target completo del secondo movimento diventa:

```text
J1=10 J2=20 J3=30 J4=<precedente> J5=<precedente> J6=<precedente>
```

Nota importante: la cache viene aggiornata quando un movimento viene accodato, non quando il movimento viene completato. Quindi una sequenza di movimenti parziali usa come riferimento logico i target già accodati.

### 7.3 Avvio movimento

Quando il planner non sta eseguendo nulla e la coda contiene almeno un elemento, estrae il prossimo `Move` e chiama `startMove()`.

`startMove()`:

1. calcola il delta per ogni giunto;
2. calcola `Dmax = max(|delta_i|)`;
3. seleziona il profilo `G0` o `G1`;
4. scala velocità e accelerazione per ciascun giunto;
5. imposta i limiti su ciascun giunto coinvolto;
6. imposta il tempo S-curve;
7. chiama `set_target_all()` una sola volta con il target completo.

Se `Dmax < min_delta_deg`, il movimento è considerato nullo e viene completato immediatamente.

---

## 8. Coordinamento multi-giunto

Per ottenere movimenti coordinati in joint-space, il planner usa lo spostamento massimo come riferimento.

Per ciascun movimento:

```text
Dmax = max_i |delta_i|
r_i = |delta_i| / Dmax
```

Per ciascun giunto coinvolto:

```text
v_i = max(min_v_deg_s, Vbase * r_i)
a_i = max(min_a_deg_s2, Abase * r_i)
t_jerk_i = Tjbase
```

Dove:

- `Vbase` è il parametro `V` del comando, oppure il default `G0/G1`;
- `Abase` viene dal profilo planner (`prof_G0` o `prof_G1`);
- `Tjbase` viene dal profilo planner (`prof_G0` o `prof_G1`).

Il giunto con delta maggiore riceve la velocità richiesta completa; gli altri ricevono limiti proporzionalmente inferiori.

Esempio:

```text
G1 J1=100 J2=50 V=30
```

Se i delta effettivi sono:

```text
J1 delta = 100 deg
J2 delta = 50 deg
```

allora:

```text
Dmax = 100
r1 = 1.0
r2 = 0.5
v1 = 30 deg/s
v2 = 15 deg/s
```

---

## 9. Blending fra segmenti

Il planner include una logica di blending per evitare stop-and-go fra segmenti consecutivi.

### 9.1 Condizioni preliminari

Il blending viene valutato solo se sono disponibili gli hook:

- `get_ref_pos_one`
- `get_ref_vel_one`
- `set_limits_one`

Inoltre il blending viene considerato solo fra movimenti dello stesso tipo:

- `G1 -> G1`
- `G0 -> G0`

Non viene fatto blending fra:

- `G0 -> G1`
- `G1 -> G0`

### 9.2 Criterio di switch

Durante un movimento, se esiste un segmento successivo in coda, il planner valuta se il target corrente è abbastanza vicino da poter iniziare il segmento successivo.

Per ogni giunto coinvolto considera:

- posizione di riferimento corrente;
- target corrente;
- velocità di riferimento corrente;
- distanza residua dal target corrente;
- eventuale inversione di direzione nel segmento successivo.

Se il segmento successivo prosegue nella stessa direzione, usa una finestra dinamica basata sulla distanza di frenata:

```text
d_brake = vel² / (2 * a)
window = max(blend_min_rem, blend_k_brake * d_brake)
```

I valori interni correnti sono:

```cpp
_blend_k_brake = 1.6f;
_blend_min_rem = 0.25f; // deg
```

Se il segmento successivo inverte la direzione, usa solo la finestra minima `_blend_min_rem`.

Se almeno un giunto è fuori dalla propria finestra, il blending non viene eseguito in quel tick.

### 9.3 Effetto del blending

Quando il blending è accettato:

- il movimento successivo viene estratto dalla coda;
- `startMove()` viene chiamato immediatamente sul nuovo target;
- i limiti vengono ricalcolati usando la posizione di riferimento corrente;
- il controller viene retargettato senza attendere lo stato settled sul waypoint intermedio.

---

## 10. Stato homed, motori ed E-stop

### 10.1 Motori disabilitati

Se `get_motors_enabled()` è disponibile e restituisce `false`, i movimenti e l'homing vengono rifiutati:

```text
error:motors_disabled
```

Nella sequenza normale usare:

```text
M17
G28
```

prima di inviare `G0`/`G1`.

### 10.2 Stato homed

Se `get_is_homed()` è disponibile e restituisce `false`, `G0` e `G1` vengono rifiutati:

```text
error:not_homed
```

`G28` imposta lo stato homed tramite l'applicazione, non direttamente nel parser.

### 10.3 E-stop latched

Dopo `M112`, lo stato `_estop` resta vero.

Comandi bloccati in E-stop:

| Comando | Risposta |
|---|---|
| `G0` | `error:estop` |
| `G1` | `error:estop` |
| `G28` | `error:estop` |
| `M17` | `error:estop` |

Comandi ancora utili/ammessi:

| Comando | Note |
|---|---|
| `M114` | lettura stato/posizione |
| `M115` | info firmware |
| `M18` | ulteriore stop/disabilitazione |
| `M112` | ripete E-stop |

---

## 11. Tabella riassuntiva comandi

| Comando | Nome | Accodato? | Richiede motori ON | Richiede homed | Risposta successo |
|---|---|---:|---:|---:|---|
| `G0` | movimento rapido joint-space | sì | sì, se hook presente | sì, se hook presente | `ok` quando accodato |
| `G1` | movimento normale joint-space | sì | sì, se hook presente | sì, se hook presente | `ok` quando accodato |
| `G28` | homing globale | no | sì, se hook presente | no | `ok` a homing concluso |
| `M17` | abilita motori | no | no | no | `ok` |
| `M18` | stop e disabilita motori | no | no | no | `ok` |
| `M400` | wait planner idle | no | no | no | `ok` immediato o differito |
| `M114` | report posizione | no | no | no | dati + `ok` |
| `M115` | firmware info | no | no | no | info + `ok` |
| `M112` | emergency stop | no | no | no | `error:estop` |

---

## 12. Codici errore implementati

| Errore | Quando può avvenire |
|---|---|
| `error:unknown_command` | comando non valido o non supportato |
| `error:line_too_long` | riga lunga almeno `line_max` caratteri |
| `error:bad_param format` | parametro senza `=` o formato non valido |
| `error:bad_param name` | nome parametro troppo lungo |
| `error:bad_param empty` | valore parametro vuoto |
| `error:bad_param V` | valore `V` non parsabile come float valido |
| `error:bad_param Jn` | valore `Jn` non parsabile come float valido |
| `error:bad_param <name>` | parametro non supportato |
| `error:missing_joint_param` | `G0`/`G1` senza alcun `J1..J6` |
| `error:motors_disabled` | movimento/homing richiesto con motori disabilitati |
| `error:not_homed` | movimento richiesto prima di homing |
| `error:busy` | queue piena, planner occupato per `G28`, oppure comando non ammesso durante `M400` pendente |
| `error:internal no_home_cb` | `G28` richiesto ma manca hook homing |
| `error:internal homing_fail` | hook homing fallito |
| `error:internal pos_fail` | `M114` non riesce a leggere le posizioni |
| `error:estop` | E-stop attivo o appena richiesto |

Nota: `missing_param` era citato in versioni precedenti della documentazione, ma nel parser attuale non risulta generato direttamente.

---

## 13. Sequenze operative consigliate

### 13.1 Startup minimo

```text
M115
M17
G28
M114
```

Output tipico:

```text
FIRMWARE_NAME:ArmFW PROTOCOL:AGC1 AXES:6 UNITS:deg,deg_s
ok
ok
ok
J:0.000,0.000,0.000,0.000,0.000,0.000
ok
```

### 13.2 Movimento singolo e attesa completamento

```text
G1 J1=10 J2=-20 V=30
M400
M114
```

Interpretazione:

- `G1` risponde `ok` appena il movimento è accodato;
- `M400` risponde `ok` solo quando la coda è vuota e il movimento corrente è settled;
- `M114` legge la posizione finale.

### 13.3 Sequenza di movimenti accodati

```text
G1 J1=10 V=20
G1 J2=15 V=20
G1 J3=-30 V=20
M400
```

I tre movimenti vengono messi in coda. `M400` permette all'host di sincronizzarsi con la fine dell'intera sequenza.

### 13.4 Stop normale

```text
M18
```

Effetti:

- ferma il planner;
- svuota la coda;
- disabilita i motori.

### 13.5 Emergency stop

```text
M112
```

Output:

```text
error:estop
```

Dopo questo comando i movimenti sono bloccati fino a reset firmware o gestione esterna dello stato latched.

---

## 14. Comportamento host consigliato

Un host seriale dovrebbe:

1. inviare una riga terminata da `\n` o `\r\n`;
2. leggere righe fino a `ok` o `error:`;
3. trattare eventuali righe dati come payload del comando;
4. ignorare righe diagnostiche note, per esempio `## ...`;
5. non assumere che `ok` su `G0`/`G1` significhi movimento concluso;
6. usare `M400` per sincronizzarsi con la fine della queue;
7. durante un `M400` pendente inviare solo query (`M114`, `M115`) o stop/emergency (`M18`, `M112`).

Pseudo-logica host:

```text
send(command)
while true:
    line = read_line()
    if line starts with "## ": continue
    if line == "ok": success
    if line starts with "error:": failure
    else: collect_data_line(line)
```

---

## 15. Limiti e note importanti

### 15.1 Nessun range check meccanico nel parser

Il parser non verifica i limiti meccanici dei target `J1..J6` e non verifica che `V` sia positiva o entro un range sicuro.

Il valore viene accettato se:

- il parametro è sintatticamente valido;
- lo stato del sistema consente il comando;
- la coda planner accetta il movimento.

La sicurezza dei limiti fisici deve essere gestita nel controller/safety layer.

### 15.2 Velocità negative

Il parser accetta un `float` valido per `V`, anche negativo. Nel planner, la velocità base viene poi protetta con:

```cpp
Vbase = max(min_v_deg_s, requested_v)
```

Quindi un `V` negativo finisce per diventare almeno `min_v_deg_s`. È comunque consigliabile lato host inviare solo `V > 0`.

### 15.3 Unità joint-space, non cartesiane

`G0` e `G1` non rappresentano movimenti cartesiani TCP. Sono movimenti coordinati in spazio giunti.

Non sono implementati:

- assi cartesiani `X/Y/Z`;
- orientamenti `A/B/C`;
- feedrate standard `F`;
- interpolazione cartesiana;
- cinematica inversa;
- unità mm;
- modalità relative/assolute tipo `G90/G91`.

### 15.4 Target assoluti

Tutti i target `J1..J6` sono assoluti. Non esiste una modalità incrementale nel parser attuale.

### 15.5 Comandi non implementati

Qualsiasi comando non elencato in questa documentazione risponde:

```text
error:unknown_command
```

Esempi non implementati:

```text
G90
G91
G92
M999
M119
M500
M501
```

---

## 16. Reference rapida

```text
M115
    Report firmware/protocollo.

M17
    Abilita motori.

G28
    Homing globale.

G0 J1=<deg> ... [V=<deg_s>]
    Movimento rapido in joint-space.

G1 J1=<deg> ... [V=<deg_s>]
    Movimento normale/coordinato in joint-space.

M400
    Attende in modo non bloccante lato parser fino a planner idle.

M114
    Report posizione giunti.

M18
    Stop immediato, svuota queue, disabilita motori.

M112
    Emergency stop latched.
```
