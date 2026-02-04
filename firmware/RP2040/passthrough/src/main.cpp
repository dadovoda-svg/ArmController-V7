#include <Arduino.h>

static constexpr uint32_t USB_BAUD  = 115200;   // ignorato su USB CDC, ma ok
static constexpr uint32_t UART_BAUD = 115200;

static constexpr size_t BUF_SZ = 256;

static inline void pump(Stream& in, Stream& out) {
  uint8_t buf[BUF_SZ];

  int avail = in.available();
  while (avail > 0) {
    size_t n = (avail > (int)BUF_SZ) ? BUF_SZ : (size_t)avail;

    // Stream::readBytes() può bloccare se usi timeout; qui meglio read() in loop
    // ma su Arduino-Pico readBytes su stream seriale è ok se n <= available.
    size_t got = in.readBytes((char*)buf, n);
    if (got == 0) break;

    out.write(buf, got);
    avail = in.available();
  }
}

void setup() {
  // --- USB CDC ---
  Serial.begin(USB_BAUD);

  // Se vuoi che la USB CDC trasmetta anche quando il terminale sul PC non è "aperto":
  // (comodo per debug continuo; se preferisci comportamento "classico", commenta)
  Serial.ignoreFlowControl(true);

  // --- UART0 (Serial1) su GP0/GP1 ---
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.setFIFOSize(256);
  Serial1.begin(UART_BAUD);

  // (Opzionale) piccola scritta di avvio, SOLO se il PC è connesso
  if (Serial) Serial.println("RP2040 bidirectional passthrough ready");
}

void loop() {
  // UART0 -> USB CDC
  pump(Serial1, Serial);

  // USB CDC -> UART0
  pump(Serial, Serial1);

  // Piccolo yield per non monopolizzare la CPU in caso di traffico continuo
  // (su RP2040 non è strettamente necessario ma aiuta)
  delayMicroseconds(50);
}

/*
Note pratiche (importanti)

Se sul PC non vedi nulla finché non apri un terminale, è normale: USB CDC spesso dipende da DTR. 
Con Serial.ignoreFlowControl(true) forzi l’invio anche senza DTR.

Il bridge è “trasparente”: inoltra byte così come sono. Quindi funziona anche con protocolli 
binari, non solo testo.

Variante “più conservativa” con controllo DTR (solo per debug USB)

Se vuoi evitare di buttare dati verso l’host quando il terminale non è aperto, puoi fare così 
solo sul verso UART→USB:

if (Serial.dtr()) pump(Serial1, Serial);
pump(Serial, Serial1);


(La direzione host→node la lasci sempre attiva.)
*/