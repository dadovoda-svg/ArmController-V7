#include <Arduino.h>

static constexpr uint32_t UART_BAUD = 115200;
static constexpr size_t   BUF_SZ    = 256;

void setup() {
  // USB CDC
  Serial.begin(115200); // baud ignorato su USB CDC, ma ok per compatibilità :contentReference[oaicite:1]{index=1}
  // Se vuoi che scriva anche quando il terminale non ha alzato DTR:
  Serial.ignoreFlowControl(true); // :contentReference[oaicite:2]{index=2}

  // UART0 su Serial1 (default pins tipicamente GP0=TX, GP1=RX; li forziamo esplicitamente)
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.setFIFOSize(1024);   // aumenta resilienza a burst :contentReference[oaicite:3]{index=3}
  Serial1.begin(UART_BAUD);
}

void loop() {
  static uint8_t buf[BUF_SZ];

  int avail = Serial1.available();
  if (avail <= 0) return;

  size_t toRead = (avail > (int)BUF_SZ) ? BUF_SZ : (size_t)avail;
  size_t n = Serial1.readBytes(reinterpret_cast<char*>(buf), toRead);

  if (n > 0) {
    Serial.write(buf, n);
    // Serial.flush(); // in genere NON necessario (USB buffered)
  }
}
