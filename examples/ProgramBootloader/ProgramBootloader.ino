// ProgramBootloader — flash the RA4M1 (Arduino UNO R4 WiFi) bootloader from an
// ESP32 host over UART, using rfpclilib. Arduino-ESP32 example.
//
// The library is framework-neutral: it talks to the target through a
// SerialInterface you implement, logs through a callback, and reads the Intel
// HEX from a filesystem path. Here we back those with Serial1 and LittleFS.

#include <rfpclilib.h>
#include <LittleFS.h>

// MD is connected to Pin A1 (GPIO17), RESET to Pin A0 (GPIO18) of the Feather.
#define GPIO_BOOT 17
#define GPIO_RST  18
// UART9: P109_TXD9 (A4/GPIO14) for RX, P110_RXD9 (A3/GPIO15) for TX.
#define UART_RX_PIN 14
#define UART_TX_PIN 15

// Adapt Serial1 to the library's SerialInterface.
class Serial1Link : public SerialInterface {
public:
  void write(uint8_t b) override { Serial1.write(b); }
  bool read(uint8_t *b) override {
    uint32_t start = millis();
    while (!Serial1.available()) {
      if (millis() - start > 200) return false;  // timeout, like uart_read_bytes
    }
    *b = (uint8_t)Serial1.read();
    return true;
  }
  void setBaud(uint32_t baud) override { Serial1.updateBaudRate(baud); }
  void delayMs(uint32_t ms) override { delay(ms); }
};

static void rfpErr(const char *msg)   { Serial.print("[ERR] "); Serial.println(msg); }
static void rfpDebug(const char *msg) { Serial.print("[dbg] "); Serial.println(msg); }

// Pulse RESET with MD held at `boot` (LOW = enter boot mode, HIGH = run).
static void pulseReset(int boot) {
  digitalWrite(GPIO_BOOT, boot);
  digitalWrite(GPIO_RST, HIGH); delay(100);
  digitalWrite(GPIO_RST, LOW);  delay(100);
  digitalWrite(GPIO_RST, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pinMode(GPIO_BOOT, OUTPUT);
  pinMode(GPIO_RST, OUTPUT);
  LittleFS.begin(true);  // mounts at /littlefs

  pulseReset(LOW);       // strap the target into boot mode (MD low)
  Serial1Link ra;
  bool ok = programArduinoBootloader("/littlefs/dfu_wifi.hex", ra, rfpErr, rfpDebug);
  pulseReset(HIGH);      // reset to run the freshly-written image (MD high)

  Serial.println(ok ? "Bootloader programmed successfully"
                    : "Bootloader programming failed");
}

void loop() {}
