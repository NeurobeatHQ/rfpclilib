#include <rfpclilib.h>

// MD is connected to Pin A1 (GPIO17) of Feather
#define GPIO_BOOT 17
// Reset is connected to Pin A0 (GPIO18) of Feather
#define GPIO_RST  18

// Using P109_TXD9 (A4 GPIO14) for RX and P110_RXD9 (A3 GPIO15) for TX
#define UART_RX_PIN 14
#define UART_TX_PIN 15

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  pinMode(GPIO_BOOT, OUTPUT);
  pinMode(GPIO_RST, OUTPUT);

  RfpCliConfig cfg;
  cfg.targetSerial = &Serial1;
  cfg.logSerial = &Serial;
  cfg.bootPin = GPIO_BOOT;
  cfg.resetPin = GPIO_RST;
  cfg.hexPath = "/dfu_wifi.hex";

  if (programArduinoBootloader(cfg)) {
    Serial.println("Bootloader programmed successfully");
  } else {
    Serial.println("Bootloader programming failed");
  }
}

void loop() {
}
