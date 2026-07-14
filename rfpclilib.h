#ifndef RFPCLI_H
#define RFPCLI_H

#include <Arduino.h>

struct RfpCliConfig {
  HardwareSerial* targetSerial = nullptr;  // UART connected to the RA target (e.g. &Serial1)
  Stream* logSerial = nullptr;             // Stream for status/debug output (e.g. &Serial)
  uint8_t bootPin = 0;                     // GPIO wired to MD/BOOT
  uint8_t resetPin = 0;                    // GPIO wired to RES
  const char* hexPath = "/dfu_wifi.hex";   // FatFS path to the Intel HEX image
};

bool programArduinoBootloader(const RfpCliConfig& cfg);

#endif  // RFPCLI_H
