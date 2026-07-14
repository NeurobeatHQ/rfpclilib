#include <Arduino.h>

#include "rfpclilib.h"
#include "hex_loader.h"


static bool readExpected(HardwareSerial& target, Stream& log,
                         const uint8_t* expected, int length) {
  uint8_t* buf = (uint8_t*)alloca(length);
  target.readBytes(buf, length);
  if (memcmp(buf, expected, length) != 0) {
    log.print("Inquiry response mismatch. Expected: 0x");
    for (int i = 0; i < length; ++i) log.print(expected[i], HEX);
    log.print(" Actual: 0x");
    for (int i = 0; i < length; ++i) log.print(buf[i], HEX);
    log.println();
    return false;
  }
  return true;
}

class CSum {
  unsigned sum;
public:
  CSum()
    : sum(0) {}

  CSum& operator+=(const char rhs) {
    sum += rhs;
    return *this;
  }

  unsigned char get() {
    return (unsigned char)((-sum) & 0xFF);
  }
};

bool programArduinoBootloader(const RfpCliConfig& cfg) {
  if (!cfg.targetSerial || !cfg.logSerial) return false;
  HardwareSerial& Serial1 = *cfg.targetSerial;
  Stream& Serial = *cfg.logSerial;
  const uint8_t GPIO_BOOT = cfg.bootPin;
  const uint8_t GPIO_RST = cfg.resetPin;

  // downloadDfuToFatFs();
  HexImage dfuImage;
  if (loadHexFromFatFs(cfg.hexPath, dfuImage)) {
    dumpHexImage(dfuImage);
  } else {
    Serial.println("[hex] parse failed");
  }

  digitalWrite(GPIO_BOOT, LOW);
  digitalWrite(GPIO_RST, HIGH);
  delay(100);
  digitalWrite(GPIO_RST, LOW);
  delay(100);
  digitalWrite(GPIO_RST, HIGH);

  int step = 1;
  int ib = -1;
  int initNotDone = 1;
  while (initNotDone) {
    Serial1.write(0x00);
    while (Serial1.available() > 0) {
      int ib = Serial1.read();
      Serial.print("After sending ");
      Serial.print(step);
      Serial.print(" I received: 0x");
      Serial.println(ib, HEX);
      initNotDone = 0;
      break;
    }
    ++step;
  }

  {
    Serial1.write(0x55);
    int initNotDone = 1;
    while (initNotDone) {
      while (Serial1.available() > 0) {
        int ib = Serial1.read();
        Serial.print("I received: 0x");
        Serial.println(ib, HEX);

        if (ib == 0xc3) {
          initNotDone = 0;
        }
      }
    }
  }

  Serial.println("Init Done");

  {
    Serial1.write(0x01);  // Start of packet
    Serial1.write(0x00);
    Serial1.write(0x01);  // 1 bytes long
    Serial1.write(0x00);  // Inquiry Command
    Serial1.write(0xFF);  // Checksum
    Serial1.write(0x03);  // End
    static const uint8_t expected[7] = { 0x81, 0x00, 0x02, 0x00, 0x00, 0xFE, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 7)) {
      return false;
    }
  }

  {
    Serial1.write(0x01);  // Start of packet
    Serial1.write(0x00);
    Serial1.write(0x01);  // 1 bytes long
    Serial1.write(0x3A);  // Signature request command
    Serial1.write(0xC5);  // Checksum
    Serial1.write(0x03);  // End

    // I received: 0x81000D3A 016E3600 0016E360 03 02 0200 B403
    // SCI op clock: 24MHz, Max UART Baud Rate 1,500,000bps
    static const uint8_t expected[18] = { 0x81, 0x00, 0x0D, 0x3A, 0x01, 0x6E, 0x36, 0x00, 0x00, 0x16, 0xE3, 0x60, 0x03, 0x02, 0x02, 0x00, 0xB4, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 18)) {
      return false;
    }
  }

  {
    Serial1.write(0x01);  // Start of packet
    Serial1.write(0x00);
    Serial1.write(0x02);  // 2 bytes long
    Serial1.write(0x3B);  // Area information request command
    Serial1.write(0x00);  // Area 0
    Serial1.write(0xC3);  // Checksum
    Serial1.write(0x03);  // End

    // I received: 0x8100123B00000000000003FFFF0000080000000008A203
    static const uint8_t expected[23] = { 0x81, 0x00, 0x12, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x08, 0xA2, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 23)) {
      return false;
    }
  }

  {
    Serial1.write(0x01);  // Start of packet
    Serial1.write(0x00);
    Serial1.write(0x02);  // 2 bytes long
    Serial1.write(0x3B);  // Area information request command
    Serial1.write(0x01);  // Area 1
    Serial1.write(0xC2);  // Checksum
    Serial1.write(0x03);  // End

    // I received: 0x8100123B014010000040101FFF0000040000000001EF03
    static const uint8_t expected[23] = { 0x81, 0x00, 0x12, 0x3B, 0x01, 0x40, 0x10, 0x00, 0x00, 0x40, 0x10, 0x1F, 0xFF, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0xEF, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 23)) {
      return false;
    }
  }

  {
    Serial1.write(0x01);  // Start of packet
    Serial1.write(0x00);
    Serial1.write(0x02);  // 2 bytes long
    Serial1.write(0x3B);  // Area information request command
    Serial1.write(0x02);  // Area 2
    Serial1.write(0xC1);  // Checksum
    Serial1.write(0x03);  // End

    // I received: 0x8100123B02010100080101003300000000000000046E03
    static const uint8_t expected[23] = { 0x81, 0x00, 0x12, 0x3B, 0x02, 0x01, 0x01, 0x00, 0x08, 0x01, 0x01, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6E, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 23)) {
      return false;
    }
  }

  {
    Serial1.write(0x01);  // Start of packet
    Serial1.write(0x00);
    Serial1.write(0x05);  // 5 bytes long
    Serial1.write(0x34);  // Baud Rate Setting Command
    Serial1.write(0x00);
    Serial1.write(0x01);
    Serial1.write(0xC2);
    Serial1.write(0x00);  // 0x1C200 => 115200 baud
    Serial1.write(0x04);  // Checksum
    Serial1.write(0x03);  // End

    // I received: 0x8100023400CA03
    static const uint8_t expected[7] = { 0x81, 0x00, 0x02, 0x34, 0x00, 0xCA, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 7)) {
      return false;
    }
  }

  Serial1.updateBaudRate(115200);
  while (Serial1.available()) {  // Consume remaining garbage
    Serial1.read();
  }

  delay(140);  // 140ms to wake up in the new baud rate

  Serial.println("new baud rate");

  {
    Serial1.write(0x01);  // Start of packet
    Serial1.write(0x00);
    Serial1.write(0x09);  // 9 bytes long
    Serial1.write(0x12);  // Erase Command
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);  // Start 0x00
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x3F);
    Serial1.write(0xFF);  // End 0x3FFF (16kByte)
    Serial1.write(0xA7);  // Checksum
    Serial1.write(0x03);  // End

    // I received: 0x8100021200EC03
    static const uint8_t expected[7] = { 0x81, 0x00, 0x02, 0x12, 0x00, 0xEC, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 7)) {
      return false;
    }
  }

  {
    Serial1.write(0x01);  // Start of packet
    Serial1.write(0x00);
    Serial1.write(0x09);  // 9 bytes long
    Serial1.write(0x13);  // Write Command
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);  // Start address: 0x00
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x3C);
    Serial1.write(0x27);  // End address: 0x3C27 (15399)
    Serial1.write(0x81);  // Checksum
    Serial1.write(0x03);  // End

    // I received: 0x8100021300EB03
    static const uint8_t expected[7] = { 0x81, 0x00, 0x02, 0x13, 0x00, 0xEB, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 7)) {
      return false;
    }
  }

  // Send 1024 Bytes... x 15 and one block of 0x28 bytes   (+40 bytes to end)
  // 15400 bytes total
  HexBlock& toWrite = dfuImage.blocks[0];
  int remaining = toWrite.data.size();
  int offset = 0;
  while (remaining) {
    int tosend = remaining > 1024 ? 1024 : remaining;

    int hh = ((tosend + 1) >> 8) & 0xff;
    int hl = (tosend + 1) & 0xff;

    CSum cs;
    Serial1.write(0x81);  // Write 1024 Bytes
    Serial1.write(hh);
    cs += hh;
    Serial1.write(hl);
    cs += hl;
    Serial1.write(0x13);
    cs += 0x13;

    //Send N = 1024 bytes from the file
    for (int i = 0; i < tosend; ++i) {
      char c = toWrite.data[offset + i];
      Serial1.write(c);
      cs += c;
    }

    unsigned char csum = cs.get();

    Serial1.write(csum);  // Checksum
    Serial1.write(0x03);

    Serial.print("Writing page in offset: ");
    Serial.print(offset);
    Serial.print(" that has length of ");
    Serial.print(tosend);
    Serial.print(" and checksum");
    Serial.println(csum, HEX);

    // I received: 0x8100021300EB03
    static const uint8_t expected[7] = { 0x81, 0x00, 0x02, 0x13, 0x00, 0xEB, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 7)) {
      return false;
    }

    offset += tosend;
    remaining -= tosend;
  }
  
  
  if (false) {
	  // The programmer of the new image also does an extra write of 28 bytes in address 0x01010018
	  // Did the other guys do bin->hex and strip this out?

    Serial1.write(0x01);  // Start of packet
    Serial1.write(0x00);
    Serial1.write(0x09);  // 9 bytes long
    Serial1.write(0x13);  // Write Command
    Serial1.write(0x01);
    Serial1.write(0x01);
    Serial1.write(0x00);
    Serial1.write(0x18);  // Start address: 0x01010018
    Serial1.write(0x01);
    Serial1.write(0x01);
    Serial1.write(0x00);
    Serial1.write(0x33);  // End address: 0x3C27 (15399)
    Serial1.write(0x95);  // Checksum
    Serial1.write(0x03);  // End

    // I received: 0x8100021300EB03
    static const uint8_t expected[7] = { 0x81, 0x00, 0x02, 0x13, 0x00, 0xEB, 0x03 };
    if (!readExpected(Serial1, Serial, expected, 7)) {
      return false;
    }
	
	// Then it writes:
	Serial1.write(0x81);  // Start of packet
	Serial1.write(0x00);
	Serial1.write(0x13);
	Serial1.write(0xff); // (28 times)
	Serial1.write(0xEC);
	Serial1.write(0x03);
  }

  // Reset MD, and reset
  digitalWrite(GPIO_BOOT, HIGH);
  digitalWrite(GPIO_RST, HIGH);
  delay(100);
  digitalWrite(GPIO_RST, LOW);
  delay(100);
  digitalWrite(GPIO_RST, HIGH);

  return true;
}
