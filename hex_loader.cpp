#include "hex_loader.h"

#include <FFat.h>

namespace {

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool hexByte(const char* s, uint8_t& out) {
  int hi = hexNibble(s[0]);
  int lo = hexNibble(s[1]);
  if (hi < 0 || lo < 0) return false;
  out = (uint8_t)((hi << 4) | lo);
  return true;
}

// Append `data` of `len` bytes at absolute `addr` into the block list,
// coalescing with the last block if it extends contiguously.
void appendBytes(std::vector<HexBlock>& blocks,
                 uint32_t addr, const uint8_t* data, size_t len) {
  if (len == 0) return;
  if (!blocks.empty()) {
    HexBlock& tail = blocks.back();
    if (addr == tail.addr + tail.data.size()) {
      tail.data.insert(tail.data.end(), data, data + len);
      return;
    }
  }
  HexBlock nb;
  nb.addr = addr;
  nb.data.assign(data, data + len);
  blocks.push_back(std::move(nb));
}

}  // namespace

bool loadHexFromFatFs(const char* path, HexImage& image) {
  if (!FFat.begin(true)) {
    Serial.println("[hex] FFat mount failed");
    return false;
  }

  File f = FFat.open(path, "r");
  if (!f) {
    Serial.printf("[hex] open %s failed\n", path);
    return false;
  }

  image.blocks.clear();
  image.hasStartAddress = false;
  image.startAddress = 0;

  uint32_t upperBase = 0;       // Type 04 (Extended Linear Address) << 16
  uint32_t segmentBase = 0;     // Type 02 (Extended Segment Address) << 4
  bool sawEof = false;
  uint32_t lineNo = 0;
  char line[600];               // 255 data bytes -> 521 hex chars + overhead

  while (f.available()) {
    int n = f.readBytesUntil('\n', line, sizeof(line) - 1);
    if (n <= 0) continue;
    line[n] = '\0';
    if (line[n - 1] == '\r') line[--n] = '\0';
    lineNo++;

    if (line[0] != ':') continue;  // skip blanks / non-record lines
    if (n < 11) {
      Serial.printf("[hex] line %u: too short\n", (unsigned)lineNo);
      f.close();
      return false;
    }

    uint8_t byteCount, addrHi, addrLo, recType;
    if (!hexByte(line + 1, byteCount) ||
        !hexByte(line + 3, addrHi) ||
        !hexByte(line + 5, addrLo) ||
        !hexByte(line + 7, recType)) {
      Serial.printf("[hex] line %u: bad header\n", (unsigned)lineNo);
      f.close();
      return false;
    }

    int expected = 1 + 2 + 4 + 2 + byteCount * 2 + 2;  // : + len + addr + type + data + cksum
    if (n < expected) {
      Serial.printf("[hex] line %u: truncated (have %d need %d)\n",
                    (unsigned)lineNo, n, expected);
      f.close();
      return false;
    }

    // Verify checksum (two's complement of sum of all bytes incl. checksum -> 0)
    uint8_t sum = byteCount + addrHi + addrLo + recType;
    uint8_t data[256];
    for (uint8_t i = 0; i < byteCount; i++) {
      if (!hexByte(line + 9 + i * 2, data[i])) {
        Serial.printf("[hex] line %u: bad data\n", (unsigned)lineNo);
        f.close();
        return false;
      }
      sum += data[i];
    }
    uint8_t cksum;
    if (!hexByte(line + 9 + byteCount * 2, cksum)) {
      Serial.printf("[hex] line %u: bad checksum field\n", (unsigned)lineNo);
      f.close();
      return false;
    }
    if ((uint8_t)(sum + cksum) != 0) {
      Serial.printf("[hex] line %u: checksum mismatch\n", (unsigned)lineNo);
      f.close();
      return false;
    }

    uint16_t loAddr = ((uint16_t)addrHi << 8) | addrLo;

    switch (recType) {
      case 0x00: {  // Data
        uint32_t abs = upperBase + segmentBase + loAddr;
        appendBytes(image.blocks, abs, data, byteCount);
        break;
      }
      case 0x01:    // EOF
        sawEof = true;
        break;
      case 0x02:    // Extended Segment Address
        if (byteCount != 2) {
          Serial.printf("[hex] line %u: bad type 02 length\n", (unsigned)lineNo);
          f.close();
          return false;
        }
        segmentBase = ((uint32_t)data[0] << 8 | data[1]) << 4;
        upperBase = 0;
        break;
      case 0x03:    // Start Segment Address (CS:IP) — ignored on ARM
        break;
      case 0x04:    // Extended Linear Address
        if (byteCount != 2) {
          Serial.printf("[hex] line %u: bad type 04 length\n", (unsigned)lineNo);
          f.close();
          return false;
        }
        upperBase = ((uint32_t)data[0] << 8 | data[1]) << 16;
        segmentBase = 0;
        break;
      case 0x05:    // Start Linear Address
        if (byteCount != 4) {
          Serial.printf("[hex] line %u: bad type 05 length\n", (unsigned)lineNo);
          f.close();
          return false;
        }
        image.startAddress = ((uint32_t)data[0] << 24) |
                             ((uint32_t)data[1] << 16) |
                             ((uint32_t)data[2] << 8) |
                             (uint32_t)data[3];
        image.hasStartAddress = true;
        break;
      default:
        Serial.printf("[hex] line %u: unknown record type 0x%02X (skipped)\n",
                      (unsigned)lineNo, recType);
        break;
    }

    if (sawEof) break;
  }

  f.close();

  if (!sawEof) {
    Serial.println("[hex] missing EOF record");
    return false;
  }
  return true;
}

void dumpHexImage(const HexImage& image) {
  Serial.printf("[hex] %u block(s), %u total bytes\n",
                (unsigned)image.blocks.size(), (unsigned)image.totalBytes());
  for (size_t i = 0; i < image.blocks.size(); i++) {
    const HexBlock& b = image.blocks[i];
    Serial.printf("  [%u] addr=0x%08lX len=%u (end=0x%08lX)\n",
                  (unsigned)i, (unsigned long)b.addr,
                  (unsigned)b.data.size(),
                  (unsigned long)(b.addr + b.data.size() - 1));
  }
  if (image.hasStartAddress) {
    Serial.printf("  start address = 0x%08lX\n", (unsigned long)image.startAddress);
  }
}
