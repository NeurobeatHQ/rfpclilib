#pragma once

#include <Arduino.h>
#include <vector>

// One contiguous span of bytes loaded from the .hex file.
struct HexBlock {
  uint32_t addr;            // absolute start address
  std::vector<uint8_t> data;
};

struct HexImage {
  std::vector<HexBlock> blocks;   // sorted by addr, no overlaps, gaps allowed
  uint32_t startAddress = 0;      // from record type 05 (0 if absent)
  bool hasStartAddress = false;

  uint32_t totalBytes() const {
    uint32_t n = 0;
    for (const auto& b : blocks) n += b.data.size();
    return n;
  }
};

// Parses an Intel HEX file from FatFS into coalesced blocks of contiguous memory.
// Adjacent / touching records are merged into a single block; non-contiguous
// jumps start a new block.
//
// Returns true on success. On failure, `image` may be partially populated.
bool loadHexFromFatFs(const char* path, HexImage& image);

// Pretty-prints block summary to Serial.
void dumpHexImage(const HexImage& image);
