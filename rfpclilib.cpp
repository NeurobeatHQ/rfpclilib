// rfpclilib.cpp — Renesas RFP/SCI serial-boot programmer for the Arduino UNO R4
// WiFi's RA4M1. Framework-neutral: it has NO Arduino or FreeRTOS dependency —
// the UART, the delay, the logging and the file path are all supplied by the
// caller.
//
// Design:
//   * Intel-HEX image read from a mounted filesystem (LittleFS/FatFS) via POSIX
//     fopen/fgets — works on arduino-esp32 and ESP-IDF (both VFS-backed).
//   * The target UART (write/read/setBaud) AND the ms delay are injected as a
//     `SerialInterface`.
//   * Two optional sinks: `err(const char*)` for failures, `debug(const char*)`
//     for progress. Messages are formatted into a local buffer (no-op if null).
//   * hex_loader is folded into this file's anonymous namespace.
//   * Single-block images only; the write range is derived from that block.
//   * The reset/boot GPIO sequences are the caller's job (see header).
//
// The RFP command/response byte sequences are preserved verbatim from the
// original (captured against real RA4M1 silicon) — do not "tidy" them.

#include "rfpclilib.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

using PrintFn = void (*)(const char *);

// ---------------------------------------------------------------------------
// Format into a local buffer and hand a const char* to a sink (`err` or
// `debug`). No-op when the sink is null.
// ---------------------------------------------------------------------------
__attribute__((format(printf, 2, 3)))
void emitf(PrintFn sink, const char *fmt, ...) {
    if (!sink) return;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    sink(buf);
}

// ---------------------------------------------------------------------------
// Intel HEX loader (folded in from the old hex_loader.*; internal only).
// ---------------------------------------------------------------------------
struct HexBlock {
    uint32_t addr;
    std::vector<uint8_t> data;
};

struct HexImage {
    std::vector<HexBlock> blocks;  // coalesced, sorted by addr, gaps allowed
    uint32_t startAddress = 0;
    bool hasStartAddress = false;

    uint32_t totalBytes() const {
        uint32_t n = 0;
        for (const auto &b : blocks) n += b.data.size();
        return n;
    }
};

int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool hexByte(const char *s, uint8_t &out) {
    int hi = hexNibble(s[0]);
    int lo = hexNibble(s[1]);
    if (hi < 0 || lo < 0) return false;
    out = static_cast<uint8_t>((hi << 4) | lo);
    return true;
}

// Append `len` bytes at absolute `addr`, coalescing with the previous block if
// it extends it contiguously.
void appendBytes(std::vector<HexBlock> &blocks, uint32_t addr,
                 const uint8_t *data, size_t len) {
    if (len == 0) return;
    if (!blocks.empty()) {
        HexBlock &tail = blocks.back();
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

// Parse an Intel HEX file into coalesced blocks of contiguous memory. Returns
// true on success; `image` may be partial on failure.
bool loadHex(const char *path, HexImage &image, PrintFn err, PrintFn debug) {
    FILE *f = fopen(path, "r");
    if (!f) {
        emitf(err, "[hex] open %s failed", path);
        return false;
    }

    image.blocks.clear();
    image.hasStartAddress = false;
    image.startAddress = 0;

    uint32_t upperBase = 0;    // type 04 (Extended Linear Address) << 16
    uint32_t segmentBase = 0;  // type 02 (Extended Segment Address) << 4
    bool sawEof = false;
    uint32_t lineNo = 0;
    char line[256];            // ~121 data bytes/record max (typical hex uses 16/32-byte records)

    while (fgets(line, sizeof(line), f)) {
        int n = static_cast<int>(strlen(line));
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) line[--n] = '\0';
        if (n == 0) continue;
        lineNo++;

        if (line[0] != ':') continue;  // skip blanks / non-record lines
        if (n < 11) {
            emitf(err, "[hex] line %u: too short", (unsigned)lineNo);
            fclose(f);
            return false;
        }

        uint8_t byteCount, addrHi, addrLo, recType;
        if (!hexByte(line + 1, byteCount) || !hexByte(line + 3, addrHi) ||
            !hexByte(line + 5, addrLo) || !hexByte(line + 7, recType)) {
            emitf(err, "[hex] line %u: bad header", (unsigned)lineNo);
            fclose(f);
            return false;
        }

        int expected = 1 + 2 + 4 + 2 + byteCount * 2 + 2;  // : len addr type data cksum
        if (n < expected) {
            emitf(err, "[hex] line %u: truncated (have %d need %d)", (unsigned)lineNo, n, expected);
            fclose(f);
            return false;
        }

        // Checksum: two's complement of the sum of all bytes (incl. cksum) is 0.
        uint8_t sum = byteCount + addrHi + addrLo + recType;
        uint8_t data[256];
        for (uint8_t i = 0; i < byteCount; i++) {
            if (!hexByte(line + 9 + i * 2, data[i])) {
                emitf(err, "[hex] line %u: bad data", (unsigned)lineNo);
                fclose(f);
                return false;
            }
            sum += data[i];
        }
        uint8_t cksum;
        if (!hexByte(line + 9 + byteCount * 2, cksum)) {
            emitf(err, "[hex] line %u: bad checksum field", (unsigned)lineNo);
            fclose(f);
            return false;
        }
        if (static_cast<uint8_t>(sum + cksum) != 0) {
            emitf(err, "[hex] line %u: checksum mismatch", (unsigned)lineNo);
            fclose(f);
            return false;
        }

        uint16_t loAddr = (static_cast<uint16_t>(addrHi) << 8) | addrLo;

        switch (recType) {
            case 0x00:  // Data
                appendBytes(image.blocks, upperBase + segmentBase + loAddr, data, byteCount);
                break;
            case 0x01:  // EOF
                sawEof = true;
                break;
            case 0x02:  // Extended Segment Address
                if (byteCount != 2) {
                    emitf(err, "[hex] line %u: bad type 02 length", (unsigned)lineNo);
                    fclose(f);
                    return false;
                }
                segmentBase = (static_cast<uint32_t>(data[0]) << 8 | data[1]) << 4;
                upperBase = 0;
                break;
            case 0x03:  // Start Segment Address — ignored on ARM
                break;
            case 0x04:  // Extended Linear Address
                if (byteCount != 2) {
                    emitf(err, "[hex] line %u: bad type 04 length", (unsigned)lineNo);
                    fclose(f);
                    return false;
                }
                upperBase = (static_cast<uint32_t>(data[0]) << 8 | data[1]) << 16;
                segmentBase = 0;
                break;
            case 0x05:  // Start Linear Address
                if (byteCount != 4) {
                    emitf(err, "[hex] line %u: bad type 05 length", (unsigned)lineNo);
                    fclose(f);
                    return false;
                }
                image.startAddress = (static_cast<uint32_t>(data[0]) << 24) |
                                     (static_cast<uint32_t>(data[1]) << 16) |
                                     (static_cast<uint32_t>(data[2]) << 8) | data[3];
                image.hasStartAddress = true;
                break;
            default:
                emitf(debug, "[hex] line %u: unknown record type 0x%02X (skipped)",
                      (unsigned)lineNo, recType);
                break;
        }
        if (sawEof) break;
    }
    fclose(f);

    if (!sawEof) {
        emitf(err, "[hex] missing EOF record");
        return false;
    }
    return true;
}

void dumpHexImage(const HexImage &image, PrintFn debug) {
    if (!debug) return;
    emitf(debug, "[hex] %u block(s), %u total bytes",
          (unsigned)image.blocks.size(), (unsigned)image.totalBytes());
    for (size_t i = 0; i < image.blocks.size(); i++) {
        const HexBlock &b = image.blocks[i];
        emitf(debug, "  [%u] addr=0x%08lX len=%u (end=0x%08lX)", (unsigned)i,
              (unsigned long)b.addr, (unsigned)b.data.size(),
              (unsigned long)(b.addr + b.data.size() - 1));
    }
    if (image.hasStartAddress)
        emitf(debug, "  start address = 0x%08lX", (unsigned long)image.startAddress);
}

// ---------------------------------------------------------------------------
// RFP/SCI boot-protocol helpers.
// ---------------------------------------------------------------------------

// Running two's-complement checksum used by the write-page (0x81) packets and
// the derived write-range command.
class CSum {
    uint32_t sum_ = 0;
public:
    CSum &operator+=(uint8_t rhs) { sum_ += rhs; return *this; }
    uint8_t get() const { return static_cast<uint8_t>((-sum_) & 0xFF); }
};

void writeAll(SerialInterface &ra4m1, const uint8_t *bytes, size_t n) {
    for (size_t i = 0; i < n; ++i) ra4m1.write(bytes[i]);
}

// Read `length` bytes and compare against `expected`; log a hex diff on mismatch.
bool readExpected(SerialInterface &ra4m1, PrintFn err, const uint8_t *expected, int length) {
    uint8_t buf[32];  // longest expected response is 23 bytes
    for (int i = 0; i < length; ++i) {
        if (!ra4m1.read(&buf[i])) {
            emitf(err, "Timeout reading response byte %d/%d", i, length);
            return false;
        }
    }
    if (memcmp(buf, expected, length) != 0) {
        if (err) {
            char msg[256];
            int o = snprintf(msg, sizeof(msg), "Response mismatch. Expected:");
            for (int i = 0; i < length; ++i)
                o += snprintf(msg + o, sizeof(msg) - o, " %02X", expected[i]);
            o += snprintf(msg + o, sizeof(msg) - o, " Actual:");
            for (int i = 0; i < length; ++i)
                o += snprintf(msg + o, sizeof(msg) - o, " %02X", buf[i]);
            err(msg);
        }
        return false;
    }
    return true;
}

// Send a fixed command packet, then verify the fixed response.
bool command(SerialInterface &ra4m1, PrintFn err,
             const uint8_t *cmd, size_t cmdLen, const uint8_t *resp, int respLen) {
    writeAll(ra4m1, cmd, cmdLen);
    return readExpected(ra4m1, err, resp, respLen);
}

}  // namespace

bool programArduinoBootloader(const char *hexPath, SerialInterface &ra4m1,
                              void (*err)(const char *), void (*debug)(const char *)) {
    HexImage dfuImage;
    if (!loadHex(hexPath, dfuImage, err, debug)) {
        emitf(err, "[hex] load failed: %s", hexPath);
        return false;  // (original continued here and later crashed on blocks[0])
    }
    dumpHexImage(dfuImage, debug);
    // This programmer handles a single-block image only (a clean bootloader hex).
    // Reject empty or multi-block images rather than silently programming part.
    if (dfuImage.blocks.size() != 1) {
        emitf(err, "[hex] expected exactly one block, got %u — use a clean single-block hex",
              (unsigned)dfuImage.blocks.size());
        return false;
    }

    // Validate the block fits the 16 KB bootloader region BEFORE touching the
    // target — otherwise we'd handshake and erase it, then bail, leaving the
    // target with a wiped bootloader.
    const HexBlock &toWrite = dfuImage.blocks[0];
    // The code-flash WRITE access unit is 8 bytes (WAU in the Area-info response),
    // so the written length must be a multiple of 8 or the target rejects the
    // write-range with Address error (0xD0). Round up; the tail is padded 0xFF.
    const size_t kWriteUnit = 8;
    size_t writeSize = (toWrite.data.size() + (kWriteUnit - 1)) & ~(kWriteUnit - 1);
    uint32_t wStart = toWrite.addr;
    uint32_t wEnd   = toWrite.addr + writeSize - 1;
    if (wEnd > 0x3FFF) {
        emitf(err, "[hex] write ends at 0x%lX, past the 16 KB bootloader region",
              (unsigned long)wEnd);
        return false;
    }

    // The caller has already strapped the target into boot mode.

    // --- Communication setting (R01AN5372 Fig 11/14/27): send 0x00 low pulses
    //     until the target ACKs with 0x00, then send Generic code 0x55 and
    //     expect Boot code 0xC3. The 1st 0x00 only wakes the SCI (no reply); the
    //     ACK comes from a later 0x00 received as data. So flush any reset/boot
    //     glitch first, and wait for an actual 0x00 — not just any byte.
    //     Both loops are bounded so a dead target fails instead of hanging. ---
    { uint8_t junk; while (ra4m1.read(&junk)) { /* drop reset/boot glitches */ } }

    // The SCI locks its baud from a near-CONTINUOUS stream of 0x00 low pulses.
    // The Arduino original spammed 0x00 non-stop (non-blocking available()); this
    // port was sending one pulse per 200 ms blocking read — far too sparse, so
    // the target never finished comm-setup and ignored the 0x55. Fix: send a
    // solid burst of 0x00 each round, then read the ACK it buffered mid-burst.
    uint8_t ib = 0;
    bool acked = false;
    for (int round = 0; round < 20 && !acked; ++round) {
        for (int j = 0; j < 32; ++j) ra4m1.write(0x00);     // ~32 back-to-back pulses
        while (ra4m1.read(&ib)) { if (ib == 0x00) { acked = true; break; } }
    }
    if (!acked) {
        emitf(err, "no 0x00 ACK — target not answering (boot mode? MD low? TX/RX/GND?)");
        return false;
    }
    // --- Generic code 0x55 -> expect Boot code 0xC3. ---
    ra4m1.write(0x55);
    bool booted = false;
    for (int i = 0; i < 40 && !booted; ++i) {   // tolerate leftover 0x00s, bounded
        if (ra4m1.read(&ib) && ib == 0xC3) booted = true;
    }
    if (!booted) {
        emitf(err, "no 0xC3 boot code after 0x55 — handshake failed");
        return false;
    }
    emitf(debug, "Init Done");

    // --- Inquiry command. ---
    {
        static const uint8_t cmd[] = {
            0x01,  // Start of packet
            0x00,
            0x01,  // 1 byte long
            0x00,  // Inquiry command
            0xFF,  // Checksum
            0x03,  // End
        };
        static const uint8_t resp[] = { 0x81, 0x00, 0x02, 0x00, 0x00, 0xFE, 0x03 };
        if (!command(ra4m1, err, cmd, sizeof(cmd), resp, sizeof(resp))) return false;
    }

    // --- Signature request command. ---
    {
        static const uint8_t cmd[] = {
            0x01,  // Start of packet
            0x00,
            0x01,  // 1 byte long
            0x3A,  // Signature request command
            0xC5,  // Checksum
            0x03,  // End
        };
        // I received: 0x81000D3A 016E3600 0016E360 03 02 0200 B403
        // SCI op clock: 24MHz, Max UART Baud Rate 1,500,000bps
        static const uint8_t resp[] = { 0x81, 0x00, 0x0D, 0x3A, 0x01, 0x6E, 0x36, 0x00, 0x00,
                                        0x16, 0xE3, 0x60, 0x03, 0x02, 0x02, 0x00, 0xB4, 0x03 };
        if (!command(ra4m1, err, cmd, sizeof(cmd), resp, sizeof(resp))) return false;
    }

    // --- Area information request, area 0. ---
    {
        static const uint8_t cmd[] = {
            0x01,  // Start of packet
            0x00,
            0x02,  // 2 bytes long
            0x3B,  // Area information request command
            0x00,  // Area 0
            0xC3,  // Checksum
            0x03,  // End
        };
        // I received: 0x8100123B00000000000003FFFF0000080000000008A203
        static const uint8_t resp[] = { 0x81, 0x00, 0x12, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
                                        0xFF, 0xFF, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x08, 0xA2, 0x03 };
        if (!command(ra4m1, err, cmd, sizeof(cmd), resp, sizeof(resp))) return false;
    }

    // --- Area information request, area 1. ---
    {
        static const uint8_t cmd[] = {
            0x01,  // Start of packet
            0x00,
            0x02,  // 2 bytes long
            0x3B,  // Area information request command
            0x01,  // Area 1
            0xC2,  // Checksum
            0x03,  // End
        };
        // I received: 0x8100123B014010000040101FFF0000040000000001EF03
        static const uint8_t resp[] = { 0x81, 0x00, 0x12, 0x3B, 0x01, 0x40, 0x10, 0x00, 0x00, 0x40, 0x10, 0x1F,
                                        0xFF, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0xEF, 0x03 };
        if (!command(ra4m1, err, cmd, sizeof(cmd), resp, sizeof(resp))) return false;
    }

    // --- Area information request, area 2. ---
    {
        static const uint8_t cmd[] = {
            0x01,  // Start of packet
            0x00,
            0x02,  // 2 bytes long
            0x3B,  // Area information request command
            0x02,  // Area 2
            0xC1,  // Checksum
            0x03,  // End
        };
        // I received: 0x8100123B02010100080101003300000000000000046E03
        static const uint8_t resp[] = { 0x81, 0x00, 0x12, 0x3B, 0x02, 0x01, 0x01, 0x00, 0x08, 0x01, 0x01, 0x00,
                                        0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6E, 0x03 };
        if (!command(ra4m1, err, cmd, sizeof(cmd), resp, sizeof(resp))) return false;
    }

    // --- Baud rate setting -> 115200, then switch the local UART to match. ---
    {
        static const uint8_t cmd[] = {
            0x01,  // Start of packet
            0x00,
            0x05,  // 5 bytes long
            0x34,  // Baud rate setting command
            0x00,
            0x01,
            0xC2,
            0x00,  // 0x1C200 => 115200 baud
            0x04,  // Checksum
            0x03,  // End
        };
        // I received: 0x8100023400CA03
        static const uint8_t resp[] = { 0x81, 0x00, 0x02, 0x34, 0x00, 0xCA, 0x03 };
        if (!command(ra4m1, err, cmd, sizeof(cmd), resp, sizeof(resp))) return false;
    }
    ra4m1.setBaud(115200);
    { uint8_t junk; while (ra4m1.read(&junk)) { /* drain stale bytes at old baud */ } }
    ra4m1.delayMs(140);  // let the target settle at the new baud

    // --- Erase 0x0000-0x3FFF (16 KB) — the max the bootloader is expected to
    //     occupy. Kept hardcoded on purpose. ---
    {
        static const uint8_t cmd[] = {
            0x01,  // Start of packet
            0x00,
            0x09,  // 9 bytes long
            0x12,  // Erase command
            0x00,
            0x00,
            0x00,
            0x00,  // Start 0x00000000
            0x00,
            0x00,
            0x3F,
            0xFF,  // End 0x00003FFF (16 kByte)
            0xA7,  // Checksum
            0x03,  // End
        };
        // I received: 0x8100021200EC03
        static const uint8_t resp[] = { 0x81, 0x00, 0x02, 0x12, 0x00, 0xEC, 0x03 };
        if (!command(ra4m1, err, cmd, sizeof(cmd), resp, sizeof(resp))) return false;
    }

    // --- Write range 0x0000..(size-1), DERIVED from the single block; its fit
    //     in the 16 KB region was validated up front. ---
    {
        uint8_t cmd[] = {
            0x01,                     // Start of packet
            0x00,
            0x09,                     // 9 bytes long
            0x13,                     // Write command
            (uint8_t)(wStart >> 24),  // Start address (big-endian)
            (uint8_t)(wStart >> 16),
            (uint8_t)(wStart >> 8),
            (uint8_t)(wStart),
            (uint8_t)(wEnd >> 24),    // End address = start + size - 1
            (uint8_t)(wEnd >> 16),
            (uint8_t)(wEnd >> 8),
            (uint8_t)(wEnd),
            0x00,                     // Checksum (filled in below)
            0x03,                     // End
        };
        CSum cs;                      // over length-hi/lo + cmd + 8 address bytes
        for (int i = 1; i <= 11; ++i) cs += cmd[i];
        cmd[12] = cs.get();
        // I received: 0x8100021300EB03
        static const uint8_t resp[] = { 0x81, 0x00, 0x02, 0x13, 0x00, 0xEB, 0x03 };
        if (!command(ra4m1, err, cmd, sizeof(cmd), resp, sizeof(resp))) return false;
    }

    // --- Stream the block in <=1024-byte pages (0x81 data packets). ---
    // I received (per page): 0x8100021300EB03
    static const uint8_t pageAck[] = { 0x81, 0x00, 0x02, 0x13, 0x00, 0xEB, 0x03 };
    emitf(debug, "writing %u bytes (%u data + %u pad)",
          (unsigned)writeSize, (unsigned)toWrite.data.size(),
          (unsigned)(writeSize - toWrite.data.size()));
    size_t remaining = writeSize;
    size_t offset = 0;
    while (remaining) {
        size_t tosend = remaining > 1024 ? 1024 : remaining;
        uint8_t hh = static_cast<uint8_t>(((tosend + 1) >> 8) & 0xFF);
        uint8_t hl = static_cast<uint8_t>((tosend + 1) & 0xFF);

        CSum cs;
        ra4m1.write(0x81);
        ra4m1.write(hh);   cs += hh;
        ra4m1.write(hl);   cs += hl;
        ra4m1.write(0x13); cs += 0x13;
        for (size_t i = 0; i < tosend; ++i) {
            uint8_t c = (offset + i < toWrite.data.size()) ? toWrite.data[offset + i] : 0xFF;
            ra4m1.write(c);
            cs += c;
        }
        uint8_t csum = cs.get();
        ra4m1.write(csum);
        ra4m1.write(0x03);

        if (!readExpected(ra4m1, err, pageAck, sizeof(pageAck))) return false;

        offset += tosend;
        remaining -= tosend;
    }

    // Unsupported, but potentially program other segments here — e.g. an extra
    // 28-byte write to address 0x01010018 (config/ID region). The reference
    // programmer did this; unclear whether a bin->hex conversion stripped it, so
    // we deliberately do NOT send it. Left for reference, in the same form:
    //
    // // --- Write range 0x01010018-0x01010033 (config/ID region). ---
    // {
    //     static const uint8_t cmd[] = {
    //         0x01,  // Start of packet
    //         0x00,
    //         0x09,  // 9 bytes long
    //         0x13,  // Write command
    //         0x01,
    //         0x01,
    //         0x00,
    //         0x18,  // Start address: 0x01010018
    //         0x01,
    //         0x01,
    //         0x00,
    //         0x33,  // End address:   0x01010033
    //         0x95,  // Checksum
    //         0x03,  // End
    //     };
    //     // I received: 0x8100021300EB03
    //     static const uint8_t resp[] = { 0x81, 0x00, 0x02, 0x13, 0x00, 0xEB, 0x03 };
    //     if (!command(ra4m1, err, cmd, sizeof(cmd), resp, sizeof(resp))) return false;
    // }
    // Then stream the 28-byte data page:
    //   0x81  0x00  0x13  0xFF (x28)  0xEC  0x03

    emitf(debug, "programming complete");
    // The caller resets the target so the freshly-written image runs.
    return true;
}
