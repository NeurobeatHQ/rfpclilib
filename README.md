# rfpclilib

Program the **Renesas RA4M1** bootloader (the MCU on the **Arduino UNO R4 WiFi**)
over UART from an ESP32 host, using the Renesas **RFP / SCI serial boot
protocol**. Framework-neutral — the same source builds under both **Arduino
(ESP32)** and **ESP-IDF**.

## What it does

Streams a single-block Intel HEX image (e.g. `dfu_wifi.hex`) into the RA4M1's
16 KB bootloader region, driving the chip's SCI boot firmware end to end:
auto-baud handshake → inquiry / signature / area queries → raise baud to
115200 → erase → write, one `<=1 KB` page at a time, each ACK-checked.

## Design

The library has **no Arduino or FreeRTOS dependency of its own**. Everything
platform-specific is injected by the caller:

- **UART + delay** — a `SerialInterface` you implement (`write` / `read` /
  `setBaud` / `delayMs`).
- **The HEX image** — a filesystem path, read with POSIX `fopen`, so any
  VFS-mounted filesystem works (LittleFS, FatFS, …).
- **Logging** — two optional callbacks: `err` (failures) and `debug` (progress).
- **Boot-mode strapping and the final reset** — GPIO, done by the caller before
  and after the call.

## API

```cpp
class SerialInterface {
public:
  virtual void write(uint8_t byte) = 0;      // TX one byte
  virtual bool read(uint8_t *byte) = 0;      // RX one byte; false on timeout
  virtual void setBaud(uint32_t baud) = 0;   // switch the local UART baud
  virtual void delayMs(uint32_t ms) = 0;     // block for `ms` milliseconds
};

bool programArduinoBootloader(const char *hexPath, SerialInterface &ra4m1,
                              void (*err)(const char *)   = nullptr,
                              void (*debug)(const char *) = nullptr);
```

Returns `true` only if every protocol step ACKed and the image streamed.

## Wiring (Adafruit Feather ESP32-S3 → UNO R4 WiFi)

| ESP32 pin        | RA4M1        | Note                          |
| ---------------- | ------------ | ----------------------------- |
| GPIO17 (A1)      | MD / BOOT    | held low to enter boot mode   |
| GPIO18 (A0)      | RES          | reset pulse                   |
| GPIO15 (A3, TX)  | RXD9 (P110)  | ESP → target                  |
| GPIO14 (A4, RX)  | TXD9 (P109)  | target → ESP                  |
| GND              | GND          |                               |

## Usage (ESP-IDF)

```cpp
class RaUart : public SerialInterface {
public:
  void write(uint8_t b) override       { uart_write_bytes(UART_NUM_1, &b, 1); }
  bool read(uint8_t *b) override        { return uart_read_bytes(UART_NUM_1, b, 1, pdMS_TO_TICKS(200)) == 1; }
  void setBaud(uint32_t baud) override  { uart_set_baudrate(UART_NUM_1, baud); }
  void delayMs(uint32_t ms) override    { vTaskDelay(pdMS_TO_TICKS(ms)); }
};

// UART @9600 8N1, BOOT/RES as GPIO outputs, and the filesystem all set up first…
strap_into_boot_mode();     // caller: MD low + reset pulse
RaUart ra;
bool ok = programArduinoBootloader("/littlefs/dfu_wifi.hex", ra, on_err, on_debug);
reset_to_run();             // caller: MD high + reset pulse
```

For an Arduino sketch, see [`examples/ProgramBootloader`](examples/ProgramBootloader/ProgramBootloader.ino),
which backs `SerialInterface` with `Serial1` + `LittleFS` and `delayMs` with `delay()`.

## Constraints

- **Single-block HEX only** (a clean bootloader image); multi-block input is
  rejected up front.
- Writes exactly the block's bytes; the erase is a fixed **16 KB**
  (`0x0000`–`0x3FFF`), the max bootloader size — a block past that is rejected.
- The signature / area responses are **hardcoded** for one RA4M1 revision
  (captured from silicon); a different part will mismatch and abort.
- The caller owns: mounting the filesystem, boot-mode strapping (MD + reset),
  and the final reset.

## Files

`rfpclilib.h` / `rfpclilib.cpp` (self-contained — the Intel-HEX parser is folded
in). Drop the folder into an Arduino `libraries/` dir, or vendor it as an ESP-IDF
component (`idf_component_register(SRCS "rfpclilib.cpp" INCLUDE_DIRS ".")`).
