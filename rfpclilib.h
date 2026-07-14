#pragma once

#include <cstdint>

// C++ interface to the RA4M1 target while it is in the Renesas SCI/RFP serial
// boot mode. Implement it over your platform's UART + timer (ESP-IDF
// uart_*/vTaskDelay, Arduino HardwareSerial/delay, ...). Providing the delay
// here is what lets the library carry no Arduino/FreeRTOS dependency of its own.
// The methods are non-const: they drive real I/O and real time.
class SerialInterface {
public:
    virtual ~SerialInterface() = default;

    // Transmit exactly one byte to the target.
    virtual void write(uint8_t byte) = 0;

    // Receive one byte. Blocks up to an implementation-defined timeout and
    // returns true (with *byte set) if a byte arrived, false on timeout.
    // (Merges the classic available()+read() into one call.)
    virtual bool read(uint8_t *byte) = 0;

    // Switch the local UART to `baud`. The RFP protocol raises the baud rate
    // mid-stream (command 0x34); this is invoked right after the target ACKs it.
    virtual void setBaud(uint32_t baud) = 0;

    // Block for `ms` milliseconds.
    virtual void delayMs(uint32_t ms) = 0;
};

// Program the Renesas RA4M1 over `ra4m1` using the RFP/SCI serial boot protocol,
// streaming the single-block Intel-HEX image at `hexPath` on a mounted
// filesystem (e.g. "/littlefs/downloads/dfu_nbtzero.hex").
//
// Preconditions (the caller owns these):
//   * The filesystem is mounted and `hexPath` exists (a clean, single-block hex).
//   * The target is ALREADY strapped into boot mode (MD/BOOT low + reset pulse).
//   * `ra4m1` is open at the target's initial boot baud (e.g. 9600).
// On success the caller resets the target so the freshly-written image runs.
//
// `err` and `debug`, if non-null, receive human-readable failure and progress
// lines respectively. Pass nullptr for a silent run.
//
// Returns true only if every protocol step was ACKed and the image streamed.
bool programArduinoBootloader(const char *hexPath,
                              SerialInterface &ra4m1,
                              void (*err)(const char *msg) = nullptr,
                              void (*debug)(const char *msg) = nullptr);
