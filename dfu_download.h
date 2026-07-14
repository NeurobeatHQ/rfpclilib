#pragma once

// Downloads the UNO_R4 dfu_wifi.hex bootloader from GitHub and stores it on
// the FatFS partition. Call once after Wi-Fi is connected.
//
// Returns true on success.
bool downloadDfuToFatFs(const char* path = "/dfu_wifi.hex",
                        bool overwrite = false);
