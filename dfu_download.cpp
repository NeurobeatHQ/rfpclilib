#include "dfu_download.h"

#include <Arduino.h>
#include <FFat.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

static const char* DFU_URL =
  "https://raw.githubusercontent.com/arduino/ArduinoCore-renesas/main/"
  "bootloaders/UNO_R4/dfu_wifi.hex";

bool downloadDfuToFatFs(const char* path, bool overwrite) {
  if (!FFat.begin(true)) {
    Serial.println("[dfu] FFat mount failed");
    return false;
  }

  if (FFat.exists(path) && !overwrite) {
    Serial.printf("[dfu] %s already present (%u bytes), skipping download\n",
                  path, (unsigned)FFat.open(path, "r").size());
    return true;
  }

  WiFiClientSecure client;
  client.setInsecure();  // GitHub redirects to raw.githubusercontent.com; skip cert pinning

  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setTimeout(15000);

  if (!http.begin(client, DFU_URL)) {
    Serial.println("[dfu] http.begin failed");
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[dfu] HTTP GET failed: %d\n", code);
    http.end();
    return false;
  }

  int contentLen = http.getSize();  // -1 if chunked
  Serial.printf("[dfu] downloading %s (%d bytes)\n", DFU_URL, contentLen);

  File f = FFat.open(path, FILE_WRITE);
  if (!f) {
    Serial.printf("[dfu] cannot open %s for write\n", path);
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[1024];
  size_t total = 0;
  uint32_t lastData = millis();

  while (http.connected() && (contentLen < 0 || (int)total < contentLen)) {
    size_t avail = stream->available();
    if (avail) {
      size_t toRead = avail > sizeof(buf) ? sizeof(buf) : avail;
      int n = stream->readBytes(buf, toRead);
      if (n > 0) {
        if ((int)f.write(buf, n) != n) {
          Serial.println("[dfu] write failed (FS full?)");
          f.close();
          http.end();
          return false;
        }
        total += n;
        lastData = millis();
      }
    } else {
      if (millis() - lastData > 10000) {
        Serial.println("[dfu] stream stalled");
        break;
      }
      delay(1);
    }
  }

  f.close();
  http.end();

  if (contentLen > 0 && (int)total != contentLen) {
    Serial.printf("[dfu] short read: %u / %d\n", (unsigned)total, contentLen);
    FFat.remove(path);
    return false;
  }

  Serial.printf("[dfu] saved %s (%u bytes)\n", path, (unsigned)total);
  return true;
}
