#include "c5_link.h"
#include "drone_scan.h"

#include <Arduino.h>
#include <cstring>
#include <cstdlib>

// === DIAGNOSTIC ============================================================
// 1 = echo every received line + parse result to the USB console
//     (pio device monitor on the StickS3). Use this to confirm the link.
// 0 = silent, normal operation.
#define C5_LINK_DEBUG 1
// ===========================================================================

namespace c5link {
namespace {

// --- WIRING (match your actual wiring!) ------------------------------------
// One-way link: C5 UART0 TXD  ->  StickS3 kRxPin.
// PROTOCOL.md uses G0. GPIO0 is the S3 boot strapping pin (see task §0) — if the
// link won't come up, move this wire to a plain GPIO and change kRxPin to match.
constexpr int      kRxPin = 0;     // StickS3 RX  <-  C5 TXD
constexpr int      kTxPin = -1;    // unused (C5 -> S3 only)
constexpr uint32_t kBaud  = 115200;
constexpr uint32_t kHeartbeatTimeoutMs = 8000;

HardwareSerial& Link = Serial1;    // Serial = USB-CDC on this build; Serial1 is free
char            s_line[256];
size_t          s_len     = 0;
unsigned long   s_last_hb = 0;
unsigned long   s_last_byte = 0;

// split src on '|' into tok[] (max maxTok). returns count. mutates src in place.
int split(char* src, char* tok[], int maxTok) {
  int n = 0;
  char* p = src;
  tok[n++] = p;
  while (*p && n < maxTok) {
    if (*p == '|') { *p = '\0'; tok[n++] = p + 1; }
    ++p;
  }
  return n;
}

void handleLine(char* line) {
#if C5_LINK_DEBUG
  Serial.print("[c5] "); Serial.println(line);
#endif
  if (line[0] == 'H') { s_last_hb = millis(); return; }  // H|planewatch-c5|<ver>
  if (line[0] != 'R') return;                            // only R| carries detections

  // R|mac|uasid|rssi|band|dlat|dlon|olat|olon|speed|track
  char* t[12];
  int n = split(line, t, 12);
  if (n < 5 || !t[1][0]) return;                         // need at least a MAC

  const char* mac = t[1];
  const char* id  = t[2];
  int8_t  rssi    = (int8_t)atoi(t[3]);
  uint8_t band    = (n > 4 && t[4][0]) ? (uint8_t)atoi(t[4]) : 24;
  bool   has_loc  = (n > 6  && t[5][0] && t[6][0]);
  double lat      = has_loc ? atof(t[5]) : 0.0;
  double lon      = has_loc ? atof(t[6]) : 0.0;
  bool   has_op   = (n > 8  && t[7][0] && t[8][0]);
  double oplat    = has_op  ? atof(t[7]) : 0.0;
  double oplon    = has_op  ? atof(t[8]) : 0.0;
  float  speed    = (n > 9  && t[9][0])  ? (float)atof(t[9])  : 0.0f;
  float  track    = (n > 10 && t[10][0]) ? (float)atof(t[10]) : 0.0f;

  drone::ingestC5(mac, id, rssi, band, has_loc, lat, lon, speed, track, has_op, oplat, oplon);
#if C5_LINK_DEBUG
  Serial.printf("[c5] -> drone mac=%s id=%s rssi=%d loc=%d op=%d\n",
                mac, id, rssi, (int)has_loc, (int)has_op);
#endif
}

}  // namespace

void begin() {
  Link.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);
#if C5_LINK_DEBUG
  Serial.printf("[c5] link up: Serial1 rx=%d tx=%d @%lu\n",
                kRxPin, kTxPin, (unsigned long)kBaud);
#endif
}

void poll() {
  while (Link.available()) {
    char c = (char)Link.read();
    s_last_byte = millis();
    if (c == '\n' || c == '\r') {
      if (s_len > 0) { s_line[s_len] = '\0'; handleLine(s_line); s_len = 0; }
    } else if (s_len < sizeof(s_line) - 1) {
      s_line[s_len++] = c;
    } else {
      s_len = 0;  // overrun — resync on next newline
    }
  }
}

bool          linked()     { return s_last_hb != 0 && (millis() - s_last_hb) < kHeartbeatTimeoutMs; }
unsigned long lastByteMs() { return s_last_byte; }

}  // namespace c5link
