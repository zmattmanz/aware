#include "c5_link.h"
#include "drone_scan.h"

#include <Arduino.h>
#include <M5Unified.h>          // M5.getPin() — ask the board for its real Grove GPIOs
#include <cstring>
#include <cstdlib>

#define C5_LINK_DEBUG 1

namespace c5link {
namespace {

constexpr int      kRxFallback = 2;
constexpr uint32_t kBaud  = 115200;
constexpr uint32_t kHeartbeatTimeoutMs = 8000;

HardwareSerial  SerialC5(1);       // UART1
int             s_rxPin     = -1;
int             s_pin1      = -1;
int             s_pin2      = -1;
bool            s_started   = false;
char            s_line[256];
size_t          s_len       = 0;
unsigned long   s_last_hb   = 0;
unsigned long   s_last_byte = 0;

unsigned long s_rx_bytes = 0, s_rx_lines = 0;
unsigned long s_n_h = 0, s_n_r = 0, s_n_w = 0, s_n_d = 0, s_n_other = 0;
unsigned long s_hb_n24 = 0, s_hb_n5 = 0;   // 2.4/5 GHz frame counts from the C5 heartbeat
char          s_last_line[80] = "";

constexpr int kMaxWifi = 24;       // bigger table so 5 GHz APs aren't evicted by the 2.4 flood
WifiSight s_wifi[kMaxWifi] = {};

void ingestWifi(const char* bssid, const char* ssid, int8_t rssi, uint8_t band) {
  int idx = -1, oldest = 0; unsigned long ot = 0xFFFFFFFFUL;
  for (int i = 0; i < kMaxWifi; ++i) {
    if (s_wifi[i].last_seen && strncmp(s_wifi[i].bssid, bssid, 17) == 0) { idx = i; break; }
    if (s_wifi[i].last_seen < ot) { ot = s_wifi[i].last_seen; oldest = i; }
  }
  if (idx < 0) {
    idx = oldest;
    memset(&s_wifi[idx], 0, sizeof(s_wifi[idx]));
    strncpy(s_wifi[idx].bssid, bssid, 17); s_wifi[idx].bssid[17] = '\0';
  }
  strncpy(s_wifi[idx].ssid, ssid ? ssid : "", sizeof(s_wifi[idx].ssid) - 1);
  s_wifi[idx].ssid[sizeof(s_wifi[idx].ssid) - 1] = '\0';
  s_wifi[idx].rssi = rssi; s_wifi[idx].band = band; s_wifi[idx].last_seen = millis();
}

int split(char* src, char* tok[], int maxTok) {
  int n = 0; char* p = src; tok[n++] = p;
  while (*p && n < maxTok) { if (*p == '|') { *p = '\0'; tok[n++] = p + 1; } ++p; }
  return n;
}

void handleLine(char* line) {
#if C5_LINK_DEBUG
  Serial.print("[c5] "); Serial.println(line);
#endif
  s_rx_lines++;
  strncpy(s_last_line, line, sizeof(s_last_line) - 1); s_last_line[sizeof(s_last_line) - 1] = '\0';
  switch (line[0]) {
    case 'H': s_n_h++; break;  case 'R': s_n_r++; break;
    case 'W': s_n_w++; break;  case 'D': s_n_d++; break;  default: s_n_other++;
  }
  if (line[0] == 'H') {                                   // H|planewatch-c5|<ver>[|up=..|hop=../..|n24=..|n5=..]
    s_last_hb = millis();
    const char* p;
    if ((p = strstr(line, "n24=")) != nullptr) s_hb_n24 = strtoul(p + 4, nullptr, 10);
    if ((p = strstr(line, "n5="))  != nullptr) s_hb_n5  = strtoul(p + 3, nullptr, 10);
    return;
  }
  if (line[0] == 'W') {                                   // W|bssid|ssid|rssi|band
    char* t[6]; int n = split(line, t, 6);
    if (n >= 5 && t[1][0]) ingestWifi(t[1], t[2], (int8_t)atoi(t[3]), (uint8_t)atoi(t[4]));
    return;
  }
  if (line[0] != 'R') return;                             // R| = drone detection

  char* t[12]; int n = split(line, t, 12);
  if (n < 5 || !t[1][0]) return;
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
}

}  // namespace

void begin() {
  if (s_started) return;
  s_pin1 = M5.getPin(m5::pin_name_t::port_a_pin1);   // Grove RX line
  s_pin2 = M5.getPin(m5::pin_name_t::port_a_pin2);
  s_rxPin = (s_pin1 >= 0) ? s_pin1 : kRxFallback;
  SerialC5.setRxBufferSize(512);
  SerialC5.begin(kBaud, SERIAL_8N1, s_rxPin, -1);
  s_len = 0;
  s_started = true;
#if C5_LINK_DEBUG
  Serial.printf("[c5] Grove port_a pin1=%d pin2=%d -> using RX=%d @%lu\n",
                s_pin1, s_pin2, s_rxPin, (unsigned long)kBaud);
#endif
}

void poll() {
  if (!s_started) return;
  int budget = 1024;
  while (budget-- > 0 && SerialC5.available() > 0) {
    char c = (char)SerialC5.read();
    s_last_byte = millis();
    s_rx_bytes++;
    if (c == '\n' || c == '\r') {
      if (s_len > 0) { s_line[s_len] = '\0'; handleLine(s_line); s_len = 0; }
    } else if (s_len < sizeof(s_line) - 1) {
      s_line[s_len++] = c;
    } else {
      s_len = 0;
    }
  }
}

bool          linked()     { return s_last_hb != 0 && (millis() - s_last_hb) < kHeartbeatTimeoutMs; }
unsigned long lastByteMs() { return s_last_byte; }
unsigned long frames24()   { return s_hb_n24; }   // cumulative 2.4 GHz frames the C5 has captured
unsigned long frames5()    { return s_hb_n5;  }   // cumulative 5 GHz frames the C5 has captured

void diag(char* out, size_t n) {
  unsigned long age = s_last_byte ? (millis() - s_last_byte) : 0;
  snprintf(out, n,
    "rx=%d(p1=%d,p2=%d) bytes=%luB H=%lu W=%lu R=%lu n24=%lu n5=%lu age=%lums last=\"%s\"",
    s_rxPin, s_pin1, s_pin2, s_rx_bytes, s_n_h, s_n_w, s_n_r, s_hb_n24, s_hb_n5, age, s_last_line);
}

size_t wifiSnapshot(WifiSight* out, size_t max) {
  size_t n = 0; unsigned long now = millis();
  for (int i = 0; i < kMaxWifi && n < max; ++i) {
    if (!s_wifi[i].last_seen || now - s_wifi[i].last_seen > 20000) continue;
    out[n++] = s_wifi[i];
  }
  return n;
}

}  // namespace c5link
