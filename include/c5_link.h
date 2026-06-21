#pragma once
#include <cstdint>
#include <cstddef>

namespace c5link {
void          begin();        // open + auto-hunt the Grove RX pin (call once, from loop)
void          poll();         // call every loop(): drains UART, parses, feeds table
bool          linked();       // true if a C5 heartbeat arrived within the last 8 s
unsigned long lastByteMs();
unsigned long frames24();     // C5's 2.4 GHz frame count (from heartbeat)
unsigned long frames5();      // C5's 5 GHz frame count (from heartbeat)
unsigned long clients();      // distinct probe-request MACs seen in last 60 s (RF busyness)
int           rxPin();        // Grove GPIO currently being listened on
unsigned long rxBytes();      // total bytes received (>0 means the wire is good)

struct WifiSight {
  char          bssid[18];
  char          ssid[24];
  int8_t        rssi;
  uint8_t       band;          // 24 = 2.4 GHz, 5 = 5 GHz
  unsigned long last_seen;
};
size_t wifiSnapshot(WifiSight* out, size_t max);
void   diag(char* out, size_t n);

}  // namespace c5link
