#pragma once
#include <cstdint>
#include <cstddef>

// Receives the ESP32-C5 WiFi Remote ID sniffer's UART stream (see PROTOCOL.md)
// and merges decoded drones into the shared drone table (drone_scan).
namespace c5link {
void          begin();        // open the UART link from the C5 (call once, from loop — NOT setup)
void          poll();         // call every loop(): drains UART, parses, feeds table
bool          linked();       // true if a C5 heartbeat arrived within the last 8 s
unsigned long lastByteMs();   // millis() of last byte received (0 = never) — diagnostics

// WiFi beacon sightings reported by the C5 (W| lines)
struct WifiSight {
  char          bssid[18];
  char          ssid[24];
  int8_t        rssi;
  uint8_t       band;          // 24 = 2.4 GHz, 5 = 5 GHz
  unsigned long last_seen;
};
size_t wifiSnapshot(WifiSight* out, size_t max);  // non-stale C5 APs -> count
void   diag(char* out, size_t n);                 // one-line link status for Serial debug

}  // namespace c5link
