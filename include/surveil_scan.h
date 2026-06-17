#pragma once
#include <cstddef>
#include <cstdint>

// Passive RF surveillance detector: Flock Safety cameras / Raven audio sensors,
// Axon body cameras. Receive-only — observe broadcasts, identify, display.
// Never connects, associates, probes, or interferes with any device.
//
// Feed from two sources:
//   ingestBle()  — called from the drone_scan NimBLE callback (shared radio)
//   ingestWifi() — called after periodic WiFi.scanNetworks() in loop()

namespace surveil {

enum Kind : uint8_t { FLOCK_CAM, FLOCK_RAVEN, AXON_CAM, GENERIC_LE };

struct Detection {
  char     label[20];    // human-readable match label (≤19 chars)
  uint8_t  mac[6];       // MAC in presentation order (OUI first)
  int8_t   rssi;         // strongest RSSI seen
  Kind     kind;
  uint8_t  confidence;   // 0–100; OUI-only=LOW(20), SSID=HIGH(80)
  uint32_t last_ms;      // millis() of last observation
};

constexpr int kMaxDetections = 16;

void   begin();
void   prune();                                   // drop entries silent >60 s; call from loop()
size_t snapshot(Detection* out, size_t max);      // copy active detections under lock

// Feed points ----------------------------------------------------------------
// Called from the NimBLE scan callback (BLE task) — must be lock-safe.
void ingestBle(const char*    name,        // device advertised name (may be nullptr/"")
               const uint8_t* mac6,        // 6 bytes, presentation order (OUI first)
               int8_t         rssi,
               uint16_t       companyId,   // manufacturer data company ID (LE byte order)
               bool           hasCompanyId);

// Called from loop() after WiFi.scanNetworks() completes.
// bssid6: 6 bytes, network byte order (OUI first), same convention as mac6 above.
void ingestWifi(const char* ssid, const uint8_t* bssid6, int8_t rssi);

}  // namespace surveil
