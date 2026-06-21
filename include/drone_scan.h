#pragma once
#include <cstddef>
#include <cstdint>

// Passive BLE scanner for ASTM F3411 / OpenDroneID Remote ID broadcasts.
// Runs continuously on its own NimBLE task; coexists with WiFi STA.
// Decodes Basic ID / Location / System messages via the vendored
// opendroneid-core-c decoder and exposes a flat snapshot to the UI.

namespace drone {

enum DroneSource  : uint8_t { SRC_BLE = 0, SRC_WIFI_2G = 1, SRC_WIFI_5G = 2 };
enum TrackerType  : uint8_t { TRK_NONE = 0, TRK_AIRTAG, TRK_TILE, TRK_SMARTTAG };

struct Drone {
  char          mac[18];
  char          id[21];        // UAS ID (serial/registration); empty until a Basic ID arrives
  int8_t        rssi;
  uint8_t       source;        // DroneSource: which radio heard this contact
  bool          has_loc;
  double        lat, lon;      // drone position (deg)
  float         speed_mps;     // horizontal ground speed
  float         dir_deg;       // track, deg from true north
  bool          has_op;
  double        op_lat, op_lon;  // operator/pilot position (deg)
  unsigned long last_seen_ms;
};

void   begin();                            // init NimBLE + start forever-scan
void   prune();                            // drop stale entries; call from loop()
size_t snapshot(Drone* out, size_t max);   // copy active drones under lock -> count

// Feed a WiFi Remote ID detection from the C5 co-processor into the shared
// table. Keyed by MAC; fields accumulate across calls just like the BLE path.
void ingestC5(const char* mac, const char* id, int8_t rssi, uint8_t band,
              bool has_loc, double lat, double lon, float speed, float dir,
              bool has_op, double op_lat, double op_lon);

// General BLE feed: every advertising device seen recently (not just drones),
// for the live RF-scan screen.
struct BleSight {
  char          mac[18];
  char          name[20];
  int8_t        rssi;
  uint16_t      company_id;
  bool          has_mfr;
  uint8_t       tracker;           // TrackerType — TRK_NONE if not a known tracker
  unsigned long first_seen_ms;     // when this MAC was first logged (stable within a rotation window)
  unsigned long last_seen_ms;
};
size_t bleSnapshot(BleSight* out, size_t max);
size_t bleCount();      // total recent BLE devices (any kind)
size_t trackerCount();  // recent devices classified as a known tracker

}  // namespace drone
