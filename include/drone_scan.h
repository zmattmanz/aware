#pragma once
#include <cstddef>
#include <cstdint>

// Passive BLE scanner for ASTM F3411 / OpenDroneID Remote ID broadcasts.
// Runs continuously on its own NimBLE task; coexists with WiFi STA.
// Decodes Basic ID / Location / System messages via the vendored
// opendroneid-core-c decoder and exposes a flat snapshot to the UI.

namespace drone {

struct Drone {
  char          mac[18];
  char          id[21];        // UAS ID (serial/registration); empty until a Basic ID arrives
  int8_t        rssi;
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

// General BLE feed: every advertising device seen recently (not just drones),
// for the live RF-scan screen.
struct BleSight {
  char          mac[18];
  char          name[20];      // empty if the device advertises no name
  int8_t        rssi;
  uint16_t      company_id;    // BLE SIG company ID from manufacturer data (0 if none)
  bool          has_mfr;       // true if the advert carried manufacturer data
  unsigned long last_seen_ms;
};
size_t bleSnapshot(BleSight* out, size_t max);  // recent BLE devices -> count

}  // namespace drone
