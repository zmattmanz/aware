#pragma once

#include <cstddef>

namespace services::adsb {

struct Aircraft {
  float lat;
  float lon;
  float nose_deg;
  float track_deg;
  float gs_knots;
  char  callsign[9];
  char  type[5];
  char  alt[12];
  char  squawk[5];   // transponder code e.g. "7700"; "" if absent
  int   alt_ft;      // barometric altitude in feet; INT_MIN if unknown/ground
  bool  emergency;   // squawk 7500/7600/7700, or non-"none" emergency field
  bool  mil;         // military (adsb.fi dbFlags bit 0)
};

constexpr size_t kMaxAircraft = 64;

size_t          aircraftCount();
const Aircraft* aircraftList();
unsigned long   lastOkMs();   // millis() of last successful fetch (0 = never)

void begin();                                                      // start background fetch task
void setCenter(double lat, double lon, float radius_km);           // update fetch params
inline void setPollFn(void (*)()) {}                               // no-op kept for ABI compat

}  // namespace services::adsb
