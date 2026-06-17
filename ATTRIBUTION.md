# Attribution

**ADS-B data layer** — `include/services/adsb_client.h`, `src/services/adsb_client.cpp`
taken verbatim from **MatixYo/ESP32-Plane-Radar** (MIT). Aircraft data from the
adsb.fi community feed (https://opendata.adsb.fi/) — free, personal use; cite adsb.fi.

**Remote ID decoder** — `src/odid/opendroneid.{c,h}` vendored from
**opendroneid/opendroneid-core-c** (Apache-2.0; full text in src/odid/LICENSE.opendroneid).

**BLE** — NimBLE-Arduino (h2zero), Apache-2.0.

Everything else (StickS3 port, washer-style UI, screen framework, drone scanner glue)
is new.
