# Aware C5 firmware

WiFi Remote ID sniffer for the **ESP32-C5** dongle. Separate firmware from the
StickS3 — flash this to the C5.

- Passive, receive-only. Sniffs OpenDroneID over WiFi (beacon + NaN) on 2.4 and
  5 GHz, decodes with vendored opendroneid-core-c, and streams `R|` lines to the
  StickS3 over UART (Serial0 / the TXD/RXD pads), 115200 8N1.
- Skeleton adapted from Plume's `c5-sniffer`; reuse your Plume C5 build env if
  the platformio.ini board/platform doesn't resolve (C5 Arduino support is new).
- Wire: C5 TXD -> StickS3 Grove RX, common GND, 3.3 V. See repo PROTOCOL.md.

First cut — untested on hardware. Most likely to need tuning: beacon IE offset
math, NaN channel dwell, and the exact UART pins for your wiring.
