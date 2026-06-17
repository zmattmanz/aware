# Aware

Screen-cycled RF/awareness device for the **M5Stack StickS3** (ESP32-S3, 240x135),
washer-detector UI language.

## Screens
- **PLANES**  — live ADS-B aircraft over WiFi (radar + selected-aircraft panel)
- **DRONES**  — passive BLE Remote ID (OpenDroneID); shows UAS ID, RSSI, and—when
                broadcast—drone position and **pilot location**. Always scanning.
- **FLAGGED** — stub (watchlist), next slice

## Controls
- **BtnA**: cycle screens
- **BtnB** (click): cycle the focused plane / drone
- **BtnB** (hold): on PLANES, cycle radar range (5/10/15/25 km)

## Build / flash
1. Edit `include/config.h` — WiFi SSID/password (home coords already set).
2. `pio run -t upload`
3. `pio device monitor`  (115200)

See ATTRIBUTION.md for vendored code + licenses.
