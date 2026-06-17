# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and flash

```bash
pio run               # compile only (use this to verify before touching hardware)
pio run -t upload     # build + flash to device (auto-detects COM port)
pio device monitor    # serial output at 115200 baud
```

First build pulls the ESP32 platform and libs — takes a few minutes once. If upload hangs, hold the **side reset button** until the green LED flashes (download mode), then re-run.

Config to edit before first flash: `include/config.h` — WiFi SSID/password. Home lat/lon defaults to Hillsborough, NC.

## Architecture in one paragraph

`main.cpp` is the UI shell: it owns every screen render, the button dispatch loop, the animation tick, and the FreeRTOS fetch task for ADS-B. Data modules (`drone_scan`, `adsb_client`, and future `c5_link`) only acquire and expose data — they never draw. Every render goes into a full-screen `M5Canvas cv` (sprite), then `cv.pushSprite(0,0)` commits it once to avoid flicker. Screens are an `enum Screen` + a `switch` in `render()`; adding one means adding an enum value, a `drawXxxScreen()`, and a case.

## The one constraint that governs every feature

**A receiver can only detect what its antenna can hear.** Before adding a capability, identify which bucket it falls in:

| Signal | How it arrives |
|---|---|
| ADS-B (aircraft) | Internet API (`adsb_client`) — NOT native RF |
| BLE Remote ID (drones) | Native BLE — `drone_scan` |
| WiFi Remote ID (drones) | Needs promiscuous mode; conflicts with WiFi STA. Use the C5 dongle over UART, or a mode-toggle that drops the plane feed |
| 5 GHz / sub-GHz | Needs external hardware (C5 dongle, CC1101, etc.) |

## Threading model

Two tasks share state:
- **Main task (core 1):** button loop, animation tick, all rendering, drone prune, idle render.
- **ADS-B fetch task (core 0):** calls `fetchUpdate()` (blocking HTTP), then briefly takes `g_aircraft_mutex` to `memcpy` into `g_local_aircraft[]`. Animation tick takes the same mutex non-blocking each frame; if locked, it skips that tick.
- **NimBLE task (implicit):** fires `ScanCb::onResult()` for every BLE advertisement. Takes `s_lock` to update the drone table and the BLE feed table. Never draws.

Rule: **any access to `s_tab[]` or `s_feed[]` in `drone_scan` requires `s_lock`.** Use `snapshot()` / `feedSnapshot()` to read from the main task.

## Chrome contract (every screen must follow this)

```cpp
drawTopBar("TITLE");
// ... draw content into cv ...
drawBottomBar(left, center, right);
// render() calls cv.pushSprite(0,0) — never call it yourself
```

Draw into `cv` (the global `M5Canvas`), never directly to `M5.Display`. Layout constants `TOPBAR_H=17`, `BOTBAR_H=19`, `CONTENT_Y=18`, `CONTENT_H=97`, `W=240`, `H=135` are the source of truth.

## Button scheme

- `BtnA.wasPressed()` → cycle screens.
- `BtnB.wasClicked()` → cycle focused item on the current screen.
- `BtnB.wasHold()` → secondary action (planes: cycle range; others: TBD).
- **Never use `wasPressed()` on BtnB** — a hold fires `wasClicked()` too, so always use `wasClicked()`/`wasHold()` to distinguish.

## Palette and colors

All colors defined as `COL_*` constants at the top of `main.cpp` via `rgb565()`. Reuse them; don't add per-screen colors. Key ones: `COL_PLANE` (red blips), `COL_DRONE` (orange), `COL_BAD` (emergency/error red), `COL_OK` (green), `COL_HEAD` (yellow label), `COL_SEL_DIM` (muted ring accent).

## Vendored files — do not edit

- `src/odid/opendroneid.{c,h}` — OpenDroneID decoder (Apache-2.0).
- `src/services/adsb_client.{h,cpp}` — originally from MatixYo/ESP32-Plane-Radar (MIT); now adapted (hex/reg/squawk fields added). Treat as semi-vendored: the original MIT header must stay, and `ATTRIBUTION.md` must reflect any further changes.

## IMU (BMI270) API — verified from M5Unified source

```cpp
M5.Imu.isEnabled()               // true after M5.begin()
M5.Imu.getAccel(&ax, &ay, &az)  // floats in g; M5.update() already calls Imu.update()
```

The auto-rotate feature in `loop()` uses `ax` with ±0.4 g threshold and 750 ms debounce. Serial prints `IMU ax/ay/az` every 500 ms to aid calibration — the owner must verify axis sign on hardware.

## Gotchas log (append when you hit new ones)

1. Any `.cpp` that calls `millis()`/`delay()` without including `M5Unified.h` or `HTTPClient.h` must `#include <Arduino.h>` explicitly.
2. NimBLE targets **2.x** API (`NimBLEScanCallbacks`, `setScanCallbacks`, `getServiceData(uuid)`). 1.x differs.
3. WiFi is 2.4 GHz only; 5 GHz SSIDs silently fail. Check the WiFi dot color first.
4. adsb.fi geographic endpoint may 401/403 on plain home IPs. Fallback: `adsb.lol` or `api.adsb.one` (same ADSBexchange-v2 JSON format — no parse changes needed).
5. Plane selection is by rank (distance order), not identity. Track by ICAO hex across fetches to lock onto one plane.
6. WiFi promiscuous + WiFi STA cannot coexist — sniffing WiFi RID on the StickS3 pauses the plane feed. Use the C5 dongle to avoid this.
7. `g_fetch_task_handle` must be non-null before calling `xTaskNotify` (set in `setup()` — safe after that).
