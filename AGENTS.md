# Aware — Agent Guide

> This file is the working brief for any coding agent on this repo. Read it
> before changing code. If you're Claude Code, also copy/symlink this to
> `CLAUDE.md`. Keep it current: when you learn something the hard way, add it
> to **Gotchas**.

## What this is

A handheld, screen-cycled **RF/awareness device** for the **M5Stack StickS3**.
It shows what's around the user: aircraft, drones, and (later) a watchlist of
flagged signals. UI language is borrowed from the owner's earlier
"washer detector": a title bar, a content area, and a 3-cell stat strip at the
bottom. Each screen is its own data source feeding that shared chrome.

## Hardware (M5StickS3)

- SoC: **ESP32-S3**-PICO-1-N8R8, dual-core Xtensa LX7 @ 240 MHz
- **8 MB flash + 8 MB PSRAM** — memory is NOT tight; use it (full-screen canvas is fine)
- Radios: **2.4 GHz WiFi + Bluetooth LE 5** (no 5 GHz, no sub-GHz, no 1090 MHz)
- Display: **240×135** landscape (rotation 1), ST7789, driven via M5Unified/M5GFX
- Inputs: **BtnA** (front "M5"), **BtnB** (side). Power button is separate.
- Also onboard but unused so far: **BMI270 6-axis IMU**, mic/speaker (ES8311),
  IR TX/RX, Grove port, Hat2 16-pin header. The Grove/Hat headers are how future
  "ears" (see below) bolt on.

## The one rule that governs everything

**A receiver can only detect what its antenna can hear.** Every feature sorts
into one of three buckets. Do not propose a feature without first placing it:

| Capability | Band | How |
|---|---|---|
| WiFi devices, drone control links | 2.4 GHz | **native** (WiFi promiscuous/scan) |
| BLE devices, **drone Remote ID** | 2.4 GHz BLE | **native** (current `drone_scan`) |
| **Aircraft (ADS-B)** | 1090 MHz | **NOT native** — pulled from an internet API (`adsb_client`), or later a UART 1090 module |
| 5 GHz WiFi | 5 GHz | needs an external **ESP32-C5** dongle over UART |
| Sub-GHz (433/315/915: garage, TPMS) | sub-GHz | needs an external **CC1101** module |

So this is a **mixed-source device**: some screens are real RF, some are
internet data. Be explicit about which any new screen is. The architectural
pattern for non-native bands is "**brain + pluggable ears**" — the StickS3 is
the brain + native 2.4 GHz ear; other bands arrive as decoded messages over
UART from an add-on, mirroring how `adsb_client` ingests JSON.

## Architecture

- **Data/render split.** Each data module owns acquisition and exposes a plain
  data snapshot; `main.cpp` owns all rendering. Never put drawing in a data
  module, never put acquisition in `main.cpp`.
- **Chrome contract.** Every screen calls `drawTopBar(title)` + content +
  `drawBottomBar(c1,c2,c3)` into the global `M5Canvas cv`, then `render()`
  pushes the sprite once. Draw into `cv`, never directly to `M5.Display`, so we
  stay flicker-free.
- **Screen framework.** `enum Screen` + `g_screen`; `BtnA` cycles. Add a screen
  by adding an enum value, a `drawXxxScreen()`, and a `case` in `render()`.
- **Button scheme (keep consistent):** `BtnA` = cycle screens. `BtnB` click =
  cycle the focused item on the current screen. `BtnB` hold = secondary action
  (currently: cycle plane radar range). Use `wasClicked()`/`wasHold()` on BtnB
  (NOT `wasPressed()`) so a hold doesn't also fire the click action.

### Current modules

- `services/adsb_client.{h,cpp}` — **VENDORED, do not edit.** ADS-B over WiFi
  from adsb.fi. From `MatixYo/ESP32-Plane-Radar` (MIT).
- `src/odid/opendroneid.{c,h}` — **VENDORED, do not edit.** OpenDroneID message
  decoder from `opendroneid/opendroneid-core-c` (Apache-2.0).
- `drone_scan.{h,cpp}` — **ours.** Passive NimBLE scan → finds ASTM Remote ID
  service data (UUID `0xFFFA`, app code `0x0D`) → feeds the 25-byte message to
  the vendored decoder → accumulates per-MAC into a mutex-guarded table →
  `snapshot()` for the UI. Runs continuously on the NimBLE task.
- `main.cpp` — chrome, screen framework, plane radar + drone screens.
- `config.h` — two namespaces: `config::` (consumed by vendored `adsb_client`,
  do not rename its keys) and `appcfg::` (our WiFi creds, home lat/lon, ranges).

## Build, flash, verify

```
pio run -t upload          # build + flash
pio device monitor         # 115200 baud; watch "adsb: N aircraft" etc.
```

- First build pulls the ESP32 platform + libs; it churns for a few minutes once.
- If upload can't connect: hold the **side reset button** until the green LED
  flashes (download mode), release, re-run upload.
- Flashed but screen dark / green LED blinking: hold the **left (power) button**
  ~6 s, then a quick press.
- **You usually cannot test on hardware yourself.** Get it compiling cleanly
  (`pio run`) and reason carefully; the owner flashes and reports back.
- **To test the drone screen without a drone:** flash a second ESP32 with an
  OpenDroneID *transmitter/spoofer* sketch (search "OpenDroneID ESP32
  transmitter" / `RemoteIDSpoofer`). It broadcasts fake drones the screen
  should pick up.

## Conventions

- **UI palette** is defined once at the top of `main.cpp` (`COL_*`, via
  `rgb565()`). Reuse it; don't invent new colors per screen.
- **Layout constants** (`TOPBAR_H`, `BOTBAR_H`, `CONTENT_Y/H`, `W=240 H=135`)
  are the source of truth. Respect them.
- **Threading:** the NimBLE scan callback runs on the BLE host task, the UI on
  the main loop task. Any shared state between them MUST be taken under
  `s_lock` (see `drone_scan.cpp`). Don't read the drone table without the
  mutex; use `snapshot()`.
- **Memory:** PSRAM is available (8 MB). Bounded fixed-size tables are still
  preferred over dynamic growth for predictability, but you are not fighting for
  KBs like on a no-PSRAM board.
- **Includes:** any `.cpp` that calls Arduino functions (`millis()`,
  `delay()`, etc.) must `#include <Arduino.h>` itself — don't rely on transitive
  includes. (This bit us once; see Gotchas.)
- **Prose code style:** small focused functions, comments explain *why* not
  *what*, match the existing terse style.

## Gotchas (real ones we hit — append as you find more)

1. **`millis()` not declared** in a TU → that file is missing `#include
   <Arduino.h>`. Files that include `M5Unified.h` or `HTTPClient.h` get it for
   free; standalone modules don't.
2. **NimBLE API drift.** Code targets **NimBLE-Arduino 2.x** (`NimBLEScanCallbacks`,
   `setScanCallbacks`, `getServiceData(uuid)`). On 1.x the callback base class
   and signatures differ. If `drone_scan.cpp` errors, suspect this first.
3. **WiFi is 2.4 GHz only.** The S3 can't join a 5 GHz network. The most common
   "it does nothing" cause is wrong creds OR a 5 GHz-only SSID. Check the wifi
   dot color (top-right) and serial before debugging code.
4. **adsb.fi access.** The geographic (`/lat/lon/dist/`) endpoint may be gated
   to contributors who run a receiver; a plain home IP can get 401/403/429.
   Symptom: WiFi dot green but planes never appear. If confirmed, switch
   `kApiBase` in `adsb_client.cpp` to a format-compatible open feed
   (adsb.lol / api.adsb.one — all ADSBexchange-v2 compatible, so the JSON parse
   is unchanged). Verify the exact path before changing.
5. **Drone screen is BLE-4 legacy only** right now. BT5 long-range (coded PHY,
   extended advertising) drones won't appear until ext-adv scanning is enabled.
6. **WiFi Remote ID vs. plane feed conflict.** Sniffing WiFi-transport Remote ID
   needs promiscuous mode, which can't coexist with WiFi STA used by the plane
   feed. BLE Remote ID has no such conflict (separate path). Any WiFi-RID screen
   must pause the plane fetch.
7. **Plane selection is by rank, not identity.** `BtnB` selects the Nth-nearest;
   across 5 s refreshes the specific aircraft under the cursor can change. To
   lock onto one plane, track by ICAO hex across fetches (not yet done).

## Roadmap (rough priority)

1. **FLAGGED watchlist screen** — let the user flag a BLE/WiFi device from a
   feed; persist to flash (LittleFS/NVS); alert on re-sighting. This is the
   first screen that needs a generic `Contact` model + a `classify()` step.
2. **BT5 extended advertising** in `drone_scan` — `CONFIG_BT_NIMBLE_EXT_ADV` +
   ext-adv scan path, to catch long-range Remote ID.
3. **Track-by-hex** for plane selection (Gotcha 7).
4. **WiFi-transport Remote ID** (mode that pauses the plane fetch; Gotcha 6).
5. **IMU use** (BMI270) — e.g. tilt-to-scroll, or a compass-stabilized radar.
6. **New "ears"** if the owner wants true-RF planes (1090 UART module) or
   sub-GHz (CC1101): add a UART ingest module shaped like `adsb_client`, emit
   into the same screen/chrome.

## Guardrails (don't)

- Don't edit anything under `src/odid/` or the `adsb_client` files — they're
  vendored. Wrap/extend instead, and keep `ATTRIBUTION.md` + the license file.
- Don't move drawing into data modules or acquisition into `main.cpp`.
- Don't access the drone table without `s_lock`.
- Don't claim a band is detectable that the radio can't hear (see The Rule).
- Don't use browser storage idioms; this is bare-metal Arduino/ESP-IDF.
