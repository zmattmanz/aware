# Aware ↔ C5 Co-processor Protocol

**Version 1** — UART 115200 8N1, newline-terminated ASCII, C5→StickS3 direction.

## Physical layer

| Signal | C5 pin | Grove wire | StickS3 pin |
|--------|--------|------------|-------------|
| TX (C5→S3) | UART0 TXD | white  | G0 (Serial1 RX) |
| RX (S3→C5) | UART0 RXD | yellow | G26 (Serial1 TX) |
| Power | 5V pad | red | 5V |
| Ground | GND pad | black | GND |

USB CDC on Boot is *disabled* on the C5 (`ARDUINO_USB_CDC_ON_BOOT=0`), so
`Serial` maps to UART0 on the TXD/RXD pads. The C5's own USB port shows
nothing — read the link on the StickS3.

> **Caution:** G0 on the StickS3 is the ESP32-S3 boot strapping pin. It works
> as Serial1 RX but if the C5 drives the line during S3 power-up the S3 can
> misboot / enter download mode. If the link won't come up, power the C5 after
> the S3 has fully booted, or move the RX wire to a plain GPIO and update
> `kRxPin` in `c5_link.cpp`.

## Message format

All lines end with `\n` (LF). Fields are separated by `|`. Pipes within field
values are escaped to `_`. No spaces around pipes.

### `R|` — Remote ID drone detection

```
R|mac|uasid|rssi|band|dlat|dlon|olat|olon|speed|track
```

| Field | Type | Notes |
|-------|------|-------|
| mac | string | WiFi transmitter MAC, lowercase `aa:bb:cc:dd:ee:ff` |
| uasid | string | ODID UAS ID from BasicID message; empty if not yet received |
| rssi | int | Received signal strength, dBm (negative) |
| band | int | `24` = 2.4 GHz, `5` = 5 GHz |
| dlat | float | Drone latitude, degrees (7 decimal places); empty if no Location |
| dlon | float | Drone longitude, degrees (7 decimal places); empty if no Location |
| olat | float | Operator latitude, degrees; empty if no System message |
| olon | float | Operator longitude, degrees; empty if no System message |
| speed | float | Horizontal ground speed, m/s; empty if no Location |
| track | float | Track/heading, degrees true North; empty if no Location |

Empty fields are represented by zero-length strings between pipes:
```
R|aa:bb:cc:dd:ee:ff|HEQ12345678|-65|24|35.1234567|-79.5678901|35.1201234|-79.5699123|3.50|270.0
R|aa:bb:cc:dd:ee:ff||-72|24||||||
```

The C5 re-sends an `R|` line for each drone at most every 3 seconds. The
StickS3 accumulates Location and System data across successive `R|` lines for
the same drone (keyed by UAS ID when available, otherwise by MAC).

### `W|` — WiFi beacon / AP sighting

```
W|bssid|ssid|rssi|band
```

| Field | Type | Notes |
|-------|------|-------|
| bssid | string | AP MAC, lowercase `aa:bb:cc:dd:ee:ff` |
| ssid  | string | Network name; empty if hidden; pipes escaped to `_` |
| rssi  | int   | Received signal strength, dBm (negative) |
| band  | int   | `24` = 2.4 GHz, `5` = 5 GHz |

Emitted for each beacon frame the C5 hears that does not carry an ODID payload.
The StickS3 stores these for up to 20 s and surfaces them on the RF Scan screen
with a `2.4` or `5g` badge.

### `H|` — Heartbeat

```
H|planewatch-c5|<version>
```

Sent once at boot and every 3 seconds. `version` is the integer protocol
version (currently `1`). The StickS3 may use this to show a C5 link-health
indicator. A missing heartbeat for > 8 s indicates the C5 is offline.

## Channels scanned

The C5 hops between ASTM F3411-22 NaN social channels:

| Band | Channel | Frequency | Dwell |
|------|---------|-----------|-------|
| 2.4 GHz | 6 | 2437 MHz | 700 ms |
| 5 GHz | 149 | 5745 MHz | 300 ms |
| 5 GHz | 44 | 5220 MHz | 300 ms |

The C5 decodes OpenDroneID payloads from 802.11 beacon vendor IEs
(Element ID `0xDD`, OUI `FA:0B:BC`, App Code `0x0D`).

NaN action frame parsing (ASTM F3411 Annex C) is not implemented in v1.
Most production drones broadcast via beacons, which covers the common case.

## StickS3 ingest (c5_link module)

`c5_link::begin()` opens Serial1 at 115200 bps (GPIO0 RX, TX unused).
`c5_link::poll()` is called from `loop()`; it drains available bytes and
calls `drone::ingestC5()` for each valid `R|` line.

`drone::ingestC5()` writes directly into the shared drone table (keyed by
MAC), setting the same `uas.*Valid` flags that `snapshot()` reads. C5-sourced
drones appear on the drones screen alongside BLE-sourced ones with no UI
change required.

## Future (not yet implemented)

- StickS3 → C5 commands (e.g. channel lock, heartbeat ack)
- NaN action frame parsing for drones that don't beacon
- On-StickS3 fallback WiFi RID mode (pauses ADS-B fetch, uses S3 radio
  to sniff 2.4 GHz ch 6 directly) — see AGENTS.md for design notes
