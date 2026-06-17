# Aware ↔ C5 Co-processor Protocol

**Version 1** — UART 115200 8N1, newline-terminated ASCII, C5→StickS3 direction.

## Physical layer

| Signal | C5 pin | Grove wire | StickS3 pin |
|--------|--------|------------|-------------|
| TX (C5→S3) | UART0 TXD | white  | G0 (Serial1 RX) |
| RX (S3→C5) | UART0 RXD | yellow | G26 (Serial1 TX) |
| Power | 5V pad | red | 5V |
| Ground | GND pad | black | GND |

The C5's debug console is on its USB-C port (native USB-Serial/JTAG). USB CDC
on Boot must be enabled so UART0 pads are free for the Grove link.

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

`c5_link::begin()` opens Serial1 at 115200 bps (GPIO0 RX, GPIO26 TX).
`c5_link::poll()` is called from `loop()`; it drains available bytes and
calls `drone::ingestExternal()` for each valid `R|` line.

`drone::ingestExternal()` searches the drone table by UAS ID first (to
merge WiFi and BLE sightings of the same drone), falls back to MAC, and
marks the entry with `SRC_WIFI_C5` in its source bitmask.

## Future (not yet implemented)

- StickS3 → C5 commands (e.g. channel lock, heartbeat ack)
- NaN action frame parsing for drones that don't beacon
- On-StickS3 fallback WiFi RID mode (pauses ADS-B fetch, uses S3 radio
  to sniff 2.4 GHz ch 6 directly) — see AGENTS.md for design notes
