// ===========================================================================
// Aware C5 — WiFi Remote ID sniffer (ESP32-C5)
//
// The "WiFi ear." Passive, receive-only promiscuous capture of WiFi-transport
// OpenDroneID (ASTM F3411) on BOTH 2.4 GHz and 5 GHz, decoded with the vendored
// opendroneid-core-c, reported to the StickS3 (the brain) over UART. The
// StickS3 keeps its own WiFi on the plane feed; this chip is the second radio,
// so WiFi Remote ID and the ADS-B plane screen run at the same time.
//
// Skeleton adapted from Plume's c5-sniffer.ino (promiscuous sniffer + channel
// hop + UART reporter + heartbeat). The detection logic is replaced: instead of
// Flock SSID/OUI signatures it extracts and decodes OpenDroneID from:
//   - beacon frames  : vendor IE 221, OUI FA:0B:BC, type 0x0D  (hand-parsed)
//   - NaN action frames : via odid_wifi_receive_message_pack_nan_action_frame()
//
// Wire protocol (see repo PROTOCOL.md), C5 -> StickS3, 115200 8N1, '\n' ASCII:
//   R|mac|uasid|rssi|band|dlat|dlon|oplat|oplon|speed|track
//   H|planewatch-c5|<ver>            heartbeat every ~3 s
// Empty fields are sent as nothing between pipes (RID arrives in pieces; the
// brain accumulates). Append-only: new fields go on the END of the R| line.
// ===========================================================================

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"

extern "C" {
#include "odid/opendroneid.h"
#include "odid/odid_wifi.h"
}

// ---- link / config ---------------------------------------------------------
#define PROTOCOL_VERSION 1
#define LINK_BAUD        115200
#define HEARTBEAT_MS     3000
#define CHANNEL_DWELL_MS 350
// UART0 (the TXD/RXD pads) is the link to the StickS3 — same as Plume.
// On the ESP32-C5 Arduino core, `Serial` maps to UART0. We deliberately do NOT
// enable USB-CDC-on-boot (it mis-maps to a nonexistent USBSerial on this chip).
// NOTE: output goes out the UART pads, NOT the USB port — to watch it, read it
// on the StickS3 (or hang a USB-TTL adapter on the TX pad). `pio device monitor`
// over the C5's own USB will show nothing. This matches how Plume's C5 worked.
#define LinkSerial Serial

// Dual-band hop list. band: 1 = 2.4 GHz, 2 = 5 GHz.
// ch6 (2.4) and ch149/44 (5) are the NaN/Wi-Fi-Aware social channels — dwell
// there matters for NaN. Beacons appear on the drone SoftAP's channel.
struct Hop { uint8_t band; uint8_t ch; };
static const Hop kHops[] = {
  {1, 1}, {1, 6}, {1, 11},
  {2, 149}, {2, 44}, {2, 36},
};
static const int kHopCount = sizeof(kHops) / sizeof(kHops[0]);

// ---- per-MAC accumulation (RID arrives one message type per frame) ---------
struct Slot {
  bool          used;
  uint8_t       mac[6];
  int8_t        rssi;
  uint8_t       band;     // 1 or 2
  ODID_UAS_Data uas;
  bool          dirty;    // new data since last emit
  uint32_t      last;
};
static const int kSlots = 16;
static Slot s_slots[kSlots];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static Slot* slot_for(const uint8_t* mac) {
  int freeIdx = -1, oldest = 0; uint32_t oldT = 0xFFFFFFFF;
  for (int i = 0; i < kSlots; ++i) {
    if (s_slots[i].used && memcmp(s_slots[i].mac, mac, 6) == 0) return &s_slots[i];
    if (!s_slots[i].used && freeIdx < 0) freeIdx = i;
    if (s_slots[i].used && s_slots[i].last < oldT) { oldT = s_slots[i].last; oldest = i; }
  }
  int idx = (freeIdx >= 0) ? freeIdx : oldest;
  Slot& s = s_slots[idx];
  memset(&s, 0, sizeof(s));
  s.used = true;
  memcpy(s.mac, mac, 6);
  return &s;
}

// merge any newly-valid sub-messages from `src` into the persistent slot
static void merge_uas(ODID_UAS_Data* dst, const ODID_UAS_Data* src) {
  for (int i = 0; i < ODID_BASIC_ID_MAX_MESSAGES; ++i)
    if (src->BasicIDValid[i]) { dst->BasicID[i] = src->BasicID[i]; dst->BasicIDValid[i] = 1; }
  if (src->LocationValid) { dst->Location = src->Location; dst->LocationValid = 1; }
  if (src->SystemValid)   { dst->System   = src->System;   dst->SystemValid = 1; }
  if (src->SelfIDValid)   { dst->SelfID   = src->SelfID;   dst->SelfIDValid = 1; }
  if (src->OperatorIDValid){ dst->OperatorID = src->OperatorID; dst->OperatorIDValid = 1; }
}

static void store(const uint8_t* mac, const ODID_UAS_Data* tmp, int8_t rssi, uint8_t band) {
  portENTER_CRITICAL(&s_mux);
  Slot* s = slot_for(mac);
  merge_uas(&s->uas, tmp);
  s->rssi = rssi; s->band = band; s->dirty = true; s->last = millis();
  portEXIT_CRITICAL(&s_mux);
}

// ---- beacon vendor-IE OpenDroneID extraction -------------------------------
// Returns true and fills `out_msg` ptr/len to the OpenDroneID message-pack bytes
// if the beacon carries the ASD-STAN vendor IE (221 / OUI FA:0B:BC / type 0x0D).
static bool beacon_odid_payload(const uint8_t* f, int len, const uint8_t** out, int* out_len) {
  // 802.11 mgmt header (24) + beacon fixed params (timestamp8 + interval2 + cap2 = 12)
  int off = 24 + 12;
  while (off + 2 <= len) {
    uint8_t eid = f[off], elen = f[off + 1];
    if (off + 2 + elen > len) break;
    if (eid == 0xDD && elen >= 4 &&
        f[off + 2] == 0xFA && f[off + 3] == 0x0B && f[off + 4] == 0xBC && f[off + 5] == 0x0D) {
      *out = f + off + 6;          // bytes after OUI(3)+type(1) = ODID message pack
      *out_len = elen - 4;
      return true;
    }
    off += 2 + elen;
  }
  return false;
}

// ---- promiscuous callback (WiFi task — keep it lean) -----------------------
static void sniffer_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t* pkt = (const wifi_promiscuous_pkt_t*)buf;
  const uint8_t* f = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;
  if (len < 28) return;
  int8_t rssi = pkt->rx_ctrl.rssi;
  uint8_t band = (pkt->rx_ctrl.channel > 14) ? 2 : 1;

  uint8_t subtype = (f[0] >> 4) & 0x0F;  // ftype is mgmt (00) here
  ODID_UAS_Data tmp;
  memset(&tmp, 0, sizeof(tmp));

  if (subtype == 8) {  // beacon
    const uint8_t* pack; int plen;
    if (!beacon_odid_payload(f, len, &pack, &plen)) return;
    // decodeOpenDroneID handles single messages and message packs (type 0xF)
    decodeOpenDroneID(&tmp, pack);
    const uint8_t* sa = f + 10;  // SA is addr2 at offset 10 in mgmt header
    store(sa, &tmp, rssi, band);
  } else if (subtype == 13) {  // action frame -> try NaN
    char mac[6];
    int r = odid_wifi_receive_message_pack_nan_action_frame(&tmp, mac, f, (size_t)len);
    if (r == 0) store((const uint8_t*)mac, &tmp, rssi, band);
  }
}

// ---- channel hopping (with band switch) ------------------------------------
static int g_hop = 0;
static uint32_t g_last_hop = 0;
static void hop() {
  static uint8_t cur_band = 0;
  const Hop& h = kHops[g_hop];
  if (h.band != cur_band) {
    esp_wifi_set_band_mode(h.band == 2 ? WIFI_BAND_MODE_5G_ONLY : WIFI_BAND_MODE_2G_ONLY);
    cur_band = h.band;
  }
  esp_wifi_set_channel(h.ch, WIFI_SECOND_CHAN_NONE);
  g_hop = (g_hop + 1) % kHopCount;
}

// ---- emit one R| line from a slot ------------------------------------------
static void emit(const Slot& s) {
  char id[ODID_ID_SIZE + 1] = "";
  for (int i = 0; i < ODID_BASIC_ID_MAX_MESSAGES; ++i)
    if (s.uas.BasicIDValid[i]) { strncpy(id, s.uas.BasicID[i].UASID, ODID_ID_SIZE); break; }

  char dlat[16] = "", dlon[16] = "", oplat[16] = "", oplon[16] = "", spd[12] = "", trk[12] = "";
  if (s.uas.LocationValid) {
    snprintf(dlat, sizeof(dlat), "%.6f", s.uas.Location.Latitude);
    snprintf(dlon, sizeof(dlon), "%.6f", s.uas.Location.Longitude);
    snprintf(spd,  sizeof(spd),  "%.1f", s.uas.Location.SpeedHorizontal);
    snprintf(trk,  sizeof(trk),  "%.0f", s.uas.Location.Direction);
  }
  if (s.uas.SystemValid) {
    snprintf(oplat, sizeof(oplat), "%.6f", s.uas.System.OperatorLatitude);
    snprintf(oplon, sizeof(oplon), "%.6f", s.uas.System.OperatorLongitude);
  }
  int bandLabel = (s.band == 2) ? 5 : 24;
  LinkSerial.printf("R|%02x:%02x:%02x:%02x:%02x:%02x|%s|%d|%d|%s|%s|%s|%s|%s|%s\n",
                    s.mac[0], s.mac[1], s.mac[2], s.mac[3], s.mac[4], s.mac[5],
                    id, s.rssi, bandLabel, dlat, dlon, oplat, oplon, spd, trk);
}

// ---------------------------------------------------------------------------
void setup() {
  LinkSerial.begin(LINK_BAUD, SERIAL_8N1);  // UART0 pads -> StickS3
  delay(200);

  WiFi.mode(WIFI_MODE_NULL);                 // bring up the driver without associating
  esp_wifi_set_ps(WIFI_PS_NONE);
  wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT };
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&sniffer_cb);
  hop();  // set first channel/band
}

void loop() {
  uint32_t now = millis();

  if (now - g_last_hop >= CHANNEL_DWELL_MS) { hop(); g_last_hop = now; }

  // flush any dirty slots as R| lines (UART writes happen here, not in the cb)
  for (int i = 0; i < kSlots; ++i) {
    Slot snap; bool go = false;
    portENTER_CRITICAL(&s_mux);
    if (s_slots[i].used && s_slots[i].dirty) { snap = s_slots[i]; s_slots[i].dirty = false; go = true; }
    portEXIT_CRITICAL(&s_mux);
    if (go) emit(snap);
  }

  static uint32_t last_hb = 0;
  if (now - last_hb >= HEARTBEAT_MS) {
    last_hb = now;
    LinkSerial.printf("H|planewatch-c5|%d\n", PROTOCOL_VERSION);
  }

  // expire slots silent for >30 s so a landed drone stops being reported
  portENTER_CRITICAL(&s_mux);
  for (int i = 0; i < kSlots; ++i)
    if (s_slots[i].used && now - s_slots[i].last > 30000) s_slots[i].used = false;
  portEXIT_CRITICAL(&s_mux);

  delay(2);
}
