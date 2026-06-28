#include "drone_scan.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// vendored Apache-2.0 decoder (C, with its own extern "C" guards)
#include "odid/opendroneid.h"

namespace drone {
namespace {

constexpr int      kMaxDrones = 12;
constexpr uint32_t kStaleMs   = 30000;  // forget a drone after 30 s of silence
constexpr uint16_t kAstmUuid  = 0xFFFA; // ASTM Remote ID service-data UUID
constexpr uint8_t  kAstmCode  = 0x0D;   // ASTM AD application code (ODID)

constexpr int      kMaxBle    = 24;     // general BLE feed capacity
constexpr uint32_t kBleStale  = 20000;  // forget a BLE sighting after 20 s

struct BleEntry {
  bool          used;
  char          mac[18];
  char          name[20];
  int8_t        rssi;
  uint16_t      company_id;
  bool          has_mfr;
  uint8_t       tracker;       // TrackerType
  unsigned long first_seen;    // set once on new entry; stable within a MAC rotation window
  unsigned long last_seen;
};
BleEntry s_ble[kMaxBle];

// One slot per tracked drone. uas accumulates across adverts because a drone
// sends Basic ID, Location, and System in *separate* advertisements — decoding
// each into the same struct fills the picture in over a second or two.
struct Entry {
  bool          used;
  char          mac[18];
  int8_t        rssi;
  uint8_t       source;   // DroneSource
  unsigned long last_seen;
  ODID_UAS_Data uas;
};

Entry             s_tab[kMaxDrones];
SemaphoreHandle_t s_lock = nullptr;
NimBLEScan*       s_scan = nullptr;

Entry* findOrCreate(const char* mac) {
  int free_idx = -1, oldest_idx = 0;
  unsigned long oldest = 0xFFFFFFFFUL;
  for (int i = 0; i < kMaxDrones; ++i) {
    if (s_tab[i].used && strncmp(s_tab[i].mac, mac, 17) == 0) return &s_tab[i];
    if (!s_tab[i].used && free_idx < 0) free_idx = i;
    if (s_tab[i].used && s_tab[i].last_seen < oldest) { oldest = s_tab[i].last_seen; oldest_idx = i; }
  }
  int idx = (free_idx >= 0) ? free_idx : oldest_idx;  // evict oldest if table full
  Entry& e = s_tab[idx];
  memset(&e, 0, sizeof(e));
  e.used = true;
  strncpy(e.mac, mac, 17);
  e.mac[17] = '\0';
  return &e;
}

// NimBLE-Arduino 2.x callback style.
class ScanCb : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    // --- general BLE sighting: log every advert (before the Remote ID filter) ---
    {
      char m[18];
      std::string ms = dev->getAddress().toString();
      strncpy(m, ms.c_str(), 17); m[17] = '\0';
      std::string nm  = dev->getName();   // passive scan: present only if in the advert itself
      std::string mfr = dev->getManufacturerData();
      bool hasMfr = mfr.size() >= 2;
      uint16_t cid = 0;
      if (hasMfr) cid = (uint16_t)((uint8_t)mfr[0] | ((uint16_t)(uint8_t)mfr[1] << 8));

      if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
      int hit = -1, fi = -1, oi = 0; unsigned long ot = 0xFFFFFFFFUL;
      for (int i = 0; i < kMaxBle; ++i) {
        if (s_ble[i].used && strncmp(s_ble[i].mac, m, 17) == 0) { hit = i; break; }
        if (!s_ble[i].used && fi < 0) fi = i;
        if (s_ble[i].used && s_ble[i].last_seen < ot) { ot = s_ble[i].last_seen; oi = i; }
      }
      int bi = (hit >= 0) ? hit : ((fi >= 0) ? fi : oi);
      BleEntry& be = s_ble[bi];
      if (hit < 0) {
        memset(&be, 0, sizeof(be)); be.used = true;
        strncpy(be.mac, m, 17); be.mac[17] = '\0';
        be.first_seen = millis();
      }
      if (!nm.empty()) { strncpy(be.name, nm.c_str(), sizeof(be.name) - 1); be.name[sizeof(be.name) - 1] = '\0'; }
      be.rssi       = (int8_t)dev->getRSSI();
      be.company_id = cid;
      be.has_mfr    = hasMfr;
      be.last_seen  = millis();

      // Tracker classification
      uint8_t trk = TRK_NONE;
      if (hasMfr && cid == 0x004C && mfr.size() >= 3 && (uint8_t)mfr[2] == 0x12)
        trk = TRK_AIRTAG;
      else if (dev->isAdvertisingService(NimBLEUUID((uint16_t)0xFEED)))
        trk = TRK_TILE;
      else if (dev->isAdvertisingService(NimBLEUUID((uint16_t)0xFD5A)))
        trk = TRK_SMARTTAG;
      be.tracker = trk;

      if (s_lock) xSemaphoreGive(s_lock);


    }

    NimBLEUUID astm((uint16_t)kAstmUuid);
    auto sd = dev->getServiceData(astm);  // std::string/vector; .data()/.size() both work
    if (sd.size() < (size_t)(2 + ODID_MESSAGE_SIZE)) return;  // need app+counter+message
    const uint8_t* p = reinterpret_cast<const uint8_t*>(sd.data());
    if (p[0] != kAstmCode) return;
    const uint8_t* msg = p + 2;  // skip app code + message counter -> 25-byte ODID message/pack

    char mac[18];
    std::string macs = dev->getAddress().toString();
    strncpy(mac, macs.c_str(), 17);
    mac[17] = '\0';

    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    Entry* e = findOrCreate(mac);
    e->rssi = (int8_t)dev->getRSSI();
    e->last_seen = millis();
    e->source = SRC_BLE;
    decodeOpenDroneID(&e->uas, msg);  // sets the relevant *Valid flag(s)
    if (s_lock) xSemaphoreGive(s_lock);
  }
};

ScanCb s_cb;

}  // namespace

void begin() {
  s_lock = xSemaphoreCreateMutex();
  NimBLEDevice::init("");
  s_scan = NimBLEDevice::getScan();
  s_scan->setScanCallbacks(&s_cb, true);   // report every advert so last_seen refreshes and ODID multi-packet builds up
  s_scan->setActiveScan(false);  // Remote ID lives in the advertisement; no scan-response needed
  s_scan->setInterval(160);      // 0.625 ms units -> 100 ms period (faster catch-rate)
  s_scan->setWindow(96);         // listen 60 ms per period (~60% duty) — snappy locator
  s_scan->start(0, false);       // 0 = scan forever
}

void prune() {
  if (!s_lock) return;
  xSemaphoreTake(s_lock, portMAX_DELAY);
  unsigned long now = millis();
  for (int i = 0; i < kMaxDrones; ++i)
    if (s_tab[i].used && (now - s_tab[i].last_seen) > kStaleMs) s_tab[i].used = false;
  for (int i = 0; i < kMaxBle; ++i)
    if (s_ble[i].used && (now - s_ble[i].last_seen) > kBleStale) s_ble[i].used = false;
  xSemaphoreGive(s_lock);
}

size_t snapshot(Drone* out, size_t max) {
  size_t n = 0;
  if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
  for (int i = 0; i < kMaxDrones && n < max; ++i) {
    Entry& e = s_tab[i];
    if (!e.used) continue;
    Drone& d = out[n];
    memset(&d, 0, sizeof(d));
    strncpy(d.mac, e.mac, 17);
    d.rssi = e.rssi;
    d.source = e.source;
    d.last_seen_ms = e.last_seen;
    for (int b = 0; b < ODID_BASIC_ID_MAX_MESSAGES; ++b) {
      if (e.uas.BasicIDValid[b]) {
        strncpy(d.id, e.uas.BasicID[b].UASID, sizeof(d.id) - 1);
        break;
      }
    }
    if (e.uas.LocationValid) {
      d.has_loc   = true;
      d.lat       = e.uas.Location.Latitude;
      d.lon       = e.uas.Location.Longitude;
      d.speed_mps = e.uas.Location.SpeedHorizontal;
      d.dir_deg   = e.uas.Location.Direction;
    }
    if (e.uas.SystemValid) {
      d.has_op = true;
      d.op_lat = e.uas.System.OperatorLatitude;
      d.op_lon = e.uas.System.OperatorLongitude;
    }
    ++n;
  }
  if (s_lock) xSemaphoreGive(s_lock);
  return n;
}

void ingestC5(const char* mac, const char* id, int8_t rssi, uint8_t band,
              bool has_loc, double lat, double lon, float speed, float dir,
              bool has_op, double op_lat, double op_lon) {
  if (!mac || !mac[0]) return;
  if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
  Entry* e = findOrCreate(mac);
  e->rssi = rssi;
  e->last_seen = millis();
  e->source = (band == 5 || band == 2) ? SRC_WIFI_5G : SRC_WIFI_2G;
  if (id && id[0]) {
    strncpy(e->uas.BasicID[0].UASID, id, sizeof(e->uas.BasicID[0].UASID) - 1);
    e->uas.BasicIDValid[0] = 1;
  }
  if (has_loc) {
    e->uas.Location.Latitude        = lat;
    e->uas.Location.Longitude       = lon;
    e->uas.Location.SpeedHorizontal = speed;
    e->uas.Location.Direction       = dir;
    e->uas.LocationValid            = 1;
  }
  if (has_op) {
    e->uas.System.OperatorLatitude  = op_lat;
    e->uas.System.OperatorLongitude = op_lon;
    e->uas.SystemValid              = 1;
  }
  if (s_lock) xSemaphoreGive(s_lock);
}

size_t bleSnapshot(BleSight* out, size_t max) {
  size_t n = 0;
  if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
  for (int i = 0; i < kMaxBle && n < max; ++i) {
    if (!s_ble[i].used) continue;
    BleSight& b = out[n];
    strncpy(b.mac, s_ble[i].mac, sizeof(b.mac));
    strncpy(b.name, s_ble[i].name, sizeof(b.name));
    b.rssi          = s_ble[i].rssi;
    b.company_id    = s_ble[i].company_id;
    b.has_mfr       = s_ble[i].has_mfr;
    b.tracker       = s_ble[i].tracker;
    b.first_seen_ms = s_ble[i].first_seen;
    b.last_seen_ms  = s_ble[i].last_seen;
    ++n;
  }
  if (s_lock) xSemaphoreGive(s_lock);
  return n;
}

size_t bleCount() {
  size_t n = 0; unsigned long now = millis();
  if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
  for (int i = 0; i < kMaxBle; ++i)
    if (s_ble[i].used && now - s_ble[i].last_seen < 30000) ++n;
  if (s_lock) xSemaphoreGive(s_lock);
  return n;
}

size_t trackerCount() {
  size_t n = 0; unsigned long now = millis();
  if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
  for (int i = 0; i < kMaxBle; ++i)
    if (s_ble[i].used && s_ble[i].tracker != TRK_NONE && now - s_ble[i].last_seen < 30000) ++n;
  if (s_lock) xSemaphoreGive(s_lock);
  return n;
}

}  // namespace drone
