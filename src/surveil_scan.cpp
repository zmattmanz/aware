#include "surveil_scan.h"

#include <Arduino.h>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace surveil {
namespace {

// ---------------------------------------------------------------------------
// Signature table
// ---------------------------------------------------------------------------

enum MatchType : uint8_t {
  BLE_NAME_PREFIX,   // pattern = null-terminated string prefix
  BLE_COMPANY_ID,    // pattern = 2 bytes little-endian company ID
  WIFI_SSID_PREFIX,  // pattern = null-terminated string prefix
  WIFI_OUI,          // pattern = 3 binary bytes (presentation order, OUI first)
};

struct Signature {
  Kind        kind;
  MatchType   how;
  const char* pattern;    // interpretation depends on MatchType (see above)
  const char* label;      // shown in the UI (keep ≤19 chars)
  uint8_t     confidence; // 0–100
};

// ---- Flock Safety / Raven: WiFi SSID during provisioning/troubleshooting ----
// Source: flock-you project (github.com/colonelpanichacks/flock-you),
//   "DeFlocking Joplin" DEFCON 31 talk (2023-08), public wardrive datasets.
//   Cameras broadcast "Flock-XXXX" AP during setup and field diagnostics.
//   This SSID prefix is Flock-specific → HIGH confidence.
// Verified: 2024.
static const Signature kSigFlockSSID = {
  FLOCK_CAM, WIFI_SSID_PREFIX, "Flock-", "Flock hotspot", 80
};

// ---- Flock Safety / Raven: Espressif WiFi OUIs ----------------------------
// Source: datasets/NitekryDPaul_wifi_ouis.md @
//   github.com/colonelpanichacks/flock-you  (retrieved 2024-11)
// These 31 OUIs were observed in the flock-you wardrive dataset correlated
// with known Flock camera deployments.
//
// ⚠ IMPORTANT — LOW CONFIDENCE (20): these are *generic* Espressif module
//   OUIs shared by millions of ESP32-based devices (smart plugs, hobby boards,
//   any IoT using Espressif silicon). An OUI match alone is NOT confirmation
//   of a Flock camera. The confidence score and "Flock? (OUI)" label reflect
//   this explicitly. Corroboration (SSID match, BLE name) raises confidence.
//
// The same OUIs apply to BLE MACs from Espressif modules (same vendor block).
static const Signature kSigOuis[] = {
  { FLOCK_CAM, WIFI_OUI, "\x70\xC9\x4E", "Flock? (OUI)", 20 }, // 70:C9:4E
  { FLOCK_CAM, WIFI_OUI, "\x3C\x91\x80", "Flock? (OUI)", 20 }, // 3C:91:80
  { FLOCK_CAM, WIFI_OUI, "\xD8\xF3\xBC", "Flock? (OUI)", 20 }, // D8:F3:BC
  { FLOCK_CAM, WIFI_OUI, "\x80\x30\x49", "Flock? (OUI)", 20 }, // 80:30:49
  { FLOCK_CAM, WIFI_OUI, "\xB8\x35\x32", "Flock? (OUI)", 20 }, // B8:35:32
  { FLOCK_CAM, WIFI_OUI, "\x14\x5A\xFC", "Flock? (OUI)", 20 }, // 14:5A:FC
  { FLOCK_CAM, WIFI_OUI, "\x74\x4C\xA1", "Flock? (OUI)", 20 }, // 74:4C:A1
  { FLOCK_CAM, WIFI_OUI, "\x08\x3A\x88", "Flock? (OUI)", 20 }, // 08:3A:88
  { FLOCK_CAM, WIFI_OUI, "\x9C\x2F\x9D", "Flock? (OUI)", 20 }, // 9C:2F:9D
  { FLOCK_CAM, WIFI_OUI, "\xC0\x35\x32", "Flock? (OUI)", 20 }, // C0:35:32
  { FLOCK_CAM, WIFI_OUI, "\x94\x08\x53", "Flock? (OUI)", 20 }, // 94:08:53
  { FLOCK_CAM, WIFI_OUI, "\xE4\xAA\xEA", "Flock? (OUI)", 20 }, // E4:AA:EA
  { FLOCK_CAM, WIFI_OUI, "\xF4\x6A\xDD", "Flock? (OUI)", 20 }, // F4:6A:DD
  { FLOCK_CAM, WIFI_OUI, "\xF8\xA2\xD6", "Flock? (OUI)", 20 }, // F8:A2:D6
  { FLOCK_CAM, WIFI_OUI, "\x24\xB2\xB9", "Flock? (OUI)", 20 }, // 24:B2:B9
  { FLOCK_CAM, WIFI_OUI, "\x00\xF4\x8D", "Flock? (OUI)", 20 }, // 00:F4:8D
  { FLOCK_CAM, WIFI_OUI, "\xD0\x39\x57", "Flock? (OUI)", 20 }, // D0:39:57
  { FLOCK_CAM, WIFI_OUI, "\xE8\xD0\xFC", "Flock? (OUI)", 20 }, // E8:D0:FC
  { FLOCK_CAM, WIFI_OUI, "\xE0\x4F\x43", "Flock? (OUI)", 20 }, // E0:4F:43
  { FLOCK_CAM, WIFI_OUI, "\xB8\x1E\xA4", "Flock? (OUI)", 20 }, // B8:1E:A4
  { FLOCK_CAM, WIFI_OUI, "\x70\x08\x94", "Flock? (OUI)", 20 }, // 70:08:94
  { FLOCK_CAM, WIFI_OUI, "\x58\x8E\x81", "Flock? (OUI)", 20 }, // 58:8E:81
  { FLOCK_CAM, WIFI_OUI, "\xEC\x1B\xBD", "Flock? (OUI)", 20 }, // EC:1B:BD
  { FLOCK_CAM, WIFI_OUI, "\x3C\x71\xBF", "Flock? (OUI)", 20 }, // 3C:71:BF
  { FLOCK_CAM, WIFI_OUI, "\x58\x00\xE3", "Flock? (OUI)", 20 }, // 58:00:E3
  { FLOCK_CAM, WIFI_OUI, "\x90\x35\xEA", "Flock? (OUI)", 20 }, // 90:35:EA
  { FLOCK_CAM, WIFI_OUI, "\x5C\x93\xA2", "Flock? (OUI)", 20 }, // 5C:93:A2
  { FLOCK_CAM, WIFI_OUI, "\x64\x6E\x69", "Flock? (OUI)", 20 }, // 64:6E:69
  { FLOCK_CAM, WIFI_OUI, "\x48\x27\xEA", "Flock? (OUI)", 20 }, // 48:27:EA
  { FLOCK_CAM, WIFI_OUI, "\xA4\xCF\x12", "Flock? (OUI)", 20 }, // A4:CF:12
  { FLOCK_CAM, WIFI_OUI, "\x82\x6B\xF2", "Flock? (OUI)", 20 }, // 82:6B:F2
};

// ---- Axon body camera (Body3 / Body4 / Body2) BLE -------------------------
// PLACEHOLDER — BLE company ID / service UUIDs / name prefix NOT yet verified.
// How to fill in: power on a known Axon device; run "pio device monitor" and
// watch for lines containing "surveil: ble raw" (enable that Serial.printf in
// ingestBle below); capture the "name=", "mfr=0x" (company ID), and
// service UUIDs. Then add the stable fields here with source citation.
// Do NOT guess identifiers — a false signature does more harm than a gap.
//
// Until filled in, these commented-out entries are skipped:
// { AXON_CAM, BLE_COMPANY_ID, "\xXX\xXX", "Axon Body3", 75 },  // TODO

constexpr size_t kOuiCount = sizeof(kSigOuis) / sizeof(kSigOuis[0]);

// ---------------------------------------------------------------------------
// Detection table
// ---------------------------------------------------------------------------

constexpr uint32_t kStaleMs = 60000;  // silence > 60 s → expire

struct Entry {
  bool     used;
  uint8_t  mac[6];
  char     label[20];
  int8_t   rssi;
  Kind     kind;
  uint8_t  confidence;
  uint32_t last_ms;
};

Entry             s_tab[kMaxDetections];
SemaphoreHandle_t s_lock = nullptr;

// ---------------------------------------------------------------------------
// Internal helpers (call with s_lock held)
// ---------------------------------------------------------------------------

static Entry* findOrCreate(const uint8_t* mac6) {
  int free_idx = -1, oldest_idx = 0;
  uint32_t oldest = 0xFFFFFFFFU;
  for (int i = 0; i < kMaxDetections; ++i) {
    if (s_tab[i].used && memcmp(s_tab[i].mac, mac6, 6) == 0) return &s_tab[i];
    if (!s_tab[i].used && free_idx < 0) free_idx = i;
    if (s_tab[i].used && s_tab[i].last_ms < oldest) {
      oldest = s_tab[i].last_ms; oldest_idx = i;
    }
  }
  int idx = (free_idx >= 0) ? free_idx : oldest_idx;
  Entry& e = s_tab[idx];
  memset(&e, 0, sizeof(e));
  e.used = true;
  memcpy(e.mac, mac6, 6);
  return &e;
}

// Upsert one signature hit. s_lock must be held.
static void upsertLocked(const uint8_t* mac6, const Signature& sig, int8_t rssi) {
  Entry* e = findOrCreate(mac6);
  if (sig.confidence > e->confidence) {
    e->kind = sig.kind;
    strncpy(e->label, sig.label, sizeof(e->label) - 1);
    e->label[sizeof(e->label) - 1] = '\0';
    e->confidence = sig.confidence;
  } else if (sig.confidence == e->confidence && e->confidence < 95) {
    // corroborating hit from a second signature → small bump
    e->confidence = (uint8_t)(e->confidence + 5);
  }
  if (rssi > e->rssi) e->rssi = rssi;
  e->last_ms = (uint32_t)millis();
  e->used    = true;
}

static inline bool ouiMatch(const uint8_t* mac6, const char* pat3) {
  return (uint8_t)pat3[0] == mac6[0] &&
         (uint8_t)pat3[1] == mac6[1] &&
         (uint8_t)pat3[2] == mac6[2];
}

static inline bool prefixMatch(const char* s, const char* prefix) {
  return s && prefix && strncmp(s, prefix, strlen(prefix)) == 0;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void begin() {
  memset(s_tab, 0, sizeof(s_tab));
  s_lock = xSemaphoreCreateMutex();
}

void prune() {
  if (!s_lock) return;
  xSemaphoreTake(s_lock, portMAX_DELAY);
  uint32_t now = (uint32_t)millis();
  for (int i = 0; i < kMaxDetections; ++i) {
    if (s_tab[i].used && (now - s_tab[i].last_ms) > kStaleMs) {
      s_tab[i].used = false;
    }
  }
  xSemaphoreGive(s_lock);
}

size_t snapshot(Detection* out, size_t max) {
  if (!s_lock) return 0;
  size_t n = 0;
  xSemaphoreTake(s_lock, portMAX_DELAY);
  for (int i = 0; i < kMaxDetections && n < max; ++i) {
    if (!s_tab[i].used) continue;
    Detection& d = out[n];
    memcpy(d.mac,  s_tab[i].mac,   6);
    strncpy(d.label, s_tab[i].label, sizeof(d.label) - 1);
    d.label[sizeof(d.label) - 1] = '\0';
    d.rssi       = s_tab[i].rssi;
    d.kind       = s_tab[i].kind;
    d.confidence = s_tab[i].confidence;
    d.last_ms    = s_tab[i].last_ms;
    ++n;
  }
  xSemaphoreGive(s_lock);
  return n;
}

void ingestBle(const char* name, const uint8_t* mac6, int8_t rssi,
               uint16_t companyId, bool hasCompanyId) {
  if (!s_lock || !mac6) return;

  // Uncomment to capture raw BLE for signature development:
  // Serial.printf("surveil: ble raw  mac=%02X:%02X:%02X:%02X:%02X:%02X "
  //               "name='%s' mfr=0x%04X rssi=%d\n",
  //               mac6[0],mac6[1],mac6[2],mac6[3],mac6[4],mac6[5],
  //               name ? name : "", companyId, rssi);

  bool matched = false;

  xSemaphoreTake(s_lock, portMAX_DELAY);

  // BLE name prefix — no verified Flock/Axon BLE names yet; placeholder comment above
  (void)name;

  // BLE company ID — no verified Axon company ID yet; placeholder comment above
  (void)hasCompanyId; (void)companyId;

  // OUI matching: Espressif OUIs shared by WiFi + BLE on the same chip.
  // BLE MACs on ESP32 are derived from the same OUI block as WiFi MACs.
  // False-positive rate is significant (any ESP32 device); LOW confidence.
  for (size_t i = 0; i < kOuiCount; ++i) {
    if (ouiMatch(mac6, kSigOuis[i].pattern)) {
      upsertLocked(mac6, kSigOuis[i], rssi);
      matched = true;
    }
  }

  xSemaphoreGive(s_lock);
  (void)matched;
}

void ingestWifi(const char* ssid, const uint8_t* bssid6, int8_t rssi) {
  if (!s_lock || !bssid6) return;

  xSemaphoreTake(s_lock, portMAX_DELAY);

  // High-confidence: Flock-XXXX SSID is specific to Flock provisioning
  if (prefixMatch(ssid, kSigFlockSSID.pattern)) {
    upsertLocked(bssid6, kSigFlockSSID, rssi);
    xSemaphoreGive(s_lock);
    return;  // SSID match overrides any OUI match; no need to continue
  }

  // Low-confidence OUI check
  for (size_t i = 0; i < kOuiCount; ++i) {
    if (ouiMatch(bssid6, kSigOuis[i].pattern)) {
      upsertLocked(bssid6, kSigOuis[i], rssi);
    }
  }

  xSemaphoreGive(s_lock);
}

}  // namespace surveil
