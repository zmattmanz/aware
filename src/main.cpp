// ===========================================================================
// Aware — M5Stack StickS3 (ESP32-S3, 240x135)
//
// Screen-cycled awareness device, washer-detector UI language.
//   BtnA            — cycle screens
//   BtnB  (click)   — cycle the focused item (plane / drone)
//   BtnB  (hold)    — on PLANES: cycle radar range (5/10/15/25 km)
//
// PLANES : live ADS-B over WiFi (data layer from MatixYo/ESP32-Plane-Radar).
// DRONES : passive BLE Remote ID (ASTM F3411 / OpenDroneID) — true native RF,
//          runs continuously alongside WiFi. Decoder vendored from
//          opendroneid-core-c (Apache-2.0). See ATTRIBUTION.md.
// SURVEIL: passive RF surveillance (Flock Safety / Axon) via WiFi+BLE signatures.
// ===========================================================================

#include <M5Unified.h>
#include <WiFi.h>
#include <math.h>

#include "config.h"
#include "services/adsb_client.h"
#include "drone_scan.h"
#include "surveil_scan.h"
#include "c5_link.h"
#include <Preferences.h>
#include <WebServer.h>
#include "esp32-hal-cpu.h"
#include <esp_sleep.h>

// ---- palette ---------------------------------------------------------------
static constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
static constexpr uint16_t COL_BG     = rgb565(8, 12, 36);
static constexpr uint16_t COL_BAR    = rgb565(18, 26, 64);
static constexpr uint16_t COL_HEAD   = rgb565(255, 214, 0);
static constexpr uint16_t COL_OK     = rgb565(80, 220, 120);
static constexpr uint16_t COL_LABEL  = rgb565(140, 150, 185);
static constexpr uint16_t COL_TEXT   = rgb565(235, 240, 255);
static constexpr uint16_t COL_PLANE  = rgb565(245, 80, 80);
static constexpr uint16_t COL_DRONE  = rgb565(255, 120, 40);
static constexpr uint16_t COL_RING   = rgb565(36, 92, 70);
static constexpr uint16_t COL_SEL    = rgb565(255, 255, 255);
static constexpr uint16_t COL_BAD    = rgb565(230, 70, 70);

// ---- airspace palette (lilac, no alpha — precomputed blends) ---------------
static constexpr uint16_t C_BG        = rgb565(0x12,0x11,0x16);
static constexpr uint16_t C_TEXT      = rgb565(0xE5,0xE1,0xEA);
static constexpr uint16_t C_DIM       = rgb565(0x6E,0x68,0x77);
static constexpr uint16_t C_ACCENT    = rgb565(0xAE,0x9F,0xCB);
static constexpr uint16_t C_PANEL     = rgb565(0x1A,0x19,0x22);
static constexpr uint16_t C_HAIRLINE  = rgb565(0x32,0x2F,0x38);
static constexpr uint16_t C_ACCENT_LO = rgb565(0x50,0x4A,0x5E);
static constexpr uint16_t C_SELROW    = rgb565(0x28,0x25,0x2F);

// ---- drone feed palette (same Lilac scheme, aliased + layout constants) ----
static constexpr uint16_t F_BG     = C_BG;
static constexpr uint16_t F_TEXT   = C_TEXT;
static constexpr uint16_t F_DIM    = C_DIM;
static constexpr uint16_t F_ACCENT = C_ACCENT;
static constexpr uint16_t F_HAIR   = C_HAIRLINE;
static constexpr uint16_t F_SELROW = C_SELROW;
static constexpr int F_ROW_H = 22;
static constexpr int F_TOP   = 23;
static constexpr int F_WIN_H = 112;

// ---- layout ----------------------------------------------------------------
static constexpr int W = 240, H = 135;
static constexpr int TOPBAR_H = 17, BOTBAR_H = 19;
static constexpr int CONTENT_Y = TOPBAR_H + 1;
static constexpr int CONTENT_H = H - TOPBAR_H - BOTBAR_H - 2;

static M5Canvas cv(&M5.Display);

// ---- screens ---------------------------------------------------------------
enum Screen { SCR_AIRSPACE = 0, SCR_DRONES, SCR_SURVEIL, SCR_SCAN, SCR_STATS, SCR_SETUP, SCR_COUNT };
static int g_screen = SCR_AIRSPACE;

// ---- RF scan screen state (2.4 GHz WiFi APs; BLE comes from drone_scan) -----
struct ApSight { char ssid[24]; char bssid[18]; int8_t rssi; uint8_t ch; };
static ApSight       g_aps[12];
static int           g_ap_count       = 0;
static bool          g_wifi_scanning  = false;
static unsigned long g_last_wifi_scan = 0;

// ---- plane state -----------------------------------------------------------
static int           g_range_idx      = appcfg::kRangeDefaultIdx;
static unsigned long g_last_fetch     = 0;
static unsigned long g_last_update_ms = 0;   // millis() of last SUCCESSFUL adsb fetch
static bool          g_have_fetched   = false;
static int           g_porder[64];   // aircraft indices sorted by distance asc
static float         g_pdist[64];    // km, indexed by aircraft index
static float         g_pbrg[64];     // deg
static int           g_pcount        = 0;
static inline float  rangeKm() { return appcfg::kRangePresetsKm[g_range_idx]; }

// ---- unified airspace selection (index into sorted contact list) -----------
static int           g_airspace_sel  = 0;

// ---- drone feed selection + slide-in animation ----------------------------
static int           g_drone_sel     = 0;
static char          g_feed_top_mac[18] = "";
static float         g_feed_anim     = 0.0f;   // px offset F_ROW_H->0 during slide
static unsigned long g_feed_anim_t   = 0;

// ---- airspace source health ------------------------------------------------
static unsigned long g_lastAdsbOkMs    = 0;     // millis() of last successful fetch (0 = never)
static unsigned long g_lastDataMs      = 0;     // millis() of last plane or BLE contact
static bool          g_bleScanRunning  = false;
static bool          g_everReceivedData= false;  // latches true on first contact of any kind

// ---- scan / surveil selection -----------------------------------------------
static int           g_scan_sel      = 0;

// ---- runtime config (NVS) --------------------------------------------------
static Preferences g_prefs;

struct Cfg { String ssid, pass; double lat, lon; };
static Cfg g_cfg;

static void loadCfg() {
  g_prefs.begin("aware", true);
  g_cfg.ssid = g_prefs.getString("ssid", appcfg::kWifiSsid);
  g_cfg.pass = g_prefs.getString("pass", appcfg::kWifiPass);
  g_cfg.lat  = g_prefs.getDouble("lat",  appcfg::kHomeLat);
  g_cfg.lon  = g_prefs.getDouble("lon",  appcfg::kHomeLon);
  g_prefs.end();
}
static void saveCfg() {
  g_prefs.begin("aware", false);
  g_prefs.putString("ssid", g_cfg.ssid); g_prefs.putString("pass", g_cfg.pass);
  g_prefs.putDouble("lat", g_cfg.lat);   g_prefs.putDouble("lon", g_cfg.lon);
  g_prefs.end();
}

// ---- web config portal -----------------------------------------------------
static WebServer*    g_srv           = nullptr;
static bool          g_portal_active = false;
static unsigned long g_portal_start  = 0;
static constexpr unsigned long kPortalTimeoutMs = 180000UL;  // 3 min

static const char* const PAGE = R"HTML(
<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<style>body{font:16px sans-serif;margin:1.2em;background:#0a0a0a;color:#fff}
input{width:100%;box-sizing:border-box;padding:.5em;margin:.25em 0}button{padding:.6em 1em;margin:.3em 0}</style>
<h3>Aware setup</h3>
<form method=POST action=/save>
 WiFi network<input name=ssid value="%SSID%">
 WiFi password<input name=pass type=password placeholder="(unchanged)">
 Latitude<input id=lat name=lat value="%LAT%">
 Longitude<input id=lon name=lon value="%LON%">
 <button type=button onclick=geo()>Use my location</button>
 <p id=msg></p>
 <button type=submit>Save &amp; reboot</button>
</form>
<script>
function geo(){
 if(!navigator.geolocation){msg.textContent='no geolocation API - enter manually';return;}
 navigator.geolocation.getCurrentPosition(
  p=>{lat.value=p.coords.latitude.toFixed(5);lon.value=p.coords.longitude.toFixed(5);msg.textContent='location set';},
  e=>{msg.textContent='location blocked ('+e.message+') - enter lat/lon manually';});
}
</script>
)HTML";

static void handleRoot() {
  char clat[16], clon[16];
  snprintf(clat, sizeof(clat), "%.5f", g_cfg.lat);
  snprintf(clon, sizeof(clon), "%.5f", g_cfg.lon);
  String p(PAGE);
  p.replace("%SSID%", g_cfg.ssid);
  p.replace("%LAT%", clat);
  p.replace("%LON%", clon);
  if (g_srv) g_srv->send(200, "text/html", p);
}

static void handleSave() {
  if (!g_srv) return;
  String ssid = g_srv->arg("ssid");
  String pass = g_srv->arg("pass");
  String slat = g_srv->arg("lat");
  String slon = g_srv->arg("lon");
  if (ssid.length()) g_cfg.ssid = ssid;
  if (pass.length()) g_cfg.pass = pass;
  if (slat.length()) g_cfg.lat  = slat.toDouble();
  if (slon.length()) g_cfg.lon  = slon.toDouble();
  saveCfg();
  g_srv->send(200, "text/html",
    "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<body style='background:#0a0a0a;color:#fff;font:16px sans-serif;margin:1.2em'>"
    "<h3>Saved - rebooting...</h3></body>");
  delay(800);
  ESP.restart();
}

static void startConfigPortal() {
  g_wifi_scanning = false;    // disarm scan loop before touching radio state
  WiFi.scanDelete();          // drop any stale/in-flight async scan results
  WiFi.mode(WIFI_AP);         // pure AP — kills STA so no scan can collide
  delay(100);                 // let the radio settle into AP mode
  WiFi.softAP("Aware-Setup");
  delay(100);                 // let AP + DHCP come up before serving
  g_srv = new WebServer(80);
  g_srv->on("/", HTTP_GET, handleRoot);
  g_srv->on("/save", HTTP_POST, handleSave);
  g_srv->begin();
  g_portal_active = true;
  g_portal_start  = millis();
  g_screen        = SCR_SETUP;
}

static void stopConfigPortal() {
  if (g_srv) { g_srv->stop(); delete g_srv; g_srv = nullptr; }
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_cfg.ssid.c_str(), g_cfg.pass.c_str());
  g_portal_active = false;
}

// ---------------------------------------------------------------------------
// battery helpers
// ---------------------------------------------------------------------------
static int battPctFromMv(int mv) {  // approx single-cell Li-ion, resting voltage
  static const int v[] = {3300,3500,3600,3700,3750,3790,3830,3870,3920,3980,4080,4200};
  static const int p[] = {   0,   5,  10,  20,  30,  40,  50,  60,  70,  80,  90, 100};
  if (mv <= v[0]) return 0;
  for (int i = 1; i < 12; ++i)
    if (mv <= v[i]) return p[i-1] + (p[i]-p[i-1])*(mv-v[i-1])/(v[i]-v[i-1]);
  return 100;
}

// ---------------------------------------------------------------------------
// BLE vendor lookup — values from Bluetooth SIG Assigned Numbers
// ---------------------------------------------------------------------------
struct Vendor { uint16_t cid; const char* name; };
static const Vendor kVendors[] = {
  { 0x004C, "Apple"     },   // iPhone/Mac/AirPods/AirTag/Watch — vendor only
  { 0x0006, "Microsoft" },
  { 0x00E0, "Google"    },
  { 0x0075, "Samsung"   },
  { 0x0059, "Nordic"    },   // dev boards & many BLE gadgets
};
static const char* bleVendor(uint16_t cid, bool hasMfr) {
  if (!hasMfr) return nullptr;
  for (auto& v : kVendors) if (v.cid == cid) return v.name;
  return nullptr;
}

// ---------------------------------------------------------------------------
// brightness arbitration — one place sets brightness; lowest level wins
// ---------------------------------------------------------------------------
static void applyBrightness(bool idle, int mv, bool onUsb) {
  int b = 90;
  if (idle)                               b = 25;              // motion-idle dim
  if (!onUsb && mv > 0 && mv < 3300)     b = (b < 40 ? b : 40);  // low-batt cap
  M5.Display.setBrightness(b);
}

// ---------------------------------------------------------------------------
// geo helpers
// ---------------------------------------------------------------------------
static inline double deg2rad(double d) { return d * 0.017453292519943295; }

static float haversineKm(double la1, double lo1, double la2, double lo2) {
  const double R = 6371.0088;
  double dlat = deg2rad(la2 - la1), dlon = deg2rad(lo2 - lo1);
  double a = sin(dlat / 2) * sin(dlat / 2) +
             cos(deg2rad(la1)) * cos(deg2rad(la2)) * sin(dlon / 2) * sin(dlon / 2);
  return (float)(2.0 * R * atan2(sqrt(a), sqrt(1 - a)));
}
static float bearingDeg(double la1, double lo1, double la2, double lo2) {
  double y = sin(deg2rad(lo2 - lo1)) * cos(deg2rad(la2));
  double x = cos(deg2rad(la1)) * sin(deg2rad(la2)) -
             sin(deg2rad(la1)) * cos(deg2rad(la2)) * cos(deg2rad(lo2 - lo1));
  double b = atan2(y, x) * 57.29577951308232;
  if (b < 0) b += 360.0;
  return (float)b;
}
static const char* compass8(float brg) {
  static const char* p[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return p[(int)((brg + 22.5f) / 45.0f) & 7];
}

// ---------------------------------------------------------------------------
// chrome
// ---------------------------------------------------------------------------
static void drawTopBar(const char* title) {
  cv.fillRect(0, 0, W, TOPBAR_H, COL_BAR);
  cv.setTextColor(COL_TEXT, COL_BAR);
  cv.setTextSize(1);
  cv.setTextDatum(middle_left);
  cv.drawString(title, 4, TOPBAR_H / 2);
  char b[8] = "";
  int lvl = battPctFromMv(M5.Power.getBatteryVoltage());
  snprintf(b, sizeof(b), "%d%%", lvl);
  cv.setTextColor(COL_LABEL, COL_BAR);
  cv.setTextDatum(middle_right);
  cv.drawString(b, W - 14, TOPBAR_H / 2);
  bool up = (WiFi.status() == WL_CONNECTED);
  cv.fillCircle(W - 6, TOPBAR_H / 2, 3, up ? COL_OK : COL_BAD);
}

static void drawBottomBar(const char* a, const char* b, const char* c) {
  int y = H - BOTBAR_H;
  cv.fillRect(0, y, W, BOTBAR_H, COL_BAR);
  cv.drawLine(80, y + 3, 80, H - 3, COL_BG);
  cv.drawLine(160, y + 3, 160, H - 3, COL_BG);
  cv.setTextSize(1);
  cv.setTextColor(COL_TEXT, COL_BAR);
  cv.setTextDatum(middle_center);
  cv.drawString(a, 40, y + BOTBAR_H / 2);
  cv.drawString(b, 120, y + BOTBAR_H / 2);
  cv.drawString(c, 200, y + BOTBAR_H / 2);
}

// horizontal RSSI proximity bar (-90..-30 dBm -> empty..full)
static void drawRssiBar(int x, int y, int w, int8_t rssi) {
  cv.drawRect(x, y, w, 6, COL_LABEL);
  float t = (rssi + 90) / 60.0f;
  if (t < 0) t = 0; if (t > 1) t = 1;
  cv.fillRect(x + 1, y + 1, (int)((w - 2) * t), 4, COL_OK);
}

// ---- airspace glyphs -------------------------------------------------------
static void drawPlane(int cx, int cy, float hdg, float scale, uint16_t col) {
  static constexpr float T[][6] = {
    { 0,-7,   1.3f,-1,  -1.3f,-1  },   // nose
    { 1.3f,-1, 1.6f,6,  -1.6f, 6  },   // body
    { 1.3f,-1,-1.6f,6,  -1.3f,-1  },   // body
    { 1.3f, 0,  7,  3,   1.4f, 2.8f},  // right wing
    {-1.3f, 0, -7,  3,  -1.4f, 2.8f},  // left wing
    { 1,    5,  3.4f,7.6f, 1,  6.9f},  // right tail
    {-1,    5, -3.4f,7.6f,-1,  6.9f},  // left tail
  };
  float a = hdg * (float)M_PI / 180.0f, c = cosf(a), s = sinf(a);
  auto P = [&](float lx, float ly, int& ox, int& oy) {
    lx *= scale; ly *= scale;
    ox = cx + (int)lroundf(lx * c - ly * s);
    oy = cy + (int)lroundf(lx * s + ly * c);
  };
  for (auto& t : T) {
    int x0,y0,x1,y1,x2,y2;
    P(t[0],t[1],x0,y0); P(t[2],t[3],x1,y1); P(t[4],t[5],x2,y2);
    cv.fillTriangle(x0,y0,x1,y1,x2,y2,col);
  }
}

static void drawQuad(int x, int y, uint16_t col, bool selected) {
  if (selected) cv.drawCircle(x, y, 9, C_ACCENT_LO);
  const int A = 5;
  cv.drawLine(x,y, x+A,y+A, col); cv.drawLine(x,y, x-A,y+A, col);
  cv.drawLine(x,y, x+A,y-A, col); cv.drawLine(x,y, x-A,y-A, col);
  cv.drawCircle(x+A,y+A,2,col); cv.drawCircle(x-A,y+A,2,col);
  cv.drawCircle(x+A,y-A,2,col); cv.drawCircle(x-A,y-A,2,col);
  cv.fillCircle(x,y,2,col);
}

// ---- airspace / drone source helpers ---------------------------------------
static const char* droneSrcTag(uint8_t s) {
  switch (s) {
    case drone::SRC_WIFI_5G: return "5G";
    case drone::SRC_WIFI_2G: return "2.4G";
    default:                 return "BLE";
  }
}
static uint16_t droneSrcCol(uint8_t s) {
  switch (s) {
    case drone::SRC_WIFI_5G: return COL_OK;     // green — the one you're verifying
    case drone::SRC_WIFI_2G: return COL_HEAD;   // amber
    default:                 return C_DIM;       // muted — BLE
  }
}

// ---- airspace radar helpers ------------------------------------------------
static constexpr float AS_R_OUTER  = 44.0f;
static constexpr float AS_R_MID    = 31.0f;
static constexpr float AS_R_NEAR   = 17.0f;
static constexpr float AS_NEAR_M   = 2000.0f;   // inner band: 0..2 km → 0..R_NEAR
static constexpr float AS_FAR_M    = 92600.0f;  // outer band: 2 km..50 nm → R_NEAR..R_OUTER

static float airspaceRadiusFor(bool isDrone, float dist_m) {
  if (isDrone) {
    return fminf(dist_m / AS_NEAR_M, 1.0f) * AS_R_NEAR;
  } else {
    float t = fminf(fmaxf((dist_m - AS_NEAR_M) / (AS_FAR_M - AS_NEAR_M), 0.0f), 1.0f);
    return AS_R_NEAR + t * (AS_R_OUTER - AS_R_NEAR);
  }
}

// ---------------------------------------------------------------------------
// AIRSPACE (unified planes + drones)
// ---------------------------------------------------------------------------
static void recomputePlanes() {
  size_t n = services::adsb::aircraftCount();
  const services::adsb::Aircraft* L = services::adsb::aircraftList();
  g_pcount = (int)n;
  for (int i = 0; i < (int)n; ++i) {
    g_pdist[i] = haversineKm(g_cfg.lat, g_cfg.lon, L[i].lat, L[i].lon);
    g_pbrg[i]  = bearingDeg(g_cfg.lat, g_cfg.lon, L[i].lat, L[i].lon);
    g_porder[i] = i;
  }
  for (int i = 1; i < (int)n; ++i) {  // insertion sort by distance
    int key = g_porder[i], j = i - 1;
    while (j >= 0 && g_pdist[g_porder[j]] > g_pdist[key]) { g_porder[j + 1] = g_porder[j]; --j; }
    g_porder[j + 1] = key;
  }
  // g_airspace_sel is clamped in drawAirspaceScreen after drones are merged in
}

enum class AirState { NORMAL, ACQUIRING, CLEAR, DEGRADED };

static void drawAirspaceScreen() {
  cv.fillScreen(C_BG);

  const int RAD_CX = 186, RAD_CY = 70;
  unsigned long now_ms = millis();

  // --- Build unified contact list (nearest first) ---
  struct AContact { bool is_drone; int idx; float dist_m; float brg_deg; };
  static AContact contacts[76];
  int nContacts = 0;

  drone::Drone drones[12];
  size_t nd = drone::snapshot(drones, 12);
  int cBle = 0, c24 = 0, c5 = 0;
  for (size_t i = 0; i < nd; ++i) {
    switch (drones[i].source) {
      case drone::SRC_WIFI_5G: ++c5;  break;
      case drone::SRC_WIFI_2G: ++c24; break;
      default:                 ++cBle; break;
    }
  }
  for (size_t i = 0; i < nd; ++i) {
    float dm = drones[i].has_loc
               ? haversineKm(g_cfg.lat, g_cfg.lon, drones[i].lat, drones[i].lon) * 1000.0f
               : 0.0f;
    float db = drones[i].has_loc ? bearingDeg(g_cfg.lat, g_cfg.lon, drones[i].lat, drones[i].lon) : 0.0f;
    contacts[nContacts++] = { true, (int)i, dm, db };
  }
  const services::adsb::Aircraft* acList = services::adsb::aircraftList();
  for (int i = 0; i < g_pcount; ++i) {
    int ai = g_porder[i];
    contacts[nContacts++] = { false, ai, g_pdist[ai] * 1000.0f, g_pbrg[ai] };
  }
  for (int i = 1; i < nContacts; ++i) {
    AContact key = contacts[i]; int j = i - 1;
    while (j >= 0 && contacts[j].dist_m > key.dist_m) { contacts[j+1] = contacts[j]; --j; }
    contacts[j+1] = key;
  }
  if (nContacts > 0) { g_everReceivedData = true; g_lastDataMs = now_ms; }
  if (g_airspace_sel >= nContacts) g_airspace_sel = 0;

  // --- Health state ---
  bool wifiUp    = (WiFi.status() == WL_CONNECTED);
  // adsbFresh: if we've never fetched, be optimistic (show "ok" if WiFi is up)
  bool adsbFresh = (g_lastAdsbOkMs > 0) ? ((now_ms - g_lastAdsbOkMs) < 60000UL) : wifiUp;
  AirState state;
  if (nContacts > 0) {
    state = AirState::NORMAL;
  } else if (!g_everReceivedData && now_ms < 20000UL) {
    state = AirState::ACQUIRING;
  } else if (!wifiUp || !g_bleScanRunning) {
    state = AirState::DEGRADED;
  } else if (!adsbFresh) {
    state = AirState::DEGRADED;
  } else {
    state = AirState::CLEAR;
  }

  // --- Vertical divider ---
  cv.drawLine(132, 8, 132, 127, C_HAIRLINE);

  // --- Radar: crosshair ---
  cv.drawLine(RAD_CX, RAD_CY - (int)AS_R_OUTER - 2, RAD_CX, RAD_CY + (int)AS_R_OUTER + 2, C_HAIRLINE);
  cv.drawLine(RAD_CX - (int)AS_R_OUTER - 2, RAD_CY, RAD_CX + (int)AS_R_OUTER + 2, RAD_CY, C_HAIRLINE);

  // --- Radar: rings ---
  uint16_t ringOuter = (state == AirState::DEGRADED && !wifiUp) ? C_HAIRLINE : C_DIM;
  cv.drawCircle(RAD_CX, RAD_CY, (int)AS_R_OUTER, ringOuter);
  cv.drawCircle(RAD_CX, RAD_CY, (int)AS_R_MID,   ringOuter);
  cv.drawCircle(RAD_CX, RAD_CY, (int)AS_R_NEAR,  C_ACCENT_LO);

  // --- Always-on sweep (stops if loop stalls — liveness cue) ---
  {
    static float sweepDeg = 0;
    sweepDeg += 6.0f; if (sweepDeg >= 360.0f) sweepDeg -= 360.0f;
    float sa = sweepDeg * (float)M_PI / 180.0f;
    cv.drawLine(RAD_CX, RAD_CY,
                RAD_CX + (int)lroundf(AS_R_OUTER * sinf(sa)),
                RAD_CY - (int)lroundf(AS_R_OUTER * cosf(sa)),
                C_ACCENT_LO);
    float sa2 = (sweepDeg - 8.0f) * (float)M_PI / 180.0f;
    cv.drawLine(RAD_CX, RAD_CY,
                RAD_CX + (int)lroundf(AS_R_OUTER * sinf(sa2)),
                RAD_CY - (int)lroundf(AS_R_OUTER * cosf(sa2)),
                C_HAIRLINE);
  }

  // --- Radar contacts (planes first so drones render on top) ---
  for (int pass = 0; pass < 2; ++pass) {
    for (int i = 0; i < nContacts; ++i) {
      const AContact& c = contacts[i];
      if ((pass == 0) == c.is_drone) continue;
      float r = airspaceRadiusFor(c.is_drone, c.dist_m);
      float a = c.brg_deg * (float)M_PI / 180.0f;
      int px = (int)lroundf(RAD_CX + r * sinf(a));
      int py = (int)lroundf(RAD_CY - r * cosf(a));
      bool sel = (i == g_airspace_sel);
      if (c.is_drone) {
        drawQuad(px, py, C_ACCENT, sel);
      } else {
        if (sel) cv.drawCircle(px, py, 8, C_ACCENT_LO);
        float hdg = acList[c.idx].track_deg;
        drawPlane(px, py, hdg > 0.0f ? hdg : c.brg_deg, 1.0f, sel ? C_ACCENT : C_TEXT);
      }
    }
  }

  // --- Live dot (blinks while fresh data, hollow when idle) ---
  bool active = (g_lastDataMs > 0) && ((now_ms - g_lastDataMs) < 8000UL);
  if (active) {
    if (((now_ms / 500) % 2) == 0) cv.fillCircle(8, 20, 2, C_ACCENT);
  } else {
    cv.drawCircle(8, 20, 2, C_DIM);
  }
  cv.setTextDatum(top_left); cv.setTextSize(1);
  cv.setTextColor(C_DIM, C_BG);
  char liveTxt[14]; snprintf(liveTxt, sizeof(liveTxt), "%d \xB7 live", nContacts);
  cv.drawString(liveTxt, 16, 17);

  if (state == AirState::NORMAL) {
    // --- Contact rows ---
    // Row 0: 20px tall (drone pilot 2nd line); rows 1-5: 14px
    static const int rowTop[] = {  30,  56,  70,  84,  98, 112 };
    static const int rowH[]   = {  20,  14,  14,  14,  14,  14 };
    static const int rowGY[]  = {  40,  63,  77,  91, 105, 119 };
    const int kMaxRows = 6;
    bool hasMore = nContacts > kMaxRows;
    int rowsToShow = hasMore ? kMaxRows - 1 : nContacts;

    for (int i = 0; i < rowsToShow; ++i) {
      const AContact& c = contacts[i];
      int rt = rowTop[i], rh = rowH[i], gy = rowGY[i];
      bool sel = (i == g_airspace_sel);
      uint16_t bg = sel ? C_SELROW : C_BG;
      if (sel) cv.fillRoundRect(4, rt, 122, rh, 3, C_SELROW);

      // Glyph at x=14
      if (c.is_drone) {
        const int A = 4;
        if (sel) cv.drawCircle(14, gy, 8, C_ACCENT_LO);
        cv.drawLine(14,gy, 14+A,gy+A,C_ACCENT); cv.drawLine(14,gy, 14-A,gy+A,C_ACCENT);
        cv.drawLine(14,gy, 14+A,gy-A,C_ACCENT); cv.drawLine(14,gy, 14-A,gy-A,C_ACCENT);
        cv.drawCircle(14+A,gy+A,1,C_ACCENT); cv.drawCircle(14-A,gy+A,1,C_ACCENT);
        cv.drawCircle(14+A,gy-A,1,C_ACCENT); cv.drawCircle(14-A,gy-A,1,C_ACCENT);
        cv.fillCircle(14,gy,2,C_ACCENT);
      } else {
        drawPlane(14, gy, 0.0f, 0.55f, sel ? C_ACCENT : C_TEXT);
      }

      // ID at x=24
      char id[10];
      if (c.is_drone) {
        snprintf(id, sizeof(id), "%.8s", drones[c.idx].id[0] ? drones[c.idx].id : "DRONE");
      } else {
        const services::adsb::Aircraft& ac = acList[c.idx];
        snprintf(id, sizeof(id), "%.8s", ac.callsign[0] ? ac.callsign : "----");
      }
      cv.setTextDatum(top_left);
      cv.setTextColor(c.is_drone ? C_ACCENT : (sel ? C_ACCENT : C_TEXT), bg);
      cv.drawString(id, 24, rt + 1);

      // Trend at x=100: "-" for planes, colored source tag for drones
      if (c.is_drone) {
        cv.setTextDatum(top_left);
        cv.setTextColor(droneSrcCol(drones[c.idx].source), bg);
        cv.drawString(droneSrcTag(drones[c.idx].source), 82, rt + 1);
      } else {
        cv.setTextColor(C_DIM, bg);
        cv.drawString("-", 100, rt + 1);
      }

      // Distance right-aligned at x=126
      char dist[10];
      if (c.is_drone) {
        if (c.dist_m < 1000.0f) snprintf(dist, sizeof(dist), "%dm",   (int)c.dist_m);
        else                    snprintf(dist, sizeof(dist), "%.1fkm", c.dist_m / 1000.0f);
      } else {
        float nm = c.dist_m / 1852.0f;
        if (nm < 10.0f) snprintf(dist, sizeof(dist), "%.1fnm", nm);
        else            snprintf(dist, sizeof(dist), "%.0fnm", nm);
      }
      cv.setTextDatum(top_right);
      cv.setTextColor(c.is_drone ? C_ACCENT : C_DIM, bg);
      cv.drawString(dist, 126, rt + 1);

      // Drone 2nd line: pilot distance
      if (c.is_drone && drones[c.idx].has_op) {
        float opM = haversineKm(g_cfg.lat, g_cfg.lon, drones[c.idx].op_lat, drones[c.idx].op_lon) * 1000.0f;
        char line2[20];
        if (opM < 1000.0f) snprintf(line2, sizeof(line2), "pilot %dm",    (int)opM);
        else               snprintf(line2, sizeof(line2), "pilot %.1fkm", opM / 1000.0f);
        cv.setTextDatum(top_left);
        cv.setTextColor(C_ACCENT, bg);
        cv.drawString(line2, 24, rt + 10);
      }
    }
    if (hasMore) {
      char more[12]; snprintf(more, sizeof(more), "+%d more", nContacts - (kMaxRows - 1));
      cv.setTextDatum(top_left);
      cv.setTextColor(C_DIM, C_BG);
      cv.drawString(more, 16, rowTop[kMaxRows - 1]);
    }

    // --- Source tally (bottom-right, below radar) ---
    {
      char buf[8];
      cv.setTextDatum(top_left); cv.setTextSize(1);
      cv.setTextColor(C_DIM,    C_BG); snprintf(buf, sizeof(buf), "ble:%d", cBle); cv.drawString(buf, 140, 119);
      cv.setTextColor(COL_HEAD, C_BG); snprintf(buf, sizeof(buf), "24:%d",  c24);  cv.drawString(buf, 172, 119);
      cv.setTextColor(COL_OK,   C_BG); snprintf(buf, sizeof(buf), "5g:%d",  c5);   cv.drawString(buf, 204, 119);
    }
  } else {
    // --- Empty states ---
    bool blink = ((now_ms / 600) % 2) == 0;
    const int midX = 66;  // center of feed area x=0..132

    cv.setTextDatum(middle_center); cv.setTextSize(1);
    if (state == AirState::ACQUIRING) {
      cv.setTextColor(C_DIM, C_BG);
      cv.drawString("waiting for", midX, 55);
      cv.drawString("a signal",    midX, 67);
    } else if (state == AirState::CLEAR) {
      cv.setTextColor(C_TEXT, C_BG);
      cv.drawString("sky clear", midX, 60);
    } else {
      const char* dead = !wifiUp ? "wifi" : (!adsbFresh ? "ads-b" : "ble");
      cv.setTextColor(C_TEXT, C_BG); cv.drawString(dead,      midX, 55);
      cv.setTextColor(C_DIM,  C_BG); cv.drawString("offline", midX, 67);
    }

    // Source status dots
    cv.setTextDatum(top_left); cv.setTextSize(1);
    const int srcY1 = 90, srcY2 = 104;
    bool adsbDot = (state == AirState::ACQUIRING) ? blink : adsbFresh;
    if (adsbDot) cv.fillCircle(12, srcY1 + 4, 2, C_ACCENT);
    else         cv.drawCircle(12, srcY1 + 4, 2, C_DIM);
    cv.setTextColor(adsbFresh ? C_TEXT : C_DIM, C_BG);
    cv.drawString(state==AirState::ACQUIRING ? "ads-b ..." : (adsbFresh ? "ads-b ok":"ads-b --"), 17, srcY1);
    bool bleDot = (state == AirState::ACQUIRING) ? blink : g_bleScanRunning;
    if (bleDot) cv.fillCircle(12, srcY2 + 4, 2, C_ACCENT);
    else        cv.drawCircle(12, srcY2 + 4, 2, C_DIM);
    cv.setTextColor(g_bleScanRunning ? C_TEXT : C_DIM, C_BG);
    cv.drawString(state==AirState::ACQUIRING ? "ble ..."   : (g_bleScanRunning ? "ble ok" : "ble --"), 17, srcY2);

    // C5 link dot + source tally (always visible so 5G count is legible at zero)
    const int srcY3 = 118;
    bool c5Dot = c5link::linked();
    if (c5Dot) cv.fillCircle(12, srcY3 + 4, 2, COL_OK);
    else       cv.drawCircle(12, srcY3 + 4, 2, C_DIM);
    cv.setTextColor(c5Dot ? COL_OK : C_DIM, C_BG);
    cv.drawString(c5Dot ? "c5 ok" : "c5 --", 17, srcY3);
    {
      char buf[8];
      cv.setTextDatum(top_left); cv.setTextSize(1);
      cv.setTextColor(C_DIM,    C_BG); snprintf(buf, sizeof(buf), "ble:%d", cBle); cv.drawString(buf, 62,  srcY3);
      cv.setTextColor(COL_HEAD, C_BG); snprintf(buf, sizeof(buf), "24:%d",  c24);  cv.drawString(buf, 92,  srcY3);
      cv.setTextColor(COL_OK,   C_BG); snprintf(buf, sizeof(buf), "5g:%d",  c5);   cv.drawString(buf, 118, srcY3);
    }
  }
}

// ---------------------------------------------------------------------------
static void drawStubScreen(const char* title, const char* note) {
  drawTopBar(title);
  cv.setTextDatum(middle_center);
  cv.setTextColor(COL_HEAD, COL_BG); cv.setTextSize(2);
  cv.drawString("COMING SOON", W / 2, CONTENT_Y + CONTENT_H / 2 - 8);
  cv.setTextSize(1); cv.setTextColor(COL_LABEL, COL_BG);
  cv.drawString(note, W / 2, CONTENT_Y + CONTENT_H / 2 + 12);
  drawBottomBar("--", "--", "--");
}

// ---------------------------------------------------------------------------
// STATS (battery / temperature / system)
// ---------------------------------------------------------------------------
static void drawStatsScreen() {
  drawTopBar("STATS");
  cv.setTextDatum(top_left);
  cv.setTextSize(1);
  int y = CONTENT_Y + 4;
  char line[44];

  float c = temperatureRead();                 // ESP32-S3 die temp (reads hotter than the case)
  float f = c * 9.0f / 5.0f + 32.0f;
  cv.setTextColor(f >= 149.0f ? COL_BAD : COL_OK, COL_BG);   // 149F = 65C
  snprintf(line, sizeof(line), "CHIP TEMP   %.0f F", f);
  cv.drawString(line, 8, y); y += 15;

  cv.setTextColor(COL_TEXT, COL_BG);
  int mv  = M5.Power.getBatteryVoltage();
  int lvl = battPctFromMv(mv);
  snprintf(line, sizeof(line), "BATTERY     %d%%", lvl);        cv.drawString(line, 8, y); y += 13;
  snprintf(line, sizeof(line), "VOLTAGE     %.2f V", mv / 1000.0f); cv.drawString(line, 8, y); y += 13;

  unsigned long up = millis() / 1000;
  snprintf(line, sizeof(line), "UPTIME      %lum %lus", up / 60, up % 60); cv.drawString(line, 8, y); y += 13;

  if (WiFi.status() == WL_CONNECTED) {
    snprintf(line, sizeof(line), "WIFI        %d dBm", (int)WiFi.RSSI());
    cv.drawString(line, 8, y);
  } else {
    cv.setTextColor(COL_BAD, COL_BG);
    cv.drawString("WIFI        offline", 8, y);
  }

  char cTemp[10], cBat[10];
  snprintf(cTemp, sizeof(cTemp), "%.0fF", f);
  snprintf(cBat,  sizeof(cBat),  "%d%%", lvl);
  drawBottomBar(cBat, cTemp, "STATS");
}

// ---------------------------------------------------------------------------
// SURVEILLANCE
// ---------------------------------------------------------------------------
static void drawSurveilScreen() {
  drawTopBar("SURVEILLANCE");
  surveil::Detection dets[surveil::kMaxDetections];
  size_t n = surveil::snapshot(dets, surveil::kMaxDetections);

  // sort by confidence desc, then rssi desc for ties
  for (int i = 1; i < (int)n; ++i) {
    surveil::Detection tmp = dets[i]; int j = i - 1;
    while (j >= 0 && (dets[j].confidence < tmp.confidence ||
           (dets[j].confidence == tmp.confidence && dets[j].rssi < tmp.rssi))) {
      dets[j+1] = dets[j]; --j;
    }
    dets[j+1] = tmp;
  }

  cv.setTextDatum(top_left);
  cv.setTextSize(1);
  int y = CONTENT_Y + 2;

  if (n == 0) {
    cv.setTextColor(COL_LABEL, COL_BG);
    cv.drawString("PASSIVE DETECT", 8, y); y += 13;
    cv.setTextColor(COL_HEAD, COL_BG); cv.setTextSize(2);
    cv.drawString("--", 8, y); y += 20;
    cv.setTextSize(1); cv.setTextColor(COL_TEXT, COL_BG);
    cv.drawString("listening...", 8, y);
  } else {
    for (size_t i = 0; i < n && y + 12 <= H - BOTBAR_H; ++i) {
      const surveil::Detection& d = dets[i];
      uint16_t col = d.confidence >= 70 ? COL_BAD :
                     d.confidence >= 40 ? COL_HEAD : COL_LABEL;
      cv.setTextColor(col, COL_BG);
      char row[28];
      snprintf(row, sizeof(row), "%-14s %3d%%", d.label, (int)d.confidence);
      cv.drawString(row, 8, y); y += 12;
      cv.setTextColor(COL_LABEL, COL_BG);
      snprintf(row, sizeof(row), "%02X:%02X:%02X  %d dBm",
               d.mac[0], d.mac[1], d.mac[2], (int)d.rssi);
      cv.drawString(row, 8, y); y += 12;
    }
  }

  char c1[12]; snprintf(c1, sizeof(c1), "SURV %d", (int)n);
  drawBottomBar(c1, "--", "RF+BLE");
}

// ---------------------------------------------------------------------------
// RF SCAN — lilac feed: WiFi 2.4 GHz (native) + BLE + 5 GHz (via C5), newest first
// ---------------------------------------------------------------------------
static void drawSrcBadge(int x, int y, uint8_t src);   // defined after drawDroneFeed

static char          g_scan_top_mac[18] = "";
static float         g_scan_anim   = 0.0f;
static unsigned long g_scan_anim_t = 0;
static int           g_scan_view   = 0;

static struct { char mac[18]; unsigned long t; } g_rf_seen[48] = {};
static int g_rf_seen_n = 0;
static unsigned long rfFirstSeen(const char* mac, bool* is_new) {
  for (int i = 0; i < g_rf_seen_n; ++i)
    if (strncmp(g_rf_seen[i].mac, mac, 17) == 0) { if (is_new) *is_new = false; return g_rf_seen[i].t; }
  int idx;
  if (g_rf_seen_n < 48) idx = g_rf_seen_n++;
  else { idx = 0; for (int i = 1; i < 48; ++i) if (g_rf_seen[i].t < g_rf_seen[idx].t) idx = i; }
  strncpy(g_rf_seen[idx].mac, mac, 17); g_rf_seen[idx].mac[17] = '\0';
  g_rf_seen[idx].t = millis();
  if (is_new) *is_new = true;
  return g_rf_seen[idx].t;
}

static void drawScanScreen() {
  cv.fillRect(0, 0, W, H, F_BG);

  struct RfRow { uint8_t src; char id[24]; char mac[18]; int8_t rssi; };
  RfRow rows[40]; int nc = 0;

  // native WiFi APs — StickS3 radio is 2.4 GHz only
  for (int i = 0; i < g_ap_count && nc < 40; ++i) {
    RfRow& c = rows[nc++]; c.src = drone::SRC_WIFI_2G;
    strncpy(c.id, g_aps[i].ssid[0] ? g_aps[i].ssid : "(hidden)", sizeof(c.id) - 1); c.id[sizeof(c.id)-1] = '\0';
    strncpy(c.mac, g_aps[i].bssid, sizeof(c.mac) - 1); c.mac[sizeof(c.mac)-1] = '\0';
    c.rssi = g_aps[i].rssi;
  }
  // BLE devices
  drone::BleSight bs[24]; size_t nb = drone::bleSnapshot(bs, 24);
  for (size_t i = 0; i < nb && nc < 40; ++i) {
    RfRow& c = rows[nc++]; c.src = drone::SRC_BLE;
    const char* label = bs[i].name[0] ? bs[i].name : bleVendor(bs[i].company_id, bs[i].has_mfr);
    strncpy(c.id, label ? label : bs[i].mac, sizeof(c.id) - 1); c.id[sizeof(c.id)-1] = '\0';
    strncpy(c.mac, bs[i].mac, sizeof(c.mac) - 1); c.mac[sizeof(c.mac)-1] = '\0';
    c.rssi = bs[i].rssi;
  }
  // C5-reported WiFi (5 GHz + any 2.4 the native scan missed) — dormant until C5 emits W|
  c5link::WifiSight ws[24]; size_t nw = c5link::wifiSnapshot(ws, 24);
  for (size_t i = 0; i < nw && nc < 40; ++i) {
    bool dup = false;
    for (int j = 0; j < nc; ++j) if (strncmp(rows[j].mac, ws[i].bssid, 17) == 0) { dup = true; break; }
    if (dup) continue;
    RfRow& c = rows[nc++]; c.src = (ws[i].band == 5) ? drone::SRC_WIFI_5G : drone::SRC_WIFI_2G;
    strncpy(c.id, ws[i].ssid[0] ? ws[i].ssid : "(hidden)", sizeof(c.id) - 1); c.id[sizeof(c.id)-1] = '\0';
    strncpy(c.mac, ws[i].bssid, sizeof(c.mac) - 1); c.mac[sizeof(c.mac)-1] = '\0';
    c.rssi = ws[i].rssi;
  }

  int cBle = 0, c24 = 0, c5 = 0;
  for (int i = 0; i < nc; ++i)
    switch (rows[i].src) { case drone::SRC_WIFI_5G: ++c5; break; case drone::SRC_WIFI_2G: ++c24; break; default: ++cBle; }

  // sort: newest first-seen arrival at the top
  unsigned long fs[40];
  for (int i = 0; i < nc; ++i) { bool nw2; fs[i] = rfFirstSeen(rows[i].mac, &nw2); }
  int order[40]; for (int i = 0; i < nc; ++i) order[i] = i;
  for (int i = 1; i < nc; ++i) { int k = order[i]; int j = i - 1;
    while (j >= 0 && fs[order[j]] < fs[k]) { order[j+1] = order[j]; --j; } order[j+1] = k; }

  // top bar
  if (((millis() / 550) % 2) == 0) cv.fillCircle(9, 11, 2, F_ACCENT); else cv.drawCircle(9, 11, 2, F_DIM);
  cv.setTextSize(1);
  cv.setTextDatum(middle_left);  cv.setTextColor(F_DIM, F_BG); cv.drawString("rf scan", 17, 11);
  char tb[12]; cv.setTextDatum(middle_right);
  cv.setTextColor(F_DIM,    F_BG); snprintf(tb, sizeof(tb), "ble %d", cBle); cv.drawString(tb, 166, 11);
  cv.setTextColor(F_DIM,    F_BG); snprintf(tb, sizeof(tb), "2.4 %d", c24);  cv.drawString(tb, 200, 11);
  cv.setTextColor(F_ACCENT, F_BG); snprintf(tb, sizeof(tb), "5g %d",  c5);   cv.drawString(tb, 232, 11);
  cv.drawFastHLine(0, 22, W, F_HAIR);

  if (nc == 0) {
    cv.setTextDatum(top_left); cv.setTextColor(F_DIM, F_BG); cv.setTextSize(1);
    cv.drawString(g_wifi_scanning ? "scanning..." : "watching wifi + ble...", 8, F_TOP + 18);
    return;
  }

  // slide when a new MAC reaches the top (only while not scrolled down)
  if (strncmp(rows[order[0]].mac, g_scan_top_mac, 17) != 0) {
    strncpy(g_scan_top_mac, rows[order[0]].mac, 17); g_scan_top_mac[17] = '\0';
    if (g_scan_view == 0) { g_scan_anim = (float)F_ROW_H; g_scan_anim_t = millis(); }
  }
  if (g_scan_anim > 0.0f) {
    unsigned long now2 = millis();
    g_scan_anim -= (float)(now2 - g_scan_anim_t) * F_ROW_H / 300.0f; g_scan_anim_t = now2;
    if (g_scan_anim < 0.0f) g_scan_anim = 0.0f;
  }
  int off = (g_scan_view == 0) ? (int)(g_scan_anim + 0.5f) : 0;

  // clamp + scroll window
  if (g_scan_sel >= nc) g_scan_sel = nc - 1;
  if (g_scan_sel < 0)   g_scan_sel = 0;
  const int kVis = 5;
  if (g_scan_sel < g_scan_view)         g_scan_view = g_scan_sel;
  if (g_scan_sel >= g_scan_view + kVis) g_scan_view = g_scan_sel - kVis + 1;

  cv.setClipRect(0, F_TOP, W, F_WIN_H);
  for (int vis = 0; vis <= kVis && (g_scan_view + vis) < nc; ++vis) {
    int idx = g_scan_view + vis;
    const RfRow& c = rows[order[idx]];
    int y = F_TOP + vis * F_ROW_H - off;
    bool sel = (idx == g_scan_sel);
    uint16_t bg = sel ? F_SELROW : F_BG;
    if (sel) { cv.fillRect(4, y, 232, F_ROW_H, F_SELROW); cv.fillRect(4, y, 2, F_ROW_H, F_ACCENT); }
    drawSrcBadge(8, y + 4, c.src);
    cv.setTextSize(1);
    cv.setTextDatum(middle_left);  cv.setTextColor(F_TEXT, bg); cv.drawString(c.id, 40, y + 11);
    char meta[8]; snprintf(meta, sizeof(meta), "%d", c.rssi);
    cv.setTextDatum(middle_right); cv.setTextColor(F_DIM, bg);  cv.drawString(meta, 232, y + 11);
    cv.drawFastHLine(8, y + F_ROW_H - 1, 224, F_HAIR);
  }
  cv.clearClipRect();
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
static void drawSetupScreen() {
  drawTopBar("SETUP");
  cv.setTextDatum(top_left);
  cv.setTextSize(1);
  int y = CONTENT_Y + 4;

  if (!g_portal_active) {
    cv.setTextColor(COL_LABEL, COL_BG);
    cv.drawString("WiFi", 8, y); y += 13;
    cv.setTextColor(COL_TEXT, COL_BG);
    cv.drawString(g_cfg.ssid.c_str(), 8, y); y += 14;
    char loc[32];
    snprintf(loc, sizeof(loc), "%.4f, %.4f", g_cfg.lat, g_cfg.lon);
    cv.setTextColor(COL_LABEL, COL_BG);
    cv.drawString(loc, 8, y); y += 14;
    cv.setTextColor(COL_TEXT, COL_BG);
    cv.drawString("B = start portal", 8, y);
    char c1[14]; snprintf(c1, sizeof(c1), "%.13s", g_cfg.ssid.c_str());
    drawBottomBar(c1, "press B", "SETUP");
  } else {
    unsigned long elapsed = millis() - g_portal_start;
    unsigned long rem = elapsed < kPortalTimeoutMs ? (kPortalTimeoutMs - elapsed) / 1000 : 0;
    cv.setTextColor(COL_LABEL, COL_BG);
    cv.drawString("Join WiFi:", 8, y); y += 13;
    cv.setTextColor(COL_HEAD, COL_BG); cv.setTextSize(2);
    cv.drawString("Aware-Setup", 8, y); y += 20;
    cv.setTextSize(1); cv.setTextColor(COL_LABEL, COL_BG);
    cv.drawString("then open:", 8, y); y += 13;
    cv.setTextColor(COL_TEXT, COL_BG);
    cv.drawString("192.168.4.1", 8, y); y += 14;
    char timer[20]; snprintf(timer, sizeof(timer), "timeout %lus", rem);
    cv.setTextColor(COL_LABEL, COL_BG);
    cv.drawString(timer, 8, y);
    drawBottomBar("PORTAL", "ACTIVE", "A=exit");
  }
}

// ---------------------------------------------------------------------------
// DRONES FEED
// ---------------------------------------------------------------------------
static void drawSrcBadge(int x, int y, uint8_t src) {
  const char* lbl; uint16_t fill = 0, stroke = 0, txt;
  switch (src) {
    case drone::SRC_WIFI_5G: lbl = "5g";  fill   = F_ACCENT; txt = F_BG;   break;
    case drone::SRC_WIFI_2G: lbl = "2.4"; stroke = F_DIM;    txt = F_TEXT; break;
    default:                 lbl = "ble"; stroke = F_HAIR;   txt = F_DIM;  break;
  }
  if (fill) cv.fillRoundRect(x, y, 24, 14, 3, fill);
  else      cv.drawRoundRect(x, y, 24, 14, 3, stroke);
  cv.setTextDatum(middle_center);
  cv.setTextColor(txt, fill ? fill : F_BG);
  cv.setTextSize(1);
  cv.drawString(lbl, x + 12, y + 7);
}

static void drawDroneFeed() {
  cv.fillRect(0, 0, W, H, F_BG);

  drone::Drone arr[12];
  size_t n = drone::snapshot(arr, 12);

  // sort: newest first
  int order[12];
  for (size_t i = 0; i < n; ++i) order[i] = (int)i;
  for (size_t i = 1; i < n; ++i) {
    int k = order[i]; int j = (int)i - 1;
    while (j >= 0 && arr[order[j]].last_seen_ms < arr[k].last_seen_ms) { order[j+1] = order[j]; --j; }
    order[j+1] = k;
  }

  int cBle = 0, c24 = 0, c5 = 0;
  for (size_t i = 0; i < n; ++i)
    switch (arr[i].source) {
      case drone::SRC_WIFI_5G: ++c5;  break;
      case drone::SRC_WIFI_2G: ++c24; break;
      default:                 ++cBle; break;
    }

  // --- slim top bar: live dot + label + source tally ---
  if (((millis() / 550) % 2) == 0) cv.fillCircle(9, 11, 2, F_ACCENT);
  else                              cv.drawCircle(9, 11, 2, F_DIM);
  cv.setTextSize(1);
  cv.setTextDatum(middle_left);  cv.setTextColor(F_DIM, F_BG); cv.drawString("remote id", 17, 11);
  char tb[12];
  cv.setTextDatum(middle_right);
  cv.setTextColor(F_DIM,    F_BG); snprintf(tb, sizeof(tb), "ble %d", cBle); cv.drawString(tb, 166, 11);
  cv.setTextColor(F_DIM,    F_BG); snprintf(tb, sizeof(tb), "2.4 %d", c24);  cv.drawString(tb, 200, 11);
  cv.setTextColor(F_ACCENT, F_BG); snprintf(tb, sizeof(tb), "5g %d",  c5);   cv.drawString(tb, 232, 11);
  cv.drawFastHLine(0, 22, W, F_HAIR);

  if (n == 0) {
    cv.setTextDatum(top_left); cv.setTextColor(F_DIM, F_BG); cv.setTextSize(1);
    cv.drawString("watching ble + wifi...", 8, F_TOP + 18);
    return;
  }

  // --- new arrival at top -> kick off the slide ---
  if (strncmp(arr[order[0]].mac, g_feed_top_mac, 17) != 0) {
    strncpy(g_feed_top_mac, arr[order[0]].mac, 17); g_feed_top_mac[17] = '\0';
    g_feed_anim   = (float)F_ROW_H;
    g_feed_anim_t = millis();
  }
  if (g_feed_anim > 0.0f) {
    unsigned long now = millis();
    g_feed_anim -= (float)(now - g_feed_anim_t) * F_ROW_H / 300.0f;
    g_feed_anim_t = now;
    if (g_feed_anim < 0.0f) g_feed_anim = 0.0f;
  }
  int off = (int)(g_feed_anim + 0.5f);

  if (g_drone_sel >= (int)n) g_drone_sel = (int)n - 1;
  if (g_drone_sel < 0)       g_drone_sel = 0;

  // --- rows clipped to the feed window so slide enters/exits cleanly ---
  cv.setClipRect(0, F_TOP, W, F_WIN_H);
  for (size_t i = 0; i < n; ++i) {
    int y = F_TOP + (int)i * F_ROW_H - off;
    if (y >= 135 || y + F_ROW_H <= F_TOP) continue;
    const drone::Drone& d = arr[order[i]];
    bool sel = ((int)i == g_drone_sel);
    uint16_t bg = sel ? F_SELROW : F_BG;

    if (sel) {
      cv.fillRect(4, y, 232, F_ROW_H, F_SELROW);
      cv.fillRect(4, y, 2, F_ROW_H, F_ACCENT);
    }

    drawSrcBadge(8, y + 4, d.source);

    cv.setTextSize(1);
    cv.setTextDatum(middle_left); cv.setTextColor(F_TEXT, bg);
    cv.drawString(d.id[0] ? d.id : "drone", 40, y + 11);

    char meta[20];
    if (sel && d.has_op) {
      float pk = haversineKm(g_cfg.lat, g_cfg.lon, d.op_lat, d.op_lon);
      if (pk < 1.0f) snprintf(meta, sizeof(meta), "pilot %dm",    (int)lroundf(pk * 1000));
      else           snprintf(meta, sizeof(meta), "pilot %.1fkm", pk);
      cv.setTextDatum(middle_left); cv.setTextColor(F_ACCENT, bg);
      cv.drawString(meta, 100, y + 11);
    } else {
      snprintf(meta, sizeof(meta), "%d", d.rssi);
      cv.setTextDatum(middle_right); cv.setTextColor(F_DIM, bg);
      cv.drawString(meta, 186, y + 11);
    }

    cv.setTextDatum(middle_right); cv.setTextColor(F_TEXT, bg);
    if (d.has_loc) {
      float km = haversineKm(g_cfg.lat, g_cfg.lon, d.lat, d.lon);
      if (km < 1.0f) snprintf(meta, sizeof(meta), "%dm",    (int)lroundf(km * 1000));
      else           snprintf(meta, sizeof(meta), "%.1fkm", km);
    } else {
      snprintf(meta, sizeof(meta), "--");
    }
    cv.drawString(meta, 232, y + 11);

    cv.drawFastHLine(8, y + F_ROW_H - 1, 224, F_HAIR);
  }
  cv.clearClipRect();
}

// ---------------------------------------------------------------------------
static void render() {
  cv.fillScreen(COL_BG);
  switch (g_screen) {
    case SCR_AIRSPACE: drawAirspaceScreen(); break;
    case SCR_DRONES:   drawDroneFeed(); break;
    case SCR_SURVEIL:  drawSurveilScreen(); break;
    case SCR_SCAN:     drawScanScreen(); break;
    case SCR_STATS:    drawStatsScreen(); break;
    case SCR_SETUP:    drawSetupScreen(); break;
  }
  cv.pushSprite(0, 0);
}

static void pollKeepAlive() { M5.update(); }

// ---------------------------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.printf("PSRAM: %u bytes\n", (unsigned)ESP.getPsramSize());
  setCpuFrequencyMhz(160);
  M5.Display.setRotation(1);
  M5.Display.setBrightness(90);
  cv.setColorDepth(16);
  cv.createSprite(W, H);

  cv.fillScreen(COL_BG);
  cv.setTextDatum(middle_center);
  cv.setTextColor(COL_TEXT, COL_BG); cv.setTextSize(1);
  cv.drawString("Aware", W / 2, H / 2 - 6);
  cv.drawString("starting...", W / 2, H / 2 + 8);
  cv.pushSprite(0, 0);

  loadCfg();
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_cfg.ssid.c_str(), g_cfg.pass.c_str());
  WiFi.setSleep(true);
  services::adsb::setPollFn(pollKeepAlive);

  surveil::begin();
  drone::begin();  // BLE Remote ID scan + surveil ingest run continuously from here on
  c5link::begin();
  g_bleScanRunning = true;
}

static void animateCardFlip(int newRot) {
  const float cx = W / 2.0f, cy = H / 2.0f;
  const int   half = 7;        // frames per phase (~14 total)
  const float zmin = 0.03f;    // how thin the "edge-on" line gets
  cv.setPivot(W / 2.0f, H / 2.0f);

  auto frame = [&](float angle, float zx) {
    int bandHalf = (int)ceilf(W * zx * 0.5f) + 1;
    M5.Display.startWrite();
    if (bandHalf < (int)cx) {
      M5.Display.fillRect(0, 0, (int)cx - bandHalf, H, COL_BG);
      M5.Display.fillRect((int)cx + bandHalf, 0, W - ((int)cx + bandHalf), H, COL_BG);
    }
    cv.pushRotateZoom(cx, cy, angle, zx, 1.0f);
    M5.Display.endWrite();
  };

  for (int i = 1; i <= half; ++i) {        // squash current view to a line
    float t = (float)i / half;
    frame(0.0f, 1.0f - (1.0f - zmin) * (t * t));        // ease-in
  }
  for (int i = 0; i <= half; ++i) {        // grow the flipped view back out
    float t = (float)i / half;
    frame(180.0f, zmin + (1.0f - zmin) * (t * (2.0f - t)));  // ease-out
  }

  M5.Display.setRotation(newRot);
  render();
}

void loop() {
  M5.update();
  c5link::poll();  // drain C5 UART every iteration so it never backs up
  { static unsigned long s_c5diag_t = 0;
    if (millis() - s_c5diag_t > 1000) {
      s_c5diag_t = millis();
      char b[180]; c5link::diag(b, sizeof(b));
      Serial.printf("[c5diag] %s\n", b);
    } }

  // pump web config portal when active; enforce 3-min timeout
  if (g_portal_active) {
    if (g_srv) g_srv->handleClient();
    if (millis() - g_portal_start > kPortalTimeoutMs) stopConfigPortal();
  }

  // BtnA: exit portal (stays on SETUP) or cycle screens normally
  if (M5.BtnA.wasPressed()) {
    if (g_portal_active) stopConfigPortal();
    else g_screen = (g_screen + 1) % SCR_COUNT;
    render();
  }

  // BtnB: click = next item / start portal on SETUP; hold (airspace) = cycle range
  if (M5.BtnB.wasClicked()) {
    if (g_screen == SCR_AIRSPACE)                       ++g_airspace_sel;  // clamped on next render
    else if (g_screen == SCR_DRONES)                    ++g_drone_sel;     // clamped on next render
    else if (g_screen == SCR_SCAN)                      g_scan_sel = g_scan_sel + 1;  // wrapped on next render
    else if (g_screen == SCR_SETUP && !g_portal_active) startConfigPortal();
    render();
  }
  if (M5.BtnB.wasHold() && g_screen == SCR_AIRSPACE) {
    g_range_idx = (g_range_idx + 1) % appcfg::kRangePresetCount;
    g_last_fetch = 0;  // force refetch at new radius
    render();
  }

  unsigned long now = millis();
  if (g_screen == SCR_AIRSPACE && WiFi.status() == WL_CONNECTED && !g_portal_active &&
      (now - g_last_fetch >= appcfg::kFetchIntervalMs || g_last_fetch == 0)) {
    Serial.printf("fetch: WiFi=%d lat=%.4f lon=%.4f range=%.1fkm\n",
                  WiFi.status(), g_cfg.lat, g_cfg.lon, rangeKm() * 1.3f);
    bool ok = services::adsb::fetchUpdate(g_cfg.lat, g_cfg.lon, rangeKm() * 1.3f);
    if (ok) {
      g_have_fetched = true;
      g_last_update_ms = millis();
      g_lastAdsbOkMs   = millis();
      recomputePlanes();
      if (g_pcount > 0) g_lastDataMs = millis();
    }
    Serial.printf("fetch: done ok=%d planes=%d\n", ok, g_pcount);
    g_last_fetch = millis();
    render();
  }

  static unsigned long last_prune = 0;
  if (now - last_prune > 2000) { last_prune = now; drone::prune(); surveil::prune(); }

  // IMU: auto-rotate (low-pass + dominant-axis, animated) + motion-idle (~10 Hz)
  static unsigned long last_imu    = 0;
  static float         gx = 0, gy = 0, gz = 0;  // low-pass gravity estimate
  static float         last_mag    = 1.0f;
  static unsigned long last_motion = 0;
  if (now - last_imu > 100) {
    last_imu = now;
    float ax, ay, az;
    M5.Imu.update();
    M5.Imu.getAccel(&ax, &ay, &az);
    const float k = 0.15f;
    gx += k * (ax - gx); gy += k * (ay - gy); gz += k * (az - gz);
    float h = (fabsf(gx) >= fabsf(gy)) ? gx : gy;  // dominant horizontal axis
    if (fabsf(h) > 0.5f) {                           // only when clearly landscape
      static int rot = 1;
      int want = (h > 0) ? 1 : 3;                    // swap 1 and 3 if screen ends up upside-down
      if (want != rot) { rot = want; animateCardFlip(rot); }
    }
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    if (fabsf(mag - last_mag) > 0.06f) last_motion = now;
    last_mag = mag;
  }

  static unsigned long last_batt = 0;
  static int low_reads = 0;
  if (now - last_batt > 5000) {
    last_batt = now;
    int  mv    = M5.Power.getBatteryVoltage();
    bool onUsb = M5.Power.isCharging();
    bool idle  = (now - last_motion) > 60000UL;
    if (onUsb || mv <= 0) {
      low_reads = 0;
      applyBrightness(false, mv, onUsb);     // on charge: restore brightness regardless of idle
    } else if (mv < 3100) {
      if (++low_reads >= 3) {                // ~15 s sustained before sleeping
        cv.fillScreen(COL_BG);
        cv.setTextColor(COL_BAD, COL_BG);
        cv.setTextDatum(middle_center);
        cv.drawString("LOW BATTERY", W / 2, H / 2);
        cv.pushSprite(0, 0);
        delay(1500);
        // TODO: configure a button wake source before sleeping so the unit can
        // be revived without a power cycle. Verify the StickS3 BtnA/PWR GPIO
        // from M5Unified / M5 docs first — do NOT guess the pin.
        esp_deep_sleep_start();
      }
    } else {
      low_reads = 0;
      applyBrightness(idle, mv, onUsb);
    }
  }

  // RF SCAN / SURVEILLANCE screens: async 2.4 GHz AP sweep.
  // Briefly perturbs the STA link; runs while either screen is active.
  if (g_screen == SCR_SCAN || g_screen == SCR_SURVEIL) {
    if (!g_wifi_scanning && now - g_last_wifi_scan > 5000) {
      WiFi.scanNetworks(true /*async*/, false /*show_hidden*/);
      g_wifi_scanning = true;
    }
    if (g_wifi_scanning) {
      int n = WiFi.scanComplete();
      if (n >= 0) {
        g_ap_count = 0;
        for (int i = 0; i < n; ++i) {
          uint8_t bssid6[6];
          memcpy(bssid6, WiFi.BSSID(i), 6);
          surveil::ingestWifi(WiFi.SSID(i).c_str(), bssid6, (int8_t)WiFi.RSSI(i));
          if (g_ap_count < 12) {
            strncpy(g_aps[g_ap_count].ssid, WiFi.SSID(i).c_str(), sizeof(g_aps[0].ssid) - 1);
            g_aps[g_ap_count].ssid[sizeof(g_aps[0].ssid) - 1] = '\0';
            String bs = WiFi.BSSIDstr(i);
            strncpy(g_aps[g_ap_count].bssid, bs.c_str(), sizeof(g_aps[0].bssid) - 1);
            g_aps[g_ap_count].bssid[sizeof(g_aps[0].bssid) - 1] = '\0';
            g_aps[g_ap_count].rssi = (int8_t)WiFi.RSSI(i);
            g_aps[g_ap_count].ch   = (uint8_t)WiFi.channel(i);
            g_ap_count++;
          }
        }
        WiFi.scanDelete();
        g_wifi_scanning = false;
        g_last_wifi_scan = millis();
        render();
      }
    }
  }

  // nudge WiFi back if it drops; not during a scan or portal
  static unsigned long last_recon = 0;
  if (WiFi.status() != WL_CONNECTED && now - last_recon > 10000 &&
      g_screen != SCR_SCAN && g_screen != SCR_SURVEIL && !g_wifi_scanning && !g_portal_active) {
    last_recon = now; WiFi.reconnect();
  }

  static unsigned long last_idle = 0;
  if (now - last_idle > 1000) { last_idle = now; render(); }

  // Drone feed runs at ~30 fps to keep the slide animation smooth
  if (g_screen == SCR_DRONES) {
    static unsigned long last_drone_render = 0;
    if (now - last_drone_render > 33) { last_drone_render = now; render(); }
  }

  delay(10);
}
