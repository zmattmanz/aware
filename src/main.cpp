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

// ---- layout ----------------------------------------------------------------
static constexpr int W = 240, H = 135;
static constexpr int TOPBAR_H = 17, BOTBAR_H = 19;
static constexpr int CONTENT_Y = TOPBAR_H + 1;
static constexpr int CONTENT_H = H - TOPBAR_H - BOTBAR_H - 2;

static M5Canvas cv(&M5.Display);

// ---- screens ---------------------------------------------------------------
enum Screen { SCR_PLANES = 0, SCR_DRONES, SCR_SURVEIL, SCR_SCAN, SCR_STATS, SCR_SETUP, SCR_COUNT };
static int g_screen = SCR_PLANES;

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
static int           g_plane_sel     = 0;  // index into g_porder
static inline float  rangeKm() { return appcfg::kRangePresetsKm[g_range_idx]; }

// ---- drone state -----------------------------------------------------------
static int           g_drone_sel     = 0;

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

static void drawPlaneMarker(int cx, int cy, float headingDeg, uint16_t col) {
  float a = (float)deg2rad(headingDeg), s = sinf(a), c = cosf(a);
  auto rot = [&](float fx, float fy, int& ox, int& oy) {
    ox = cx + (int)lroundf(fx * c + fy * s);
    oy = cy + (int)lroundf(fx * s - fy * c);
  };
  int x0, y0, x1, y1, x2, y2;
  rot(0, 5, x0, y0); rot(-3, -4, x1, y1); rot(3, -4, x2, y2);
  cv.fillTriangle(x0, y0, x1, y1, x2, y2, col);
}

// horizontal RSSI proximity bar (-90..-30 dBm -> empty..full)
static void drawRssiBar(int x, int y, int w, int8_t rssi) {
  cv.drawRect(x, y, w, 6, COL_LABEL);
  float t = (rssi + 90) / 60.0f;
  if (t < 0) t = 0; if (t > 1) t = 1;
  cv.fillRect(x + 1, y + 1, (int)((w - 2) * t), 4, COL_OK);
}

// ---------------------------------------------------------------------------
// PLANES
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
  if (g_plane_sel >= g_pcount) g_plane_sel = (g_pcount > 0) ? g_pcount - 1 : 0;
}

static void drawPlaneScreen() {
  drawTopBar("PLANE RADAR");

  {
    char upd[12];
    if (g_last_update_ms == 0) {
      strcpy(upd, "upd --");
    } else {
      unsigned long age = (millis() - g_last_update_ms) / 1000;
      if (age < 60) snprintf(upd, sizeof(upd), "upd %lus", age);
      else          snprintf(upd, sizeof(upd), "upd %lum", age / 60);
    }
    bool stale = g_last_update_ms != 0 &&
                 (millis() - g_last_update_ms) > (2UL * appcfg::kFetchIntervalMs);
    cv.setTextColor(stale ? COL_BAD : COL_LABEL, COL_BAR);
    cv.setTextSize(1);
    cv.setTextDatum(middle_right);
    cv.drawString(upd, W - 50, TOPBAR_H / 2);
  }

  const int cx = 60, cy = CONTENT_Y + CONTENT_H / 2, R = 42;
  cv.drawCircle(cx, cy, R, COL_RING);
  cv.drawCircle(cx, cy, (R * 2) / 3, COL_RING);
  cv.drawCircle(cx, cy, R / 3, COL_RING);
  cv.drawLine(cx - R, cy, cx + R, cy, COL_RING);
  cv.drawLine(cx, cy - R, cx, cy + R, COL_RING);
  cv.fillCircle(cx, cy, 2, COL_TEXT);

  const services::adsb::Aircraft* L = services::adsb::aircraftList();
  int sel_ac = (g_pcount > 0) ? g_porder[g_plane_sel] : -1;
  for (int i = 0; i < g_pcount; ++i) {
    float rn = g_pdist[i] / rangeKm();
    float a  = (float)deg2rad(g_pbrg[i]);
    int px, py;
    if (rn > 1.0f) {
      px = cx + (int)lroundf(sinf(a) * R);
      py = cy - (int)lroundf(cosf(a) * R);
      cv.fillCircle(px, py, 1, COL_PLANE);
    } else {
      px = cx + (int)lroundf(sinf(a) * rn * R);
      py = cy - (int)lroundf(cosf(a) * rn * R);
      drawPlaneMarker(px, py, L[i].track_deg, COL_PLANE);
    }
    if (i == sel_ac) cv.drawCircle(px, py, 6, COL_SEL);  // selection ring
  }

  const int rx = 124;
  cv.setTextSize(1);
  cv.setTextDatum(top_left);
  cv.setTextColor(COL_LABEL, COL_BG);
  cv.drawString("SELECTED", rx, CONTENT_Y + 2);

  if (sel_ac < 0) {
    bool wifi = (WiFi.status() == WL_CONNECTED);
    cv.setTextColor(COL_HEAD, COL_BG); cv.setTextSize(2);
    cv.drawString(wifi ? "--" : "WIFI", rx, CONTENT_Y + 14);
    cv.setTextSize(1);
    cv.setTextColor(wifi ? COL_TEXT : COL_HEAD, COL_BG);
    cv.drawString(wifi ? (g_have_fetched ? "no aircraft" : "scanning...") : "connecting...",
                  rx, CONTENT_Y + 34);
    if (!wifi) { cv.setTextColor(COL_LABEL, COL_BG); cv.drawString("no wifi", rx, CONTENT_Y + 46); }
  } else {
    const services::adsb::Aircraft& ac = L[sel_ac];
    char cs[10]; snprintf(cs, sizeof(cs), "%s", ac.callsign[0] ? ac.callsign : "----");
    cv.setTextColor(COL_HEAD, COL_BG); cv.setTextSize(2);
    cv.drawString(cs, rx, CONTENT_Y + 13);
    cv.setTextSize(1); cv.setTextColor(COL_TEXT, COL_BG);
    char line[24];
    snprintf(line, sizeof(line), "%.1f mi %s", g_pdist[sel_ac] * 0.621371f, compass8(g_pbrg[sel_ac]));
    cv.drawString(line, rx, CONTENT_Y + 34);
    cv.drawString(ac.alt[0] ? ac.alt : "-- ft", rx, CONTENT_Y + 46);
    snprintf(line, sizeof(line), "%d kt", (int)lroundf(ac.gs_knots));
    cv.drawString(line, rx, CONTENT_Y + 58);
  }

  char c1[16], c2[16], c3[16];
  snprintf(c1, sizeof(c1), "PLANES %d", g_pcount);
  if (g_pcount > 0) snprintf(c2, sizeof(c2), "#%d/%d", g_plane_sel + 1, g_pcount);
  else              snprintf(c2, sizeof(c2), "#-/-");
  snprintf(c3, sizeof(c3), "RNG %dkm", (int)rangeKm());
  drawBottomBar(c1, c2, c3);
}

// ---------------------------------------------------------------------------
// DRONES
// ---------------------------------------------------------------------------
static void drawDroneScreen() {
  drawTopBar("DRONES");

  drone::Drone arr[12];
  size_t n = drone::snapshot(arr, 12);

  // order by signal strength (strongest first)
  int order[12];
  for (size_t i = 0; i < n; ++i) order[i] = (int)i;
  for (size_t i = 1; i < n; ++i) {
    int key = order[i]; int j = (int)i - 1;
    while (j >= 0 && arr[order[j]].rssi < arr[key].rssi) { order[j + 1] = order[j]; --j; }
    order[j + 1] = key;
  }
  if (g_drone_sel >= (int)n) g_drone_sel = (n > 0) ? (int)n - 1 : 0;

  if (n == 0) {
    cv.setTextDatum(top_left);
    cv.setTextColor(COL_LABEL, COL_BG); cv.setTextSize(1);
    cv.drawString("REMOTE ID", 8, CONTENT_Y + 2);
    cv.setTextColor(COL_HEAD, COL_BG); cv.setTextSize(2);
    cv.drawString("--", 8, CONTENT_Y + 16);
    cv.setTextColor(COL_TEXT, COL_BG); cv.setTextSize(1);
    cv.drawString("watching (BLE)...", 8, CONTENT_Y + 40);
    drawBottomBar("DRONES 0", "#-/-", "BLE");
    return;
  }

  const drone::Drone& d = arr[order[g_drone_sel]];
  cv.setTextDatum(top_left);
  cv.setTextColor(COL_LABEL, COL_BG); cv.setTextSize(1);
  cv.drawString("UAS ID", 8, CONTENT_Y + 2);

  cv.setTextColor(COL_DRONE, COL_BG); cv.setTextSize(2);
  char id[16]; snprintf(id, sizeof(id), "%s", d.id[0] ? d.id : "DRONE");
  cv.drawString(id, 8, CONTENT_Y + 13);

  cv.setTextSize(1);
  cv.setTextColor(COL_TEXT, COL_BG);
  char line[28];
  snprintf(line, sizeof(line), "%d dBm", d.rssi);
  cv.drawString(line, 8, CONTENT_Y + 36);
  drawRssiBar(70, CONTENT_Y + 37, 70, d.rssi);

  int yy = CONTENT_Y + 50;
  if (d.has_loc) {
    float dk = haversineKm(g_cfg.lat, g_cfg.lon, d.lat, d.lon);
    float br = bearingDeg(g_cfg.lat, g_cfg.lon, d.lat, d.lon);
    snprintf(line, sizeof(line), "uav %.1fmi %s  %dmph", dk * 0.621371f, compass8(br),
             (int)lroundf(d.speed_mps * 2.23694f));
    cv.drawString(line, 8, yy); yy += 12;
  }
  if (d.has_op) {
    float ok = haversineKm(g_cfg.lat, g_cfg.lon, d.op_lat, d.op_lon);
    float ob = bearingDeg(g_cfg.lat, g_cfg.lon, d.op_lat, d.op_lon);
    cv.setTextColor(COL_HEAD, COL_BG);
    snprintf(line, sizeof(line), "PILOT %.1fmi %s", ok * 0.621371f, compass8(ob));
    cv.drawString(line, 8, yy);
    cv.setTextColor(COL_TEXT, COL_BG);
  }

  char c1[16], c2[16];
  snprintf(c1, sizeof(c1), "DRONES %d", (int)n);
  snprintf(c2, sizeof(c2), "#%d/%d", g_drone_sel + 1, (int)n);
  drawBottomBar(c1, c2, d.has_op ? "PILOT!" : "BLE");
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
// RF SCAN — merged WiFi 2.4 GHz + BLE feed, sorted by RSSI
// ---------------------------------------------------------------------------
static void drawScanScreen() {
  struct RfContact { uint8_t src; char id[24]; char mac[18]; int8_t rssi; };
  constexpr int kMaxC = 36;
  RfContact contacts[kMaxC];
  int nc = 0;

  for (int i = 0; i < g_ap_count && nc < kMaxC; ++i) {
    RfContact& c = contacts[nc++];
    c.src = 0;
    strncpy(c.id, g_aps[i].ssid[0] ? g_aps[i].ssid : "(hidden)", sizeof(c.id) - 1);
    c.id[sizeof(c.id) - 1] = '\0';
    strncpy(c.mac, g_aps[i].bssid, sizeof(c.mac) - 1);
    c.mac[sizeof(c.mac) - 1] = '\0';
    c.rssi = g_aps[i].rssi;
  }

  drone::BleSight bs[24];
  size_t nb = drone::bleSnapshot(bs, 24);
  for (size_t i = 0; i < nb && nc < kMaxC; ++i) {
    RfContact& c = contacts[nc++];
    c.src = 1;
    const char* label = bs[i].name[0] ? bs[i].name
                      : bleVendor(bs[i].company_id, bs[i].has_mfr);
    strncpy(c.id, label ? label : bs[i].mac, sizeof(c.id) - 1);
    c.id[sizeof(c.id) - 1] = '\0';
    strncpy(c.mac, bs[i].mac, sizeof(c.mac) - 1);
    c.mac[sizeof(c.mac) - 1] = '\0';
    c.rssi = bs[i].rssi;
  }

  // sort by RSSI descending (insertion sort; n ≤ 36)
  for (int i = 1; i < nc; ++i) {
    RfContact tmp = contacts[i]; int j = i - 1;
    while (j >= 0 && contacts[j].rssi < tmp.rssi) { contacts[j+1] = contacts[j]; --j; }
    contacts[j+1] = tmp;
  }

  if (nc == 0) g_scan_sel = 0;
  else         g_scan_sel = g_scan_sel % nc;  // wrap

  drawTopBar("RF SCAN");
  cv.setTextDatum(top_left);
  cv.setTextSize(1);
  char hdr[20]; snprintf(hdr, sizeof(hdr), "CONTACTS %d", nc);
  cv.setTextColor(COL_HEAD, COL_BG);
  cv.drawString(hdr, 8, CONTENT_Y + 2);

  constexpr int kVis = 6;
  int page_start = (g_scan_sel / kVis) * kVis;
  int y = CONTENT_Y + 15;
  for (int i = page_start; i < nc && i < page_start + kVis; ++i) {
    const RfContact& c = contacts[i];
    cv.setTextColor(i == g_scan_sel ? COL_HEAD : COL_TEXT, COL_BG);
    char row[30];
    snprintf(row, sizeof(row), "%c %-19.19s%4d",
             c.src == 0 ? 'W' : 'B', c.id, (int)c.rssi);
    cv.drawString(row, 4, y); y += 13;
  }
  if (nc == 0) {
    cv.setTextColor(COL_LABEL, COL_BG);
    cv.drawString(g_wifi_scanning ? "scanning..." : "--", 8, y);
  }

  char c1[10], c2[10], c3[14];
  snprintf(c1, sizeof(c1), "RF %d", nc);
  if (nc > 0) snprintf(c2, sizeof(c2), "#%d/%d", g_scan_sel + 1, nc);
  else        snprintf(c2, sizeof(c2), "#-/-");
  snprintf(c3, sizeof(c3), "W%d B%u", g_ap_count, (unsigned)nb);
  drawBottomBar(c1, c2, c3);
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

static void render() {
  cv.fillScreen(COL_BG);
  switch (g_screen) {
    case SCR_PLANES:  drawPlaneScreen(); break;
    case SCR_DRONES:  drawDroneScreen(); break;
    case SCR_SURVEIL: drawSurveilScreen(); break;
    case SCR_SCAN:    drawScanScreen(); break;
    case SCR_STATS:   drawStatsScreen(); break;
    case SCR_SETUP:   drawSetupScreen(); break;
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
}

void loop() {
  M5.update();

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

  // BtnB: click = next item / start portal on SETUP; hold (planes) = cycle range
  if (M5.BtnB.wasClicked()) {
    if (g_screen == SCR_PLANES && g_pcount > 0) g_plane_sel = (g_plane_sel + 1) % g_pcount;
    else if (g_screen == SCR_DRONES)            g_drone_sel = g_drone_sel + 1;  // clamped on next render
    else if (g_screen == SCR_SCAN)              g_scan_sel  = g_scan_sel  + 1;  // wrapped on next render
    else if (g_screen == SCR_SETUP && !g_portal_active) startConfigPortal();
    render();
  }
  if (M5.BtnB.wasHold() && g_screen == SCR_PLANES) {
    g_range_idx = (g_range_idx + 1) % appcfg::kRangePresetCount;
    g_last_fetch = 0;  // force refetch at new radius
    render();
  }

  unsigned long now = millis();
  if (g_screen == SCR_PLANES && WiFi.status() == WL_CONNECTED && !g_portal_active &&
      (now - g_last_fetch >= appcfg::kFetchIntervalMs || g_last_fetch == 0)) {
    bool ok = services::adsb::fetchUpdate(g_cfg.lat, g_cfg.lon, rangeKm() * 1.3f);
    if (ok) { g_have_fetched = true; g_last_update_ms = millis(); recomputePlanes(); }
    g_last_fetch = millis();
    render();
  }

  static unsigned long last_prune = 0;
  if (now - last_prune > 2000) { last_prune = now; drone::prune(); surveil::prune(); }

  // IMU: auto-rotate (low-pass + dominant-axis) + motion-idle tracking (~8 Hz)
  static unsigned long last_imu    = 0;
  static int           imu_rot     = 1;
  static float         gx = 0, gy = 0, gz = 0;  // low-pass gravity estimate
  static float         last_mag    = 1.0f;
  static unsigned long last_motion = 0;
  if (now - last_imu > 125) {
    last_imu = now;
    float ax, ay, az;
    M5.Imu.update();
    M5.Imu.getAccel(&ax, &ay, &az);
    const float k = 0.15f;
    gx += k * (ax - gx); gy += k * (ay - gy); gz += k * (az - gz);
    float h = (fabsf(gx) >= fabsf(gy)) ? gx : gy;  // dominant horizontal axis
    if (fabsf(h) > 0.5f) {                           // only decide when clearly landscape
      int want = (h > 0) ? 1 : 3;                    // swap 1 and 3 if screen appears upside-down
      if (want != imu_rot) { imu_rot = want; M5.Display.setRotation(imu_rot); render(); }
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

  delay(10);
}
