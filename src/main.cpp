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
#include "c5_link.h"
#include <Preferences.h>
#include <WebServer.h>
#include "esp32-hal-cpu.h"
#include <esp_sleep.h>
#include "fonts/pjs_small.h"
#include "fonts/pjs_body.h"
#include "fonts/pjs_num.h"

// ---- palette ---------------------------------------------------------------
static constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
// ---- dark "ink" design system ----
static constexpr uint16_t BG      = rgb565(0x0C,0x0C,0x10);   // screen background
static constexpr uint16_t FG      = rgb565(0xF4,0xF4,0xF7);   // primary text
static constexpr uint16_t MUTE    = rgb565(0x8A,0x8A,0x94);   // secondary text / values
static constexpr uint16_t LINE    = rgb565(0x2A,0x2A,0x32);   // borders / hairlines
static constexpr uint16_t CARD    = rgb565(0x1A,0x1A,0x20);   // card fill
static constexpr uint16_t LILAC   = rgb565(0xB9,0xA6,0xF0);   // featured / action / accent
static constexpr uint16_t DEEP_ACC= rgb565(0x9B,0x7E,0xDC);   // deep accent (PURPLE w/o name clash)
static constexpr uint16_t VERD    = rgb565(0x34,0xC7,0x59);   // healthy dot / text (GREEN w/o name clash)
static constexpr uint16_t MINT    = rgb565(0x2E,0x81,0x59);   // healthy hero card (medium green)
static constexpr uint16_t HEAD_BG = rgb565(0xEC,0xED,0xEF);   // title pill (light)
static constexpr uint16_t HEAD_FG = rgb565(0x18,0x18,0x1B);   // title pill text (dark)
static constexpr uint16_t PILL_BG = rgb565(0x1C,0x1C,0x24);   // right header pill (dark)
// type pills: dark hue fill + light hue text; flip to light on selected (lilac) row
static constexpr uint16_t B5_BG=rgb565(0x3E,0x34,0x60), B5_FG=rgb565(0xC9,0xB8,0xF2);   // 5G
static constexpr uint16_t BB_BG=rgb565(0x23,0x4E,0x5A), BB_FG=rgb565(0x8F,0xD3,0xE5);   // BLE
static constexpr uint16_t B2_BG=rgb565(0x5E,0x44,0x26), B2_FG=rgb565(0xE3,0xB8,0x83);   // 2.4
static constexpr uint16_t BSEL_BG=rgb565(0xEC,0xEC,0xF2);                                // badge on selected row

// existing names kept, remapped to dark theme
static constexpr uint16_t C_BG=BG, C_TEXT=FG, C_DIM=MUTE, C_ACCENT=DEEP_ACC;
static constexpr uint16_t C_PANEL=CARD, C_HAIRLINE=LINE, C_ACCENT_LO=LILAC, C_SELROW=LILAC;
static constexpr uint16_t F_BG=C_BG, F_TEXT=C_TEXT, F_DIM=C_DIM, F_ACCENT=C_ACCENT, F_HAIR=C_HAIRLINE, F_SELROW=C_SELROW;

// status / radar palette
static constexpr uint16_t COL_BG=BG, COL_TEXT=FG, COL_LABEL=MUTE, COL_BAR=CARD;
static constexpr uint16_t COL_OK=VERD, COL_HEAD=rgb565(0xE3,0xA9,0x4C), COL_BAD=rgb565(0xF0,0x5A,0x5A);
static constexpr uint16_t COL_RING=rgb565(0x2E,0x2A,0x42), COL_PLANE=FG, COL_DRONE=LILAC, COL_SEL=LILAC;
static constexpr int F_ROW_H = 27;     // single-line rows (name only), ~4 visible
static constexpr int F_TOP   = 27;
static constexpr int F_WIN_H = 108;

// ---- layout ----------------------------------------------------------------
static constexpr int W = 240, H = 135;
static constexpr int TOPBAR_H = 17, BOTBAR_H = 19;
static constexpr int CONTENT_Y = TOPBAR_H + 1;
static constexpr int CONTENT_H = H - TOPBAR_H - BOTBAR_H - 2;

static M5Canvas cv(&M5.Display);

// Font switch helpers — swap cv.unloadFont() for cv.loadFont(pjs_xxx) once VLW data is in include/fonts/
// Spaces in setTextSize( N ) are intentional: prevent replace_all from matching these lines
static inline void fontSmall() { cv.loadFont(pjs_small); cv.setTextSize( 1 ); }  // PJS Medium 12px
static inline void fontBody () { cv.loadFont(pjs_body);  cv.setTextSize( 1 ); }  // PJS SemiBold 16px
static inline void fontNum  () { cv.loadFont(pjs_num);   cv.setTextSize( 1 ); }  // PJS Bold 30px

// ---- screens ---------------------------------------------------------------
enum Screen { SCR_AIRSPACE = 0, SCR_CONN, SCR_SCAN, SCR_BLE, SCR_STATS, SCR_SETUP, SCR_COUNT };
static int           g_screen      = SCR_AIRSPACE;
static unsigned long g_popup_until = 0;

static const char* screenName(int s) {
  switch (s) {
    case SCR_AIRSPACE: return "AIRSPACE";
    case SCR_CONN:     return "CONNECTIONS";
    case SCR_SCAN:     return "RF SCAN";
    case SCR_BLE:      return "BLE";
    case SCR_STATS:    return "STATS";
    case SCR_SETUP:    return "SETUP";
    default:           return "";
  }
}

static int           g_scan_sel  = 0;
static int           g_scan_view = 0;
static bool          g_paused    = false;
static bool          g_detail    = false;

static void gotoScreen(int s) {
  s = ((s % SCR_COUNT) + SCR_COUNT) % SCR_COUNT;
  if (s != g_screen) {
    g_screen = s;
    g_paused = false; g_detail = false;
    g_scan_sel = 0;   g_scan_view = 0;
    g_popup_until = millis() + 1200;
  }
}

// ---- RF scan screen state (2.4 GHz WiFi APs; BLE comes from drone_scan) -----
struct ApSight { char ssid[24]; char bssid[18]; int8_t rssi; uint8_t ch; };
static ApSight       g_aps[12];
static int           g_ap_count       = 0;
static bool          g_wifi_scanning  = false;
static unsigned long g_last_wifi_scan = 0;

// ---- plane state -----------------------------------------------------------
static int           g_range_idx      = appcfg::kRangeDefaultIdx;
static int           g_porder[64];   // aircraft indices sorted by distance asc
static float         g_pdist[64];    // km, indexed by aircraft index
static float         g_pbrg[64];     // deg
static int           g_pcount        = 0;
static inline float  rangeKm() { return appcfg::kRangePresetsKm[g_range_idx]; }

// ---- unified airspace selection (index into sorted contact list) -----------
static int           g_airspace_sel  = 0;

// ---- airspace source health ------------------------------------------------
static unsigned long g_lastAdsbOkMs    = 0;     // millis() of last successful fetch (0 = never)
static unsigned long g_lastDataMs      = 0;     // millis() of last plane or BLE contact
static bool          g_bleScanRunning  = false;
static bool          g_everReceivedData= false;  // latches true on first contact of any kind

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
static void drawDot(int x, int y, uint16_t c) { cv.fillSmoothCircle(x, y, 4, c); }

static void drawCard(int x, int y, int w, int h, uint16_t bg, bool border) {
  cv.fillSmoothRoundRect(x, y, w, h, 8, bg);
  if (border) cv.drawRoundRect(x, y, w, h, 8, LINE);
}

static void drawTypeBadge(int x, int cy, uint8_t src, bool onSel = false) {
  const char* lbl; uint16_t bg, fg;
  switch (src) {
    case drone::SRC_WIFI_5G: lbl = "5G";  bg = B5_BG; fg = B5_FG; break;
    case drone::SRC_WIFI_2G: lbl = "2.4"; bg = B2_BG; fg = B2_FG; break;
    default:                 lbl = "BLE"; bg = BB_BG; fg = BB_FG; break;
  }
  if (onSel) { bg = BSEL_BG; fg = HEAD_FG; }
  int w = 34, h = 18;
  cv.fillSmoothRoundRect(x, cy - h / 2, w, h, h / 2, bg);
  fontSmall(); cv.setTextDatum(middle_center); cv.setTextColor(fg, bg);
  cv.drawString(lbl, x + w / 2, cy);
}

static void drawPillHeader(const char* title, bool liveDot, const char* rightText) {
  int ph = 18, py = 5, px = 6, dot = liveDot ? 9 : 0;
  fontBody();
  int pw = cv.textWidth(title) + 16 + dot;
  cv.fillSmoothRoundRect(px, py, pw, ph, ph / 2, HEAD_BG);
  if (liveDot) cv.fillSmoothCircle(px + 11, py + ph / 2, 2, MUTE);
  cv.setTextColor(HEAD_FG, HEAD_BG); cv.setTextDatum(middle_left);
  cv.drawString(title, px + 8 + dot, py + ph / 2);
  if (rightText && rightText[0]) {
    fontSmall();
    int rw = cv.textWidth(rightText) + 16, rx = W - 6 - rw;
    cv.fillSmoothRoundRect(rx, py, rw, ph, ph / 2, PILL_BG);
    cv.setTextColor(FG, PILL_BG); cv.setTextDatum(middle_center);
    cv.drawString(rightText, rx + rw / 2, py + ph / 2);
  }
}

static void drawTopBar(const char* title) {
  char b[8]; snprintf(b, sizeof(b), "%d%%", battPctFromMv(M5.Power.getBatteryVoltage()));
  drawPillHeader(title, false, b);
}

static void drawBottomBar(const char* a, const char* b, const char* c) {
  int y = H - BOTBAR_H;
  cv.fillRect(0, y, W, BOTBAR_H, COL_BAR);
  cv.drawLine(80, y + 3, 80, H - 3, COL_BG);
  cv.drawLine(160, y + 3, 160, H - 3, COL_BG);
  fontSmall();
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
  cv.fillSmoothCircle(x,y,2,col);
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
  cv.fillScreen(BG);
  unsigned long now_ms = millis();

  auto lerp565 = [](uint16_t a, uint16_t b, float t) -> uint16_t {
    int ar=(a>>11)&0x1F, ag=(a>>5)&0x3F, ab=a&0x1F;
    int br=(b>>11)&0x1F, bg=(b>>5)&0x3F, bb=b&0x1F;
    return (uint16_t)(((ar+(int)((br-ar)*t))<<11)|((ag+(int)((bg-ag)*t))<<5)|(ab+(int)((bb-ab)*t)));
  };

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

  // --- Pill header ---
  bool dataLive = (g_lastDataMs > 0) && ((now_ms - g_lastDataMs) < 8000UL);
  char hdrR[20];
  if (nContacts > 0)
    snprintf(hdrR, sizeof(hdrR), "%d %s", nContacts, nContacts == 1 ? "contact" : "contacts");
  else if (state == AirState::CLEAR)      snprintf(hdrR, sizeof(hdrR), "clear");
  else if (state == AirState::ACQUIRING)  snprintf(hdrR, sizeof(hdrR), "scanning");
  else                                    snprintf(hdrR, sizeof(hdrR), "offline");
  drawPillHeader("AIRSPACE", dataLive, hdrR);

  // --- Radar scope (left, cx=64, cy=82, R=46) ---
  static constexpr int RCX = 64, RCY = 82, RRAD = 46;
  static constexpr uint16_t RADAR_BG = rgb565(0x10,0x10,0x18);

  cv.fillSmoothCircle(RCX, RCY, RRAD, RADAR_BG);
  cv.drawLine(RCX, RCY - RRAD - 2, RCX, RCY + RRAD + 2, LINE);
  cv.drawLine(RCX - RRAD - 2, RCY, RCX + RRAD + 2, RCY, LINE);

  // Sweep fade trail with fillArc
  {
    static float sweepDeg = 0.0f;
    sweepDeg += 5.0f;
    if (sweepDeg >= 360.0f) sweepDeg -= 360.0f;

    auto safeArc = [&](float a0, float a1, uint16_t col) {
      while (a0 < 0.0f) a0 += 360.0f;
      while (a1 < a0)   a1 += 360.0f;
      if (a1 > 360.0f) {
        cv.fillArc(RCX, RCY, 0, RRAD, a0, 360.0f, col);
        cv.fillArc(RCX, RCY, 0, RRAD, 0.0f, a1 - 360.0f, col);
      } else {
        cv.fillArc(RCX, RCY, 0, RRAD, a0, a1, col);
      }
    };

    const float STEP_W = 6.0f;
    const int SWEEP_STEPS = 10;
    for (int s = SWEEP_STEPS - 1; s >= 0; --s) {
      float a1 = sweepDeg - s * STEP_W;
      float a0 = a1 - STEP_W;
      float t  = 1.0f - (float)s / SWEEP_STEPS;
      safeArc(a0, a1, lerp565(RADAR_BG, LILAC, t * 0.55f));
    }
    float sa = sweepDeg * (float)M_PI / 180.0f;
    cv.drawLine(RCX, RCY,
                RCX + (int)lroundf(RRAD * sinf(sa)),
                RCY - (int)lroundf(RRAD * cosf(sa)),
                LILAC);
  }

  // Rings (drawn over sweep so they stay crisp)
  cv.drawCircle(RCX, RCY, RRAD,       LINE);
  cv.drawCircle(RCX, RCY, RRAD * 2/3, LINE);
  cv.drawCircle(RCX, RCY, RRAD / 3,   LILAC);

  // Contacts: planes first, drones on top
  for (int pass = 0; pass < 2; ++pass) {
    for (int i = 0; i < nContacts; ++i) {
      const AContact& c = contacts[i];
      if ((pass == 0) == c.is_drone) continue;
      float r = airspaceRadiusFor(c.is_drone, c.dist_m);
      float a = c.brg_deg * (float)M_PI / 180.0f;
      int px = RCX + (int)lroundf(r * sinf(a));
      int py = RCY - (int)lroundf(r * cosf(a));
      bool sel = (i == g_airspace_sel);
      if (c.is_drone) {
        if (sel) cv.drawCircle(px, py, 7, LILAC);
        cv.fillSmoothCircle(px, py, 3, DEEP_ACC);
      } else {
        float hdg = acList[c.idx].track_deg > 0.0f ? acList[c.idx].track_deg : c.brg_deg;
        if (sel) cv.drawCircle(px, py, 8, LILAC);
        drawPlane(px, py, hdg, 0.75f, sel ? LILAC : FG);
      }
    }
  }
  // You-are-here: lilac square at radar center
  cv.fillRect(RCX - 2, RCY - 2, 5, 5, LILAC);

  // --- Right panel ---
  static constexpr int cX = 124, cY = 78, cW = 112, cH = 46;
  int midRX = cX + cW / 2;

  // Source health dots (wifi · ble · c5)
  {
    const bool oks[]      = { wifiUp, g_bleScanRunning, c5link::linked() };
    const char* lbls[]    = { "wifi", "ble", "c5" };
    int dx = cX + 8;
    for (int k = 0; k < 3; ++k) {
      cv.fillSmoothCircle(dx, 30, 2, oks[k] ? VERD : COL_BAD);
      cv.setTextDatum(middle_left); fontSmall();
      cv.setTextColor(MUTE, BG);
      cv.drawString(lbls[k], dx + 5, 30);
      dx += 36;
    }
  }

  // Big count or empty-state label
  {
    cv.setTextDatum(middle_center);
    if (nContacts > 0) {
      fontBody(); cv.setTextColor(FG, BG);
      char cnt[4]; snprintf(cnt, sizeof(cnt), "%d", nContacts);
      cv.drawString(cnt, midRX, 52);
      fontSmall(); cv.setTextColor(MUTE, BG);
      cv.drawString(nContacts == 1 ? "contact" : "contacts", midRX, 66);
    } else {
      fontSmall(); cv.setTextColor(MUTE, BG);
      if      (state == AirState::ACQUIRING) cv.drawString("scanning...", midRX, 55);
      else if (state == AirState::CLEAR)     cv.drawString("sky clear",   midRX, 55);
      else                                   cv.drawString("offline",     midRX, 55);
    }
  }

  // Contact card for selected entry
  if (nContacts > 0 && g_airspace_sel < nContacts) {
    const AContact& sel = contacts[g_airspace_sel];
    drawCard(cX, cY, cW, cH, CARD, true);
    int tx = cX + 8, ty = cY + 6;
    fontSmall();

    if (sel.is_drone) {
      const drone::Drone& d = drones[sel.idx];
      cv.setTextDatum(top_left);
      cv.setTextColor(LILAC, CARD);
      cv.drawString(d.id[0] ? d.id : "DRONE", tx, ty);
      cv.setTextDatum(top_right);
      cv.setTextColor(droneSrcCol(d.source), CARD);
      cv.drawString(droneSrcTag(d.source), cX + cW - 8, ty);
      // distance + bearing
      char dist[16];
      if (sel.dist_m < 1000.0f) snprintf(dist, sizeof(dist), "%dm %s",    (int)sel.dist_m,        compass8(sel.brg_deg));
      else                      snprintf(dist, sizeof(dist), "%.1fkm %s",  sel.dist_m / 1000.0f,   compass8(sel.brg_deg));
      cv.setTextDatum(top_left); cv.setTextColor(MUTE, CARD);
      cv.drawString(dist, tx, ty + 14);
      // rssi or pilot dist on right
      cv.setTextDatum(top_right); cv.setTextColor(MUTE, CARD);
      if (d.has_op) {
        float opM = haversineKm(g_cfg.lat, g_cfg.lon, d.op_lat, d.op_lon) * 1000.0f;
        char op[16];
        if (opM < 1000.0f) snprintf(op, sizeof(op), "pilot %dm",    (int)opM);
        else               snprintf(op, sizeof(op), "pilot %.1fkm",  opM / 1000.0f);
        cv.drawString(op, cX + cW - 8, ty + 14);
      } else {
        char rssi[8]; snprintf(rssi, sizeof(rssi), "%ddBm", d.rssi);
        cv.drawString(rssi, cX + cW - 8, ty + 14);
      }
      // mac (dim, bottom line)
      cv.setTextDatum(top_left); cv.setTextColor(LINE, CARD);
      cv.drawString(d.mac, tx, ty + 28);
    } else {
      const services::adsb::Aircraft& ac = acList[sel.idx];
      cv.setTextDatum(top_left);
      cv.setTextColor(ac.emergency ? COL_BAD : FG, CARD);
      cv.drawString(ac.callsign[0] ? ac.callsign : "----", tx, ty);
      cv.setTextDatum(top_right);
      if (ac.mil) { cv.setTextColor(COL_HEAD, CARD); cv.drawString("MIL", cX + cW - 8, ty); }
      else if (ac.type[0]) { cv.setTextColor(MUTE, CARD); cv.drawString(ac.type, cX + cW - 8, ty); }
      // distance + bearing
      float nm = sel.dist_m / 1852.0f;
      char dist[16];
      if (nm < 10.0f) snprintf(dist, sizeof(dist), "%.1fnm %s", nm, compass8(sel.brg_deg));
      else            snprintf(dist, sizeof(dist), "%.0fnm %s",  nm, compass8(sel.brg_deg));
      cv.setTextDatum(top_left); cv.setTextColor(MUTE, CARD);
      cv.drawString(dist, tx, ty + 14);
      if (ac.alt[0]) { cv.setTextDatum(top_right); cv.setTextColor(MUTE, CARD); cv.drawString(ac.alt, cX + cW - 8, ty + 14); }
      // speed or emergency squawk
      cv.setTextDatum(top_left);
      if (ac.emergency && ac.squawk[0]) {
        cv.setTextColor(COL_BAD, CARD);
        char sq[10]; snprintf(sq, sizeof(sq), "SQK %s", ac.squawk);
        cv.drawString(sq, tx, ty + 28);
      } else if (ac.gs_knots > 0.0f) {
        cv.setTextColor(MUTE, CARD);
        char spd[10]; snprintf(spd, sizeof(spd), "%.0fkt", ac.gs_knots);
        cv.drawString(spd, tx, ty + 28);
      }
    }
  } else {
    drawCard(cX, cY, cW, cH, CARD, false);
    cv.setTextDatum(middle_center); fontSmall(); cv.setTextColor(MUTE, CARD);
    cv.drawString("no contact", cX + cW / 2, cY + cH / 2);
  }
}

// ---------------------------------------------------------------------------
static void drawStubScreen(const char* title, const char* note) {
  drawTopBar(title);
  cv.setTextDatum(middle_center);
  cv.setTextColor(COL_HEAD, COL_BG); fontBody();
  cv.drawString("COMING SOON", W / 2, CONTENT_Y + CONTENT_H / 2 - 8);
  fontSmall(); cv.setTextColor(COL_LABEL, COL_BG);
  cv.drawString(note, W / 2, CONTENT_Y + CONTENT_H / 2 + 12);
  drawBottomBar("--", "--", "--");
}

// ---------------------------------------------------------------------------
// STATS (battery / temperature / system)
// ---------------------------------------------------------------------------
static void drawStatsScreen() {
  drawTopBar("STATS");
  cv.setTextDatum(top_left);
  fontSmall();
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
// CONNECTIONS — at-a-glance health of each data channel
// ---------------------------------------------------------------------------
static void drawConnScreen() {
  cv.fillRect(0, 0, W, H, BG);

  // ---- data ----
  bool c5up = c5link::linked();
  c5link::WifiSight ws[24]; size_t nw = c5link::wifiSnapshot(ws, 24);
  int w5 = 0; for (size_t i = 0; i < nw; ++i) if (ws[i].band == 5) ++w5;
  size_t nAc  = services::adsb::aircraftCount();
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  static unsigned long s_lb = 0, s_lt = 0; static float s_kbps = 0;
  { unsigned long nm = millis(), b = c5link::rxBytes();
    if (s_lt && nm > s_lt) { float dt = (nm - s_lt) / 1000.0f;
      if (dt >= 0.5f) { s_kbps = ((float)(b - s_lb) / 1024.0f) / dt; s_lb = b; s_lt = nm; } }
    else { s_lb = b; s_lt = nm; } }
  if (s_kbps < 0 || !c5up) s_kbps = 0;

  // ---- hero (fills from the top, no header) ----
  int hx = 6, hy = 6, hw = 228, hh = 50;
  uint16_t hbg  = c5up ? MINT : CARD;
  uint16_t hsub = c5up ? rgb565(0xC8,0xE6,0xD4) : MUTE;
  drawCard(hx, hy, hw, hh, hbg, false);
  cv.fillSmoothCircle(hx + 12, hy + 22, 4, c5up ? VERD : COL_BAD);
  fontBody(); cv.setTextDatum(top_left); cv.setTextColor(FG, hbg);
  cv.setClipRect(hx + 24, hy, hw - 30, 24);
  cv.drawString("C5 co-processor", hx + 24, hy + 9);
  cv.clearClipRect();
  char kb[14];
  if (c5up) snprintf(kb, sizeof(kb), "%.1f KB/s", s_kbps); else snprintf(kb, sizeof(kb), "--");
  fontSmall();
  cv.setTextDatum(top_left);  cv.setTextColor(hsub, hbg);
  cv.drawString(c5up ? "linked - 5 GHz" : "no link", hx + 24, hy + 32);
  cv.setTextDatum(top_right); cv.setTextColor(c5up ? FG : MUTE, hbg);
  cv.drawString(kb, hx + hw - 12, hy + 32);

  // ---- 2x2 grid, even gutters, vertically centered ----
  auto tile = [&](int x, int y, int w, int h, const char* label, uint16_t dot, const char* val) {
    drawCard(x, y, w, h, CARD, false);
    int cy = y + h / 2;
    cv.fillSmoothCircle(x + 13, cy, 4, dot);
    fontSmall();
    cv.setTextDatum(middle_left);  cv.setTextColor(FG, CARD);   cv.drawString(label, x + 24, cy);
    cv.setTextDatum(middle_right); cv.setTextColor(MUTE, CARD); cv.drawString(val, x + w - 10, cy);
  };
  int gh = 31, tw = (hw - 6) / 2;
  int gy1 = hy + hh + 6, gy2 = gy1 + gh + 5;
  int x1 = hx, x2 = hx + tw + 6;
  char v[12];
  snprintf(v, sizeof(v), "%d dev", (int)drone::bleCount());
  tile(x1, gy1, tw, gh, "BLE",     g_bleScanRunning ? VERD : COL_BAD, v);
  snprintf(v, sizeof(v), "%d ap", g_ap_count);
  tile(x2, gy1, tw, gh, "2.4 GHz", (g_ap_count > 0) ? VERD : MUTE,    v);
  snprintf(v, sizeof(v), "%d ap", w5);
  tile(x1, gy2, tw, gh, "5 GHz",   c5up ? VERD : COL_BAD,             v);
  snprintf(v, sizeof(v), "%d ac", (int)nAc);
  tile(x2, gy2, tw, gh, "ADS-B",   (nAc > 0) ? VERD : (wifiUp ? MUTE : COL_BAD), v);
}

// ---------------------------------------------------------------------------
// RF SCAN — lilac feed: WiFi 2.4 GHz (native) + BLE + 5 GHz (via C5), newest first
// ---------------------------------------------------------------------------
static void drawSrcBadge(int x, int y, uint8_t src);   // forward decl; defined below
static void drawSignalBars(int x, int yBottom, int8_t rssi, uint16_t on);  // forward decl; defined below

struct RfRow { uint8_t src; char id[24]; char mac[18]; int8_t rssi; uint8_t tracker; };

static RfRow         g_disp[40];
static int           g_dispN  = 0;
static RfRow         g_detail_row{};

static drone::BleSight g_bleFrozen[24];
static int             g_bleFrozenN = 0;
static bool            g_bleTrkOnly = false;

enum RfFilter { RF_ALL = 0, RF_BLE, RF_24, RF_5, RF_NEAR, RF_COUNT };
static int           g_filter    = RF_ALL;
static bool          g_hide_home = true;
static const int8_t  kNearRssi   = -70;

static const char* filterName(int f) {
  switch (f) { case RF_BLE:  return "BLE";  case RF_24:   return "2.4G";
               case RF_5:    return "5G";   case RF_NEAR: return "near";
               default:      return "all"; }
}

static const char* signalWord(int8_t rssi) {
  if (rssi >= -60) return "strong";
  if (rssi >= -75) return "medium";
  return "weak";
}

static bool keepRow(uint8_t src, int8_t rssi, const char* id, const char* home_ssid) {
  if (home_ssid[0] && id && id[0] && strcmp(id, home_ssid) == 0) return false;
  switch (g_filter) {
    case RF_BLE:  return src == drone::SRC_BLE;
    case RF_24:   return src == drone::SRC_WIFI_2G;
    case RF_5:    return src == drone::SRC_WIFI_5G;
    case RF_NEAR: return rssi >= kNearRssi;
    default:      return true;
  }
}

static char          g_scan_top_mac[18] = "";
static float         g_scan_anim   = 0.0f;
static unsigned long g_scan_anim_t = 0;

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

static const char* trackerLabel(uint8_t t) {
  switch (t) {
    case drone::TRK_AIRTAG:   return "AirTag";
    case drone::TRK_TILE:     return "Tile";
    case drone::TRK_SMARTTAG: return "SmartTag";
    default:                  return "tracker";
  }
}

static void drawBleScreen() {
  cv.fillRect(0, 0, W, H, BG);

  drone::BleSight b[24];
  size_t n   = drone::bleSnapshot(b, 24);
  size_t trk = drone::trackerCount();

  // trackers first, then strongest RSSI
  for (size_t i = 0; i < n; ++i)
    for (size_t j = i + 1; j < n; ++j) {
      bool ti = b[i].tracker != drone::TRK_NONE, tj = b[j].tracker != drone::TRK_NONE;
      if ((tj && !ti) || (ti == tj && b[j].rssi > b[i].rssi)) { auto t = b[i]; b[i] = b[j]; b[j] = t; }
    }

  // header: light "BLE" pill + red "N trackers" pill (neutral when none)
  drawPillHeader("BLE", false, "");
  char rp[16];
  if      (trk == 0) snprintf(rp, sizeof(rp), "%u near", (unsigned)n);
  else if (trk == 1) snprintf(rp, sizeof(rp), "1 tracker");
  else               snprintf(rp, sizeof(rp), "%u trackers", (unsigned)trk);
  uint16_t rpbg = trk ? rgb565(0xF0,0x5A,0x5A) : PILL_BG;
  uint16_t rpfg = trk ? rgb565(0xFF,0xFF,0xFF) : FG;
  fontSmall(); int rw = cv.textWidth(rp) + 16, rx = W - 6 - rw;
  cv.fillSmoothRoundRect(rx, 5, rw, 18, 9, rpbg);
  cv.setTextColor(rpfg, rpbg); cv.setTextDatum(middle_center); cv.drawString(rp, rx + rw / 2, 14);

  if (n == 0) {
    cv.setTextDatum(middle_center); cv.setTextColor(MUTE, BG); fontSmall();
    cv.drawString("watching ble...", W / 2, H / 2 + 6);
    return;
  }

  const uint16_t TRK_BG  = rgb565(0x3A,0x1E,0x22);
  const uint16_t TRK_DOT = rgb565(0xF0,0x5A,0x5A);
  const uint16_t TRK_FG  = rgb565(0xEE,0x8C,0x8C);
  const uint16_t TRK_SUB = rgb565(0xE8,0x6E,0x6E);

  unsigned long now = millis();
  int shown = (int)(n < 4 ? n : 4);
  cv.setClipRect(0, F_TOP, W, F_WIN_H);
  for (int i = 0; i < shown; ++i) {
    int y = F_TOP + i * F_ROW_H, cy = y + (F_ROW_H - 4) / 2;
    bool isTrk = b[i].tracker != drone::TRK_NONE;
    uint16_t rbg = isTrk ? TRK_BG : CARD;
    cv.fillSmoothRoundRect(4, y, 232, F_ROW_H - 4, 8, rbg);

    const char* vnd = b[i].name[0] ? b[i].name : bleVendor(b[i].company_id, b[i].has_mfr);
    const char* label = isTrk ? trackerLabel(b[i].tracker) : (vnd ? vnd : b[i].mac);
    int nameX = 14;
    if (isTrk) { cv.fillSmoothCircle(16, cy, 4, TRK_DOT); nameX = 30; }

    char rs[8]; snprintf(rs, sizeof(rs), "%d", b[i].rssi);
    fontSmall(); cv.setTextDatum(middle_right);
    cv.setTextColor(isTrk ? TRK_SUB : MUTE, rbg); cv.drawString(rs, 226, cy);
    int rightEdge = 226 - cv.textWidth(rs) - 8;

    if (isTrk) {
      char dur[20]; snprintf(dur, sizeof(dur), "following %lus", (now - b[i].first_seen_ms) / 1000);
      cv.setTextDatum(middle_right); cv.setTextColor(TRK_SUB, rbg);
      cv.drawString(dur, rightEdge, cy);
      rightEdge -= cv.textWidth(dur) + 8;
    }

    int nameW = rightEdge - nameX;
    cv.setClipRect(nameX, F_TOP, nameW > 0 ? nameW : 0, F_WIN_H);
    fontBody(); cv.setTextDatum(middle_left);
    cv.setTextColor(isTrk ? TRK_FG : FG, rbg); cv.drawString(label, nameX, cy);
    cv.setClipRect(0, F_TOP, W, F_WIN_H);
  }
  cv.clearClipRect();
}

static uint16_t srcColor(uint8_t src) {
  switch (src) { case drone::SRC_WIFI_5G: return COL_OK;
                 case drone::SRC_WIFI_2G: return COL_HEAD;
                 default:                 return F_ACCENT; }
}

static void drawRfBadge(int x, int yTop, uint8_t src) {
  const char* lbl = (src == drone::SRC_WIFI_5G) ? "5G" : (src == drone::SRC_WIFI_2G) ? "2.4" : "BLE";
  cv.fillSmoothRoundRect(x, yTop, 34, 15, 3, srcColor(src));
  fontSmall(); cv.setTextDatum(middle_center); cv.setTextColor(F_BG, srcColor(src));
  cv.drawString(lbl, x + 17, yTop + 8);
}

static void drawSignalBars(int x, int yBottom, int8_t rssi, uint16_t on) {
  int lvl = (rssi >= -55) ? 4 : (rssi >= -67) ? 3 : (rssi >= -78) ? 2 : 1;
  const int h[4] = {4, 7, 10, 13};
  for (int i = 0; i < 4; ++i)
    cv.fillRect(x + i * 8, yBottom - h[i], 6, h[i], (i < lvl) ? on : F_HAIR);
}

static void drawScanScreen() {
  cv.fillRect(0, 0, W, H, F_BG);

  RfRow rows[40]; int nc = 0;
  if (g_paused) { nc = g_dispN; for (int i = 0; i < nc; ++i) rows[i] = g_disp[i]; }

  // Collapse a WiFi network to ONE row: merge by SSID (mesh APs share a name);
  // dedup hidden networks by BSSID. Keep the first BSSID (stable identity for the
  // slide animation) and upgrade to the strongest RSSI/band seen for that name.
  auto addWifi = [&](const char* ssid, const char* bssid, int8_t rssi, uint8_t src) {
    if (nc >= 40) return;
    if (ssid && ssid[0]) {
      for (int j = 0; j < nc; ++j)
        if (rows[j].src != drone::SRC_BLE &&
            strncmp(rows[j].id, ssid, sizeof(rows[j].id) - 1) == 0) {
          if (rssi > rows[j].rssi) { rows[j].rssi = rssi; rows[j].src = src; }
          return;
        }
    } else {
      for (int j = 0; j < nc; ++j)
        if (rows[j].src != drone::SRC_BLE && strncmp(rows[j].mac, bssid, 17) == 0) return;
    }
    RfRow& c = rows[nc++]; c.src = src;
    strncpy(c.id, (ssid && ssid[0]) ? ssid : "(hidden)", sizeof(c.id) - 1); c.id[sizeof(c.id) - 1] = '\0';
    strncpy(c.mac, bssid, sizeof(c.mac) - 1); c.mac[sizeof(c.mac) - 1] = '\0';
    c.rssi = rssi;
  };

  char home_ssid[33]; home_ssid[0] = '\0';
  if (g_hide_home && WiFi.status() == WL_CONNECTED) {
    strncpy(home_ssid, WiFi.SSID().c_str(), sizeof(home_ssid) - 1);
    home_ssid[sizeof(home_ssid) - 1] = '\0';
  }

  if (!g_paused) {
    // native WiFi APs — StickS3 radio is 2.4 GHz only
    for (int i = 0; i < g_ap_count; ++i) {
      if (!keepRow(drone::SRC_WIFI_2G, g_aps[i].rssi, g_aps[i].ssid, home_ssid)) continue;
      addWifi(g_aps[i].ssid, g_aps[i].bssid, g_aps[i].rssi, drone::SRC_WIFI_2G);
    }

    // BLE devices (added directly — skipped by addWifi's src != SRC_BLE guard)
    drone::BleSight bs[24]; size_t nb = drone::bleSnapshot(bs, 24);
    for (size_t i = 0; i < nb && nc < 40; ++i) {
      const char* label = bs[i].name[0] ? bs[i].name : bleVendor(bs[i].company_id, bs[i].has_mfr);
      const char* id    = label ? label : bs[i].mac;
      if (!keepRow(drone::SRC_BLE, bs[i].rssi, id, home_ssid)) continue;
      RfRow& c = rows[nc++]; c.src = drone::SRC_BLE;
      strncpy(c.id, id, sizeof(c.id) - 1); c.id[sizeof(c.id)-1] = '\0';
      strncpy(c.mac, bs[i].mac, sizeof(c.mac) - 1); c.mac[sizeof(c.mac)-1] = '\0';
      c.rssi = bs[i].rssi;
    }

    // C5-reported WiFi (5 GHz + any 2.4 the native scan missed)
    c5link::WifiSight ws[24]; size_t nw = c5link::wifiSnapshot(ws, 24);
    for (size_t i = 0; i < nw; ++i) {
      uint8_t src = (ws[i].band == 5) ? drone::SRC_WIFI_5G : drone::SRC_WIFI_2G;
      if (!keepRow(src, ws[i].rssi, ws[i].ssid, home_ssid)) continue;
      addWifi(ws[i].ssid, ws[i].bssid, ws[i].rssi, src);
    }
  }

  // sort: newest first-seen arrival at the top
  unsigned long fs[40];
  for (int i = 0; i < nc; ++i) { bool nw2; fs[i] = rfFirstSeen(rows[i].mac, &nw2); }
  int order[40]; for (int i = 0; i < nc; ++i) order[i] = i;
  for (int i = 1; i < nc; ++i) { int k = order[i]; int j = i - 1;
    while (j >= 0 && fs[order[j]] < fs[k]) { order[j+1] = order[j]; --j; } order[j+1] = k; }

  if (!g_paused) { for (int i = 0; i < nc; ++i) g_disp[i] = rows[order[i]]; g_dispN = nc; }

  // ---- header: light title pill + N found / filter / paused ----
  char rt[14];
  if (g_paused)                snprintf(rt, sizeof(rt), "paused");
  else if (g_filter != RF_ALL) snprintf(rt, sizeof(rt), "%s", filterName(g_filter));
  else                         snprintf(rt, sizeof(rt), "%d found", nc);
  bool scanLive = !g_paused && (((millis() / 600) % 2) == 0);
  drawPillHeader("RF SCAN", scanLive, rt);

  if (nc == 0) {
    cv.setTextDatum(middle_center); cv.setTextColor(MUTE, BG); fontSmall();
    cv.drawString(g_wifi_scanning ? "scanning..." : "watching wifi + ble...", W / 2, H / 2 + 6);
    return;
  }

  if (!g_paused && strncmp(rows[order[0]].mac, g_scan_top_mac, 17) != 0) {
    strncpy(g_scan_top_mac, rows[order[0]].mac, 17); g_scan_top_mac[17] = '\0';
    if (g_scan_view == 0 && g_scan_anim <= 0.0f) { g_scan_anim = (float)F_ROW_H; g_scan_anim_t = millis(); }
  }
  if (g_scan_anim > 0.0f) {
    unsigned long now2 = millis();
    float dt = (float)(now2 - g_scan_anim_t); g_scan_anim_t = now2;
    g_scan_anim *= expf(-dt / 180.0f);
    if (g_scan_anim < 0.5f) g_scan_anim = 0.0f;
  }
  int off = (g_scan_view == 0) ? (int)(g_scan_anim + 0.5f) : 0;

  if (g_scan_sel >= nc) g_scan_sel = nc - 1;
  if (g_scan_sel < 0)   g_scan_sel = 0;
  const int kVis = 3;
  if (g_scan_sel < g_scan_view)         g_scan_view = g_scan_sel;
  if (g_scan_sel >= g_scan_view + kVis) g_scan_view = g_scan_sel - kVis + 1;

  cv.setClipRect(0, F_TOP, W, F_WIN_H);
  for (int vis = 0; vis <= kVis && (g_scan_view + vis) < nc; ++vis) {
    int idx = g_scan_view + vis;
    const RfRow& c = rows[order[idx]];
    int y  = F_TOP + vis * F_ROW_H - off;
    int cy = y + (F_ROW_H - 4) / 2;
    bool sel = (idx == g_scan_sel);

    uint16_t rbg = sel ? LILAC : CARD;
    uint16_t rfg = sel ? HEAD_FG : FG;
    cv.fillSmoothRoundRect(4, y, 232, F_ROW_H - 4, 8, rbg);

    drawTypeBadge(12, cy, c.src, sel);

    char rs[8]; snprintf(rs, sizeof(rs), "%d", c.rssi);
    fontSmall(); cv.setTextDatum(middle_right);
    cv.setTextColor(sel ? HEAD_FG : MUTE, rbg); cv.drawString(rs, 226, cy);

    int nameW = (226 - cv.textWidth(rs) - 8) - 52;
    cv.setClipRect(52, F_TOP, nameW > 0 ? nameW : 0, F_WIN_H);
    fontBody(); cv.setTextDatum(middle_left);
    cv.setTextColor(rfg, rbg); cv.drawString(c.id, 52, cy);
    cv.setClipRect(0, F_TOP, W, F_WIN_H);
  }
  cv.clearClipRect();
}

// ---------------------------------------------------------------------------
// RF DETAIL
// ---------------------------------------------------------------------------
static bool liveRssi(const char* mac, uint8_t src, int8_t* out) {
  if (src == drone::SRC_BLE) {
    drone::BleSight b[24]; size_t n = drone::bleSnapshot(b, 24);
    for (size_t i = 0; i < n; ++i) if (strncmp(b[i].mac, mac, 17) == 0) { *out = b[i].rssi; return true; }
  } else {
    c5link::WifiSight w[24]; size_t n = c5link::wifiSnapshot(w, 24);
    for (size_t i = 0; i < n; ++i) if (strncmp(w[i].bssid, mac, 17) == 0) { *out = w[i].rssi; return true; }
    for (int i = 0; i < g_ap_count; ++i) if (strncmp(g_aps[i].bssid, mac, 17) == 0) { *out = g_aps[i].rssi; return true; }
  }
  return false;
}

static const char* srcName(uint8_t s) {
  switch (s) { case drone::SRC_WIFI_5G: return "WiFi 5 GHz";
               case drone::SRC_WIFI_2G: return "WiFi 2.4 GHz";
               default:                  return "BLE"; }
}

static void drawDetailScreen() {
  cv.fillRect(0, 0, W, H, F_BG);
  fontSmall(); cv.setTextDatum(middle_left);
  cv.setTextColor(F_DIM, F_BG); cv.drawString("detail", 8, 11);
  if (((millis() / 500) % 2) == 0) cv.fillSmoothCircle(W - 10, 11, 2, COL_OK);
  cv.drawFastHLine(0, 22, W, F_HAIR);

  const RfRow& r = g_detail_row;

  drawSrcBadge(8, 30, r.src);
  fontBody(); cv.setTextDatum(top_left); cv.setTextColor(F_TEXT, F_BG);
  cv.drawString(r.id[0] ? r.id : "(unknown)", 40, 28);

  int8_t rssi; bool live = liveRssi(r.mac, r.src, &rssi); if (!live) rssi = r.rssi;
  float frac = (rssi + 90) / 50.0f; if (frac < 0) frac = 0; if (frac > 1) frac = 1;
  uint16_t sc = (rssi >= -55) ? COL_OK : (rssi >= -70) ? COL_HEAD : COL_BAD;
  fontSmall(); cv.setTextDatum(top_left); cv.setTextColor(F_DIM, F_BG);
  cv.drawString(live ? "signal" : "signal (last seen)", 8, 46);
  int bx = 8, by = 56, bw = 180, bh = 18;
  cv.drawRoundRect(bx, by, bw, bh, 3, F_HAIR);
  if (frac > 0.02f) cv.fillSmoothRoundRect(bx, by, (int)(bw * frac), bh, 3, sc);
  char rb[8]; snprintf(rb, sizeof(rb), "%d", rssi);
  fontBody(); cv.setTextDatum(middle_right); cv.setTextColor(sc, F_BG);
  cv.drawString(rb, 232, by + bh / 2);

  int y = 82; fontSmall();
  auto field = [&](const char* k, const char* v, uint16_t vc) {
    cv.setTextDatum(top_left);  cv.setTextColor(F_DIM, F_BG); cv.drawString(k, 8, y);
    cv.setTextDatum(top_right); cv.setTextColor(vc,    F_BG); cv.drawString(v, W - 8, y);
    y += 14;
  };
  field("mac", r.mac, F_TEXT);
  if (r.src == drone::SRC_BLE) {
    drone::BleSight bs[24]; size_t nb = drone::bleSnapshot(bs, 24);
    for (size_t i = 0; i < nb; ++i)
      if (strncmp(bs[i].mac, r.mac, 17) == 0) {
        const char* v = bleVendor(bs[i].company_id, bs[i].has_mfr);
        field("vendor", v ? v : "-", F_TEXT);
        if (bs[i].tracker != drone::TRK_NONE) field("tracker", trackerLabel(bs[i].tracker), COL_BAD);
        break;
      }
  } else {
    field("band", r.src == drone::SRC_WIFI_5G ? "5 GHz" : "2.4 GHz", F_TEXT);
  }
  bool nw = false; unsigned long fs = rfFirstSeen(r.mac, &nw);
  char age[12]; snprintf(age, sizeof(age), "%lus", fs ? (millis() - fs) / 1000 : 0UL);
  field("seen for", age, F_TEXT);

  cv.setTextDatum(bottom_center); cv.setTextColor(F_DIM, F_BG);
  cv.drawString("BtnB next   BtnA back", W / 2, H - 3);
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
static void drawSetupScreen() {
  cv.fillRect(0, 0, W, H, BG);
  drawTopBar("SETUP");

  if (!g_portal_active) {
    bool wup = (WiFi.status() == WL_CONNECTED);

    auto inforow = [&](int y, const char* lab, const char* val, bool dot, bool dotOk) {
      drawCard(4, y, 232, 28, CARD, false);
      int cy = y + 14;
      fontSmall(); cv.setTextDatum(middle_left); cv.setTextColor(MUTE, CARD); cv.drawString(lab, 16, cy);
      cv.setClipRect(70, y, 158, 28);
      fontBody(); cv.setTextColor(FG, CARD); cv.drawString(val, 70, cy);
      cv.clearClipRect();
      if (dot) cv.fillSmoothCircle(W - 16, cy, 4, dotOk ? VERD : COL_BAD);
    };

    const char* ssid = g_cfg.ssid.length() ? g_cfg.ssid.c_str() : "(not set)";
    inforow(28, "WIFI", ssid, true, wup);
    char loc[24]; snprintf(loc, sizeof(loc), "%.2f, %.2f", g_cfg.lat, g_cfg.lon);
    inforow(60, "LOCATION", loc, false, false);

    int ay = 92, ah = 38;
    drawCard(4, ay, 232, ah, LILAC, false);
    fontBody(); cv.setTextDatum(top_left); cv.setTextColor(HEAD_FG, LILAC);
    cv.drawString("Setup portal", 16, ay + 7);
    fontSmall(); cv.setTextColor(rgb565(0x5A,0x4E,0x80), LILAC);
    cv.drawString("hold B to start", 16, ay + 25);

  } else {
    unsigned long elapsed = millis() - g_portal_start;
    unsigned long rem = elapsed < kPortalTimeoutMs ? (kPortalTimeoutMs - elapsed) / 1000 : 0;

    drawCard(4, 28, 232, 44, LILAC, false);
    fontSmall(); cv.setTextDatum(top_left); cv.setTextColor(rgb565(0x5A,0x4E,0x80), LILAC);
    cv.drawString("join wifi", 16, 35);
    fontBody(); cv.setTextColor(HEAD_FG, LILAC); cv.drawString("Aware-Setup", 16, 48);

    drawCard(4, 78, 232, 28, CARD, false);
    fontSmall(); cv.setTextDatum(middle_left); cv.setTextColor(MUTE, CARD); cv.drawString("OPEN", 16, 92);
    fontBody(); cv.setTextColor(FG, CARD); cv.drawString("192.168.4.1", 70, 92);

    char timer[28]; snprintf(timer, sizeof(timer), "timeout %lus  -  A exits", rem);
    fontSmall(); cv.setTextDatum(top_left); cv.setTextColor(MUTE, BG); cv.drawString(timer, 8, 113);
  }
}

// ---------------------------------------------------------------------------
static void drawSrcBadge(int x, int y, uint8_t src) {
  const char* lbl; uint16_t fill = 0, stroke = 0, txt;
  switch (src) {
    case drone::SRC_WIFI_5G: lbl = "5g";  fill   = F_ACCENT; txt = F_BG;   break;
    case drone::SRC_WIFI_2G: lbl = "2.4"; stroke = F_DIM;    txt = F_TEXT; break;
    default:                 lbl = "ble"; stroke = F_HAIR;   txt = F_DIM;  break;
  }
  if (fill) cv.fillSmoothRoundRect(x, y, 24, 14, 3, fill);
  else      cv.drawRoundRect(x, y, 24, 14, 3, stroke);
  cv.setTextDatum(middle_center);
  cv.setTextColor(txt, fill ? fill : F_BG);
  fontSmall();
  cv.drawString(lbl, x + 12, y + 7);
}

// ---------------------------------------------------------------------------
static void render() {
  if (g_detail) { cv.fillScreen(F_BG); drawDetailScreen(); cv.pushSprite(0, 0); return; }
  cv.fillScreen(COL_BG);
  switch (g_screen) {
    case SCR_AIRSPACE: drawAirspaceScreen(); break;
    case SCR_CONN:     drawConnScreen(); break;
    case SCR_SCAN:     drawScanScreen(); break;
    case SCR_BLE:      drawBleScreen();  break;
    case SCR_STATS:    drawStatsScreen(); break;
    case SCR_SETUP:    drawSetupScreen(); break;
  }
  if (g_popup_until && millis() < g_popup_until) {
    const char* nm = screenName(g_screen);
    fontBody();
    int tw = cv.textWidth(nm) + 28, th = 30;
    int bx = (W - tw) / 2, by = (H - th) / 2;
    cv.fillSmoothRoundRect(bx, by, tw, th, 6, F_SELROW);
    cv.drawRoundRect(bx, by, tw, th, 6, F_ACCENT);
    cv.setTextDatum(middle_center);
    cv.setTextColor(F_TEXT, F_SELROW);
    cv.drawString(nm, W / 2, by + th / 2);
    fontSmall();
  }
  cv.pushSprite(0, 0);
}

// ---------------------------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.printf("PSRAM: %u bytes\n", (unsigned)ESP.getPsramSize());
  setCpuFrequencyMhz(160);
  M5.Display.setRotation(1);
  M5.Display.setBrightness(90);
  cv.setColorDepth(16);
  cv.setPsram(true);                         // 64 KB canvas in PSRAM -> frees internal RAM for WiFi/BLE/TLS
  if (!cv.createSprite(W, H)) {
    cv.setPsram(false);                      // fall back to internal RAM
    if (!cv.createSprite(W, H)) {
      M5.Display.fillScreen(TFT_BLACK);
      M5.Display.setCursor(4, 4);
      M5.Display.print("canvas alloc failed");
      Serial.println("FATAL: createSprite failed");
      for (;;) delay(1000);                  // fail loudly instead of crashing in pushSprite
    }
  }

  cv.fillScreen(COL_BG);
  cv.setTextDatum(middle_center);
  cv.setTextColor(COL_TEXT, COL_BG); fontSmall();
  cv.drawString("Aware", W / 2, H / 2 - 6);
  cv.drawString("starting...", W / 2, H / 2 + 8);
  cv.pushSprite(0, 0);
  g_popup_until = millis() + 1200;

  loadCfg();
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_cfg.ssid.c_str(), g_cfg.pass.c_str());
  WiFi.setSleep(true);
  services::adsb::begin();
  services::adsb::setCenter(g_cfg.lat, g_cfg.lon, rangeKm() * 1.3f);

  drone::begin();  // BLE Remote ID scan runs continuously from here on
  g_bleScanRunning = true;
}

void loop() {
  M5.update();
  static bool s_c5_up = false;
  if (!s_c5_up) { s_c5_up = true; c5link::begin(); }
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

  // BtnA click: scroll screens (always)
  if (M5.BtnA.wasClicked()) {
    if (g_portal_active) stopConfigPortal();
    else                 gotoScreen(g_screen + 1);
    render();
  }

  // BtnA long-press: toggle pause / unpause the feed (SCAN or BLE)
  if (M5.BtnA.wasHold() && (g_screen == SCR_SCAN || g_screen == SCR_BLE)) {
    g_paused = !g_paused;
    if (g_paused) g_scan_sel = 0;
    else          g_detail   = false;
    render();
  }

  // BtnB click: live = change filter; paused = scroll; in item = next contact
  if (M5.BtnB.wasClicked()) {
    if (g_screen == SCR_SCAN) {
      if (g_detail) {
        if (g_dispN > 0) { g_scan_sel = (g_scan_sel + 1) % g_dispN; g_detail_row = g_disp[g_scan_sel]; }
      } else if (!g_paused) {
        g_filter = (g_filter + 1) % RF_COUNT;
      } else {
        g_scan_sel = g_scan_sel + 1;
      }
    } else if (g_screen == SCR_BLE) {
      if (g_detail) {
        if (g_bleFrozenN > 0) {
          g_scan_sel = (g_scan_sel + 1) % g_bleFrozenN;
          const drone::BleSight& bs = g_bleFrozen[g_scan_sel];
          g_detail_row.src = drone::SRC_BLE; g_detail_row.rssi = bs.rssi;
          g_detail_row.tracker = bs.tracker;
          strncpy(g_detail_row.mac, bs.mac, sizeof(g_detail_row.mac) - 1);
          g_detail_row.mac[sizeof(g_detail_row.mac) - 1] = '\0';
          const char* lbl = bs.name[0] ? bs.name : bs.mac;
          strncpy(g_detail_row.id, lbl, sizeof(g_detail_row.id) - 1);
          g_detail_row.id[sizeof(g_detail_row.id) - 1] = '\0';
        }
      } else if (!g_paused) {
        g_bleTrkOnly = !g_bleTrkOnly;
      } else {
        g_scan_sel++;
      }
    } else if (g_screen == SCR_AIRSPACE) {
      ++g_airspace_sel;
    } else if (g_screen == SCR_SETUP && !g_portal_active) {
      startConfigPortal();
    }
    render();
  }

  // BtnB long-press: paused = drill into item; in item = back to list; airspace = range
  if (M5.BtnB.wasHold()) {
    if (g_screen == SCR_AIRSPACE) {
      g_range_idx = (g_range_idx + 1) % appcfg::kRangePresetCount;
      services::adsb::setCenter(g_cfg.lat, g_cfg.lon, rangeKm() * 1.3f);
    } else if (g_screen == SCR_SCAN || g_screen == SCR_BLE) {
      if (g_detail) {
        g_detail = false;
      } else if (g_paused) {
        if (g_screen == SCR_BLE && g_bleFrozenN > 0) {
          int s = g_scan_sel; if (s < 0) s = 0; if (s >= g_bleFrozenN) s = g_bleFrozenN - 1;
          const drone::BleSight& bs = g_bleFrozen[s];
          g_detail_row.src = drone::SRC_BLE; g_detail_row.rssi = bs.rssi;
          g_detail_row.tracker = bs.tracker;
          strncpy(g_detail_row.mac, bs.mac, sizeof(g_detail_row.mac) - 1);
          g_detail_row.mac[sizeof(g_detail_row.mac) - 1] = '\0';
          const char* lbl = bs.name[0] ? bs.name : bs.mac;
          strncpy(g_detail_row.id, lbl, sizeof(g_detail_row.id) - 1);
          g_detail_row.id[sizeof(g_detail_row.id) - 1] = '\0';
          g_detail = true;
        } else if (g_screen == SCR_SCAN && g_dispN > 0) {
          int s = g_scan_sel; if (s < 0) s = 0; if (s >= g_dispN) s = g_dispN - 1;
          g_detail_row = g_disp[s]; g_detail = true;
        }
      }
    }
    render();
  }

  unsigned long now = millis();
  // 1 s recompute: pick up whatever the background fetch task published.
  { static unsigned long s_last_recompute = 0;
    if (now - s_last_recompute >= 1000) {
      s_last_recompute = now;
      unsigned long ok_ms = services::adsb::lastOkMs();
      if (ok_ms > g_lastAdsbOkMs) {
        g_lastAdsbOkMs = ok_ms;
        recomputePlanes();
        if (g_pcount > 0) g_lastDataMs = now;
        if (g_screen == SCR_AIRSPACE) render();
      }
    }
  }

  static unsigned long last_prune = 0;
  if (now - last_prune > 2000) { last_prune = now; drone::prune(); }

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
      if (want != rot) { rot = want; M5.Display.setRotation(rot); render(); }
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

  // RF SCAN screen: async 2.4 GHz AP sweep.
  // Briefly perturbs the STA link; runs while the screen is active.
  if (g_screen == SCR_SCAN) {
    if (!g_wifi_scanning && now - g_last_wifi_scan > 5000) {
      WiFi.scanNetworks(true /*async*/, false /*show_hidden*/);
      g_wifi_scanning = true;
    }
    if (g_wifi_scanning) {
      int n = WiFi.scanComplete();
      if (n >= 0) {
        g_ap_count = 0;
        for (int i = 0; i < n; ++i) {
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
      g_screen != SCR_SCAN && !g_wifi_scanning && !g_portal_active) {
    last_recon = now; WiFi.reconnect();
  }

  // drive the feed slide-in at ~30 fps while it's animating (the 1 Hz idle render is too slow)
  static unsigned long last_anim = 0;
  if (g_screen == SCR_SCAN && g_scan_anim > 0.0f && now - last_anim > 16) {
    last_anim = now; render();
  }

  if (g_popup_until && now >= g_popup_until) { g_popup_until = 0; render(); }

  static unsigned long last_idle = 0;
  if (now - last_idle > 1000) { last_idle = now; render(); }

  static unsigned long last_detail = 0;
  if (g_detail && now - last_detail > 300) { last_detail = now; render(); }

  delay(10);
}
