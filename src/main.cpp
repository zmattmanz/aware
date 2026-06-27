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
#include "esp_wifi.h"
#include <climits>
#include "fonts/pjs_small.h"
#include "fonts/pjs_body.h"
#include "fonts/pjs_num.h"
#include "audio/aware_boot.h"
#include "audio/aircraft_wav.h"
#include "audio/drone_wav.h"
#include "audio/tracker_wav.h"

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
static unsigned long g_screen_since = 0;   // ms when current screen was entered (glyph entrance)

static const char* screenName(int s) {
  switch (s) {
    case SCR_AIRSPACE: return "Airspace";
    case SCR_CONN:     return "Connections";
    case SCR_SCAN:     return "RF Scan";
    case SCR_BLE:      return "BLE";
    case SCR_STATS:    return "Stats";
    case SCR_SETUP:    return "Setup";
    default:           return "";
  }
}

static int           g_scan_sel  = 0;
static int           g_scan_view = 0;
static bool          g_paused    = false;
static int           g_dispN     = 0;
static constexpr float TICKER_PXPS = 18.0f;   // steady ticker speed (px/sec)
static float         g_feed_scroll = 0.0f;
static float         g_feed_glide  = 0.0f;
static unsigned long g_feed_t      = 0;
static int           g_feed_dir    = 1;
static bool          g_feed_manual = false;
static unsigned long g_feed_touch  = 0;
static float         g_feed_vel    = TICKER_PXPS;  // target scroll velocity (px/s); tilt drives it
static float         g_feed_vel_eased = TICKER_PXPS; // smoothed velocity actually applied per frame
static float         g_tilt_neutral = 99.0f;       // adaptive resting pitch (99 = uninitialised)
static bool          g_tilt_scrub  = false;        // past the deadzone right now
static int           g_tilt_dir    = 0;            // -1 up, +1 down (for the chevron)
static float         g_tilt_e      = 0.0f;         // live tilt deviation from neutral (for the HUD)
static bool          g_detail    = false;

// ---- cursor mode (feed screens) ----
static bool          g_cursor_mode   = false;      // selection cursor active on feed screen
static char          g_cursor_mac[18] = {};         // pinned entry MAC — survives feed reorder
static unsigned long g_cursor_idle_ms = 0;          // time of last cursor gesture (8 s timeout)
static float         g_cursor_anim_y = 0.0f;        // animated highlight Y (pixels in list coords)

// ---------------------------------------------------------------------------
// Locator mode — BLE proximity beep (faster = closer)
// ---------------------------------------------------------------------------
static bool          g_locating            = false;
static char          g_locate_mac[18]      = {};   // target BLE MAC
static char          g_locate_id[24]       = {};   // target display label
static float         g_rssi_smooth         = -100.0f;
static unsigned long g_locate_last_seen_ms = 0;
static unsigned long g_locate_last_beep_ms = 0;
// Tunables — all named, no magic numbers
static constexpr float         kLocFar          = -90.0f;  // dBm far end → slow beep
static constexpr float         kLocNear         = -45.0f;  // dBm near end → solid tone
static constexpr int           kLocSlowMs       = 1200;    // ms between beeps at far
static constexpr int           kLocFastMs       =   60;    // ms between beeps at near
static constexpr float         kLocAlpha        =  0.30f;  // EMA smoothing factor
static constexpr unsigned long kLocLostMs       = 2500;    // ms without sighting = signal lost
static constexpr uint16_t      kLocFreq         = 2700;    // Hz — cutting but not shrill
static constexpr int           kLocDurMs        =   35;    // ms per locator beep
static constexpr uint16_t      kLocSearchFreq   =  500;    // Hz — "searching" low tone
static constexpr int           kLocSearchDurMs  =  500;    // ms for searching beep
static constexpr unsigned long kLocSearchPeriod = 2000;    // ms between searching beeps

static int           g_airspace_sel = 0;
static bool          g_air_detail   = false;
static unsigned long g_last_motion = 0;
static float         g_bri         = 90.0f;
static int           g_mv_cache    = 4000;
static bool          g_usb_cache   = false;
static constexpr int BRI_HI = 60;
static constexpr int BRI_LO = 12;
struct AContact {
  bool  is_drone; float dist_m, brg_deg;
  char  id[14]; char type[10];
  int   alt_ft; float trk_deg;
  bool  has_op; double op_lat, op_lon;
};
static AContact g_air[80];
static int      g_air_count = 0;

static void gotoScreen(int s) {
  s = ((s % SCR_COUNT) + SCR_COUNT) % SCR_COUNT;
  if (s != g_screen) {
    g_screen = s;
    g_paused = false; g_detail = false;
    g_scan_sel = 0;   g_scan_view = 0;
    g_feed_scroll = 0; g_feed_manual = false; g_dispN = 0;
    g_air_detail = false; g_airspace_sel = 0;
    g_cursor_mode = false; g_cursor_mac[0] = '\0'; g_cursor_anim_y = 0.0f;
    g_popup_until = millis() + 1200;
    g_screen_since = millis();
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
  cv.setClipRect(x, cy - h / 2, w, h);
  cv.drawString(lbl, x + w / 2, cy);
  cv.clearClipRect();
}

static void drawPillHeader(const char* title, bool liveDot, const char* rightText, bool center = false) {
  int ph = 18, py = 5, px = 6, dot = liveDot ? 9 : 0;
  fontSmall();
  int pw = cv.textWidth(title) + 16 + dot;
  cv.fillSmoothRoundRect(px, py, pw, ph, ph / 2, HEAD_BG);
  cv.setTextColor(HEAD_FG, HEAD_BG);
  cv.setClipRect(px, py, pw, ph);
  if (center) {
    int cx = px + pw / 2, tw = cv.textWidth(title);
    cv.setTextDatum(middle_center);
    cv.drawString(title, cx, py + ph / 2);
    if (liveDot) cv.fillSmoothCircle(cx - tw / 2 - 6, py + ph / 2, 2, MUTE);
  } else {
    if (liveDot) cv.fillSmoothCircle(px + 11, py + ph / 2, 2, MUTE);
    cv.setTextDatum(middle_left);
    cv.drawString(title, px + 8 + dot, py + ph / 2);
  }
  cv.clearClipRect();
  if (rightText && rightText[0]) {
    fontSmall();
    int rw = cv.textWidth(rightText) + 16, rx = W - 6 - rw;
    cv.fillSmoothRoundRect(rx, py, rw, ph, ph / 2, PILL_BG);
    cv.setTextColor(FG, PILL_BG); cv.setTextDatum(middle_center);
    cv.setClipRect(rx, py, rw, ph);
    cv.drawString(rightText, rx + rw / 2, py + ph / 2);
    cv.clearClipRect();
  }
}

static void drawTopBar(const char* title) {
  drawPillHeader(title, false, "");                 // battery drawn as an icon, not a text pill
  int  pct = battPctFromMv(M5.Power.getBatteryVoltage());
  bool chg = M5.Power.isCharging();
  uint16_t col = chg ? VERD : (pct <= 12 ? COL_BAD : (pct <= 30 ? COL_HEAD : FG));
  const int w = 20, h = 11, y = 5 + (18 - h) / 2, x = W - 8 - w;
  cv.drawRoundRect(x, y, w, h, 2, MUTE);
  cv.fillRect(x + w, y + 3, 2, h - 6, MUTE);        // terminal nub
  int fw = (w - 4) * pct / 100; if (pct > 0 && fw < 2) fw = 2; if (fw > w - 4) fw = w - 4;
  cv.fillRect(x + 2, y + 2, fw, h - 4, col);
  char b[6]; snprintf(b, sizeof(b), "%d%%", pct);
  fontSmall(); cv.setTextDatum(middle_right); cv.setTextColor(col, BG);
  cv.drawString(b, x - 5, 5 + 9);
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

static void buildAirContacts() {
  if (g_paused || g_air_detail) return;
  int n = 0;
  drone::Drone dr[12]; size_t nd = drone::snapshot(dr, 12);
  for (size_t i = 0; i < nd && n < 80; ++i) {
    AContact c{}; c.is_drone = true;
    c.dist_m  = dr[i].has_loc ? haversineKm(g_cfg.lat, g_cfg.lon, dr[i].lat, dr[i].lon) * 1000.0f : 0.0f;
    c.brg_deg = dr[i].has_loc ? bearingDeg(g_cfg.lat, g_cfg.lon, dr[i].lat, dr[i].lon) : 0.0f;
    snprintf(c.id,   sizeof(c.id),   "%.13s", dr[i].id[0] ? dr[i].id : "DRONE");
    snprintf(c.type, sizeof(c.type), "drone");
    c.alt_ft = INT_MIN; c.trk_deg = dr[i].dir_deg;
    c.has_op = dr[i].has_op; c.op_lat = dr[i].op_lat; c.op_lon = dr[i].op_lon;
    g_air[n++] = c;
  }
  const services::adsb::Aircraft* L = services::adsb::aircraftList();
  for (int i = 0; i < g_pcount && n < 80; ++i) {
    int ai = g_porder[i]; const services::adsb::Aircraft& a = L[ai];
    AContact c{}; c.is_drone = false;
    c.dist_m = g_pdist[ai] * 1000.0f; c.brg_deg = g_pbrg[ai];
    snprintf(c.id,   sizeof(c.id),   "%.13s", a.callsign[0] ? a.callsign : "----");
    snprintf(c.type, sizeof(c.type), "%s", a.emergency ? "EMERG" : (a.mil ? "mil" : "aircraft"));
    c.alt_ft = a.alt_ft; c.trk_deg = a.track_deg; c.has_op = false;
    g_air[n++] = c;
  }
  for (int i = 1; i < n; ++i) { AContact k = g_air[i]; int j = i - 1;
    while (j >= 0 && g_air[j].dist_m > k.dist_m) { g_air[j+1] = g_air[j]; --j; } g_air[j+1] = k; }
  g_air_count = n;
  if (n > 0) { g_everReceivedData = true; g_lastDataMs = millis(); }
  if (g_airspace_sel >= n) g_airspace_sel = 0;
  if (g_airspace_sel < 0)  g_airspace_sel = 0;
}

// ---------------------------------------------------------------------------
// VoiceAnnouncer — screen-gated, new-contact-only, cooldown-capped playback.
// No heap: fixed C-string seen table. Global isPlaying() guard prevents overlap.
// kIdLen = 21 covers drone Remote ID serials (20 chars) and MACs (17 chars).
// ---------------------------------------------------------------------------
struct VoiceAnnouncer {
  static constexpr int kMaxSeen = 32;
  static constexpr int kIdLen   = 21;

  const uint8_t* clip       = nullptr;
  size_t         clip_len   = 0;
  unsigned long  cooldown_ms = 9000;
  char           seen[kMaxSeen][kIdLen] = {};
  int            seen_n     = 0;
  unsigned long  last_ms    = 0;

  void init(const uint8_t* c, size_t len, unsigned long cd) {
    clip = c; clip_len = len; cooldown_ms = cd;
  }

  // ids[0..n-1] = current live contact IDs for this source.
  // screen_active = the owning screen is currently displayed.
  void check(const char* const* ids, int n, bool screen_active) {
    // Prune seen entries that are no longer in the live set
    int w = 0;
    for (int i = 0; i < seen_n; ++i) {
      bool alive = false;
      for (int j = 0; j < n && !alive; ++j)
        if (ids[j] && strncmp(seen[i], ids[j], kIdLen - 1) == 0) alive = true;
      if (alive) { if (w != i) memcpy(seen[w], seen[i], kIdLen); ++w; }
    }
    seen_n = w;

    // Detect new IDs and add to seen
    bool new_found = false;
    for (int j = 0; j < n; ++j) {
      if (!ids[j] || !ids[j][0]) continue;
      bool known = false;
      for (int i = 0; i < seen_n && !known; ++i)
        if (strncmp(seen[i], ids[j], kIdLen - 1) == 0) known = true;
      if (!known) {
        new_found = true;
        if (seen_n < kMaxSeen) {
          strncpy(seen[seen_n], ids[j], kIdLen - 1);
          seen[seen_n][kIdLen - 1] = '\0';
          ++seen_n;
        }
      }
    }

    if (!new_found || !screen_active) return;
    unsigned long now = millis();
    if (now - last_ms >= cooldown_ms && !M5.Speaker.isPlaying()) {
      last_ms = now;
      M5.Speaker.playWav(clip, clip_len);
    }
  }
};

static VoiceAnnouncer g_aircraft_va;   // callsign/hex, gated on SCR_AIRSPACE, 9 s cooldown
static VoiceAnnouncer g_drone_va;      // RID serial or MAC, gated on SCR_AIRSPACE, 9 s cooldown
static VoiceAnnouncer g_tracker_va;    // tracker MAC (following only), gated on SCR_BLE, 60 s cooldown

// Tracker "following" thresholds — announce only for close, persistent trackers.
// MAC rotation: AirTags rotate ~every 24 h when separated (vs 15 min when paired),
// so a 10-min first_seen window catches separated tags without firing on paired ones.
static constexpr int8_t  kFollowRssiMin    = -75;               // dBm — must be nearby
static constexpr unsigned long kFollowPersistMs = 10UL * 60 * 1000; // 10 min continuous presence

static void drawAirspaceScreen() {
  cv.fillScreen(BG);
  buildAirContacts();
  int n = g_air_count;
  if (g_airspace_sel >= n) g_airspace_sel = (n > 0) ? n - 1 : 0;

  // headerless — plane glyph (top-left) + entry toast carry identity
  if (g_paused) { cv.fillRect(6, 6, 3, 11, LILAC); cv.fillRect(11, 6, 3, 11, LILAC); }

  // ================= RADAR =================
  const int cx = 64, cy = 74, R = 48;
  const uint16_t RADAR_BG = rgb565(0x2C,0x26,0x4C), RING = rgb565(0x55,0x4C,0x80);
  auto lerp565 = [](uint16_t a, uint16_t b, float t) -> uint16_t {
    if (t < 0) t = 0; if (t > 1) t = 1;
    int ar=(a>>11)&0x1F, ag=(a>>5)&0x3F, ab=a&0x1F, br=(b>>11)&0x1F, bg=(b>>5)&0x3F, bb=b&0x1F;
    return (uint16_t)(((ar+(int)lroundf((br-ar)*t))<<11)|((ag+(int)lroundf((bg-ag)*t))<<5)|(ab+(int)lroundf((bb-ab)*t)));
  };
  auto radiusFor = [&](float dm) -> float {
    float km = dm / 1000.0f, r = (float)R * (log10f(1.0f + km) / log10f(41.0f));
    if (r < 4.0f) r = 4.0f; if (r > (float)R) r = (float)R; return r;
  };
  cv.fillSmoothCircle(cx, cy, R, RADAR_BG);
  cv.drawCircle(cx, cy, R,           RING);
  cv.drawCircle(cx, cy, (R * 2) / 3, RING);
  cv.drawCircle(cx, cy, R / 3,       RING);
  cv.drawLine(cx - R, cy, cx + R, cy, RING);
  cv.drawLine(cx, cy - R, cx, cy + R, RING);
  float sweep = fmodf((float)millis() * 0.10f, 360.0f);
  const int TRAIL = 10; const float step = 5.0f;
  for (int k = TRAIL - 1; k >= 0; --k) {
    float a1 = sweep - k * step, a0 = a1 - step; if (a0 < 0) { a0 += 360.0f; a1 += 360.0f; }
    float t = 1.0f - (float)k / (float)TRAIL;
    cv.fillArc(cx, cy, 0, R - 1, a0, a1, lerp565(RADAR_BG, LILAC, t * 0.85f));
  }
  for (int i = 0; i < n; ++i) {
    float rr = radiusFor(g_air[i].dist_m), a = g_air[i].brg_deg * (float)M_PI / 180.0f;
    int px = cx + (int)lroundf(rr * sinf(a)), py = cy - (int)lroundf(rr * cosf(a));
    if (i == g_airspace_sel) { cv.fillSmoothCircle(px, py, 3, LILAC);
      cv.drawCircle(px, py, 6, LILAC); cv.drawCircle(px, py, 8, LILAC); }
    else cv.fillSmoothCircle(px, py, 2, FG);
  }
  cv.drawCircle(cx, cy, 6, rgb565(0x12,0x12,0x16));
  cv.fillRect(cx - 3, cy - 3, 6, 6, LILAC);
  // ================= end radar =================

  // ---- right: scrollable contact list ----
  const int LX = 114, LR = 236, top = 8, rowH = 22, kVis = 5;
  const uint16_t AMBER = rgb565(0xE3,0xA9,0x4C), HILITE = rgb565(0x26,0x22,0x3A);
  if (n == 0) {
    fontSmall(); cv.setTextDatum(middle_center); cv.setTextColor(MUTE, BG);
    cv.drawString("clear skies", (LX + LR) / 2, 78);
    return;
  }
  static int s_view = 0;
  if (g_airspace_sel < s_view)         s_view = g_airspace_sel;
  if (g_airspace_sel >= s_view + kVis) s_view = g_airspace_sel - kVis + 1;
  if (s_view < 0) s_view = 0;

  cv.setClipRect(LX, top, LR - LX, H - top);
  for (int vis = 0; vis <= kVis && (s_view + vis) < n; ++vis) {
    int idx = s_view + vis;
    const AContact& c = g_air[idx];
    int y = top + vis * rowH, cy2 = y + rowH / 2;
    bool sel   = (idx == g_airspace_sel);
    bool emerg = (strcmp(c.type, "EMERG") == 0);
    bool mil   = (strcmp(c.type, "mil")   == 0);
    uint16_t rbg = sel ? HILITE : BG;
    if (sel) cv.fillSmoothRoundRect(LX, y, LR - LX, rowH - 2, 5, HILITE);

    uint16_t dotc = emerg ? COL_BAD : (c.is_drone ? LILAC : rgb565(0xB0,0xB0,0xBC));
    cv.fillSmoothCircle(LX + 6, cy2, 3, dotc);

    float nm = c.dist_m / 1852.0f;
    char ds[10]; snprintf(ds, sizeof(ds), "%.1fnm", nm);
    fontSmall(); cv.setTextDatum(middle_right); cv.setTextColor(sel ? FG : MUTE, rbg);
    cv.setClipRect(LX, y, LR - LX, rowH); cv.drawString(ds, LR - 4, cy2); cv.clearClipRect();
    int distLeft = LR - 4 - cv.textWidth(ds) - 6;

    uint16_t namec = emerg ? COL_BAD : (mil ? AMBER : FG);
    fontBody(); cv.setTextDatum(middle_left); cv.setTextColor(namec, rbg);
    int nameW = distLeft - (LX + 16);
    cv.setClipRect(LX + 16, y, nameW > 0 ? nameW : 0, rowH);
    cv.drawString(c.id, LX + 16, cy2);
    cv.clearClipRect();
  }
  cv.clearClipRect();
}

static void drawAirspaceDetail() {
  cv.fillRect(0, 0, W, H, BG);
  buildAirContacts();
  int n = g_air_count;
  if (n == 0) { fontBody(); cv.setTextDatum(middle_center); cv.setTextColor(MUTE, BG);
                cv.drawString("no contacts", W / 2, H / 2); return; }
  if (g_airspace_sel >= n) g_airspace_sel = 0;
  const AContact& c = g_air[g_airspace_sel];

  char pos[12]; snprintf(pos, sizeof(pos), "%d/%d", g_airspace_sel + 1, n);
  drawPillHeader(c.is_drone ? "Drone" : "Aircraft", true, pos);

  fontBody(); cv.setTextDatum(top_left); cv.setTextColor(c.is_drone ? LILAC : FG, BG);
  cv.setClipRect(8, 28, 224, 22); cv.drawString(c.id, 8, 30); cv.clearClipRect();
  fontSmall(); cv.setTextColor(MUTE, BG); cv.drawString(c.type, 8, 52);

  auto row = [&](int ry, const char* k, const char* v) {
    drawCard(6, ry, 228, 22, CARD, false);
    fontSmall(); cv.setTextDatum(middle_left);  cv.setTextColor(MUTE, CARD); cv.drawString(k, 16, ry + 11);
    cv.setTextDatum(middle_right); cv.setTextColor(FG, CARD); cv.drawString(v, 218, ry + 11);
  };
  char v[24];
  if (c.dist_m < 1000.0f) snprintf(v, sizeof(v), "%dm", (int)c.dist_m);
  else                    snprintf(v, sizeof(v), "%.1f km", c.dist_m / 1000.0f);
  row(62, "DISTANCE", v);
  snprintf(v, sizeof(v), "%.0f deg", c.brg_deg); row(86, "BEARING", v);
  if (!c.is_drone) {
    if (c.alt_ft != INT_MIN) snprintf(v, sizeof(v), "%d ft", c.alt_ft); else snprintf(v, sizeof(v), "--");
    row(110, "ALTITUDE", v);
  } else if (c.has_op) {
    snprintf(v, sizeof(v), "%.3f, %.3f", c.op_lat, c.op_lon); row(110, "PILOT", v);
  } else {
    snprintf(v, sizeof(v), "%.0f deg", c.trk_deg); row(110, "HEADING", v);
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
  cv.fillRect(0, 0, W, H, BG);
  // headerless — gauge glyph + entry toast

  // ---- readings ----
  float c   = temperatureRead();
  float f   = c * 9.0f / 5.0f + 32.0f;
  int   mv  = M5.Power.getBatteryVoltage();
  int   lvl = battPctFromMv(mv);
  bool  wup = (WiFi.status() == WL_CONNECTED);
  int   rssi = wup ? (int)WiFi.RSSI() : 0;

  char nTemp[8], nBat[8], nVolt[8], nWifi[8];
  snprintf(nTemp, sizeof(nTemp), "%.0f", f);
  snprintf(nBat,  sizeof(nBat),  "%d",   lvl);
  snprintf(nVolt, sizeof(nVolt), "%.2f", mv / 1000.0f);
  snprintf(nWifi, sizeof(nWifi), wup ? "%d" : "--", rssi);

  const uint16_t T_GREEN  = rgb565(0x2E,0x81,0x59);
  const uint16_t T_PURPLE = rgb565(0x6E,0x4C,0x82);   // nudged red-ward so the panel reads purple, not blue
  const uint16_t T_TEAL   = rgb565(0x2D,0x6E,0x7C);
  const uint16_t T_BROWN  = rgb565(0x7A,0x57,0x33);
  // value + label share one colour per tile (set at the call site)

  // ---- 2x2 grid: even gutters; number high, unit centered beside it, label on the floor ----
  const int P    = 7;                          // outer margin + middle gutter
  const int top  = 6;                           // headerless
  const int tw   = (W - P * 3) / 2;            // 109
  const int th   = 56;
  const int vgap = 7;
  const int x1 = P, x2 = W - P - tw;
  const int y1 = top, y2 = top + th + vgap;

  const int PADX     = 14;                     // text inset from the tile's left edge
  const int VAL_BASE = 22;                     // value baseline from the tile top
  const int LBL_BASE = th - 8;                 // label baseline from the tile top

  auto tile = [&](int x, int y, uint16_t bg, uint16_t col,
                  const char* num, const char* unit, const char* label) {
    cv.fillSmoothRoundRect(x, y, tw, th, 13, bg);
    // value + unit, one line at the top, body size
    fontBody(); cv.setTextDatum(baseline_left); cv.setTextColor(col);
    cv.drawString(num, x + PADX, y + VAL_BASE);
    int nw = cv.textWidth(num);
    if (unit && unit[0]) {
      fontSmall(); cv.setTextDatum(baseline_left); cv.setTextColor(col);
      cv.drawString(unit, x + PADX + nw + 4, y + VAL_BASE);
    }
    // label — same colour as the value, pinned to the bottom
    fontSmall(); cv.setTextDatum(baseline_left); cv.setTextColor(col);
    cv.drawString(label, x + PADX, y + LBL_BASE);
  };

  uint16_t tempCol = (f >= 149.0f) ? COL_BAD : FG;   // overheat flag
  tile(x1, y1, T_GREEN,  tempCol, nTemp, "\xB0""F",       "CHIP TEMP");
  tile(x2, y1, T_PURPLE, FG,      nBat,  "%",             "BATTERY");
  tile(x1, y2, T_TEAL,   FG,      nVolt, "V",             "VOLTAGE");
  tile(x2, y2, T_BROWN,  FG,      nWifi, wup ? "dBm" : "", "WIFI");
}

// ---------------------------------------------------------------------------
// CONNECTIONS — at-a-glance health of each data channel
// ---------------------------------------------------------------------------
static void drawConnScreen() {
  cv.fillRect(0, 0, W, H, BG);
  // headerless — node glyph + entry toast

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

  // ---- hero: title (line 1), subtitle + big KB/s (line 2) ----
  int hx = 6, hy = 6, hw = 228, hh = 50;
  uint16_t hbg  = c5up ? MINT : CARD;
  uint16_t hsub = c5up ? rgb565(0xC8,0xE6,0xD4) : MUTE;
  drawCard(hx, hy, hw, hh, hbg, false);
  cv.fillSmoothCircle(hx + 12, hy + 22, 4, c5up ? VERD : COL_BAD);

  fontBody(); cv.setTextDatum(top_left); cv.setTextColor(FG, hbg);
  cv.setClipRect(hx + 24, hy, 150, 22);
  cv.drawString("C5 co-processor", hx + 24, hy + 5);
  cv.clearClipRect();

  char kb[14];
  if (c5up) snprintf(kb, sizeof(kb), "%.1f KB/s", s_kbps); else snprintf(kb, sizeof(kb), "--");
  fontSmall(); cv.setTextDatum(top_left); cv.setTextColor(hsub, hbg);
  cv.drawString(c5up ? "linked \xB7 5 GHz radio" : "no link", hx + 24, hy + 25);
  fontSmall(); cv.setTextDatum(top_right); cv.setTextColor(c5up ? FG : MUTE, hbg);
  cv.drawString(kb, hx + hw - 10, hy + 25);

  // ---- 2x2 grid ----
  auto tile = [&](int x, int y, int w, int h, const char* label, uint16_t dot, const char* val) {
    drawCard(x, y, w, h, CARD, false);
    int cy = y + h / 2;
    cv.fillSmoothCircle(x + 13, cy, 4, dot);
    fontSmall();
    cv.setTextDatum(middle_left);  cv.setTextColor(FG, CARD);   cv.drawString(label, x + 24, cy);
    cv.setTextDatum(middle_right); cv.setTextColor(MUTE, CARD); cv.drawString(val, x + w - 10, cy);
  };
  int gh = 31, tw = (hw - 6) / 2;
  int gy1 = hy + hh + 4, gy2 = gy1 + gh + 3;
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

// --- persistent feed table: rows live in g_disp[]; updated in place, expired on a TTL, order stable ---
static unsigned long g_disp_seen[40];

static void feedUpsert(uint8_t src, uint8_t tracker, const char* id, const char* mac, int8_t rssi) {
  for (int i = 0; i < g_dispN; ++i)
    if (strncmp(g_disp[i].mac, mac, 17) == 0) {                 // already tracked -> update its slot in place
      g_disp[i].src = src; g_disp[i].tracker = tracker; g_disp[i].rssi = rssi;
      strncpy(g_disp[i].id, id, sizeof(g_disp[i].id)-1); g_disp[i].id[sizeof(g_disp[i].id)-1] = '\0';
      g_disp_seen[i] = millis(); return;
    }
  if (g_dispN >= 40) return;                                    // new device -> append
  int i = g_dispN++;
  g_disp[i].src = src; g_disp[i].tracker = tracker; g_disp[i].rssi = rssi;
  strncpy(g_disp[i].id,  id,  sizeof(g_disp[i].id)-1);  g_disp[i].id[sizeof(g_disp[i].id)-1]   = '\0';
  strncpy(g_disp[i].mac, mac, sizeof(g_disp[i].mac)-1); g_disp[i].mac[sizeof(g_disp[i].mac)-1] = '\0';
  g_disp_seen[i] = millis();
}

static void feedExpire(unsigned long ttl) {                     // drop devices unseen for > ttl, keep order
  unsigned long now = millis(); int w = 0;
  for (int r = 0; r < g_dispN; ++r)
    if (now - g_disp_seen[r] <= ttl) {
      if (w != r) { g_disp[w] = g_disp[r]; g_disp_seen[w] = g_disp_seen[r]; }
      ++w;
    }
  g_dispN = w;
}

static void feedSortByFirstSeen() {                             // stable: first-seen is immutable per MAC
  unsigned long fs[40];
  for (int i = 0; i < g_dispN; ++i) { bool nw; fs[i] = rfFirstSeen(g_disp[i].mac, &nw); }
  for (int i = 1; i < g_dispN; ++i) {
    RfRow kr = g_disp[i]; unsigned long kf = fs[i], ks = g_disp_seen[i]; int j = i - 1;
    while (j >= 0 && fs[j] < kf) { g_disp[j+1]=g_disp[j]; fs[j+1]=fs[j]; g_disp_seen[j+1]=g_disp_seen[j]; --j; }
    g_disp[j+1] = kr; fs[j+1] = kf; g_disp_seen[j+1] = ks;
  }
}

static void feedSortBleStable() {                               // trackers first, then first-seen (both stable)
  unsigned long fs[40];
  for (int i = 0; i < g_dispN; ++i) { bool nw; fs[i] = rfFirstSeen(g_disp[i].mac, &nw); }
  for (int i = 0; i < g_dispN; ++i)
    for (int j = i + 1; j < g_dispN; ++j) {
      bool ti = g_disp[i].tracker != drone::TRK_NONE, tj = g_disp[j].tracker != drone::TRK_NONE;
      if ((tj && !ti) || (ti == tj && fs[j] > fs[i])) {
        RfRow t = g_disp[i]; g_disp[i] = g_disp[j]; g_disp[j] = t;
        unsigned long a = fs[i]; fs[i] = fs[j]; fs[j] = a;
        unsigned long b = g_disp_seen[i]; g_disp_seen[i] = g_disp_seen[j]; g_disp_seen[j] = b;
      }
    }
}

static void drawFeedList(const RfRow* rows, int n, int top, int win);   // defined below
static bool isMuted(const char* mac);                                   // defined below
static uint16_t lerp565(uint16_t a, uint16_t b, float t);              // defined below

// Re-resolve g_cursor_mac → g_scan_sel after a feed rebuild. Keeps cursor on the
// same identity when entries expire or new ones are appended; no-op when cursor off.
static void cursorResolve() {
  if (!g_cursor_mode || !g_cursor_mac[0]) return;
  for (int i = 0; i < g_dispN; ++i)
    if (strncmp(g_disp[i].mac, g_cursor_mac, 17) == 0) { g_scan_sel = i; return; }
  if (g_scan_sel >= g_dispN) g_scan_sel = g_dispN > 0 ? g_dispN - 1 : 0;
}

static void drawBleScreen() {
  cv.fillRect(0, 0, W, H, BG);

  static unsigned long s_bleBuild = 0;
  if (!g_paused && (g_dispN == 0 || millis() - s_bleBuild > 250)) {
    drone::BleSight b[24]; size_t n = drone::bleSnapshot(b, 24);
    for (size_t i = 0; i < n; ++i) {
      bool isTrk = b[i].tracker != drone::TRK_NONE;
      const char* vnd = b[i].name[0] ? b[i].name : bleVendor(b[i].company_id, b[i].has_mfr);
      const char* id  = isTrk ? trackerLabel(b[i].tracker) : (vnd ? vnd : b[i].mac);
      feedUpsert(drone::SRC_BLE, b[i].tracker, id, b[i].mac, b[i].rssi);
    }
    feedExpire(5000);
    if (g_cursor_mode) cursorResolve(); else feedSortBleStable();
    s_bleBuild = millis();
  }

  int trk = 0; for (int i = 0; i < g_dispN; ++i) if (g_disp[i].tracker != drone::TRK_NONE && !isMuted(g_disp[i].mac)) trk++;
  int ftop = trk ? 24 : 6;                       // tracker alert pushes the feed down only when present
  drawFeedList(g_disp, g_dispN, ftop, H - ftop);
  cv.fillRect(0, 0, W, ftop, BG);                // clean the strip above the feed

  if (trk) {                                     // self-effacing alert: hidden when no trackers around
    const uint16_t RED = rgb565(0xF0,0x5A,0x5A);
    char rp[20];
    if (trk == 1) snprintf(rp, sizeof(rp), "1 tracker near");
    else          snprintf(rp, sizeof(rp), "%d trackers near", trk);
    fontSmall();
    int rw = cv.textWidth(rp) + 26;
    cv.fillSmoothRoundRect(6, 4, rw, 17, 8, RED);
    cv.fillSmoothCircle(16, 12, 3, rgb565(0xFF,0xFF,0xFF));
    cv.setTextColor(rgb565(0xFF,0xFF,0xFF), RED); cv.setTextDatum(middle_left);
    cv.setClipRect(6, 4, rw, 17); cv.drawString(rp, 24, 12); cv.clearClipRect();
  }
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

static void drawFeedList(const RfRow* rows, int n, int top, int win) {
  const int ROW = F_ROW_H;
  const uint16_t TRK_DOT = rgb565(0xF0,0x5A,0x5A);
  const uint16_t TRK_BG  = rgb565(0x3A,0x1E,0x22);
  const uint16_t TRK_FG  = rgb565(0xEE,0x8C,0x8C);

  if (n <= 0) {
    fontSmall(); cv.setTextDatum(middle_center); cv.setTextColor(MUTE, BG);
    cv.drawString("watching...", W / 2, top + win / 2);
    g_feed_scroll = 0; return;
  }

  unsigned long now = millis();
  float dt = g_feed_t ? (float)(now - g_feed_t) : 0.0f; g_feed_t = now;
  if (dt > 40.0f) dt = 40.0f;                       // a stalled frame can't lurch the scroll far
  if (g_feed_manual && !g_cursor_mode && now - g_feed_touch > 7000) g_feed_manual = false;

  int kVis = win / ROW; if (kVis < 1) kVis = 1;
  float total = (float)n * ROW;
  int   tot   = (int)total;
  bool ticker = (!g_feed_manual && !g_paused && n > kVis);

  if (g_feed_manual) {                              // cursor mode: ease to keep sel visible (no wrap)
    if (g_scan_sel < 0) g_scan_sel = 0; if (g_scan_sel >= n) g_scan_sel = n - 1;
    float maxS = total - win; if (maxS < 0) maxS = 0;
    float tgt = g_feed_scroll;
    if (g_scan_sel * ROW < tgt)             tgt = (float)(g_scan_sel * ROW);
    if ((g_scan_sel + 1) * ROW > tgt + win) tgt = (float)((g_scan_sel + 1) * ROW - win);
    if (tgt < 0) tgt = 0; if (tgt > maxS) tgt = maxS;
    g_feed_scroll += (tgt - g_feed_scroll) * (1.0f - expf(-dt / 55.0f));
  } else if (ticker) {                              // tilt-controlled flow, seamless wrap both ways
    g_feed_vel_eased += (g_feed_vel - g_feed_vel_eased) * (1.0f - expf(-dt / 55.0f));   // smooth speed (snappier)
    g_feed_scroll += g_feed_vel_eased * dt / 1000.0f;
    while (g_feed_scroll >= total) g_feed_scroll -= total;
    while (g_feed_scroll <  0.0f)  g_feed_scroll += total;
  } else {
    g_feed_scroll = 0;
    if (g_scan_sel < 0) g_scan_sel = 0; if (g_scan_sel >= n) g_scan_sel = n - 1;
  }

  int sInt = (int)(g_feed_scroll + 0.5f);

  auto row = [&](const RfRow& c, int y, bool sel) {
    int cy = y + (ROW - 4) / 2;
    bool isTrk = (c.tracker != drone::TRK_NONE);
    uint16_t rbg = sel ? LILAC : (isTrk ? TRK_BG : CARD);
    uint16_t fg  = sel ? HEAD_FG : (isTrk ? TRK_FG : FG);
    cv.fillSmoothRoundRect(4, y, 232, ROW - 4, 8, rbg);
    { bool nw3; unsigned long fseen = rfFirstSeen(c.mac, &nw3);
      if (!sel && (now - fseen) < 3000) cv.fillSmoothRoundRect(7, y + 5, 3, ROW - 14, 1, VERD); }
    int nameX;
    if (isTrk) { cv.fillSmoothCircle(16, cy, 4, sel ? HEAD_FG : TRK_DOT); nameX = 30; }
    else       { drawTypeBadge(12, cy, c.src, sel); nameX = 52; }
    char rs[8]; snprintf(rs, sizeof(rs), "%d", c.rssi);
    fontSmall(); cv.setTextDatum(middle_right);
    cv.setTextColor(sel ? HEAD_FG : MUTE, rbg); cv.drawString(rs, 226, cy);
    int nameW = (226 - cv.textWidth(rs) - 8) - nameX;
    cv.setClipRect(nameX, top, nameW > 0 ? nameW : 0, win);
    fontBody(); cv.setTextDatum(middle_left);
    cv.setTextColor(fg, rbg); cv.drawString(c.id, nameX, cy);
    cv.setClipRect(0, top, W, win);
  };

  cv.setClipRect(0, top, W, win);
  if (ticker) {
    int topIdx = -1, topY = 32767;
    for (int i = 0; i < n; ++i) {
      int base = (i * ROW + sInt) % tot;            // index 0 at the top; flow advances downward
      for (int k = 0; k < 2; ++k) {
        int y = top + base - k * tot;               // k=1 = the wrap copy re-entering at the top
        if (y + ROW <= top || y >= top + win) continue;
        row(rows[i], y, false);
        if (y < topY) { topY = y; topIdx = i; }
      }
    }
    if (topIdx >= 0) g_scan_sel = topIdx;            // drill target = the row currently at the top
  } else {
    // animate cursor highlight toward g_scan_sel; snap instantly in non-cursor manual
    float anim_target = (float)(g_scan_sel * ROW);
    if (g_cursor_mode) {
      g_cursor_anim_y += (anim_target - g_cursor_anim_y) * (1.0f - expf(-dt / 55.0f));
    } else {
      g_cursor_anim_y = anim_target;
    }
    int anim_sel = (int)lroundf(g_cursor_anim_y / ROW);
    if (anim_sel < 0) anim_sel = 0;
    if (anim_sel >= n) anim_sel = n - 1;

    for (int i = 0; i < n; ++i) {
      int y = top + i * ROW - sInt;
      if (y + ROW <= top || y >= top + win) continue;
      row(rows[i], y, (g_feed_manual && i == anim_sel));
    }
  }
  cv.clearClipRect();

  if (g_tilt_scrub && ticker) {                     // faint chevron points the scrub direction
    const uint16_t cc = rgb565(0x4A,0x4A,0x55);
    int cxp = W / 2;
    if (g_tilt_dir < 0) { int yy = top + 4;       cv.fillTriangle(cxp-7, yy+5, cxp+7, yy+5, cxp, yy, cc); }
    else                { int yy = top + win - 5; cv.fillTriangle(cxp-7, yy-5, cxp+7, yy-5, cxp, yy, cc); }
  }

  // ===== tilt HUD =====
  {
    const float HUD_DEAD = 0.11f, HUD_RANGE = 0.45f;   // HUD_DEAD must match the tick's TILT_DEAD
    const int   cxp = W / 2, half = 34, ty = H - 5;

    // (A) live hash indicator — fades in while tilting, marker rides toward the hashes
    static float s_hud = 0.0f;
    float tgt = (fabsf(g_tilt_e) > 0.05f) ? 1.0f : 0.0f;
    s_hud += (tgt - s_hud) * 0.20f;
    if (s_hud > 0.02f) {
      uint16_t base = lerp565(BG, rgb565(0x6A,0x6A,0x78), s_hud);
      uint16_t hot  = lerp565(BG, rgb565(0xF4,0xF4,0xF7), s_hud);
      cv.drawFastHLine(cxp - half, ty, half * 2, base);
      int dz = (int)(half * (HUD_DEAD / HUD_RANGE));     // deadzone-edge "scroll starts here" hashes
      cv.drawFastVLine(cxp - dz, ty - 3, 6, base);
      cv.drawFastVLine(cxp + dz, ty - 3, 6, base);
      cv.drawFastVLine(cxp,      ty - 2, 4, base);        // center (neutral) tick
      float pe = g_tilt_e / HUD_RANGE; if (pe > 1) pe = 1; if (pe < -1) pe = -1;
      int  mx   = cxp + (int)(pe * half);
      bool past = fabsf(g_tilt_e) > HUD_DEAD;
      cv.fillSmoothCircle(mx, ty, past ? 3 : 2, past ? hot : base);
    }

    // (B) entry hint — rocking device + "tilt to scroll", after the name toast
    unsigned long te = millis() - g_screen_since;
    float hop = 0.0f;
    if      (te < 1300) hop = 0.0f;
    else if (te < 1600) hop = (te - 1300) / 300.0f;
    else if (te < 2700) hop = 1.0f;
    else if (te < 3100) hop = 1.0f - (te - 2700) / 400.0f;
    if (hop > 0.02f) {
      uint16_t hc = lerp565(BG, rgb565(0xF4,0xF4,0xF7), hop * 0.9f);
      int hx = W / 2, hy = H - 30;
      float th = sinf(millis() / 150.0f) * 0.22f;        // rock ±~12 deg
      float co = cosf(th), si = sinf(th);
      const float pxs[4] = { -11, 11, 11, -11 };
      const float pys[4] = {  -6, -6,  6,   6 };
      int rx[4], ry[4];
      for (int i = 0; i < 4; ++i) {
        rx[i] = hx + (int)lroundf(pxs[i] * co - pys[i] * si);
        ry[i] = hy + (int)lroundf(pxs[i] * si + pys[i] * co);
      }
      cv.fillTriangle(rx[0], ry[0], rx[1], ry[1], rx[2], ry[2], hc);
      cv.fillTriangle(rx[0], ry[0], rx[2], ry[2], rx[3], ry[3], hc);
      uint16_t slit = lerp565(BG, rgb565(0x1C,0x1C,0x24), hop);
      cv.drawLine(rx[0] + (rx[3]-rx[0]) / 4, ry[0] + (ry[3]-ry[0]) / 4,
                  rx[1] + (rx[2]-rx[1]) / 4, ry[1] + (ry[2]-ry[1]) / 4, slit);
      cv.drawLine(hx - 21, hy - 3, hx - 16, hy, hc); cv.drawLine(hx - 21, hy + 3, hx - 16, hy, hc);
      cv.drawLine(hx + 21, hy - 3, hx + 16, hy, hc); cv.drawLine(hx + 21, hy + 3, hx + 16, hy, hc);
      fontSmall(); cv.setTextDatum(middle_center); cv.setTextColor(hc);
      cv.drawString("tilt to scroll", hx, hy + 17);
    }
  }

  // position counter: "4/17" pill in the top-right of the feed window during cursor mode
  if (g_cursor_mode && n > 0) {
    char pi[12]; snprintf(pi, sizeof(pi), "%d/%d", g_scan_sel + 1, n);
    fontSmall();
    int pw = cv.textWidth(pi) + 12;
    cv.fillSmoothRoundRect(W - 6 - pw, top + 4, pw, 15, 7, PILL_BG);
    cv.setTextColor(MUTE, PILL_BG); cv.setTextDatum(middle_center);
    cv.drawString(pi, W - 6 - pw / 2, top + 12);
  }
}

static void drawScanScreen() {
  cv.fillRect(0, 0, W, H, BG);

  static unsigned long s_scanBuild = 0;
  if (!g_paused && (g_dispN == 0 || millis() - s_scanBuild > 250)) {   // merge-refresh 4 Hz; scroll 30 fps
    char home_ssid[33]; home_ssid[0] = '\0';
    if (g_hide_home && WiFi.status() == WL_CONNECTED) {
      strncpy(home_ssid, WiFi.SSID().c_str(), sizeof(home_ssid)-1); home_ssid[sizeof(home_ssid)-1]='\0'; }
    for (int i = 0; i < g_ap_count; ++i) {
      if (!keepRow(drone::SRC_WIFI_2G, g_aps[i].rssi, g_aps[i].ssid, home_ssid)) continue;
      feedUpsert(drone::SRC_WIFI_2G, drone::TRK_NONE,
                 g_aps[i].ssid[0] ? g_aps[i].ssid : "(hidden)", g_aps[i].bssid, g_aps[i].rssi);
    }
    { drone::BleSight bs[24]; size_t nb = drone::bleSnapshot(bs, 24);
      for (size_t i = 0; i < nb; ++i) {
        const char* label = bs[i].name[0] ? bs[i].name : bleVendor(bs[i].company_id, bs[i].has_mfr);
        const char* id = label ? label : bs[i].mac;
        if (!keepRow(drone::SRC_BLE, bs[i].rssi, id, home_ssid)) continue;
        feedUpsert(drone::SRC_BLE, bs[i].tracker, id, bs[i].mac, bs[i].rssi);
      } }
    { c5link::WifiSight ws[24]; size_t nw = c5link::wifiSnapshot(ws, 24);
      for (size_t i = 0; i < nw; ++i) {
        uint8_t src = (ws[i].band == 5) ? drone::SRC_WIFI_5G : drone::SRC_WIFI_2G;
        if (!keepRow(src, ws[i].rssi, ws[i].ssid, home_ssid)) continue;
        feedUpsert(src, drone::TRK_NONE, ws[i].ssid[0] ? ws[i].ssid : "(hidden)", ws[i].bssid, ws[i].rssi);
      } }
    feedExpire(5000);
    if (g_cursor_mode) cursorResolve(); else feedSortByFirstSeen();
    s_scanBuild = millis();
  }

  drawFeedList(g_disp, g_dispN, 6, H - 6);
  cv.fillRect(0, 0, W, 6, BG);                  // trim any AA bleed at the very top edge
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

// ---- small color lerp for the breathing dot / glow ----
static uint16_t lerp565(uint16_t a, uint16_t b, float t) {
  if (t < 0) t = 0; if (t > 1) t = 1;
  int ar=(a>>11)&0x1F, ag=(a>>5)&0x3F, ab=a&0x1F;
  int br=(b>>11)&0x1F, bg=(b>>5)&0x3F, bb=b&0x1F;
  int r=ar+(int)((br-ar)*t+0.5f), g=ag+(int)((bg-ag)*t+0.5f), bl=ab+(int)((bb-ab)*t+0.5f);
  return (uint16_t)((r<<11)|(g<<5)|bl);
}

static const char* trackerNetwork(uint8_t t) {
  switch (t) {
    case drone::TRK_AIRTAG:   return "APPLE FIND MY";
    case drone::TRK_TILE:     return "TILE NETWORK";
    case drone::TRK_SMARTTAG: return "SAMSUNG SMARTTHINGS";
    default:                  return "FIND MY NETWORK";
  }
}

// ---- per-MAC mute (suppresses the tracker alert for a while) ----
static char          g_mute_mac[8][18];
static unsigned long g_mute_until[8] = {0};
static bool isMuted(const char* mac) {
  unsigned long now = millis();
  for (int i = 0; i < 8; ++i)
    if (g_mute_until[i] > now && strncmp(g_mute_mac[i], mac, 17) == 0) return true;
  return false;
}
static unsigned long muteRemaining(const char* mac) {
  unsigned long now = millis();
  for (int i = 0; i < 8; ++i)
    if (g_mute_until[i] > now && strncmp(g_mute_mac[i], mac, 17) == 0) return g_mute_until[i] - now;
  return 0;
}
static void muteAdd(const char* mac, unsigned long ms) {
  unsigned long now = millis(); int slot = -1;
  for (int i = 0; i < 8; ++i) { if (strncmp(g_mute_mac[i], mac, 17) == 0) { slot = i; break; }
                                if (slot < 0 && g_mute_until[i] <= now) slot = i; }
  if (slot < 0) slot = 0;
  strncpy(g_mute_mac[slot], mac, 17); g_mute_mac[slot][17] = '\0';
  g_mute_until[slot] = now + ms;
}

static void drawDetailScreen() {
  const RfRow& r = g_detail_row;
  bool isTrk  = (r.tracker != drone::TRK_NONE);
  bool named  = r.id[0] && strncmp(r.id, r.mac, 17) != 0;   // has a real name (not just its MAC)

  const uint16_t ACC    = isTrk ? COL_BAD : LILAC;
  const uint16_t BANNER = isTrk ? rgb565(0x40,0x1E,0x22) : rgb565(0x24,0x21,0x3C);
  const uint16_t NAMEC  = isTrk ? rgb565(0xF2,0xC6,0xC8) : FG;
  const uint16_t SUBC   = isTrk ? rgb565(0xCE,0x74,0x78) : MUTE;
  const uint16_t CARD2  = rgb565(0x16,0x14,0x18);
  const uint16_t DIMBAR = isTrk ? rgb565(0x4C,0x30,0x34) : rgb565(0x34,0x31,0x42);

  cv.fillRect(0, 0, W, H, BG);

  // ---- live RSSI, EMA, peak, sparkline history (seeded full so it reads as a line immediately) ----
  int8_t rssi; bool live = liveRssi(r.mac, r.src, &rssi);
  static char  st_mac[18] = "";
  static float ema = -90.0f, refEma = -90.0f;
  static int8_t peak = -120, hist[40]; static int histN = 0;
  static unsigned long lastPush = 0, refMs = 0;
  if (strncmp(st_mac, r.mac, 17) != 0) {                    // new contact -> reset + seed
    strncpy(st_mac, r.mac, 17); st_mac[17] = '\0';
    ema = live ? (float)rssi : (float)r.rssi;
    peak = (int8_t)ema; refEma = ema; refMs = millis(); lastPush = millis();
    for (int i = 0; i < 40; ++i) hist[i] = (int8_t)ema;     // full flat line at the current level
    histN = 40;
  } else if (live) {
    ema += 0.25f * ((float)rssi - ema);
  }
  if (live && (int8_t)ema > peak) peak = (int8_t)ema;
  if (millis() - lastPush > 150) {                          // ~6 s window across 40 samples
    lastPush = millis();
    memmove(hist, hist + 1, 39); hist[39] = (int8_t)ema;
  }
  float trend = ema - refEma;
  if (millis() - refMs > 900) { refEma = ema; refMs = millis(); }
  int rint = (int)lroundf(ema);

  // ---- banner ----
  const int bx = 5, by = 4, bw = W - 10, bh = 34;
  cv.fillSmoothRoundRect(bx, by, bw, bh, 11, BANNER);
  float pulse = 0.5f + 0.5f * sinf(millis() * 0.005f);
  uint16_t dotc = live ? lerp565(BANNER, ACC, 0.12f + 0.88f * pulse) : lerp565(BANNER, ACC, 0.35f);
  cv.fillSmoothCircle(bx + 15, by + bh / 2, 4, dotc);
  int nameClip = isTrk ? 148 : (bw - 27 - 12);             // full width when there's no status word
  fontBody(); cv.setTextDatum(top_left); cv.setTextColor(NAMEC, BANNER);
  cv.setClipRect(bx + 27, by, nameClip, bh);
  cv.drawString(r.id[0] ? r.id : "(unknown)", bx + 27, by + 4);
  cv.clearClipRect();
  fontSmall(); cv.setTextDatum(top_left); cv.setTextColor(SUBC, BANNER);
  cv.drawString(isTrk ? trackerNetwork(r.tracker) : srcName(r.src), bx + 27, by + 20);
  if (isTrk) {                                             // status word only for trackers (no "SIGNAL")
    fontBody(); cv.setTextDatum(middle_right); cv.setTextColor(ACC, BANNER);
    cv.drawString("TRACKING", bx + bw - 12, by + bh / 2);
  }

  // ---- left card: bars, dBm, direction ----
  const int lx = 5, ly = 42, lw = 138, lh = 50;
  cv.fillSmoothRoundRect(lx, ly, lw, lh, 10, CARD2);
  int bars = (int)lroundf((ema + 100.0f) / 12.0f); if (bars < 0) bars = 0; if (bars > 5) bars = 5;
  int barBottom = ly + 30, barX = lx + 14;
  for (int i = 0; i < 5; ++i) {
    int hh = 7 + i * 4, x = barX + i * 8, yy = barBottom - hh;
    cv.fillSmoothRoundRect(x, yy, 5, hh, 1, (i < bars) ? ACC : DIMBAR);
  }
  char dbm[12]; snprintf(dbm, sizeof(dbm), "%d dBm", rint);
  fontBody(); cv.setTextDatum(bottom_left); cv.setTextColor(live ? FG : MUTE, CARD2);
  cv.drawString(dbm, lx + 14, ly + lh - 8);
  int ax = lx + 112, ay = ly + 18;                        // direction column, clear of the dBm
  const char* dirw; uint16_t dirc;
  if      (trend >  2.0f) { dirw = "closer";  dirc = ACC;
    cv.fillTriangle(ax-6, ay+2, ax+6, ay+2, ax, ay-7, dirc); cv.fillRect(ax-2, ay+2, 4, 7, dirc); }
  else if (trend < -2.0f) { dirw = "farther"; dirc = isTrk ? COL_HEAD : MUTE;
    cv.fillTriangle(ax-6, ay-2, ax+6, ay-2, ax, ay+7, dirc); cv.fillRect(ax-2, ay-7, 4, 7, dirc); }
  else                    { dirw = "holding"; dirc = MUTE;
    cv.fillRect(ax-6, ay-1, 12, 3, dirc); }
  fontSmall(); cv.setTextDatum(top_center); cv.setTextColor(dirc, CARD2);   // small so it can't hit the dBm
  cv.drawString(dirw, ax, ly + lh - 20);

  // ---- right card: sparkline + closest ----
  const int rx = lx + lw + 4, ry = 42, rh = 50, rw = W - 5 - rx;
  cv.fillSmoothRoundRect(rx, ry, rw, rh, 10, CARD2);
  int gx0 = rx + 8, gx1 = rx + rw - 8, gy0 = ry + 8, gy1 = ry + 30;
  auto ymap = [&](int8_t v){ float t=((float)v+90.0f)/50.0f; if(t<0)t=0; if(t>1)t=1; return (int)(gy1-t*(gy1-gy0)); };
  int px = gx0, py = ymap(hist[0]);
  for (int i = 1; i < histN; ++i) {
    int cx = gx0 + (gx1 - gx0) * i / (histN - 1), cy = ymap(hist[i]);
    cv.drawLine(px, py, cx, cy, ACC); cv.drawLine(px, py + 1, cx, cy + 1, ACC);
    px = cx; py = cy;
  }
  cv.fillSmoothCircle(px, py, 5, lerp565(CARD2, ACC, 0.35f));
  cv.fillSmoothCircle(px, py, 3, ACC);
  float dm = powf(10.0f, ((float)(-59) - (float)peak) / 25.0f);
  char cm[16];
  if (dm < 1.0f) snprintf(cm, sizeof(cm), "closest <1 m");
  else           snprintf(cm, sizeof(cm), "closest %d m", (int)lroundf(dm));
  fontSmall(); cv.setTextDatum(bottom_center); cv.setTextColor(MUTE, CARD2);
  cv.drawString(cm, rx + rw / 2, ry + rh - 8);

  // ---- bottom: id (only if it adds info) + co-presence, and the pill ----
  bool nw = false; unsigned long fs = rfFirstSeen(r.mac, &nw);
  unsigned long secs = fs ? (millis() - fs) / 1000 : 0;
  const char* du = isTrk ? "with you" : "seen";
  char idline[48];
  if (named) {
    if (secs >= 60) snprintf(idline, sizeof(idline), "id %.11s  \xB7  %s %lum", r.mac, du, secs / 60);
    else            snprintf(idline, sizeof(idline), "id %.11s  \xB7  %s %lus", r.mac, du, secs);
  } else {                                                 // banner already shows the MAC -> don't repeat it
    if (secs >= 60) snprintf(idline, sizeof(idline), "%s %lum", du, secs / 60);
    else            snprintf(idline, sizeof(idline), "%s %lus", du, secs);
  }

  fontSmall();
  char pill[18]; uint16_t pbg, pfg;
  unsigned long mrem = muteRemaining(r.mac);
  if      (mrem)  { snprintf(pill, sizeof(pill), "muted %lum", mrem / 60000UL + 1); pbg = rgb565(0x33,0x33,0x3C); pfg = MUTE; }
  else if (isTrk) { snprintf(pill, sizeof(pill), "hold B \xB7 mute 1h");             pbg = rgb565(0x5A,0x4E,0x86); pfg = rgb565(0xFF,0xFF,0xFF); }
  else            { snprintf(pill, sizeof(pill), "hold A \xB7 back");                pbg = rgb565(0x2A,0x2A,0x33); pfg = MUTE; }
  int pw = cv.textWidth(pill) + 18, ppx = W - 6 - pw, ppy = 96;
  cv.fillSmoothRoundRect(ppx, ppy, pw, 19, 9, pbg);
  cv.setTextColor(pfg, pbg); cv.setTextDatum(middle_center);
  cv.setClipRect(ppx, ppy, pw, 19); cv.drawString(pill, ppx + pw / 2, ppy + 10); cv.clearClipRect();

  cv.setTextDatum(middle_left); cv.setTextColor(MUTE, BG);
  cv.setClipRect(6, ppy, ppx - 12, 19);
  cv.drawString(idline, 8, ppy + 10);
  cv.clearClipRect();
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
static void drawSetupScreen() {
  cv.fillRect(0, 0, W, H, BG);
  // headerless — settings glyph (bottom-right); battery is now a row

  if (!g_portal_active) {
    bool wup = (WiFi.status() == WL_CONNECTED);

    auto inforow = [&](int y, const char* lab, const char* val, bool dot, bool dotOk) {
      drawCard(4, y, 232, 28, CARD, false);
      int cy = y + 14;
      int rightEdge = dot ? (W - 28) : (W - 14);
      fontSmall(); cv.setTextDatum(middle_left); cv.setTextColor(MUTE, CARD);
      cv.setClipRect(16, y, 60, 28);
      cv.drawString(lab, 16, cy);
      cv.clearClipRect();
      fontBody(); cv.setTextDatum(middle_right); cv.setTextColor(FG, CARD);
      cv.setClipRect(82, y, rightEdge - 82, 28);
      cv.drawString(val, rightEdge, cy - 1);
      cv.clearClipRect();
      if (dot) cv.fillSmoothCircle(W - 16, cy, 4, dotOk ? VERD : COL_BAD);
    };

    const char* ssid = g_cfg.ssid.length() ? g_cfg.ssid.c_str() : "(not set)";
    inforow(6,  "WIFI", ssid, true, wup);
    char loc[24]; snprintf(loc, sizeof(loc), "%.2f, %.2f", g_cfg.lat, g_cfg.lon);
    inforow(38, "LOCATION", loc, false, false);

    int  pct = battPctFromMv(M5.Power.getBatteryVoltage());
    bool chg = M5.Power.isCharging();
    char bs[12]; snprintf(bs, sizeof(bs), chg ? "%d%%  +" : "%d%%", pct);
    inforow(70, "BATTERY", bs, true, chg || pct > 20);

    int ay = 102, ah = 28;
    drawCard(4, ay, 232, ah, LILAC, false);
    fontBody(); cv.setTextDatum(middle_left); cv.setTextColor(HEAD_FG, LILAC);
    cv.drawString("Setup portal", 16, ay + ah / 2);
    fontSmall(); cv.setTextDatum(middle_right); cv.setTextColor(rgb565(0x5A,0x4E,0x80), LILAC);
    cv.drawString("hold B", W - 14, ay + ah / 2);

  } else {
    unsigned long elapsed = millis() - g_portal_start;
    unsigned long rem = elapsed < kPortalTimeoutMs ? (kPortalTimeoutMs - elapsed) / 1000 : 0;

    drawCard(4, 6, 232, 44, LILAC, false);
    fontSmall(); cv.setTextDatum(top_left); cv.setTextColor(rgb565(0x5A,0x4E,0x80), LILAC);
    cv.drawString("join wifi", 16, 13);
    fontBody(); cv.setTextColor(HEAD_FG, LILAC); cv.drawString("Aware-Setup", 16, 26);

    drawCard(4, 56, 232, 28, CARD, false);
    fontSmall(); cv.setTextDatum(middle_left); cv.setTextColor(MUTE, CARD); cv.drawString("OPEN", 16, 70);
    fontBody(); cv.setTextColor(FG, CARD); cv.drawString("192.168.4.1", 70, 70);

    char timer[28]; snprintf(timer, sizeof(timer), "timeout %lus  -  A exits", rem);
    fontSmall(); cv.setTextDatum(top_left); cv.setTextColor(MUTE, BG); cv.drawString(timer, 8, 92);
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
// --- corner identity glyph -------------------------------------------------
// White line-art mark, top-right. Bright flash on entry -> faint resting
// watermark. Drawn AFTER the feed so it overlays. Gated to feeds in render().
static void drawCornerGlyph() {
  unsigned long t = millis() - g_screen_since;
  float op; int yoff;
  if (t < 220)      { float k = t / 220.0f;          op = 0.92f * k;          yoff = (int)(-6.0f * (1.0f - k)); }
  else if (t < 720) { float k = (t - 220) / 500.0f;  op = 0.92f - 0.60f * k;  yoff = 0; }   // settle to ~0.32
  else              {                                 op = 0.32f;             yoff = 0; }

  const uint16_t c = lerp565(BG, rgb565(0xFF, 0xFF, 0xFF), op);
  const int cx = (g_screen == SCR_AIRSPACE) ? 16 : (W - 16);
  const int cy = ((g_screen == SCR_SETUP) ? (H - 15) : 15) + yoff;

  switch (g_screen) {
    case SCR_SCAN: {                                   // radiating waves
      cv.fillSmoothCircle(cx, cy + 7, 2, c);
      cv.drawArc(cx, cy + 7, 4, 6, 215, 325, c);
      cv.drawArc(cx, cy + 7, 8, 10, 215, 325, c);
      break;
    }
    case SCR_BLE: {                                    // bluetooth rune (2px lines)
      auto L = [&](int x0, int y0, int x1, int y1) {
        cv.drawLine(cx + x0,     cy + y0, cx + x1,     cy + y1, c);
        cv.drawLine(cx + x0 + 1, cy + y0, cx + x1 + 1, cy + y1, c);
      };
      L(0, -8,  0,  8);     // spine
      L(0, -8,  4, -3);     // top flag out
      L(4, -3, -4,  3);     // top flag cross
      L(0,  8,  4,  3);     // bottom flag out
      L(4,  3, -4, -3);     // bottom flag cross
      break;
    }
    case SCR_AIRSPACE: {                               // little plane (future)
      cv.fillTriangle(cx,     cy - 7, cx - 2, cy - 1, cx + 2, cy - 1, c);  // nose
      cv.fillTriangle(cx - 7, cy + 3, cx + 7, cy + 3, cx,     cy - 2, c);  // wings
      cv.fillTriangle(cx - 3, cy + 7, cx + 3, cy + 7, cx,     cy + 2, c);  // tail
      break;
    }
    case SCR_STATS: {                                  // bar chart (future)
      cv.fillRect(cx - 6, cy + 1, 3,  5, c);
      cv.fillRect(cx - 1, cy - 2, 3,  8, c);
      cv.fillRect(cx + 4, cy - 5, 3, 11, c);
      break;
    }
    case SCR_CONN: {                                   // node graph
      const int ox[3] = { cx,     cx - 7, cx + 7 };
      const int oy[3] = { cy - 7, cy + 5, cy + 5 };
      for (int i = 0; i < 3; ++i) {
        cv.drawLine(cx, cy, ox[i], oy[i], c);
        cv.fillSmoothCircle(ox[i], oy[i], 2, c);
      }
      cv.fillSmoothCircle(cx, cy, 2, c);
      break;
    }
    case SCR_SETUP: {                                  // settings sliders
      const int kx[3] = { 4, -3, 6 };
      for (int i = 0; i < 3; ++i) {
        int yy = cy - 6 + i * 6;
        cv.drawLine(cx - 8, yy, cx + 8, yy, c);
        cv.fillSmoothCircle(cx + kx[i], yy, 2, c);
      }
      break;
    }
    default: break;
  }
}

// ---------------------------------------------------------------------------
// Locator output — single function so the output device (onboard speaker now,
// hat-LEDC later) can be swapped by changing only this one call site.
// ---------------------------------------------------------------------------
static void locatorBeep(uint16_t freq_hz, int dur_ms) {
  M5.Speaker.tone(freq_hz, (uint32_t)dur_ms);
}

static void drawLocateScreen() {
  unsigned long now = millis();
  bool lost = (g_locate_last_seen_ms == 0 || now - g_locate_last_seen_ms > kLocLostMs);

  drawTopBar("LOCATE");
  drawBottomBar("A: exit", "", "");

  // Target label
  fontBody();
  cv.setTextDatum(middle_center);
  cv.setTextColor(FG, BG);
  cv.drawString(g_locate_id[0] ? g_locate_id : g_locate_mac, W / 2, CONTENT_Y + 16);

  if (lost) {
    fontSmall();
    cv.setTextColor(COL_BAD, BG);
    cv.drawString("SIGNAL LOST", W / 2, CONTENT_Y + 40);
  } else {
    // RSSI value
    char buf[16]; snprintf(buf, sizeof(buf), "%d dBm", (int)g_rssi_smooth);
    fontSmall();
    cv.setTextColor(MUTE, BG);
    cv.drawString(buf, W / 2, CONTENT_Y + 40);

    // Distance hint
    const char* hint; uint16_t hcol;
    if      (g_rssi_smooth >= kLocNear)        { hint = "NEAR";    hcol = COL_OK;   }
    else if (g_rssi_smooth >= -65.0f)           { hint = "CLOSE";   hcol = LILAC;    }
    else if (g_rssi_smooth >= kLocFar + 25.0f)  { hint = "FAR";     hcol = COL_HEAD; }
    else                                         { hint = "DISTANT"; hcol = MUTE;     }
    fontBody();
    cv.setTextColor(hcol, BG);
    cv.drawString(hint, W / 2, CONTENT_Y + 60);

    // RSSI bar
    float rssi_c = (g_rssi_smooth < kLocFar) ? kLocFar : (g_rssi_smooth > kLocNear ? kLocNear : g_rssi_smooth);
    float frac   = (rssi_c - kLocFar) / (kLocNear - kLocFar);
    int   bx = 20, by = CONTENT_Y + 78, bw = W - 40, bh = 8;
    cv.fillRect(bx, by, bw, bh, CARD);
    cv.fillRect(bx, by, (int)(frac * bw), bh, hcol);
  }
}

static void render() {
  // FPS counter — logs every 5 s with render budget context
  { static int s_n = 0; static unsigned long s_t = 0;
    if (++s_n == 1 && s_t == 0) s_t = millis();
    unsigned long nm = millis();
    if (nm - s_t >= 5000) {
      Serial.printf("[fps] %.1f fps  psram=%uB  sprite=%dx%d-16bpp  spi=80MHz\n",
                    s_n * 1000.0f / (float)(nm - s_t),
                    (unsigned)ESP.getPsramSize(), W, H);
      s_n = 0; s_t = nm;
    }
  }
  if (g_locating) { cv.fillScreen(BG); drawLocateScreen(); cv.pushSprite(0, 0); return; }
  if (g_detail) { cv.fillScreen(F_BG); drawDetailScreen(); cv.pushSprite(0, 0); return; }
  if (g_screen == SCR_AIRSPACE && g_air_detail) { cv.fillScreen(BG); drawAirspaceDetail(); cv.pushSprite(0, 0); return; }
  cv.fillScreen(COL_BG);
  switch (g_screen) {
    case SCR_AIRSPACE: drawAirspaceScreen(); break;
    case SCR_CONN:     drawConnScreen(); break;
    case SCR_SCAN:     drawScanScreen(); break;
    case SCR_BLE:      drawBleScreen();  break;
    case SCR_STATS:    drawStatsScreen(); break;
    case SCR_SETUP:    drawSetupScreen(); break;
  }
  drawCornerGlyph();
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
  cfg.output_power = false;       // do not raise EXT_5V at boot — silences Speaker Hat (PAM8303 on floating input)
  M5.begin(cfg);
  M5.Power.setExtOutput(false);   // belt-and-suspenders: keep Hat/Grove 5V off
  M5.Speaker.begin();
  M5.Speaker.setVolume(appcfg::kSpeakerVolume);   // see config.h (default 200 ≈ 78%)
  M5.Speaker.playWav(aware_boot_wav, aware_boot_wav_len);  // boot chime — non-blocking (DMA)
  g_aircraft_va.init(aircraft_wav, aircraft_wav_len, 9000);
  g_drone_va.init(drone_wav,    drone_wav_len,    9000);
  g_tracker_va.init(tracker_wav, tracker_wav_len, 60000);
  Serial.printf("PSRAM: %u bytes\n", (unsigned)ESP.getPsramSize());
  setCpuFrequencyMhz(160);
  M5.Display.setRotation(1);
  M5.Display.setBrightness(60);
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

  // --- experimental: raise the panel SPI write clock to shorten full-frame pushes ---
  {
    auto bus = reinterpret_cast<lgfx::Bus_SPI*>(M5.Display.getPanel()->getBus());
    auto bc  = bus->config();
    bc.freq_write = 80000000;                  // try 80 MHz (default is ~40); halves the push time
    bus->config(bc);
  }

  {
    esp_reset_reason_t rr = esp_reset_reason();
    const char* rn;
    switch (rr) {
      case ESP_RST_POWERON:   rn = "POWERON";   break;
      case ESP_RST_EXT:       rn = "EXT";       break;
      case ESP_RST_SW:        rn = "SW";        break;
      case ESP_RST_PANIC:     rn = "PANIC";     break;
      case ESP_RST_INT_WDT:   rn = "INT_WDT";   break;
      case ESP_RST_TASK_WDT:  rn = "TASK_WDT";  break;
      case ESP_RST_WDT:       rn = "WDT";       break;
      case ESP_RST_DEEPSLEEP: rn = "DEEPSLEEP"; break;
      case ESP_RST_BROWNOUT:  rn = "BROWNOUT";  break;
      default:                rn = "OTHER";     break;
    }
    Serial.printf("[boot] reset_reason=%d (%s)\n", (int)rr, rn);
    bool clean = (rr == ESP_RST_POWERON || rr == ESP_RST_SW);
    cv.fillScreen(COL_BG);
    cv.setTextDatum(middle_center);
    fontSmall(); cv.setTextColor(COL_TEXT, COL_BG);
    cv.drawString("Aware", W / 2, H / 2 - 16);
    fontBody(); cv.setTextColor(clean ? COL_TEXT : COL_BAD, COL_BG);
    cv.drawString(rn, W / 2, H / 2 + 8);
    cv.pushSprite(0, 0);
    delay(clean ? 600 : 2000);   // linger on a crash/brownout reason so it's readable
  }
  g_popup_until = millis() + 600;

  loadCfg();
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_cfg.ssid.c_str(), g_cfg.pass.c_str());
  WiFi.setSleep(true);
  esp_wifi_set_max_tx_power(34);   // ~8.5 dBm: trims the TX current spike that trips brownout
  services::adsb::begin();
  services::adsb::setCenter(g_cfg.lat, g_cfg.lon, rangeKm() * 1.3f);

  drone::begin();  // BLE Remote ID scan runs continuously from here on
  g_bleScanRunning = true;
}

void loop() {
  M5.update();
  if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed()) g_last_motion = millis();
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

  // BtnA click: exit cursor / exit locating / exit detail / cycle screens
  if (M5.BtnA.wasClicked()) {
    if (g_cursor_mode && (g_screen == SCR_SCAN || g_screen == SCR_BLE)) {
      if (millis() - g_cursor_idle_ms > 300)  // ignore the click that tails the entering A-hold
        { g_cursor_mode = false; g_feed_manual = false; }
    }
    else if (g_locating)      { g_locating = false; }
    else if (g_portal_active) stopConfigPortal();
    else if (g_detail)        g_detail = false;          // back to the list
    else                      gotoScreen(g_screen + 1);
    render();
  }

  // BtnA long-press: feed -> FREEZE the feed + drop a cursor on the current top
  //                  row (enter-only); detail -> locate this row; airspace -> pause
  if (M5.BtnA.wasHold()) {
    if ((g_screen == SCR_SCAN || g_screen == SCR_BLE) && !g_detail) {
      if (!g_cursor_mode) {                        // enter only — never toggle out here
        g_cursor_mode = true; g_feed_manual = true; g_feed_touch = millis();
        g_cursor_idle_ms = millis();
        if (g_dispN > 0) {
          if (g_scan_sel < 0) g_scan_sel = 0;
          if (g_scan_sel >= g_dispN) g_scan_sel = g_dispN - 1;
          strncpy(g_cursor_mac, g_disp[g_scan_sel].mac, 17); g_cursor_mac[17] = '\0';
        }
        g_feed_scroll   = (float)(g_scan_sel * F_ROW_H);    // snap: highlighted row at top NOW
        g_cursor_anim_y = (float)(g_scan_sel * F_ROW_H);    // seed: no jump on entry
        render();
      }
    } else if ((g_screen == SCR_SCAN || g_screen == SCR_BLE) && g_detail) {
      strncpy(g_locate_mac, g_detail_row.mac, sizeof(g_locate_mac) - 1);
      g_locate_mac[sizeof(g_locate_mac) - 1] = '\0';
      strncpy(g_locate_id,  g_detail_row.id,  sizeof(g_locate_id)  - 1);
      g_locate_id[sizeof(g_locate_id) - 1] = '\0';
      g_rssi_smooth         = (float)g_detail_row.rssi;
      g_locate_last_seen_ms = millis();
      g_locate_last_beep_ms = 0;
      g_locating            = true;
      render();
    } else if (g_screen == SCR_AIRSPACE) {
      g_paused = !g_paused;
      if (!g_paused) g_detail = false;
      render();
    }
  }

  // BtnB click: feed = enter cursor / select entry; airspace = cycle; setup = portal
  if (M5.BtnB.wasClicked()) {
    if (g_screen == SCR_SCAN || g_screen == SCR_BLE) {
      if (g_detail) {
        // in detail: cycle selection (legacy behaviour)
        if (g_dispN > 0) { g_scan_sel = (g_scan_sel + 1) % g_dispN; g_detail_row = g_disp[g_scan_sel]; }
      } else if (g_cursor_mode) {
        // cursor active: B click selects the highlighted entry
        if (g_dispN > 0 && g_scan_sel >= 0 && g_scan_sel < g_dispN) {
          RfRow& sel = g_disp[g_scan_sel];
          if (sel.src == drone::SRC_BLE) {
            // BLE entry → enter locator mode directly
            strncpy(g_locate_mac, sel.mac, sizeof(g_locate_mac) - 1);
            g_locate_mac[sizeof(g_locate_mac) - 1] = '\0';
            strncpy(g_locate_id,  sel.id,  sizeof(g_locate_id)  - 1);
            g_locate_id[sizeof(g_locate_id) - 1] = '\0';
            g_rssi_smooth         = (float)sel.rssi;
            g_locate_last_seen_ms = millis();
            g_locate_last_beep_ms = 0;
            g_locating            = true;
            g_cursor_mode = false; g_feed_manual = false;
          } else {
            // non-BLE → open detail view
            g_detail_row = sel; g_detail = true;
            g_cursor_mode = false; g_feed_manual = false;
          }
        }
      }
      // cursor inactive: B does nothing on the live feed — A-hold starts picking
    } else if (g_screen == SCR_AIRSPACE) {
      if (g_air_count > 0) g_airspace_sel = (g_airspace_sel + 1) % g_air_count;
    } else if (g_screen == SCR_SETUP && !g_portal_active) {
      startConfigPortal();
    }
    render();
  }

  // BtnB long-press: paused = drill into item; in item = back to list; airspace = range
  if (M5.BtnB.wasHold()) {
    if (g_screen == SCR_AIRSPACE) {
      if (g_air_count > 0) {
        g_air_detail = !g_air_detail;
      } else {
        g_range_idx = (g_range_idx + 1) % appcfg::kRangePresetCount;
        services::adsb::setCenter(g_cfg.lat, g_cfg.lon, rangeKm() * 1.3f);
      }
    } else if (g_screen == SCR_SCAN || g_screen == SCR_BLE) {
      if (g_cursor_mode && !g_detail) {
        // B hold exits cursor mode
        g_cursor_mode = false; g_feed_manual = false;
      } else if (g_detail) {
        if (g_detail_row.tracker != drone::TRK_NONE) muteAdd(g_detail_row.mac, 3600000UL);  // mute 1h
        else g_detail = false;
      } else if (g_dispN > 0) {
        g_feed_manual = true; g_feed_touch = millis();
        int s = g_scan_sel; if (s < 0) s = 0; if (s >= g_dispN) s = g_dispN - 1;
        g_detail_row = g_disp[s]; g_detail = true;
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
        {
          const services::adsb::Aircraft* L = services::adsb::aircraftList();
          size_t nac = services::adsb::aircraftCount();
          const char* ids[64];
          for (size_t i = 0; i < nac; ++i) ids[i] = L[i].callsign;
          if (!g_locating) g_aircraft_va.check(ids, (int)nac, g_screen == SCR_AIRSPACE);
        }
        if (g_screen == SCR_AIRSPACE) render();
      }
    }
  }

  static unsigned long last_prune = 0;
  if (now - last_prune > 2000) {
    last_prune = now;
    drone::prune();

    // Drone announcement: prefer Remote ID serial as dedup key; fall back to MAC.
    // Both BLE and C5 WiFi drones share the same s_tab, so one snapshot covers both.
    {
      drone::Drone dr[12]; int nd = (int)drone::snapshot(dr, 12);
      const char* ids[12];
      for (int i = 0; i < nd; ++i) ids[i] = dr[i].id[0] ? dr[i].id : dr[i].mac;
      if (!g_locating) g_drone_va.check(ids, nd, g_screen == SCR_AIRSPACE);
    }

    // Tracker announcement: only fire for "following" contacts —
    // those with sustained proximity (RSSI) and presence over kFollowPersistMs.
    // MAC is the only key; rotation resets the clock (acceptable — see threshold notes above).
    {
      drone::BleSight bl[24]; int nb = (int)drone::bleSnapshot(bl, 24);
      const char* ids[24]; int ntrk = 0;
      for (int i = 0; i < nb; ++i) {
        if (bl[i].tracker == drone::TRK_NONE) continue;
        if (bl[i].rssi < kFollowRssiMin) continue;
        if (now - bl[i].first_seen_ms < kFollowPersistMs) continue;
        ids[ntrk++] = bl[i].mac;
      }
      if (!g_locating) g_tracker_va.check(ids, ntrk, g_screen == SCR_BLE);
    }
  }

  // Locator mode: poll BLE table for target RSSI and drive the beep engine
  if (g_locating) {
    static unsigned long s_loc_poll = 0;
    if (now - s_loc_poll >= 100) {
      s_loc_poll = now;
      drone::BleSight bl[24]; int nb = (int)drone::bleSnapshot(bl, 24);
      for (int i = 0; i < nb; ++i) {
        if (strncmp(bl[i].mac, g_locate_mac, 17) == 0) {
          g_rssi_smooth += kLocAlpha * ((float)bl[i].rssi - g_rssi_smooth);
          g_locate_last_seen_ms = now;
          break;
        }
      }
      render();
    }
    bool lost = (now - g_locate_last_seen_ms > kLocLostMs);
    if (lost) {
      if (now - g_locate_last_beep_ms >= kLocSearchPeriod) {
        locatorBeep(kLocSearchFreq, kLocSearchDurMs);
        g_locate_last_beep_ms = now;
      }
    } else if (g_rssi_smooth >= kLocNear) {
      // Very close: near-continuous rapid beep
      if (now - g_locate_last_beep_ms >= (unsigned long)kLocFastMs) {
        locatorBeep(kLocFreq, kLocDurMs);
        g_locate_last_beep_ms = now;
      }
    } else {
      float rssi_c = (g_rssi_smooth < kLocFar) ? kLocFar : g_rssi_smooth;
      float t = (rssi_c - kLocFar) / (kLocNear - kLocFar);
      unsigned long interval = (unsigned long)(kLocSlowMs + (float)(kLocFastMs - kLocSlowMs) * t);
      if (now - g_locate_last_beep_ms >= interval) {
        locatorBeep(kLocFreq, kLocDurMs);
        g_locate_last_beep_ms = now;
      }
    }
  }

  // IMU: gesture + auto-rotate + motion-idle at 100 Hz (10 ms)
  // LP k scaled from original 0.15 @ 50 ms → 0.033 @ 10 ms (same ~300 ms time constant).
  static unsigned long last_imu    = 0;
  static float         gx = 0, gy = 0, gz = 0;  // low-pass gravity estimate
  static float         last_mag    = 1.0f;
  if (now - last_imu > 10) {
    last_imu = now;
    float ax, ay, az;
    M5.Imu.update();
    M5.Imu.getAccel(&ax, &ay, &az);
    const float k = 0.09f;    // ~110 ms LP — snappier tilt response (was 0.033/~300 ms)
    gx += k * (ax - gx); gy += k * (ay - gy); gz += k * (az - gz);
    float h = (fabsf(gx) >= fabsf(gy)) ? gx : gy;  // dominant horizontal axis
    // auto-rotate: limit to 20 Hz — no need to check orientation on every IMU tick
    { static unsigned long s_rot_t = 0;
      if (now - s_rot_t >= 50 && fabsf(h) > 0.5f && g_screen != SCR_SCAN && g_screen != SCR_BLE) {
        s_rot_t = now;
        static int rot = 1;
        int want = (h > 0) ? 1 : 3;            // swap 1 and 3 if screen ends up upside-down
        if (want != rot) { rot = want; M5.Display.setRotation(rot); render(); }
      } }
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    if (fabsf(mag - last_mag) > 0.06f) g_last_motion = now;
    last_mag = mag;

    // ---- tilt-shuttle: roll axis sets feed scroll velocity ----
    {
      const float TILT_SIGN = 1.0f;     // set -1 if forward tilt scrolls the wrong way
      const float TILT_DEAD = 0.11f;    // narrower neutral band (~6 deg) — reacts to smaller tilt
      const float TILT_SENS = 850.0f;   // px/s per g past the deadzone — more scroll per degree
      float tilt = TILT_SIGN * gx;      // LEFT-RIGHT (roll) axis = gx; if it does nothing, try gy or gz
      if (g_tilt_neutral > 90.0f) g_tilt_neutral = tilt;          // first-run seed
      if ((g_screen == SCR_SCAN || g_screen == SCR_BLE) && !g_detail && !g_paused) {
        float e = tilt - g_tilt_neutral;
        g_tilt_e = e;
        if (e > TILT_DEAD) {                                      // tip forward -> faster down
          g_feed_vel = TICKER_PXPS + (e - TILT_DEAD) * TILT_SENS; g_tilt_scrub = true; g_tilt_dir = 1;
        } else if (e < -TILT_DEAD) {                             // tip back -> reverse (up)
          g_feed_vel = -((-e - TILT_DEAD) * TILT_SENS);          g_tilt_scrub = true; g_tilt_dir = -1;
        } else {                                                 // neutral -> normal auto ticker
          g_feed_vel = TICKER_PXPS;                              g_tilt_scrub = false; g_tilt_dir = 0;
        }
        if (g_feed_vel >  300.0f) g_feed_vel =  300.0f;
        if (g_feed_vel < -300.0f) g_feed_vel = -300.0f;
        // neutral re-center — k scaled for 10 ms to preserve original time constants (~975 ms / ~6.2 s)
        g_tilt_neutral += (g_tilt_scrub ? 0.0016f : 0.010f) * (tilt - g_tilt_neutral);
      } else {
        g_feed_vel = TICKER_PXPS; g_tilt_scrub = false; g_tilt_dir = 0; g_tilt_e = 0.0f;
      }
    }

    // ---- cursor mode: tilt to move the highlight up/down. Uses the SAME calibrated
    //      left/right (roll) signal as scrolling — g_tilt_e is neutral-subtracted, so
    //      it works at any hold angle. Tilt-and-hold auto-repeats; return to level to
    //      stop. (The ticker is frozen in cursor mode, so this is unambiguous.)
    if (g_cursor_mode && (g_screen == SCR_SCAN || g_screen == SCR_BLE) && !g_detail) {
      const float STEP_SIGN = 1.0f;          // flip to -1 if up/down come out reversed
      const float STEP_DEAD = 0.12f;         // tilt past this (g, from neutral) = move
      const unsigned long FIRST_MS  = 420;   // delay before a held tilt starts repeating
      const unsigned long REPEAT_MS = 170;   // step interval while held
      static unsigned long s_next = 0;
      static int s_dir = 0;
      float e = STEP_SIGN * g_tilt_e;
      int dir = (e >= STEP_DEAD) ? 1 : (e <= -STEP_DEAD) ? -1 : 0;
      if (dir == 0) {
        s_dir = 0;                            // back to level — disarm
      } else {
        unsigned long t = millis();
        bool fire = false;
        if (dir != s_dir)      { fire = true; s_dir = dir; s_next = t + FIRST_MS; } // first step now
        else if (t >= s_next)  { fire = true; s_next = t + REPEAT_MS; }             // held -> repeat
        if (fire && g_dispN > 0) {
          g_scan_sel += dir;
          if (g_scan_sel < 0)        g_scan_sel = 0;
          if (g_scan_sel >= g_dispN) g_scan_sel = g_dispN - 1;
          strncpy(g_cursor_mac, g_disp[g_scan_sel].mac, 17);
          g_cursor_mac[17] = '\0';
          g_cursor_idle_ms = millis();
        }
      }
    }
  }

  static unsigned long last_batt = 0;
  static int low_reads = 0;
  if (now - last_batt > 5000) {
    last_batt = now;
    int  mv    = M5.Power.getBatteryVoltage();
    bool onUsb = M5.Power.isCharging();
    g_mv_cache = mv; g_usb_cache = onUsb;
    if (onUsb || mv < 2600) {     // on USB, or no/garbled cell read -> never "low"
      low_reads = 0;
    } else if (mv < 3100) {
      if (++low_reads >= 3) {                // ~15 s sustained low (and NOT on USB)
        cv.fillScreen(COL_BG);
        cv.setTextColor(COL_BAD, COL_BG);
        cv.setTextDatum(middle_center);
        cv.drawString("LOW BATTERY", W / 2, H / 2);
        cv.pushSprite(0, 0);
        delay(1500);
        low_reads = 0;                       // warn only -- do NOT deep-sleep
        // esp_deep_sleep_start() removed: no wake source is configured, so with USB
        // attached it can re-wake immediately (reset loop) and on battery it bricks
        // until a power-cycle. Re-enable only after a BtnA/PWR wake GPIO is set up.
      }
    } else {
      low_reads = 0;
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

  static unsigned long last_feed = 0;
  if ((g_screen == SCR_SCAN || g_screen == SCR_BLE) && !g_detail && now - last_feed > 16) {
    last_feed = now; render();
  }
  static unsigned long last_air = 0;
  if (g_screen == SCR_AIRSPACE && !g_air_detail && now - last_air > 33) { last_air = now; render(); }

  if (g_popup_until && now >= g_popup_until) { g_popup_until = 0; render(); }

  // cursor idle timeout: 8 s without a pitch gesture exits cursor mode
  if (g_cursor_mode && (g_screen == SCR_SCAN || g_screen == SCR_BLE) &&
      now - g_cursor_idle_ms > 8000UL) {
    g_cursor_mode = false; g_feed_manual = false;
    render();
  }

  static unsigned long last_idle = 0;
  if (now - last_idle > 1000) { last_idle = now; render(); }

  static unsigned long last_detail = 0;
  if (g_detail && now - last_detail > 100) { last_detail = now; render(); }   // 10 Hz finder

  {
    static unsigned long s_bri_t = 0;
    if (now - s_bri_t >= 16) {
      s_bri_t = now;
      bool awake  = g_usb_cache || (now - g_last_motion) < 5000UL;
      int  target = awake ? BRI_HI : BRI_LO;
      if (!g_usb_cache && g_mv_cache > 0 && g_mv_cache < 3300 && target > 40) target = 40;
      float rate = (target > g_bri) ? 5.0f : 2.0f;
      if      (g_bri < target) { g_bri += rate; if (g_bri > target) g_bri = target; }
      else if (g_bri > target) { g_bri -= rate; if (g_bri < target) g_bri = target; }
      M5.Display.setBrightness((uint8_t)(g_bri + 0.5f));
    }
  }

  delay(1);
}
