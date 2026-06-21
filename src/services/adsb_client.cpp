#include "services/adsb_client.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <ArduinoJson.h>

#include <climits>
#include <cstring>

#include "config.h"

namespace services::adsb {

namespace {

// adsb.lol is an open ADSBexchange-v2 mirror; same JSON shape, no feeder gate.
constexpr char kApiBase[] = "https://api.adsb.lol/v2/lat/";
constexpr float kKmPerNm = 1.852f;
constexpr int kConnectAttemptMs = 200;
constexpr unsigned long kRequestTimeoutMs = 10000;
constexpr unsigned long kFetchIntervalMs  = 4000;

Aircraft               s_buf[2][kMaxAircraft];
size_t                 s_cnt[2]       = {0, 0};
volatile int           s_front        = 0;
volatile bool          s_have_center  = false;
double                 s_lat = 0, s_lon = 0;
float                  s_radius_km    = 0;
volatile unsigned long s_last_ok_ms   = 0;

void pollNetwork() {}   // no-op: fetch runs in its own task now

int performGetWithPoll(HTTPClient& http) {
  http.setConnectTimeout(kConnectAttemptMs);
  const unsigned long deadline = millis() + kRequestTimeoutMs;
  while (millis() < deadline) {
    pollNetwork();
    const int code = http.GET();
    if (code > 0) {
      return code;
    }
    if (code != HTTPC_ERROR_CONNECTION_REFUSED &&
        code != HTTPC_ERROR_NOT_CONNECTED) {
      return code;
    }
    delay(5);
  }
  return HTTPC_ERROR_READ_TIMEOUT;
}

float kmToNauticalMiles(float km) { return km / kKmPerNm; }

bool readJsonFloat(const JsonObject& obj, const char* key, float* out) {
  if (obj[key].is<float>() || obj[key].is<double>() || obj[key].is<int>()) {
    *out = obj[key].as<float>();
    return true;
  }
  return false;
}

float pickNoseHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickTrackHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickGroundSpeed(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "gs", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "tas", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "ias", &v)) {
    return v;
  }
  return 0.0f;
}

bool isOnGround(const JsonObject& plane) {
  if (!plane["alt_baro"].is<const char*>()) {
    return false;
  }
  return strcmp(plane["alt_baro"].as<const char*>(), "ground") == 0;
}

void copyJsonStringTrimmed(const JsonObject& obj, const char* key, char* out,
                           size_t out_len) {
  out[0] = '\0';
  if (out_len == 0 || !obj[key].is<const char*>()) {
    return;
  }
  const char* s = obj[key].as<const char*>();
  size_t n = strnlen(s, out_len - 1);
  while (n > 0 && s[n - 1] == ' ') {
    --n;
  }
  memcpy(out, s, n);
  out[n] = '\0';
}

void formatAltitudeTag(const JsonObject& plane, char* out, size_t out_len) {
  out[0] = '\0';
  if (out_len == 0) {
    return;
  }

  if (plane["alt_baro"].is<const char*>()) {
    const char* s = plane["alt_baro"].as<const char*>();
    if (strcmp(s, "ground") == 0) {
      strncpy(out, "GND", out_len - 1);
      out[out_len - 1] = '\0';
      return;
    }
  }

  float alt = 0.0f;
  if (readJsonFloat(plane, "alt_baro", &alt) ||
      readJsonFloat(plane, "alt_geom", &alt)) {
    snprintf(out, out_len, "%d ft", static_cast<int>(lroundf(alt)));
  }
}

void fillTagFields(Aircraft* ac, const JsonObject& plane) {
  copyJsonStringTrimmed(plane, "flight", ac->callsign, sizeof(ac->callsign));
  if (ac->callsign[0] == '\0') {
    copyJsonStringTrimmed(plane, "hex", ac->callsign, sizeof(ac->callsign));
  }

  copyJsonStringTrimmed(plane, "t", ac->type, sizeof(ac->type));
  formatAltitudeTag(plane, ac->alt, sizeof(ac->alt));
}

}  // namespace

size_t          aircraftCount() { return s_cnt[s_front]; }
const Aircraft* aircraftList()  { return s_buf[s_front]; }
unsigned long   lastOkMs()      { return s_last_ok_ms; }

void setCenter(double lat, double lon, float radius_km) {
  s_lat = lat; s_lon = lon; s_radius_km = radius_km; s_have_center = true;
}

bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km) {
  const float dist_nm = kmToNauticalMiles(fetch_radius_km);

  String url = kApiBase;
  url += String(center_lat, 6);
  url += "/lon/";
  url += String(center_lon, 6);
  url += "/dist/";
  url += String(dist_nm, 1);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  if (!http.begin(client, url)) {
    Serial.println("adsb: http.begin failed");
    return false;
  }
  http.setTimeout(kRequestTimeoutMs);

  Serial.printf("adsb: heap before=%u\n", ESP.getFreeHeap());
  const int code = performGetWithPoll(http);
  if (code != HTTP_CODE_OK) {
    Serial.printf("adsb: HTTP %d\n", code);
    http.end();
    return false;
  }

  // Only decode the fields this app uses — avoids storing the full JSON tree.
  JsonDocument filter;
  filter["ac"][0]["lat"]          = true;
  filter["ac"][0]["lon"]          = true;
  filter["ac"][0]["flight"]       = true;
  filter["ac"][0]["hex"]          = true;
  filter["ac"][0]["t"]            = true;
  filter["ac"][0]["alt_baro"]     = true;
  filter["ac"][0]["alt_geom"]     = true;
  filter["ac"][0]["gs"]           = true;
  filter["ac"][0]["tas"]          = true;
  filter["ac"][0]["ias"]          = true;
  filter["ac"][0]["track"]        = true;
  filter["ac"][0]["true_heading"] = true;
  filter["ac"][0]["mag_heading"]  = true;
  filter["ac"][0]["dir"]          = true;
  filter["ac"][0]["squawk"]       = true;
  filter["ac"][0]["emergency"]    = true;
  filter["ac"][0]["dbFlags"]      = true;

  // Parse directly from the HTTP stream — no intermediate String copy.
  WiFiClient* stream = http.getStreamPtr();
  JsonDocument doc;
  const DeserializationError err =
      deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();
  Serial.printf("adsb: post-parse heap=%u\n", ESP.getFreeHeap());

  if (err) {
    Serial.printf("adsb: JSON error: %s\n", err.c_str());
    return false;
  }

  // Write into the back buffer, publish atomically on success.
  const int back = 1 - s_front;
  Aircraft* fill = s_buf[back];
  size_t n = 0;

  JsonArray ac = doc["ac"].as<JsonArray>();
  if (!ac.isNull()) {
    for (JsonObject plane : ac) {
      if (n >= kMaxAircraft) break;
      if (!plane["lat"].is<float>() || !plane["lon"].is<float>()) continue;
      if (isOnGround(plane) && !config::kAdsbShowGroundAircraft) continue;

      fill[n].lat       = plane["lat"].as<float>();
      fill[n].lon       = plane["lon"].as<float>();
      fill[n].nose_deg  = pickNoseHeading(plane);
      fill[n].track_deg = pickTrackHeading(plane);
      fill[n].gs_knots  = pickGroundSpeed(plane);
      fillTagFields(&fill[n], plane);

      copyJsonStringTrimmed(plane, "squawk", fill[n].squawk, sizeof(fill[n].squawk));

      float altf = 0.0f;
      fill[n].alt_ft = (readJsonFloat(plane, "alt_baro", &altf) ||
                        readJsonFloat(plane, "alt_geom", &altf))
                       ? (int)lroundf(altf) : INT_MIN;

      float flags = 0.0f;
      fill[n].mil = readJsonFloat(plane, "dbFlags", &flags) && ((int)flags & 1);

      const char* sq = fill[n].squawk;
      bool emSquawk = sq[0] && (!strcmp(sq, "7500") || !strcmp(sq, "7600") || !strcmp(sq, "7700"));
      bool emField  = false;
      if (plane["emergency"].is<const char*>()) {
        const char* e = plane["emergency"].as<const char*>();
        emField = e && e[0] && strcmp(e, "none") != 0;
      }
      fill[n].emergency = emSquawk || emField;

      ++n;
    }
  }

  s_cnt[back] = n;
  s_front = back;          // atomic publish — UI now reads the new buffer
  s_last_ok_ms = millis();
  Serial.printf("adsb: %u aircraft\n", static_cast<unsigned>(n));
  return true;
}

static void adsbTask(void*) {
  for (;;) {
    if (s_have_center && WiFi.status() == WL_CONNECTED)
      fetchUpdate(s_lat, s_lon, s_radius_km);
    vTaskDelay(pdMS_TO_TICKS(kFetchIntervalMs));
  }
}

void begin() {
  xTaskCreatePinnedToCore(adsbTask, "adsb", 16384, nullptr, 1, nullptr, 0);
}

}  // namespace services::adsb
