#pragma once

// ---------------------------------------------------------------------------
// config.h — Aware
// Split into two namespaces:
//   config::   — consumed by the lifted adsb_client.cpp (do not rename)
//   appcfg::   — our own app settings
// ---------------------------------------------------------------------------

namespace config {
// Required by the lifted adsb_client.cpp. Hide aircraft reporting "ground".
constexpr bool kAdsbShowGroundAircraft = false;
}  // namespace config

namespace appcfg {

// --- WiFi -------------------------------------------------------------------
// Credentials are entered on-device via the SETUP screen (BtnB) and stored
// in NVS. These are empty fallbacks only — do NOT commit real credentials.
constexpr char kWifiSsid[] = "";
constexpr char kWifiPass[] = "";

// --- Home location ----------------------------------------------------------
// Radar is centered here. StickS3 has no GPS, so this is fixed unless you add
// a Grove GPS unit later. Default: Hillsborough, NC.
constexpr double kHomeLat = 36.0746;
constexpr double kHomeLon = -79.0997;

// --- Radar ranges -----------------------------------------------------------
// Outer-ring distance in km. Side button (BtnB) cycles through these.
constexpr float kRangePresetsKm[] = {5.0f, 10.0f, 15.0f, 25.0f};
constexpr int   kRangePresetCount = 4;
constexpr int   kRangeDefaultIdx  = 1;  // 10 km

// How often to refetch ADS-B (ms). adsb.fi is a free community feed; be kind.
constexpr unsigned long kFetchIntervalMs = 25000;

// --- Audio ------------------------------------------------------------------
// Onboard-speaker master volume, 0–255. The onboard amp is small (~1 W), so
// this is the whole range you have on the built-in speaker.
//   200 ≈ 78% — loud, safe on battery.
//   255 = max — loudest, but a sustained loud tone can brown-out / reboot the
//         Stick on battery (fine on USB). Bump to 255 if you mostly run on USB.
constexpr uint8_t kSpeakerVolume = 169;

}  // namespace appcfg
