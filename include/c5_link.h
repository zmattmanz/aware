#pragma once
#include <cstdint>

// Receives the ESP32-C5 WiFi Remote ID sniffer's UART stream (see PROTOCOL.md)
// and merges decoded drones into the shared drone table (drone_scan).
namespace c5link {
void          begin();        // open the UART link from the C5
void          poll();         // call every loop(): drains UART, parses, feeds table
bool          linked();       // true if a C5 heartbeat arrived within the last 8 s
unsigned long lastByteMs();   // millis() of last byte received (0 = never) — diagnostics
}  // namespace c5link
