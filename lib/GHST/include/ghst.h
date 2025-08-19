#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>


#define MAX_PAYLOAD    64
#define DUP_SEND       0
#define TX_LINK_TIMEOUT_MS   150
#define TX_FAIL_STREAK_DOWN  5

namespace ghst {

// Protocol constants
static constexpr uint8_t  SYNC       = 0x3B;
static constexpr uint8_t  MAX_LEN    = 48;   // LEN byte upper bound (payload+CRC8)
static constexpr uint32_t BAUD       = 420000;
static constexpr uint32_t CFG        = SERIAL_8N1;
static constexpr bool     RX_INVERT  = true;   // TX16S GHST on S.Port is inverted to us
static constexpr bool     TX_INVERT  = false;
static constexpr uint8_t  PROTO_ID   = 3;      // our app header value

// Pull ONE complete GHST frame from a HardwareSerial.
// Returns true if a full frame (3+LEN bytes) was copied to 'out' (len in outLen).
bool get_frame(HardwareSerial& s, uint8_t* out, int& outLen);

// Best-effort channel decode (first 4 ch = 12-bit packed, next 4 = 8-bit).
// Prints a readable snapshot to Serial; safe to call with any GHST frame.
// No return — this is for debug/visibility.
void debug_decode_channels(const uint8_t* frame, size_t len);

} // namespace ghst
