#include "ghst.h"

namespace {
  // Small static buffer for frame assembly in get_frame()
  uint8_t s_buf[3 + ghst::MAX_LEN];
  int     s_have = 0;
}

namespace ghst {

bool get_frame(HardwareSerial& s, uint8_t* out, int& outLen)
{
  // Slurp bytes (bounded)
  while (s.available() && s_have < (int)sizeof(s_buf)) {
    s_buf[s_have++] = (uint8_t)s.read();
  }

  // Need at least sync/type/len
  if (s_have < 3) { outLen = 0; return false; }

  // Find sync
  int start = -1;
  for (int i = 0; i < s_have; ++i) {
    if (s_buf[i] == SYNC) { start = i; break; }
  }
  if (start < 0) { s_have = 0; outLen = 0; return false; }

  // Align to sync
  if (start > 0) {
    memmove(s_buf, s_buf + start, s_have - start);
    s_have -= start;
  }
  if (s_have < 3) { outLen = 0; return false; }

  // Length byte is at index 2
  uint8_t len = s_buf[2];
  if (len == 0 || len > MAX_LEN) {
    // bad LEN → drop this sync byte and rescan next loop
    memmove(s_buf, s_buf + 1, s_have - 1);
    s_have -= 1;
    outLen = 0;
    return false;
  }

  const int total = 3 + len;
  if (s_have < total) { outLen = 0; return false; }

  // Full frame ready
  memcpy(out, s_buf, total);
  outLen = total;

  // Consume from the ring
  memmove(s_buf, s_buf + total, s_have - total);
  s_have -= total;
  return true;
}

// Quick visual decoder for 8 channels (matches EdgeTX packing we observed)
void debug_decode_channels(const uint8_t* frame, size_t len)
{
  if (len < 15) return; // addr + len + type + payload + crc

  const uint8_t* payload = frame + 3; // skip addr,len,type
  int ch[8];

  // 4x 12-bit packed (LSB-first)
  uint32_t bits = 0;
  uint8_t  avail = 0;
  int      outi = 0;

  for (int i = 0; i < 4; i++) {
    while (avail < 12) {
      bits |= ((uint32_t)(*payload++)) << avail;
      avail += 8;
    }
    ch[outi++] = bits & 0x0FFF; // 0..4095
    bits   >>= 12;
    avail  -= 12;
  }

  // 4x 8-bit
  for (int i = 4; i < 8; i++) ch[i] = *payload++;

  // Pretty print (center first 4 to ±2048; scale last 4 to approximate ±1024)
  Serial.print("[GHST] ");
  for (int i = 0; i < 8; i++) {
    int val = (i < 4) ? (ch[i] - 2048) : ((ch[i] - 128) * 8);
    Serial.printf("CH%02d=%5d  ", i + 1, val);
  }
  Serial.println();
}

} // namespace ghst
