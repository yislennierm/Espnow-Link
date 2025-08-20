#pragma once
#include <Arduino.h>   // <-- gives uint8_t, uint32_t, SERIAL_8N1

namespace ghst {
    constexpr uint32_t GHST_UL_RC_CHANS_HS4_12_5TO8 = 0x16;
    constexpr uint32_t GHST_UL_RC_CHANS_HS4_5TO8    = 0x17;

    constexpr uint32_t BAUD = 420000;
    constexpr uint32_t CFG  = SERIAL_8N1;

    constexpr bool TX_INVERT = true;
    constexpr bool RX_INVERT = true;

    bool isCompleteFrame(const uint8_t *buf, int len);
    void debug_decode_channels(const uint8_t *frame, int len);
    void debug_dump_frame(const uint8_t *frame, int len);
}

