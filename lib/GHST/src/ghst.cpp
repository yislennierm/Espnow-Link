#include <Arduino.h>
#include "ghst.h"

namespace ghst {

static HardwareSerial *ghstSerial = nullptr;
static uint16_t channels[16] = {2048};
static unsigned long lastDebug = 0;

// Frame IDs
constexpr uint8_t GHST_ADDR_TX = 0x80;
constexpr uint8_t GHST_FRAME_TYPE_CHANNEL  = 0x3B;
constexpr uint8_t GHST_FRAME_TYPE_CHANNEL16 = 0x3C;


// Very simple check: GHST frames have a length byte at index 1
bool isCompleteFrame(const uint8_t *buf, int len) {
    if (len < 2) return false;         // too short to contain header
    uint8_t frameLen = buf[1] + 2;     // length byte + header + CRC
    return (len >= frameLen);
}

void init(HardwareSerial &serial) {
    ghstSerial = &serial;
    serial.begin(BAUD, CFG);
}

static int buildChannelsFrame8(uint8_t *buf, int maxlen) {
    if (maxlen < 14) return 0;
    buf[0] = GHST_ADDR_TX;
    buf[1] = 12;
    buf[2] = GHST_FRAME_TYPE_CHANNEL;
    for (int i=0; i<8; i++) {
        uint16_t val = channels[i] & 0x0FFF;
        buf[3 + i*2] = val & 0xFF;
        buf[4 + i*2] = (val >> 8) & 0xFF;
    }
    return 3 + 16;
}






static int buildChannelsFrame16(uint8_t *buf, int maxlen) {
    if (maxlen < 30) return 0;
    buf[0] = GHST_ADDR_TX;
    buf[1] = 28;
    buf[2] = GHST_FRAME_TYPE_CHANNEL16;
    for (int i=0; i<16; i++) {
        uint16_t val = channels[i] & 0x0FFF;
        buf[3 + i*2] = val & 0xFF;
        buf[4 + i*2] = (val >> 8) & 0xFF;
    }
    return 3 + 32;
}

int buildChannelsFrame(uint8_t *buf, int maxLen, int numCh) {
    if (numCh <= 8) return buildChannelsFrame8(buf, maxLen);
    else return buildChannelsFrame16(buf, maxLen);
}

void setChannel(int idx, uint16_t value) {
    if (idx < 0 || idx >= 16) return;
    channels[idx] = value;
}
void debug_dump_frame(const uint8_t *frame, int len) {
    if (!frame) return;
    Serial.printf("[GHST RAW] len=%d: ", len);
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", frame[i]);
    }
    Serial.println();
}

void debug_decode_channels(const uint8_t *frame, int len) {
    if (len < 2) return;
    uint8_t type = frame[2];   // frame ID
    const uint8_t *payload = frame + 3;
    
    if (type == GHST_UL_RC_CHANS_HS4_12_5TO8 || type == GHST_UL_RC_CHANS_HS4_5TO8) {
        // decode first 4x12bit packed
        uint32_t bits = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
        for (int i=0; i<4; i++) {
            int ch = bits & 0xFFF;
            bits >>= 12;
            Serial.printf("CH%d=%d ", i+1, ch);
        }
        // then decode next 4x8bit
        for (int i=0; i<4; i++) {
            Serial.printf("CH%d=%d ", i+5, payload[4+i]);
        }
        Serial.println();
    }
}





void debugTxChannels(int numCh) {
    unsigned long now = millis();
    if (now - lastDebug < 200) return;
    lastDebug = now;

    Serial.print("[GHST] TX ");
    for (int i=0; i<numCh; i++) {
        Serial.printf("CH%d=%d ", i+1, channels[i]);
    }
    Serial.println();
}

} // namespace ghst
