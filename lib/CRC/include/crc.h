#pragma once
#include <Arduino.h>

// ----------------- Packet format -----------------
#pragma pack(push, 1)
struct RadioHdr {
    uint32_t seq;
    uint32_t tx_micros;
    uint16_t len;   // bytes in 'payload'
    uint8_t  proto; // 1=CRSF, 2=SBUS, 3=GHST
};
#pragma pack(pop)

// CRC16-CCITT helper
uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF);
