#include <Arduino.h>

// --------- Platform -------

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// ------------- Application ---------

#include "status.h"
#include "link.h"
#include "ghst.h"

// ----------------- Compile-time config -----------------
#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif
#ifndef LED_PIN
#define LED_PIN 2
#endif
#ifndef UART_IN_NUM
#define UART_IN_NUM 1
#endif
#ifndef UART_OUT_NUM
#define UART_OUT_NUM 1
#endif
#ifndef RX_PIN
#define RX_PIN 16
#endif
#ifndef TX_PIN
#define TX_PIN 17
#endif
#ifndef MAX_PAYLOAD
#define MAX_PAYLOAD 64
#endif
#ifndef DUP_SEND
#define DUP_SEND 0
#endif
#ifndef FAILSAFE_MS
#define FAILSAFE_MS 120
#endif
#ifndef PRINT_STATS_MS
#define PRINT_STATS_MS 1000
#endif

// ----------------- Role selection -----------------
#if !defined(ROLE_TX) && !defined(ROLE_RX)
#warning "No ROLE_* defined; defaulting to RX"
#define ROLE_RX
#endif

// ----------------- Protocol selection -----------------
#if !defined(PROTO_GHST) && !defined(PROTO_CRSF) && !defined(PROTO_SBUS)
#warning "No PROTO_* defined; defaulting to GHST"
#define PROTO_GHST
#endif

#if defined(PROTO_GHST) // Ghost: 420k 8N1, inverted RX
static constexpr uint32_t PROTO_BAUD = 420000;
static constexpr uint32_t PROTO_CFG = SERIAL_8N1;
static constexpr bool RX_INVERT = true;
static constexpr bool TX_INVERT = false;
static constexpr uint8_t PROTO_ID = 3;
#elif defined(PROTO_CRSF) // CRSF: 420k 8N1, non-inverted
static constexpr uint32_t PROTO_BAUD = 420000;
static constexpr uint32_t PROTO_CFG = SERIAL_8N1;
static constexpr bool RX_INVERT = false;
static constexpr bool TX_INVERT = false;
static constexpr uint8_t PROTO_ID = 1;
#elif defined(PROTO_SBUS) // SBUS: 100k 8E2, inverted
static constexpr uint32_t PROTO_BAUD = 100000;
static constexpr uint32_t PROTO_CFG = SERIAL_8E2;
static constexpr bool RX_INVERT = true;
static constexpr bool TX_INVERT = true;
static constexpr uint8_t PROTO_ID = 2;
#endif

// ----------------- Peer MAC (TX build) -----------------
#ifndef PEER0
#define PEER0 0x00
#endif
#ifndef PEER1
#define PEER1 0x00
#endif
#ifndef PEER2
#define PEER2 0x00
#endif
#ifndef PEER3
#define PEER3 0x00
#endif
#ifndef PEER4
#define PEER4 0x00
#endif
#ifndef PEER5
#define PEER5 0x00
#endif

// ----------------- Packet format -----------------
#pragma pack(push, 1)
struct RadioHdr
{
  uint32_t seq;
  uint32_t tx_micros;
  uint16_t len;  // bytes in 'payload'
  uint8_t proto; // 1=CRSF, 2=SBUS, 3=GHST
};
#pragma pack(pop)

// ----------------- Globals -----------------

// ----------------- Globals -----------------
HardwareSerial UartIn(UART_IN_NUM);
HardwareSerial UartOut(UART_OUT_NUM);

static volatile uint32_t g_lastSeq = 0;
static volatile uint32_t g_pktsOk = 0;
static volatile uint32_t g_pktsCrc = 0;
static volatile uint32_t g_pktsDup = 0;
static volatile uint32_t g_pktsToolong = 0;
static volatile uint32_t g_lastRxMs = 0;
static uint32_t g_lastPrintMs = 0;

// Link debug
static bool g_rxLinkUp = false;
static bool g_txLinkUp = false;
static uint32_t g_txLastOkMs = 0;
static uint32_t g_txLastSendMs = 0;
static uint32_t g_txFailStreak = 0;

#if defined(ROLE_TX)
static uint32_t g_seq = 0;
#endif

// ----------------- CRC16-CCITT -----------------
static uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF)
{
  for (size_t i = 0; i < len; ++i)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b)
    {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ----------------- RX GHST Decoder -----------------
#if defined(ROLE_RX) && defined(PROTO_GHST)
static void decodeGhostChannels(const uint8_t *frame, size_t len)
{
  if (len < 15)
    return;

  const uint8_t *payload = frame + 3; // skip addr,len,type
  int channels[8];

  // --- First 4 channels (12-bit packed) ---
  uint32_t bits = 0;
  uint8_t bitsavail = 0;
  int outch = 0;
  for (int i = 0; i < 4; i++)
  {
    while (bitsavail < 12)
    {
      bits |= ((uint32_t)(*payload++)) << bitsavail;
      bitsavail += 8;
    }
    channels[outch++] = bits & 0xFFF;
    bits >>= 12;
    bitsavail -= 12;
  }

  // --- Next 4 channels (8-bit raw) ---
  for (int i = 4; i < 8; i++)
  {
    channels[i] = *payload++;
  }

  Serial.print("[GHST] Channels: ");
  for (int i = 0; i < 8; i++)
  {
    int val = (i < 4) ? (channels[i] - 2048) : ((channels[i] - 128) * 8);
    Serial.printf("CH%d=%d ", i + 1, val);
  }
  Serial.println();
}
#endif

// ----------------- TX: GHST frame parser -----------------
#if defined(ROLE_TX) && defined(PROTO_GHST)
static const uint8_t GHST_SYNC = 0x3B;
static const uint8_t GHST_MAX_LEN = 48;

static bool ghst_get_frame(HardwareSerial &s, uint8_t *out, int &outLen)
{
  static uint8_t buf[3 + GHST_MAX_LEN];
  static int have = 0;

  while (s.available() && have < (int)sizeof(buf))
  {
    buf[have++] = (uint8_t)s.read();
  }
  if (have < 3)
  {
    outLen = 0;
    return false;
  }

  int start = -1;
  for (int i = 0; i < have; ++i)
  {
    if (buf[i] == GHST_SYNC)
    {
      start = i;
      break;
    }
  }
  if (start < 0)
  {
    have = 0;
    outLen = 0;
    return false;
  }
  if (start > 0)
  {
    memmove(buf, buf + start, have - start);
    have -= start;
  }
  if (have < 3)
  {
    outLen = 0;
    return false;
  }

  uint8_t len = buf[2];
  if (len == 0 || len > GHST_MAX_LEN)
  {
    memmove(buf, buf + 1, have - 1);
    have -= 1;
    outLen = 0;
    return false;
  }

  int total = 3 + len;
  if (have < total)
  {
    outLen = 0;
    return false;
  }

  memcpy(out, buf, total);
  outLen = total;

  memmove(buf, buf + total, have - total);
  have -= total;
  return true;
}
#endif

// ----------------- Application RX callback -----------------
#if defined(ROLE_RX)
// correct signature
static void app_rx_cb(const uint8_t *mac, const uint8_t *data, int len)
{
  if (len < (int)(sizeof(RadioHdr) + 2))
    return;

  const RadioHdr *hdr = (const RadioHdr *)data;
  const uint8_t *payload = data + sizeof(RadioHdr);
  const int plen = hdr->len;

  if ((int)sizeof(RadioHdr) + plen + 2 != len)
  {
    g_pktsToolong++;
    return;
  }
  if (plen <= 0 || plen > (int)MAX_PAYLOAD)
  {
    g_pktsToolong++;
    return;
  }
  if (hdr->proto != PROTO_ID)
  {
    return;
  } // other protocol

  uint16_t expect;
  memcpy(&expect, data + sizeof(RadioHdr) + plen, 2);
  uint16_t calc = crc16_ccitt(data, sizeof(RadioHdr) + plen);
  if (calc != expect)
  {
    g_pktsCrc++;
    return;
  }

  if (hdr->seq <= g_lastSeq)
  {
    g_pktsDup++;
    return;
  }
  g_lastSeq = hdr->seq;

  UartOut.write(payload, plen);
  g_pktsOk++;
  g_lastRxMs = millis();

  if (!g_rxLinkUp)
  {
    g_rxLinkUp = true;
    Serial.printf("[RX] LINK UP (seq=%lu)\n", (unsigned long)hdr->seq);
  }
}

#endif

// ----------------- Setup -----------------
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  led_init(LED_PIN);

  Serial.begin(115200);
  delay(100);
  Serial.println("\nESP-NOW Serial Tunnel (cleaned)");

  led_set_state(LED_BLINK_FAST);

  if (!wifi_init(ESPNOW_CHANNEL))
  {
    Serial.println("[ERR] wifi_init failed");
    while (1)
    {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }

#if defined(ROLE_TX)
  uint8_t peer[6] = {PEER0, PEER1, PEER2, PEER3, PEER4, PEER5};
  wifi_set_peer(peer);
  UartIn.begin(PROTO_BAUD, PROTO_CFG, RX_PIN, -1, RX_INVERT);
  Serial.println("[ROLE] TX");
#elif defined(ROLE_RX)
  wifi_register_app_rx(app_rx_cb);
  UartOut.begin(PROTO_BAUD, PROTO_CFG, -1, TX_PIN, TX_INVERT);
  Serial.println("[ROLE] RX");
#endif

  Serial.printf("[PROTO] ID=%u baud=%u invert(rx=%d tx=%d)\n",
                PROTO_ID, PROTO_BAUD, RX_INVERT, TX_INVERT);
}

// ----------------- Loop -----------------
void loop()
{
  uint32_t now = millis();

#if defined(ROLE_TX)
  static uint8_t pkt[sizeof(RadioHdr) + MAX_PAYLOAD + 2];
  static uint8_t ghstFrame[MAX_PAYLOAD];
  int ghLen = 0;

  while (ghst_get_frame(UartIn, ghstFrame, ghLen))
  {
    if (ghLen <= 0 || ghLen > (int)MAX_PAYLOAD)
      continue;

    RadioHdr hdr{};
    hdr.seq = g_seq++;
    hdr.tx_micros = micros();
    hdr.len = (uint16_t)ghLen;
    hdr.proto = PROTO_ID;

    memcpy(pkt, &hdr, sizeof(RadioHdr));
    memcpy(pkt + sizeof(RadioHdr), ghstFrame, ghLen);
    uint16_t crc = crc16_ccitt(pkt, sizeof(RadioHdr) + ghLen);
    memcpy(pkt + sizeof(RadioHdr) + ghLen, &crc, 2);

    wifi_send(pkt, sizeof(RadioHdr) + ghLen + 2);
#if DUP_SEND
    wifi_send(pkt, sizeof(RadioHdr) + ghLen + 2);
#endif
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
#endif

  if (now - g_lastPrintMs >= PRINT_STATS_MS)
  {
    g_lastPrintMs = now;
    wifi_debug();
  }
}
