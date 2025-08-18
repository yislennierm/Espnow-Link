#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// ----------------- Compile-time config -----------------
#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 6
#endif
#ifndef LED_PIN
#define LED_PIN 2
#endif
#ifndef UART_IN_NUM
#define UART_IN_NUM 1
#endif
#ifndef UART_OUT_NUM
#define UART_OUT_NUM 2
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
#ifndef TX_LINK_TIMEOUT_MS
#define TX_LINK_TIMEOUT_MS 150
#endif
#ifndef TX_FAIL_STREAK_DOWN
#define TX_FAIL_STREAK_DOWN 5
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

#if defined(PROTO_GHST)      // Ghost: 420k 8N1, non-inverted
static constexpr uint32_t PROTO_BAUD = 420000;
static constexpr uint32_t PROTO_CFG  = SERIAL_8N1;
static constexpr bool RX_INVERT = false;
static constexpr bool TX_INVERT = false;
static constexpr uint8_t PROTO_ID = 3;
#elif defined(PROTO_CRSF)    // CRSF: 420k 8N1, non-inverted
static constexpr uint32_t PROTO_BAUD = 420000;
static constexpr uint32_t PROTO_CFG  = SERIAL_8N1;
static constexpr bool RX_INVERT = false;
static constexpr bool TX_INVERT = false;
static constexpr uint8_t PROTO_ID = 1;
#elif defined(PROTO_SBUS)    // SBUS: 100k 8E2, inverted
static constexpr uint32_t PROTO_BAUD = 100000;
static constexpr uint32_t PROTO_CFG  = SERIAL_8E2;
static constexpr bool RX_INVERT = true;
static constexpr bool TX_INVERT = true;
static constexpr uint8_t PROTO_ID = 2;
#endif

// ----------------- Peer MAC for TX build -----------------
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
#pragma pack(push,1)
struct RadioHdr {
  uint32_t seq;
  uint32_t tx_micros;
  uint16_t len;    // bytes in 'payload'
  uint8_t  proto;  // 1=CRSF, 2=SBUS, 3=GHST
};
#pragma pack(pop)

static_assert(MAX_PAYLOAD <= 200, "Keep MAX_PAYLOAD <= 200");

// ----------------- Globals -----------------
HardwareSerial UartIn(UART_IN_NUM);
HardwareSerial UartOut(UART_OUT_NUM);

static volatile uint32_t g_lastSeq = 0;
static volatile uint32_t g_pktsOk = 0, g_pktsCrc = 0, g_pktsDup = 0, g_pktsToolong = 0;
static volatile uint32_t g_lastRxMs = 0;
static uint32_t g_lastPrintMs = 0;

// Link debug
static bool g_rxLinkUp = false;
static bool g_txLinkUp = false;
static uint32_t g_txLastOkMs = 0;
static uint32_t g_txLastSendMs = 0;
static uint32_t g_txFailStreak = 0;

#if defined(ROLE_TX)
static uint8_t g_peer[6] = {0};   // filled from PEER* defines
#endif

// ----------------- CRC16-CCITT -----------------
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ----------------- Wi-Fi helpers -----------------
static void lock_wifi_channel(uint8_t ch) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);     // drop any AP state
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}
static void print_mac(const uint8_t mac[6], const char* label) {
  Serial.printf("%s %02X:%02X:%02X:%02X:%02X:%02X\n", label,
    mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

// ----------------- ESP-NOW callbacks -----------------
#if defined(ROLE_RX)
static void on_recv_cb(const uint8_t* /*mac*/, const uint8_t* data, int len) {
  if (len < (int)(sizeof(RadioHdr) + 2)) return;

  const RadioHdr* hdr = (const RadioHdr*)data;
  const uint8_t* payload = data + sizeof(RadioHdr);
  const int plen = hdr->len;

  if ((int)sizeof(RadioHdr) + plen + 2 != len) { g_pktsToolong++; return; }
  if (plen <= 0 || plen > (int)MAX_PAYLOAD)     { g_pktsToolong++; return; }
  if (hdr->proto != PROTO_ID)                   { return; } // other protocol

  uint16_t expect; memcpy(&expect, data + sizeof(RadioHdr) + plen, 2);
  uint16_t calc = crc16_ccitt(data, sizeof(RadioHdr) + plen);
  if (calc != expect) { g_pktsCrc++; return; }

  if (hdr->seq <= g_lastSeq) { g_pktsDup++; return; }
  g_lastSeq = hdr->seq;

  UartOut.write(payload, plen);
  g_pktsOk++;
  g_lastRxMs = millis();

  if (!g_rxLinkUp) {
    g_rxLinkUp = true;
    Serial.printf("[RX] LINK UP (seq=%lu)\n", (unsigned long)hdr->seq);
  }
}
#endif

#if defined(ROLE_TX)
static void on_send_cb(const uint8_t* /*mac*/, esp_now_send_status_t status) {
  g_txLastSendMs = millis();
  if (status == ESP_NOW_SEND_SUCCESS) {
    g_txFailStreak = 0;
    if (!g_txLinkUp) {
      g_txLinkUp = true;
      Serial.println("[TX] LINK UP");
    }
    g_txLastOkMs = g_txLastSendMs;
  } else {
    g_txFailStreak++;
    if (g_txLinkUp && g_txFailStreak >= TX_FAIL_STREAK_DOWN) {
      g_txLinkUp = false;
      Serial.println("[TX] LINK DOWN (consecutive send fails)");
    }
  }
}
#endif

// ----------------- Setup -----------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);               // USB console only
  delay(100);
  Serial.println("\nESP-NOW Serial Tunnel (GHST-framed)");

  lock_wifi_channel(ESPNOW_CHANNEL);

  uint8_t self[6];
  esp_wifi_get_mac(WIFI_IF_STA, self);
  print_mac(self, "[Self MAC]");

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED");
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }

#if defined(ROLE_TX)
  {
    esp_now_peer_info_t peer{};
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;
    peer.peer_addr[0] = (uint8_t)PEER0;
    peer.peer_addr[1] = (uint8_t)PEER1;
    peer.peer_addr[2] = (uint8_t)PEER2;
    peer.peer_addr[3] = (uint8_t)PEER3;
    peer.peer_addr[4] = (uint8_t)PEER4;
    peer.peer_addr[5] = (uint8_t)PEER5;

    bool allZero = true;
    for (int i = 0; i < 6; ++i) if (peer.peer_addr[i]) { allZero = false; break; }
    if (allZero) {
      Serial.println("** ERROR: PEER0..5 not set to RX MAC! **");
    }

    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("esp_now_add_peer FAILED");
    } else {
      memcpy(g_peer, peer.peer_addr, 6);
      Serial.print("[Peer] ");
      print_mac(g_peer, "TX->RX");
    }

    esp_now_register_send_cb(on_send_cb);
    Serial.println("[ROLE] TX");
  }
#elif defined(ROLE_RX)
  {
    esp_now_register_recv_cb(on_recv_cb);
    Serial.println("[ROLE] RX");
  }
#endif

#if defined(ROLE_TX)
  UartIn.begin(PROTO_BAUD, PROTO_CFG, RX_PIN, -1, RX_INVERT);
#else
  UartOut.begin(PROTO_BAUD, PROTO_CFG, -1, TX_PIN, TX_INVERT);
#endif

  Serial.printf("[PROTO] %s  baud=%u  cfg=0x%08lx  invert(rx=%d tx=%d)\n",
#if defined(PROTO_GHST)
                "GHST",
#elif defined(PROTO_CRSF)
                "CRSF",
#else
                "SBUS",
#endif
                PROTO_BAUD, (unsigned long)PROTO_CFG, (int)RX_INVERT, (int)TX_INVERT);
  Serial.printf("[WiFi] Locked to channel %d (PS off)\n", ESPNOW_CHANNEL);
  Serial.flush();
}

// ----------------- TX helpers: GHST frame parser -----------------
#if defined(ROLE_TX) && defined(PROTO_GHST)
// GHST frame = 0x3B, TYPE, LEN, then LEN bytes (payload+CRC8) → total = 3 + LEN
static const uint8_t GHST_SYNC = 0x3B;
static const uint8_t GHST_MAX_LEN = 48;

static bool ghst_get_frame(HardwareSerial& s, uint8_t* out, int& outLen) {
  static uint8_t buf[3 + GHST_MAX_LEN];
  static int have = 0;

  while (s.available() && have < (int)sizeof(buf)) {
    buf[have++] = (uint8_t)s.read();
  }
  if (have < 3) { outLen = 0; return false; }

  int start = -1;
  for (int i = 0; i < have; ++i) { if (buf[i] == GHST_SYNC) { start = i; break; } }
  if (start < 0) { have = 0; outLen = 0; return false; }
  if (start > 0) { memmove(buf, buf + start, have - start); have -= start; }
  if (have < 3) { outLen = 0; return false; }

  uint8_t len = buf[2];
  if (len == 0 || len > GHST_MAX_LEN) {
    // drop sync and rescan
    memmove(buf, buf + 1, have - 1);
    have -= 1;
    outLen = 0; return false;
  }

  int total = 3 + len;
  if (have < total) { outLen = 0; return false; }

  memcpy(out, buf, total);
  outLen = total;

  memmove(buf, buf + total, have - total);
  have -= total;
  return true;
}
#endif

// ----------------- Loop -----------------
void loop() {
#if defined(ROLE_TX)
  static uint32_t seq = 0;
  static uint8_t pkt[sizeof(RadioHdr) + MAX_PAYLOAD + 2];

  int ghLen = 0;
#if defined(PROTO_GHST)
  static uint8_t ghstFrame[MAX_PAYLOAD];
  while (ghst_get_frame(UartIn, ghstFrame, ghLen)) {
    if (ghLen <= 0 || ghLen > (int)MAX_PAYLOAD) continue;

    RadioHdr hdr{};
    hdr.seq = seq++;
    hdr.tx_micros = micros();
    hdr.len = (uint16_t)ghLen;
    hdr.proto = PROTO_ID;

    memcpy(pkt, &hdr, sizeof(RadioHdr));
    memcpy(pkt + sizeof(RadioHdr), ghstFrame, ghLen);

    uint16_t crc = crc16_ccitt(pkt, sizeof(RadioHdr) + ghLen);
    memcpy(pkt + sizeof(RadioHdr) + ghLen, &crc, 2);

    // ---- UNICAST send (more reliable than broadcast) ----
    esp_now_send(g_peer, pkt, sizeof(RadioHdr) + ghLen + 2);
#if DUP_SEND
    esp_now_send(g_peer, pkt, sizeof(RadioHdr) + ghLen + 2);
#endif
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
#else
  // generic tunneling fallback (unused in GHST build)
  int avail = UartIn.available();
  if (avail > 0) {
    int toRead = min(avail, (int)MAX_PAYLOAD);
    RadioHdr hdr{};
    hdr.seq = seq++;
    hdr.tx_micros = micros();
    hdr.len = (uint16_t)toRead;
    hdr.proto = PROTO_ID;

    memcpy(pkt, &hdr, sizeof(RadioHdr));
    int got = UartIn.readBytes(pkt + sizeof(RadioHdr), toRead);
    uint16_t crc = crc16_ccitt(pkt, sizeof(RadioHdr) + got);
    memcpy(pkt + sizeof(RadioHdr) + got, &crc, 2);
    esp_now_send(g_peer, pkt, sizeof(RadioHdr) + got + 2);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
#endif

  // Link timeout
  uint32_t now = millis();
  if (g_txLinkUp && (now - g_txLastOkMs) > TX_LINK_TIMEOUT_MS) {
    g_txLinkUp = false;
    Serial.println("[TX] LINK DOWN (timeout)");
  }

  if (now - g_lastPrintMs >= PRINT_STATS_MS) {
    g_lastPrintMs = now;
    Serial.printf("[TX] lastOk=%lu ms ago  failStreak=%lu  link=%s\n",
      (unsigned long)(now - g_txLastOkMs),
      (unsigned long)g_txFailStreak,
      g_txLinkUp ? "YES" : "NO");
  }

#elif defined(ROLE_RX)
  uint32_t now = millis();
  bool link = (now - g_lastRxMs) < FAILSAFE_MS;
  digitalWrite(LED_PIN, link ? HIGH : LOW);

  if (g_rxLinkUp && !link) { g_rxLinkUp = false; Serial.println("[RX] LINK DOWN (timeout)"); }

  if (now - g_lastPrintMs >= PRINT_STATS_MS) {
    g_lastPrintMs = now;
    Serial.printf("ok=%lu crc=%lu dup=%lu toolong=%lu lastSeq=%lu link=%s\n",
      (unsigned long)g_pktsOk, (unsigned long)g_pktsCrc,
      (unsigned long)g_pktsDup, (unsigned long)g_pktsToolong,
      (unsigned long)g_lastSeq, link ? "YES" : "NO");
  }
#endif
}
