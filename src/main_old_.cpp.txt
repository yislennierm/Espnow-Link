#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// = Compile-time config =====
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

// Role and protocol selection
#if !defined(ROLE_TX) && !defined(ROLE_RX)
#warning "No ROLE_* defined; defaulting to RX"
#define ROLE_RX
#endif

#if !defined(PROTO_CRSF) && !defined(PROTO_SBUS)
#warning "No PROTO_* defined; defaulting to CRSF"
#define PROTO_CRSF
#endif

// UART configs by protocol
#if defined(PROTO_CRSF)
static constexpr uint32_t PROTO_BAUD = 420000;
static constexpr uint32_t PROTO_CFG  = SERIAL_8N1;
static constexpr bool RX_INVERT = false;
static constexpr bool TX_INVERT = false;
static constexpr uint8_t PROTO_ID = 1;
#elif defined(PROTO_SBUS)
static constexpr uint32_t PROTO_BAUD = 100000;
static constexpr uint32_t PROTO_CFG  = SERIAL_8E2;
static constexpr bool RX_INVERT = true;   // SBUS inverted input
static constexpr bool TX_INVERT = true;   // SBUS inverted output
static constexpr uint8_t PROTO_ID = 2;
#endif

// Peer MAC for TX. Provide these -DPEERn flags on TX builds.
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

// ===== Packet format =====
#pragma pack(push,1)
struct RadioHdr {
  uint32_t seq;
  uint32_t tx_micros;
  uint16_t len;
  uint8_t  proto; // 1=CRSF, 2=SBUS
};
#pragma pack(pop)

static_assert(MAX_PAYLOAD <= 200, "Keep ESP-NOW payloads small (<=200 bytes).");

// ===== Globals =====
HardwareSerial UartIn(UART_IN_NUM);
HardwareSerial UartOut(UART_OUT_NUM);

static volatile uint32_t g_lastSeq = 0;
static volatile uint32_t g_pktsOk = 0, g_pktsCrc = 0, g_pktsDup = 0, g_pktsToolong = 0;
static volatile uint32_t g_lastRxMs = 0;
static uint32_t g_lastPrintMs = 0;

// ----- Link debug -----
static bool g_rxLinkUp = false;      // RX: last-known link state
static bool g_txLinkUp = false;      // TX: last-known link state
static uint32_t g_txLastOkMs = 0;    // TX: last successful delivery time
static uint32_t g_txLastSendMs = 0;  // TX: last time we attempted a send
static uint32_t g_txFailStreak = 0;  // TX: consecutive send failures

// ===== CRC16-CCITT (0x1021, init 0xFFFF) =====
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

// ===== Wi-Fi / ESP-NOW helpers =====
static void lock_wifi_channel(uint8_t ch) {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

static void print_mac(const uint8_t mac[6], const char* label) {
  Serial.printf("%s %02X:%02X:%02X:%02X:%02X:%02X\n", label,
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

#if defined(ROLE_RX)
static void on_recv_cb(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < (int)(sizeof(RadioHdr) + 2)) { return; }

  const RadioHdr* hdr = (const RadioHdr*)data;
  const uint8_t* payload = data + sizeof(RadioHdr);
  const int plen = hdr->len;

  if (plen < 0 || (int)sizeof(RadioHdr) + plen + 2 != len) { g_pktsToolong++; return; }
  if (plen > (int)MAX_PAYLOAD) { g_pktsToolong++; return; }
  if (hdr->proto != PROTO_ID) { return; } // drop mismatched protocol

  // CRC check
  uint16_t expect = 0;
  memcpy(&expect, data + sizeof(RadioHdr) + plen, 2);
  uint16_t calc = crc16_ccitt(data, sizeof(RadioHdr) + plen);
  if (calc != expect) { g_pktsCrc++; return; }

  // Dedup (allow strictly increasing)
  if (hdr->seq <= g_lastSeq) { g_pktsDup++; return; }
  g_lastSeq = hdr->seq;

  // Write out to UART
  UartOut.write(payload, plen);
  g_pktsOk++;
  g_lastRxMs = millis();

  // --- LINK UP transition message (RX) ---
  if (!g_rxLinkUp) {
    g_rxLinkUp = true;
    Serial.printf("[RX] LINK UP  (seq=%lu)\n", (unsigned long)hdr->seq);
  }
}
#endif

#if defined(ROLE_TX)
static void on_send_cb(const uint8_t* mac, esp_now_send_status_t status) {
  g_txLastSendMs = millis();
  if (status == ESP_NOW_SEND_SUCCESS) {
    g_txFailStreak = 0;
    if (!g_txLinkUp) {
      g_txLinkUp = true;
      Serial.print("[TX] LINK UP  (peer ");
      if (mac) Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
      Serial.println(")");
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

// ===== Setup / Loop =====
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(100);
  Serial.println("\nESP-NOW Serial Tunnel (Unified)");

  lock_wifi_channel(ESPNOW_CHANNEL);

  // Print our MAC
  uint8_t self[6];
  esp_wifi_get_mac(WIFI_IF_STA, self);
  print_mac(self, "[Self MAC]");

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED");
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }

#if defined(ROLE_TX)
  {
    // Configure peer (RX) MAC
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

    if (peer.peer_addr[0]==0 && peer.peer_addr[1]==0 && peer.peer_addr[2]==0 &&
        peer.peer_addr[3]==0 && peer.peer_addr[4]==0 && peer.peer_addr[5]==0) {
      Serial.println("** Set PEER0..5 build flags to your RX MAC! **");
    }

    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("esp_now_add_peer FAILED");
    }

    esp_now_register_send_cb(on_send_cb);
    Serial.println("[ROLE] TX");
  }
#elif defined(ROLE_RX)
  {
    Serial.println("[ROLE] RX");
    esp_now_register_recv_cb(on_recv_cb);
  }
#endif

  // UARTs
#if defined(ROLE_TX)
  UartIn.begin(PROTO_BAUD, PROTO_CFG, RX_PIN, -1, RX_INVERT);
#elif defined(ROLE_RX)
  UartOut.begin(PROTO_BAUD, PROTO_CFG, -1, TX_PIN, TX_INVERT);
#endif

  Serial.printf("[PROTO] %s  baud=%u  cfg=0x%08lx  invert(rx=%d tx=%d)\n",
#if defined(PROTO_CRSF)
                "CRSF",
#else
                "SBUS",
#endif
                PROTO_BAUD, (unsigned long)PROTO_CFG, (int)RX_INVERT, (int)TX_INVERT);

  Serial.printf("[WiFi] Locked to channel %d\n", ESPNOW_CHANNEL);
  Serial.flush();
}

void loop() {
#if defined(ROLE_TX)
  static uint32_t seq = 0;
  static uint8_t buf[sizeof(RadioHdr) + MAX_PAYLOAD + 2];

  // Read available bytes from UART (up to MAX_PAYLOAD each cycle)
  int avail = UartIn.available();
  if (avail > 0) {
    int toRead = avail;
    if (toRead > (int)MAX_PAYLOAD) toRead = MAX_PAYLOAD;

    // Fill header
    RadioHdr hdr{};
    hdr.seq = seq++;
    hdr.tx_micros = micros();
    hdr.len = (uint16_t)toRead;
    hdr.proto = PROTO_ID;

    // Copy into buffer
    memcpy(buf, &hdr, sizeof(RadioHdr));
    int got = UartIn.readBytes(buf + sizeof(RadioHdr), toRead);

    // CRC
    uint16_t crc = crc16_ccitt(buf, sizeof(RadioHdr) + got);
    memcpy(buf + sizeof(RadioHdr) + got, &crc, 2);

    // Send once (optionally duplicate)
    esp_now_send(nullptr, buf, sizeof(RadioHdr) + got + 2); // nullptr = all peers (we added one)
#if DUP_SEND
    esp_now_send(nullptr, buf, sizeof(RadioHdr) + got + 2);
#endif
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // blink on send
  }

  // --- NEW: forward from USB serial monitor if anything typed there ---
  if (Serial.available() > 0) {
    int toRead = Serial.available();
    if (toRead > (int)MAX_PAYLOAD) toRead = MAX_PAYLOAD;

    RadioHdr hdr{};
    hdr.seq = seq++;
    hdr.tx_micros = micros();
    hdr.len = (uint16_t)toRead;
    hdr.proto = PROTO_ID;

    memcpy(buf, &hdr, sizeof(RadioHdr));
    int got = Serial.readBytes((char*)buf + sizeof(RadioHdr), toRead);

    uint16_t crc = crc16_ccitt(buf, sizeof(RadioHdr) + got);
    memcpy(buf + sizeof(RadioHdr) + got, &crc, 2);

    esp_now_send(nullptr, buf, sizeof(RadioHdr) + got + 2);
    Serial.print("[TX] forwarded from usb: ");
    Serial.write((char*)buf + sizeof(RadioHdr), got);
    Serial.println();
  }

  // --- TX link down by timeout ---
  uint32_t now = millis();
  if (g_txLinkUp) {
    if ((now - g_txLastOkMs) > TX_LINK_TIMEOUT_MS) {
      g_txLinkUp = false;
      Serial.println("[TX] LINK DOWN (timeout)");
    }
  }

  // TX stats
  if (now - g_lastPrintMs >= PRINT_STATS_MS) {
    g_lastPrintMs = now;
    Serial.printf("[TX] lastOk=%lu ms ago  failStreak=%lu  link=%s\n",
      (unsigned long)(now - g_txLastOkMs),
      (unsigned long)g_txFailStreak,
      g_txLinkUp ? "YES" : "NO");
  }

#elif defined(ROLE_RX)
  // Link LED & stats
  uint32_t now = millis();
  bool link = (now - g_lastRxMs) < FAILSAFE_MS;
  digitalWrite(LED_PIN, link ? HIGH : LOW);

  // --- LINK DOWN transition message (RX) ---
  if (g_rxLinkUp && !link) {
    g_rxLinkUp = false;
    Serial.println("[RX] LINK DOWN (timeout)");
  }

  if (now - g_lastPrintMs >= PRINT_STATS_MS) {
    g_lastPrintMs = now;
    Serial.printf("ok=%lu crc=%lu dup=%lu toolong=%lu lastSeq=%lu link=%s\n",
      (unsigned long)g_pktsOk, (unsigned long)g_pktsCrc,
      (unsigned long)g_pktsDup, (unsigned long)g_pktsToolong,
      (unsigned long)g_lastSeq, link ? "YES" : "NO");
  }
#endif
}
