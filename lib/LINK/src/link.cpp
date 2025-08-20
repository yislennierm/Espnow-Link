#include "link.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "status.h"

// ==================
// Globals
// ==================
static QueueHandle_t txQueue = nullptr;
static QueueHandle_t rxQueue = nullptr;

static LinkState currentState = LINK_INIT;

struct FrameItem {
    uint8_t data[250];
    size_t len;
};

#ifdef ROLE_TX
static uint8_t peerMac[6] = {
    (uint8_t)PEER0, (uint8_t)PEER1, (uint8_t)PEER2,
    (uint8_t)PEER3, (uint8_t)PEER4, (uint8_t)PEER5
};
static esp_now_peer_info_t peerInfo;
#endif

// Track last activity times
static unsigned long lastFrameMs = 0;
static unsigned long lastLinkMs  = 0;

// ==================
// Callbacks
// ==================
#ifdef ROLE_TX
static void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        lastLinkMs = millis();
        status_set(STATUS_LINK);
        // leave INPUT as is, only RX sets that
        Serial.println("[LINK TX] Send OK");
    } else {
        Serial.println("[LINK TX] Send FAIL");
        status_clear(STATUS_LINK);
        currentState = LINK_ERROR;
    }
}
#endif

#ifdef ROLE_RX
static void on_recv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    Serial.printf("[LINK RX] Frame received, len=%d\n", len);
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();

    if (len <= 0) return;
    FrameItem item;
    if (len > sizeof(item.data)) return;
    memcpy(item.data, data, len);
    item.len = len;
    if (rxQueue) xQueueSend(rxQueue, &item, 0);

    // mark link and input activity
    lastFrameMs = millis();
    status_set(STATUS_LINK | STATUS_INPUT);
}
#endif

// ==================
// Init
// ==================
void link_init() {


#ifdef ROLE_TX
    esp_now_register_send_cb(on_sent);
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, peerMac, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[LINK] Failed to add peer");
        status_clear(STATUS_LINK);
        currentState = LINK_ERROR;
        return;
    }
#endif

#ifdef ROLE_RX
    esp_now_register_recv_cb(on_recv);
#endif

    txQueue = xQueueCreate(10, sizeof(FrameItem));
    rxQueue = xQueueCreate(10, sizeof(FrameItem));

    status_clear(STATUS_LINK | STATUS_INPUT);  // start "no link"
    currentState = LINK_IDLE;
}

// ==================
// Send
// ==================
#ifdef ROLE_TX
bool link_send(const uint8_t *data, size_t len) {
    if (!txQueue) return false;
    FrameItem item;
    if (len > sizeof(item.data)) return false;
    memcpy(item.data, data, len);
    item.len = len;
    return xQueueSend(txQueue, &item, 0) == pdTRUE;
}
#endif

// ==================
// Loop
// ==================
void link_loop() {
    unsigned long now = millis();

    switch (currentState) {
    case LINK_IDLE:
#ifdef ROLE_TX
        if (txQueue) {
            FrameItem item;
            if (xQueueReceive(txQueue, &item, 0) == pdTRUE) {
                esp_err_t res = esp_now_send(peerMac, item.data, item.len);
                if (res != ESP_OK) {
                    Serial.printf("[LINK] esp_now_send error=%d\n", res);
                    status_clear(STATUS_LINK);
                    currentState = LINK_ERROR;
                }
            }
        }
#endif
#ifdef ROLE_RX
        if (rxQueue) {
            FrameItem item;
            if (xQueueReceive(rxQueue, &item, 0) == pdTRUE) {
                Serial1.write(item.data, item.len);
            }
        }
#endif

        // Link alive timeout (5s without activity = drop LINK)
        if ((status_check() & STATUS_LINK) && (now - lastFrameMs > 5000)) {
            status_clear(STATUS_LINK | STATUS_INPUT);
        }

        // Input timeout (2s without frame = clear INPUT only)
        if ((status_check() & STATUS_INPUT) && (now - lastFrameMs > 2000)) {
            status_clear(STATUS_INPUT);
        }

        break;

    case LINK_ERROR:
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        break;

    default:
        break;
    }
}
