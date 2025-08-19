#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#include "link.h"

// ----------------- internal state -----------------
static bool g_linkUp = false;
static uint32_t g_lastOkMs = 0;
static uint32_t g_lastSendMs = 0;
static uint32_t g_failStreak = 0;
static uint8_t g_peer[6] = {0};

// ----------------- helpers -----------------
static void print_mac(const uint8_t mac[6], const char* label) {
    Serial.printf("%s %02X:%02X:%02X:%02X:%02X:%02X\n", label,
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ----------------- send callback -----------------
static void on_send_cb(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    g_lastSendMs = millis();
    if (status == ESP_NOW_SEND_SUCCESS) {
        g_failStreak = 0;
        if (!g_linkUp) {
            g_linkUp = true;
            Serial.println("[WIFI] LINK UP");
        }
        g_lastOkMs = g_lastSendMs;
    } else {
        g_failStreak++;
        if (g_linkUp && g_failStreak >= 5) { // configurable if needed
            g_linkUp = false;
            Serial.println("[WIFI] LINK DOWN (consecutive send fails)");
        }
    }
}

// ----------------- public API -----------------
bool wifi_init(uint8_t channel) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);
    WiFi.setSleep(false);

    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    uint8_t self[6];
    esp_wifi_get_mac(WIFI_IF_STA, self);
    print_mac(self, "[Self MAC]");

    if (esp_now_init() != ESP_OK) {
        Serial.println("esp_now_init FAILED");
        return false;
    }

    esp_now_register_send_cb(on_send_cb);
    Serial.printf("[WIFI] ESPNOW init OK (channel %d)\n", channel);
    return true;
}

bool wifi_set_peer(const uint8_t* mac) {
    if (!mac) return false;

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;     // use current channel
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("esp_now_add_peer FAILED");
        return false;
    }
    memcpy(g_peer, mac, 6);
    print_mac(g_peer, "[Peer]");
    return true;
}

bool wifi_send(const uint8_t* data, int len) {
    if (!g_peer[0] && !g_peer[1] && !g_peer[2] &&
        !g_peer[3] && !g_peer[4] && !g_peer[5]) {
        Serial.println("[WIFI] ERROR: No peer set!");
        return false;
    }
    return (esp_now_send(g_peer, data, len) == ESP_OK);
}

void wifi_debug() {
    uint32_t now = millis();
    Serial.printf("[WIFI] lastOk=%lu ms ago  failStreak=%lu  link=%s\n",
                  (unsigned long)(now - g_lastOkMs),
                  (unsigned long)g_failStreak,
                  g_linkUp ? "YES" : "NO");
}

void wifi_register_app_rx(esp_now_recv_cb_t cb) {
    esp_now_register_recv_cb(cb);
}
