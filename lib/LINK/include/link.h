#pragma once
#include <Arduino.h>
#include <esp_now.h>


bool wifi_init(uint8_t channel);
bool wifi_set_peer(const uint8_t* mac);
bool wifi_send(const uint8_t* data, int len);
void wifi_debug();
void wifi_register_app_rx(esp_now_recv_cb_t cb);
