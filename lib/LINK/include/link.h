#pragma once
#include <Arduino.h>

void link_init();
void link_loop();
// link.h

void link_set_recv_cb(void (*cb)(const uint8_t *, const uint8_t *, int));

#ifdef ROLE_TX
bool link_send(const uint8_t *data, size_t len);
#endif
// ==================
// State Machine
// ==================
enum LinkState {
    LINK_INIT = 0,
    LINK_IDLE,
    LINK_ERROR
};