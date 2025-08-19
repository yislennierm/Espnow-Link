#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


#ifdef __cplusplus
extern "C" {
#endif

void status_task(void *pvParameters);
void link_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

// =========================
// Status Bitfield Definitions
// =========================
#define STATUS_LINK     (1 << 0)   // ESPNOW link status
#define STATUS_INPUT    (1 << 1)   // Input data from handset
#define STATUS_PROTO    (1 << 2)   // Protocol OK
#define STATUS_TLM      (1 << 3)   // Telemetry OK
// add more bits as needed

// =========================
// API
// =========================
extern volatile uint32_t STATUS_REG;
extern SemaphoreHandle_t status_lock;


void status_init(uint8_t ledPin);

// Thread-safe accessors
inline void status_set_bits(uint32_t bits) {
    if (xSemaphoreTake(status_lock, portMAX_DELAY)) {
        STATUS_REG |= bits;
        xSemaphoreGive(status_lock);
    }
}

inline void status_clear_bits(uint32_t bits) {
    if (xSemaphoreTake(status_lock, portMAX_DELAY)) {
        STATUS_REG &= ~bits;
        xSemaphoreGive(status_lock);
    }
}

inline uint32_t status_get() {
    uint32_t value = 0;
    if (xSemaphoreTake(status_lock, portMAX_DELAY)) {
        value = STATUS_REG;
        xSemaphoreGive(status_lock);
    }
    return value;
}
