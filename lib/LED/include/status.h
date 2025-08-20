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

void status_set(uint32_t bits);
void status_clear(uint32_t bits);
uint32_t status_check();
