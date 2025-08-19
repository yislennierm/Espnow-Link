#include <Arduino.h>
#include "status.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static uint8_t ledPin = 255;

// Global STATUS register + lock
SemaphoreHandle_t status_lock = nullptr;
volatile uint32_t STATUS_REG = 0;

void status_init(uint8_t pin) {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

    // Create the global lock
    status_lock = xSemaphoreCreateMutex();

#if CONFIG_FREERTOS_UNICORE
    // ESP32-S2 single-core
    xTaskCreate([](void*) {
#else
    // ESP32/ESP32-S3 dual-core
    xTaskCreatePinnedToCore([](void*) {
#endif
        Serial.println("Status Task - Started");

        for (;;) {
            uint32_t s;
            if (xSemaphoreTake(status_lock, portMAX_DELAY) == pdTRUE) {
                s = STATUS_REG;
                xSemaphoreGive(status_lock);
            } else {
                s = 0; // fallback
            }

            // Priority: LINK > INPUT > PROTO > heartbeat
            if (s & STATUS_LINK) {
                digitalWrite(ledPin, HIGH);
                vTaskDelay(pdMS_TO_TICKS(200));
            } 
            else if (s & STATUS_INPUT) {
                digitalWrite(ledPin, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
                digitalWrite(ledPin, LOW);
                vTaskDelay(pdMS_TO_TICKS(900));
            } 
            else if (s & STATUS_PROTO) {
                digitalWrite(ledPin, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
                digitalWrite(ledPin, LOW);
                vTaskDelay(pdMS_TO_TICKS(100));
            } 
            else {
                // default slow heartbeat
                digitalWrite(ledPin, HIGH);
                vTaskDelay(pdMS_TO_TICKS(50));
                digitalWrite(ledPin, LOW);
                vTaskDelay(pdMS_TO_TICKS(1950));
            }
        }
#if CONFIG_FREERTOS_UNICORE
    }, "statusTask", 2048, nullptr, 1, nullptr);
#else
    }, "statusTask", 4096, nullptr, 1, nullptr, tskNO_AFFINITY);
#endif
}

void status_set(uint32_t bits) {
    if (!status_lock) return;  // safety
    if (xSemaphoreTake(status_lock, portMAX_DELAY) == pdTRUE) {
        STATUS_REG |= bits;
        xSemaphoreGive(status_lock);
    }
}

void status_clear(uint32_t bits) {
    if (!status_lock) return;
    if (xSemaphoreTake(status_lock, portMAX_DELAY) == pdTRUE) {
        STATUS_REG &= ~bits;
        xSemaphoreGive(status_lock);
    }
}

uint32_t status_check() {
    if (!status_lock) return STATUS_REG;
    uint32_t v = 0;
    if (xSemaphoreTake(status_lock, portMAX_DELAY) == pdTRUE) {
        v = STATUS_REG;
        xSemaphoreGive(status_lock);
    }
    return v;
}
