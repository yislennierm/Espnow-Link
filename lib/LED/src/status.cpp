#include <Arduino.h>
#include "status.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Global STATUS register + lock
SemaphoreHandle_t status_lock = nullptr;
volatile uint32_t STATUS_REG = 0;

void status_init(uint8_t pin)
{

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Create the global lock
    status_lock = xSemaphoreCreateMutex();

#if CONFIG_FREERTOS_UNICORE
    // ESP32-S2 single-core
    xTaskCreate([](void *)
                {
#else
    // ESP32/ESP32-S3 dual-core
    xTaskCreatePinnedToCore([](void *)
                            {
#endif
                    Serial.println("Status Task - Started");

                    for (;;)
                    {
                        uint32_t s;
                        if (xSemaphoreTake(status_lock, portMAX_DELAY) == pdTRUE)
                        {
                            s = STATUS_REG;
                            xSemaphoreGive(status_lock);
                        }
                        else
                        {
                            s = 0;
                        }

                        if (s & STATUS_TLM)
                        {
                            // Error → uneven pattern
                            digitalWrite(LED_PIN, HIGH);
                            vTaskDelay(pdMS_TO_TICKS(100));
                            digitalWrite(LED_PIN, LOW);
                            vTaskDelay(pdMS_TO_TICKS(400));
                            digitalWrite(LED_PIN, HIGH);
                            vTaskDelay(pdMS_TO_TICKS(100));
                            digitalWrite(LED_PIN, LOW);
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        }
                        else if ((s & STATUS_LINK) && (s & STATUS_INPUT))
                        {
                            // Link + Input = solid ON
                            digitalWrite(LED_PIN, HIGH);
                            vTaskDelay(pdMS_TO_TICKS(200)); // small delay to avoid hogging CPU
                        }
                        else if (s & STATUS_LINK)
                        {
                            // Link but no input = slow heartbeat
                            digitalWrite(LED_PIN, HIGH);
                            vTaskDelay(pdMS_TO_TICKS(50));
                            digitalWrite(LED_PIN, LOW);
                            vTaskDelay(pdMS_TO_TICKS(1950));
                        }
                        else
                        {
                            // No link = fast blink
                            digitalWrite(LED_PIN, HIGH);
                            vTaskDelay(pdMS_TO_TICKS(30));
                            digitalWrite(LED_PIN, LOW);
                            vTaskDelay(pdMS_TO_TICKS(30));
                        }
                    }
#if CONFIG_FREERTOS_UNICORE
                },
                "statusTask", 4096, nullptr, 1, nullptr);
#else
                            },
                            "statusTask", 4096, nullptr, 1, nullptr, tskNO_AFFINITY);
#endif
}

void status_set(uint32_t bits)
{
    if (!status_lock)
        return; // safety
    if (xSemaphoreTake(status_lock, portMAX_DELAY) == pdTRUE)
    {
        STATUS_REG |= bits;
        xSemaphoreGive(status_lock);
    }
}

void status_clear(uint32_t bits)
{
    if (!status_lock)
        return;
    if (xSemaphoreTake(status_lock, portMAX_DELAY) == pdTRUE)
    {
        STATUS_REG &= ~bits;
        xSemaphoreGive(status_lock);
    }
}

uint32_t status_set()
{
    if (!status_lock)
        return STATUS_REG;
    uint32_t v = 0;
    if (xSemaphoreTake(status_lock, portMAX_DELAY) == pdTRUE)
    {
        v = STATUS_REG;
        xSemaphoreGive(status_lock);
    }
    return v;
}
