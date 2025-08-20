#include <Arduino.h>
#include "txrx_common.h"
#include "status.h"
#include "link.h"
#include "ghst.h"



QueueHandle_t txQueue = nullptr;
QueueHandle_t rxQueue = nullptr;



void common_setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(3000));
    status_init(LED_PIN);
    vTaskDelay(pdMS_TO_TICKS(500));
    link_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.println("Link Task - Started");
}
