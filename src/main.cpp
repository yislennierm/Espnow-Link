#include <Arduino.h>
#include "link.h"
#include "ghst.h"
#include "status.h"
#include <WiFi.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

QueueHandle_t txQueue;
QueueHandle_t rxQueue;

#ifdef ROLE_TX
// ---- TX: read GHST from UART and forward via ESPNOW ----
void uartReaderTask(void *pv) {
  uint8_t buf[64];
  int pos = 0;

  for (;;) {
    while (Serial2.available()) {
      buf[pos++] = Serial2.read();
      if (ghst::isCompleteFrame(buf, pos)) {
        int len = buf[1] + 2; // GHST frame length
        uint8_t *frame = (uint8_t*)malloc(len);
        memcpy(frame, buf, len);
        xQueueSend(txQueue, &frame, portMAX_DELAY);
        pos = 0;
      }
      if (pos >= sizeof(buf)) pos = 0;
    }
    vTaskDelay(1);
  }
}

void espnowSenderTask(void *pv) {
  uint8_t *frame;
  for (;;) {
    if (xQueueReceive(txQueue, &frame, portMAX_DELAY)) {
      link_send(frame, frame[1] + 2);
      ghst::debug_decode_channels(frame); // debug locally
      free(frame);
    }
  }
}
#endif

#ifdef ROLE_RX
// ---- RX: receive from ESPNOW and forward to FC ----
void on_recv_cb(const uint8_t* mac, const uint8_t* data, int len) {
  uint8_t *frame = (uint8_t*)malloc(len);
  memcpy(frame, data, len);
  xQueueSendFromISR(rxQueue, &frame, NULL);
}

void fcForwarderTask(void *pv) {
  uint8_t *frame;
  for (;;) {
    if (xQueueReceive(rxQueue, &frame, portMAX_DELAY)) {
      Serial1.write(frame, frame[1] + 2);   // forward to FC
      ghst::debug_decode_channels(frame);   // debug
      free(frame);
    }
  }
}
#endif

// ---- Common Setup ----
void setup() {
  
  
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(20));

  
  status_init(LED_PIN);
  vTaskDelay(pdMS_TO_TICKS(20));
  Serial.println("Status Task - Started");
  
  link_init();
  vTaskDelay(pdMS_TO_TICKS(100));
  Serial.println("Link Task - Started");





#ifdef ROLE_TX
  Serial2.begin(ghst::BAUD, ghst::CFG, RX_PIN, TX_PIN, ghst::TX_INVERT);
  txQueue = xQueueCreate(10, sizeof(uint8_t*));
  xTaskCreate(uartReaderTask, "uartReader", 4096, NULL, 1, NULL);
  xTaskCreate(espnowSenderTask, "espnowSender", 4096, NULL, 1, NULL);
#endif

#ifdef ROLE_RX
  Serial1.begin(ghst::BAUD, ghst::CFG, RX_PIN, TX_PIN, ghst::RX_INVERT);
  rxQueue = xQueueCreate(10, sizeof(uint8_t*));
  //link_set_recv_cb(on_recv_cb);
  xTaskCreate(fcForwarderTask, "fcForwarder", 4096, NULL, 1, NULL);
#endif
  
}

void loop() {
  vTaskDelay(portMAX_DELAY); 
}
