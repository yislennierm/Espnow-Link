#include <Arduino.h>
#include "ghst.h"
#include "link.h"
#include "status.h"
#include "txrx_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"



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
      int len = frame[1] + 2;  // GHST frame length
      link_send(frame, len);
      ghst::debug_dump_frame(frame, len);
      //ghst::debug_decode_channels(frame, len);
      free(frame);
    }
  }
}

void setup() {
  common_setup();

  Serial1.begin(ghst::BAUD, ghst::CFG, RX_PIN, TX_PIN, ghst::TX_INVERT);
  txQueue = xQueueCreate(10, sizeof(uint8_t*));

  xTaskCreate(uartReaderTask, "uartReader", 4096, NULL, 1, NULL);
  xTaskCreate(espnowSenderTask, "espnowSender", 4096, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
