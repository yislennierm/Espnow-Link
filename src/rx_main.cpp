#include <Arduino.h>
#include "ghst.h"
#include "link.h"
#include "status.h"
#include "txrx_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


// ---- RX: receive from ESPNOW and forward to FC ----
void fcForwarderTask(void *pv) {
  uint8_t *frame;
  for (;;) {
    if (xQueueReceive(rxQueue, &frame, portMAX_DELAY)) {
      int len = frame[1] + 2;  // GHST frame length
      Serial1.write(frame, len);   // forward to FC
      ghst::debug_decode_channels(frame, len);
      free(frame);
    }
  }
}

void setup() {
  common_setup();

  Serial1.begin(ghst::BAUD, ghst::CFG, RX_PIN, TX_PIN, ghst::RX_INVERT);
  rxQueue = xQueueCreate(10, sizeof(uint8_t*));

  xTaskCreate(fcForwarderTask, "fcForwarder", 4096, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
