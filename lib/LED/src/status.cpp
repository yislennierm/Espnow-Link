// status_led.cpp
#include "status.h"

static int ledPin = -1;
static LedState currentState = LED_OFF;

static void led_task(void *pv) {
  unsigned long lastToggle = 0;
  bool ledOn = false;

  for (;;) {
    unsigned long now = millis();
    unsigned long interval = 1000; // default slow blink

    switch (currentState) {
      case LED_OFF:
        digitalWrite(ledPin, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;

      case LED_SOLID:
        digitalWrite(ledPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;

      case LED_BLINK_SLOW:
        interval = 500; // 1Hz (500ms on/off)
        break;

      case LED_BLINK_FAST:
        interval = 125; // 4Hz
        break;

      case LED_ERROR:
        // Example SOS pattern: blink 3 short, 3 long, 3 short
        // (for now, just use very fast blink)
        interval = 50;
        break;
    }

    if (now - lastToggle > interval) {
      lastToggle = now;
      ledOn = !ledOn;
      digitalWrite(ledPin, ledOn ? HIGH : LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void led_init(int pin) {
  ledPin = pin;
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  xTaskCreatePinnedToCore(led_task, "StatusLED", 2048, NULL, 1, NULL, 0);
}

void led_set_state(LedState state) {
  currentState = state;
}
