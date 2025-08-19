// status_led.h
#pragma once
#include <Arduino.h>


enum LedState {
  LED_OFF,
  LED_SOLID,
  LED_BLINK_SLOW,   // e.g. 1Hz
  LED_BLINK_FAST,   // NO LINK
  LED_ERROR,        // maybe SOS blink pattern
};

void led_init(int pin);
void led_set_state(LedState state);
