#ifndef TXRX_COMMON_H
#define TXRX_COMMON_H

#pragma once
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


#include "link.h"
#include "status.h"
#include "ghst.h"

extern QueueHandle_t rxQueue;
extern QueueHandle_t txQueue;

void common_setup();

#endif