#pragma once
#ifndef FALL_DETECT_MODULE_H
#define FALL_DETECT_MODULE_H

#include "esp_err.h"

#include "Arduino.h"
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sntp_client.h"

struct fall_detect_module_config {
  // resources
  QueueHandle_t out_queue;
  TwoWire *wire;

  // services
  sntp_client_handle sntp_client;
};

esp_err_t
fall_detect_module_init(fall_detect_module_config *module_cfg);
esp_err_t
fall_detect_module_del();

esp_err_t
fall_detect_module_start();
esp_err_t
fall_detect_module_stop();

#endif