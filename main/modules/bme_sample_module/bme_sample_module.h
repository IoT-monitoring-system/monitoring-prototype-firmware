#pragma once
#ifndef BME_SAMPLE_MODULE_H
#define BME_SAMPLE_MODULE_H

#include "esp_err.h"

#include "Arduino.h"
#include "FSManager.h"
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sntp_client.h"

struct bme_sample_module_config {
  // resources
  QueueHandle_t out_queue;
  TwoWire *wire;

  // services
  FSManager *fs_manager;
  sntp_client_handle sntp_client;
};

esp_err_t
bme_sample_module_init(bme_sample_module_config *module_cfg);
esp_err_t
bme_sample_module_del();

esp_err_t
bme_sample_module_start();
esp_err_t
bme_sample_module_stop();

#endif