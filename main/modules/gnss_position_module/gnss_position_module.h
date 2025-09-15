#pragma once
#ifndef GNSS_POSITION_MODULE_H
#define GNSS_POSITION_MODULE_H

#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sntp_client.h"

#ifdef __cplusplus
extern "C" {
#endif

struct gnss_position_module_config {
  // resources
  QueueHandle_t out_queue;

  // services
  sntp_client_handle sntp_client;
};

esp_err_t
gnss_position_module_init(struct gnss_position_module_config *module_cfg);
esp_err_t
gnss_position_module_del();

esp_err_t
gnss_position_module_start();
esp_err_t
gnss_position_module_stop();

#ifdef __cplusplus
}
#endif
#endif