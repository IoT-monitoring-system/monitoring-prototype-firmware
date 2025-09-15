#pragma once
#ifndef MQTT_TRANSMISSION_MODULE_H
#define MQTT_TRANSMISSION_MODULE_H

#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

struct mqtt_transmission_module_config {
  QueueHandle_t *in_queue;
};

esp_err_t
mqtt_transmission_module_init(QueueHandle_t *out_queue);
esp_err_t
mqtt_transmission_module_del();

esp_err_t
mqtt_transmission_module_start();
esp_err_t
mqtt_transmission_module_stop();

#ifdef __cplusplus
}
#endif
#endif