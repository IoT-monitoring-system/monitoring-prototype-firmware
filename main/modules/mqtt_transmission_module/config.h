#pragma once
#ifndef MQTT_TRANSMISSION_MODULE_CONFIG_H
#define MQTT_TRANSMISSION_MODULE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define DATA_AGGREGATION_MAX_PAYLOADS       25U
#define DATA_AGGREGATION_CBOR_OVERHEAD_COEF 1.2f /* Assuming 20% overhead */
#define DATA_AGGREGATION_TASK_TIMEOUT_MS    10000U

#define MQTT_SERVER           "192.168.0.123"
#define MQTT_PORT             8884U
#define MQTT_USERNAME         "esp32_1"
#define MQTT_PASSWORD         "Test12345"
#define MQTT_DATA_OUT_TOPIC   "/IoT-Monitoring-System/DEVICE_OUT/DATA"
#define MQTT_BUFFER_COUNT     3U
#define MQTT_MAX_MESSAGE_SIZE 4096U
#define MQTT_TASK_TIMEOUT_MS  10000U

#define MQTT_DEVICE_ID "esp32_node_1"

#ifdef __cplusplus
}
#endif
#endif