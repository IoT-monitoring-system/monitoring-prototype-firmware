#include "float.h"
#include "math.h"
#include "stdio.h"

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "cbor_sensor_encoder.h"

#include "mqtt_client_simple.h"

#include "config.h"
#include "mqtt_transmission_module.h"

typedef struct {
  uint8_t buffer[MQTT_MAX_MESSAGE_SIZE];
  uint32_t length;
} mqtt_message;

extern const uint8_t client_crt_start[] asm("_binary_client_crt_start");
extern const uint8_t client_crt_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_end[] asm("_binary_client_key_end");
extern const uint8_t root_cert_pem_start[] asm("_binary_rootCA_pem_start");
extern const uint8_t root_cert_pem_end[] asm("_binary_rootCA_pem_end");

static mqtt_client_simple_handle mqtt_client;

static TaskHandle_t task_sensor_data_aggregation_handle;
static TaskHandle_t task_mqtt_sending_handle;

static QueueHandle_t mqtt_free_queue;
static QueueHandle_t mqtt_filled_queue;

static QueueHandle_t queue;

static char DEVICE_ID[16];

EXT_RAM_BSS_ATTR mqtt_message mqtt_buffers[MQTT_BUFFER_COUNT];

static const char *TAG = "mqtt_transmission_module";

static esp_err_t
init_mqtt();

static void
task_sensor_data_aggregation(void *arg);

static void
task_mqtt_sending(void *arg);

static esp_err_t
mqtt_data_event_handler(mqtt_client_simple_handle mqtt_client, int32_t event_id, esp_mqtt_event_handle_t event_handle);

static void
mqtt_transmission_module_cleanup();

esp_err_t
mqtt_transmission_module_init(QueueHandle_t *out_queue) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(out_queue, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  queue = xQueueCreate(DATA_AGGREGATION_MAX_PAYLOADS, sizeof(sensor_payload_t));
  ESP_GOTO_ON_FALSE(queue, ESP_ERR_NO_MEM, err, TAG, "failed to create queue");
  *out_queue = queue;

  mqtt_free_queue = xQueueCreate(MQTT_BUFFER_COUNT, sizeof(mqtt_message *));
  ESP_GOTO_ON_FALSE(mqtt_free_queue, ESP_ERR_NO_MEM, err, TAG, "failed to create mqtt_free_queue");

  mqtt_filled_queue = xQueueCreate(MQTT_BUFFER_COUNT, sizeof(mqtt_message *));
  ESP_GOTO_ON_FALSE(mqtt_filled_queue, ESP_ERR_NO_MEM, err, TAG, "failed to create mqtt_filled_queue");

  for (int i = 0; i < MQTT_BUFFER_COUNT; ++i) {
    mqtt_message *msg = &mqtt_buffers[i];
    xQueueSend(mqtt_free_queue, &msg, 0);
  }

  ESP_GOTO_ON_ERROR(init_mqtt(), err, TAG, "failed to initialize MQTT");

  ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(task_mqtt_sending, "mqttTsk", 3072U, NULL, 9, &task_mqtt_sending_handle, 1U) ==
                        pdPASS,
                    ESP_FAIL, err, TAG, "failed to create mqttTsk");
  ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(task_sensor_data_aggregation, "dataAggrTsk", 3072U, NULL, 8,
                                            &task_sensor_data_aggregation_handle, 0U) == pdPASS,
                    ESP_FAIL, err, TAG, "failed to create dataAggrTsk");

  return ESP_OK;
err:
  mqtt_transmission_module_cleanup();
  return ret;
}
esp_err_t
mqtt_transmission_module_del() {
  mqtt_transmission_module_stop();
  mqtt_transmission_module_cleanup();

  ESP_LOGI(TAG, "Deleted MQTT Transmission module");

  return ESP_OK;
}

esp_err_t
mqtt_transmission_module_start() {
  vTaskResume(task_mqtt_sending_handle);
  vTaskResume(task_sensor_data_aggregation_handle);

  ESP_LOGI(TAG, "Started MQTT Transmission module");

  return ESP_OK;
}
esp_err_t
mqtt_transmission_module_stop() {
  xTaskNotifyGive(task_sensor_data_aggregation_handle);
  xTaskNotifyGive(task_mqtt_sending_handle);

  ESP_LOGI(TAG, "Stopped MQTT Transmission module");

  return ESP_OK;
}

static esp_err_t
init_mqtt() {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_ERROR(mqtt_client_simple_init(&mqtt_client), TAG, "mqtt_client_simple init failed");

  ESP_RETURN_ON_ERROR(
      mqtt_client_simple_register_event_handler(mqtt_client, MQTT_CLIENT_SIMPLE_DATA_EVENT, mqtt_data_event_handler), TAG,
      "mqtt_client_simple_register_event_handler failed");

  struct mqtt_address_config addr_cfg = {
      .hostname = MQTT_SERVER,
      .transport = MQTT_TRANSPORT_OVER_SSL,
      .path = "/",
      .port = MQTT_PORT,
  };
  ESP_RETURN_ON_ERROR(mqtt_client_simple_set_address_cfg(mqtt_client, &addr_cfg), TAG,
                      "mqtt_client_simple_set_address_cfg failed");

  struct mqtt_credentials_config cred_cfg = {
      .username = MQTT_USERNAME,
  };
  ESP_RETURN_ON_ERROR(mqtt_client_simple_set_credentials_cfg(mqtt_client, &cred_cfg), TAG,
                      "mqtt_client_simple_set_credentials_cfg failed");

  struct mqtt_verification_config verif_cfg = {
      .certificate = (const char *)root_cert_pem_start,
  };
  ESP_RETURN_ON_ERROR(mqtt_client_simple_set_verification_cfg(mqtt_client, &verif_cfg), TAG,
                      "mqtt_client_simple_set_verification_cfg failed");

  struct mqtt_auth_config auth_cfg = {
      .password = MQTT_PASSWORD,
      .certificate = (const char *)client_crt_start,
      .key = (const char *)client_key_start,
  };
  ESP_RETURN_ON_ERROR(mqtt_client_simple_set_authentication_cfg(mqtt_client, &auth_cfg), TAG,
                      "mqtt_client_simple_set_authentication_cfg failed");

  struct mqtt_session_config sesh_cfg = {
      .disable_clean_session = true,

  };
  ESP_RETURN_ON_ERROR(mqtt_client_simple_set_session_cfg(mqtt_client, &sesh_cfg), TAG,
                      "mqtt_client_simple_set_session_cfg failed");

  ESP_RETURN_ON_ERROR(mqtt_client_simple_connect(mqtt_client, 15000), TAG, "mqtt_client_simple_connect failed");
  ESP_RETURN_ON_ERROR(mqtt_client_simple_subscribe(mqtt_client, "/IoT-Clock-RoomMonitor/DEVICE_IN/CTRL", 0), TAG,
                      "mqtt_client_simple_subscribe failed");

  ESP_LOGD(TAG, "Initialized MQTT module");

  return ret;
}

void
task_sensor_data_aggregation(void *arg) {
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, 0)) {
      vTaskSuspend(NULL);
    }

    mqtt_message *msg = NULL;
    if (xQueueReceive(mqtt_free_queue, &msg, pdMS_TO_TICKS(DATA_AGGREGATION_TASK_TIMEOUT_MS)) == pdTRUE) {
      uint32_t payloads_received = 0U;

      CborEncoder encoder, map_outer, map_data, sensors_array;

      cbor_encoder_init(&encoder, msg->buffer, sizeof(msg->buffer), 0);
      cbor_encoder_create_map(&encoder, &map_outer, CborIndefiniteLength);

      cbor_encode_text_stringz(&map_outer, "data");

      cbor_encoder_create_map(&map_outer, &map_data, CborIndefiniteLength);
      cbor_encode_text_stringz(&map_data, "deviceId");
      cbor_encode_text_stringz(&map_data, MQTT_DEVICE_ID);

      cbor_encode_text_stringz(&map_data, "sensor_data");

      cbor_encoder_create_array(&map_data, &sensors_array, DATA_AGGREGATION_MAX_PAYLOADS);

      uint32_t prev_buffer_size = cbor_encoder_get_buffer_size(&sensors_array, msg->buffer);
      while (payloads_received < DATA_AGGREGATION_MAX_PAYLOADS && prev_buffer_size <= MQTT_MAX_MESSAGE_SIZE) {
        sensor_payload_t payload;
        if (xQueueReceive(queue, &payload, portMAX_DELAY) != pdTRUE) {
          break;
        }

        ESP_RETURN_VOID_ON_ERROR(encode_sensor_payload(&payload, &sensors_array), TAG, "Failed to encode a sensor payload");

        /* Debug start */
        // uint32_t buffer_size = cbor_encoder_get_buffer_size(&sensors_array, msg->buffer);
        // uint32_t encoded_size = buffer_size - prev_buffer_size;
        // prev_buffer_size = buffer_size;
        // ESP_LOGD(TAG, "Encoded a payload, sensor:%s size:%lu", payload.sensor, encoded_size);
        /* Debug end */

        payloads_received++;
      }

      cbor_encoder_close_container(&map_data, &sensors_array);
      cbor_encoder_close_container(&map_outer, &map_data);
      cbor_encoder_close_container(&encoder, &map_outer);

      msg->length = cbor_encoder_get_buffer_size(&encoder, msg->buffer);

      // ESP_LOGD(TAG, "Encoded an mqtt message, length:%lu, sensor payloads:%lu", msg->length, payloads_received);

      if (xQueueSend(mqtt_filled_queue, &msg, pdMS_TO_TICKS(1000U)) != pdTRUE) {
        ESP_LOGE(TAG, "Filled queue full, dropping msg");
        memset(msg->buffer, 0, sizeof(msg->buffer));
        xQueueSend(mqtt_free_queue, &msg, 0);
      }
    }
  }
}

void
task_mqtt_sending(void *arg) {
  esp_err_t ret = ESP_OK;
  mqtt_message *msg = NULL;
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, 0)) {
      vTaskSuspend(NULL);
    }

    if (xQueueReceive(mqtt_filled_queue, &msg, pdMS_TO_TICKS(MQTT_TASK_TIMEOUT_MS)) == pdTRUE) {
      ret = mqtt_client_simple_publish(mqtt_client, MQTT_DATA_OUT_TOPIC, (char *)msg->buffer, msg->length, 0, 0);
      if (ESP_ERR_INVALID_STATE == ret) {
        mqtt_client_simple_reconnect(mqtt_client, (uint32_t)portMAX_DELAY);
      } else {
        ESP_LOGD(TAG, "Sent an MQTT message via MQTT, size:%lu", msg->length);
      }

      memset(msg->buffer, 0, sizeof(msg->buffer));

      xQueueSend(mqtt_free_queue, &msg, 0);
    }
  }
}
esp_err_t
mqtt_data_event_handler(mqtt_client_simple_handle mqtt_client, int32_t event_id, esp_mqtt_event_handle_t event_handle) {
  //   - msg_id               message id
  //   - topic                pointer to the received topic
  //   - topic_len            length of the topic
  //   - data                 pointer to the received data
  //   - data_len             length of the data for this event
  //   - current_data_offset  offset of the current data for this event
  //   - total_data_len       total length of the data received
  //   - retain               retain flag of the message
  //   - qos                  QoS level of the message
  //   - dup                  dup flag of the message
  //   Note: Multiple MQTT_EVENT_DATA could be fired for one
  // message, if it is longer than internal buffer. In that
  // case only first event contains topic pointer and length,
  // other contain data only with current data length and
  // current data offset updating.
  esp_err_t ret = ESP_OK;
  int msg_id = event_handle->msg_id;

  if (msg_id < 0) {
    ESP_LOGE(TAG, "Error receiving a message on %s; Error: %i", event_handle->topic_len ? event_handle->topic : "None", msg_id);
    return ret;
  }

  ESP_LOGD(TAG, "Received on %.*s; Content: %.*s", event_handle->topic_len, event_handle->topic, event_handle->data_len,
           event_handle->data);
  return ret;
}

static void
mqtt_transmission_module_cleanup() {
  /* If NULL is unchecked the method will delete the calling task */
  if (task_mqtt_sending_handle)
    vTaskDelete(task_mqtt_sending_handle);
  if (task_sensor_data_aggregation_handle)
    vTaskDelete(task_sensor_data_aggregation_handle);

  mqtt_client_simple_del(mqtt_client);

  /* If NULL the method will fail at assert */
  vQueueDelete(queue);
  vQueueDelete(mqtt_free_queue);
  vQueueDelete(mqtt_filled_queue);

  task_mqtt_sending_handle = NULL;
  task_sensor_data_aggregation_handle = NULL;

  queue = NULL;
  mqtt_free_queue = NULL;
  mqtt_filled_queue = NULL;

  memset(mqtt_buffers, 0, sizeof(mqtt_buffers));
}
