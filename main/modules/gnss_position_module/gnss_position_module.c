#include "float.h"
#include "math.h"
#include "stdio.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "cbor_sensor_encoder.h"

#include "gnss_receiver.h"

#include "config.h"
#include "gnss_position_module.h"

static gnss_receiver_handle gnss_receiver;
static TaskHandle_t task_gnss_sampling_handle;
static QueueHandle_t gnss_event_data_queue_handle;

/* Managed by main app*/
static sntp_client_handle sntp_client;
/* Managed by main app*/
static QueueHandle_t out_queue;

static const char *TAG = "gnss_position_module";

static esp_err_t
init_gnss_module();

static void
gnss_data_event_handler(struct gnss_receiver_sentence sentence);

static void
task_gnss_sampling(void *arg);

static void
gnss_position_module_cleanup();

esp_err_t
gnss_position_module_init(struct gnss_position_module_config *module_cfg) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(module_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(module_cfg->out_queue, ESP_ERR_INVALID_ARG, TAG, "out_queue is invalid");
  ESP_RETURN_ON_FALSE(module_cfg->sntp_client, ESP_ERR_INVALID_ARG, TAG, "sntp_client is invalid");

  out_queue = module_cfg->out_queue;
  sntp_client = module_cfg->sntp_client;

  gnss_event_data_queue_handle = xQueueCreate(TASK_GNSS_SAMPLING_QUEUE_SIZE, sizeof(struct gnss_receiver_sentence));
  ESP_GOTO_ON_FALSE(gnss_event_data_queue_handle, ESP_ERR_NO_MEM, err, TAG, "failed to create gnss_event_data_queue_handle");

  ESP_GOTO_ON_ERROR(init_gnss_module(), err, TAG, "init_gnss_module failed");

  ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(task_gnss_sampling, "gnssTsk", 3072U, NULL, 6, &task_gnss_sampling_handle, 0U) ==
                        pdPASS,
                    ESP_FAIL, err, TAG, "failed to create gnssTsk");

  return ESP_OK;
err:
  gnss_position_module_cleanup();
  return ret;
}
esp_err_t
gnss_position_module_del() {
  gnss_position_module_stop();
  gnss_position_module_cleanup();
  ESP_LOGI(TAG, "Deleted GNSS module");
  return ESP_OK;
}

esp_err_t
gnss_position_module_start() {
  vTaskResume(task_gnss_sampling_handle);
  ESP_RETURN_ON_ERROR(gnss_receiver_start(gnss_receiver), TAG, "gnss_receiver_start failed");
  ESP_LOGI(TAG, "Started GNSS module");
  return ESP_OK;
}
esp_err_t
gnss_position_module_stop() {
  ESP_RETURN_ON_ERROR(gnss_receiver_stop(gnss_receiver), TAG, "gnss_receiver_stop failed");
  xTaskNotifyGive(task_gnss_sampling_handle);
  ESP_LOGI(TAG, "Stopped GNSS module");
  return ESP_OK;
}

static esp_err_t
init_gnss_module() {
  uart_config_t uart_config = {
      .baud_rate = 115000,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_RETURN_ON_ERROR(gnss_receiver_init(&uart_config, UART_NUM_1, GPIO_NUM_12, GPIO_NUM_13, &gnss_receiver), TAG,
                      "gnss_receiver_init failed");
  ESP_RETURN_ON_ERROR(gnss_receiver_register_event_handler(gnss_receiver, gnss_data_event_handler), TAG,
                      "gnss_receiver_register_event_handler failed");

  enum minmea_sentence_id sentences[] = {
      MINMEA_SENTENCE_RMC,
  };
  ESP_RETURN_ON_ERROR(
      gnss_receiver_update_parse_sentences(gnss_receiver, sentences, (sizeof(sentences) / sizeof(enum minmea_sentence_id))), TAG,
      "gnss_receiver_update_parse_sentences failed");
  ESP_RETURN_ON_ERROR(gnss_receiver_start(gnss_receiver), TAG, "gnss_receiver_start failed");

  ESP_LOGI(TAG, "Initialized GNSS module");

  return ESP_OK;
}

static void
task_gnss_sampling(void *arg) {
  sensor_payload_t payload;

  strncpy(payload.sensor, "GNSS", SENSOR_NAME_MAX_LEN - 1);
  payload.sensor[SENSOR_NAME_MAX_LEN - 1] = '\0';

  payload.field_count = 2U;

  strncpy(payload.fields[0].name, "lat", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[0].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[0].type = SENSOR_FIELD_DATATYPE_FLOAT;

  strncpy(payload.fields[1].name, "lon", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[1].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[1].type = SENSOR_FIELD_DATATYPE_FLOAT;

  // Getting a reliable fix from the GNSS module is problematic
  // Therefore a set of predetermined coordinates is used
  float latitudes[] = {
      59.422597f, 59.422790f, 59.422980f, 59.423212f, 59.423221f, 59.423208f, 59.422705f,
      59.422361f, 59.421739f, 59.421851f, 59.422329f, 59.422806f, 59.422705f, 59.422567f,
  };
  float longitudes[] = {
      17.830827f, 17.830010f, 17.829397f, 17.829352f, 17.828007f, 17.826988f, 17.826722f,
      17.826582f, 17.827448f, 17.828842f, 17.829555f, 17.829541f, 17.830358f, 17.830867f,
  };
  uint8_t num_points = sizeof(latitudes) / sizeof(float);

  struct gnss_receiver_sentence sentence;

  // Track the coordinate pairs
  uint16_t counter_points = 0U;

  // Track the interpolation process
  uint16_t counter_interpolation = 1U;
  uint16_t interpolation_points = 6U;
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, 0)) {
      vTaskSuspend(NULL);
    }

    payload.timestamp = (uint64_t)esp_timer_get_time() + sntp_client_get_boot_posix_time_us(sntp_client);

    float lambda = (float)counter_interpolation / (float)interpolation_points;

    uint16_t next_point = (counter_points + 1 < num_points) ? counter_points + 1 : 0U;
    float lat = (1 - lambda) * latitudes[counter_points] + lambda * latitudes[next_point];
    float lon = (1 - lambda) * longitudes[counter_points] + lambda * longitudes[next_point];

    payload.fields[0].value.f = lat;
    payload.fields[1].value.f = lon;

    if (counter_interpolation % interpolation_points == 0) {
      counter_interpolation = 1U;
      counter_points = next_point;
    } else {
      counter_interpolation++;
    }

    if (xQueueSend(out_queue, &payload, pdMS_TO_TICKS(TASK_QUEUE_SEND_TIMEOUT_MS)) != pdTRUE) {
      ESP_LOGE(TAG, "Failed sending from GNSS sampling task");
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_GNSS_SAMPLING_REPORT_PERIOD_MS));
  }
}

static void
gnss_data_event_handler(struct gnss_receiver_sentence sentence) {
  if (xQueueSend(gnss_event_data_queue_handle, &sentence, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to send gnss event data to a queue");
  }
}

static void
gnss_position_module_cleanup() {
  if (task_gnss_sampling_handle)
    vTaskDelete(task_gnss_sampling_handle);

  vQueueDelete(gnss_event_data_queue_handle);

  task_gnss_sampling_handle = NULL;
  gnss_event_data_queue_handle = NULL;
}
