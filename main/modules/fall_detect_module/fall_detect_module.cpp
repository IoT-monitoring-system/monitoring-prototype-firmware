#include "float.h"
#include "math.h"
#include "stdio.h"

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "Arduino.h"
#include "Wire.h"

#include "cbor_sensor_encoder.h"

#include "SparkFun_ADXL345.h"

#include "config.h"
#include "fall_detect_module.h"

typedef struct {
  int x;
  int y;
  int z;
  uint64_t timestamp;
} adxl345_data;

static ADXL345 adxl345;
static TaskHandle_t task_fall_detection_handle;
static TaskHandle_t task_adxl345_sampling_handle;
static QueueHandle_t fall_detection_queue_handle;
static SemaphoreHandle_t fall_detect_adxl_sample_sync_semphr_handle;

/* Managed by main app*/
static sntp_client_handle sntp_client;
/* Managed by main app*/
static QueueHandle_t out_queue;

static const char *TAG = "fall_detect_module";

static esp_err_t
init_adxl345(TwoWire *wire);

static void
task_fall_detection(void *arg);

static void
task_adxl345_sampling(void *arg);

static bool
process_window(adxl345_data *sensor_data, uint8_t win_size, uint8_t offset);

static void
fall_detect_module_cleanup();

esp_err_t
fall_detect_module_init(fall_detect_module_config *module_cfg) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(module_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(module_cfg->out_queue, ESP_ERR_INVALID_ARG, TAG, "out_queue is invalid");
  ESP_RETURN_ON_FALSE(module_cfg->sntp_client, ESP_ERR_INVALID_ARG, TAG, "sntp_client is invalid");
  ESP_RETURN_ON_FALSE(module_cfg->wire, ESP_ERR_INVALID_ARG, TAG, "wire is invalid");

  out_queue = module_cfg->out_queue;
  sntp_client = module_cfg->sntp_client;

  fall_detection_queue_handle = xQueueCreate(TASK_FALL_DETECTION_DATA_WINDOW_SIZE, sizeof(adxl345_data));
  ESP_GOTO_ON_FALSE(fall_detection_queue_handle, ESP_ERR_NO_MEM, err, TAG, "failed to create fall_detection_queue");

  fall_detect_adxl_sample_sync_semphr_handle = xSemaphoreCreateBinary();
  ESP_GOTO_ON_FALSE(fall_detect_adxl_sample_sync_semphr_handle, ESP_ERR_NO_MEM, err, TAG,
                    "failed to create fall_detect_adxl_sample_sync_semphr");

  ESP_GOTO_ON_ERROR(init_adxl345(module_cfg->wire), err, TAG, "init_adxl345 failed");

  ESP_GOTO_ON_FALSE(
      xTaskCreatePinnedToCore(task_fall_detection, "fallDetectTsk", 3072U, NULL, 7, &task_fall_detection_handle, 0U) == pdPASS,
      ESP_FAIL, err, TAG, "failed to create fallDetectTsk");

  ESP_GOTO_ON_FALSE(
      xTaskCreatePinnedToCore(task_adxl345_sampling, "adxlSampling", 3072U, NULL, 6, &task_adxl345_sampling_handle, 0U) == pdPASS,
      ESP_FAIL, err, TAG, "failed to create adxlSampling");

  return ESP_OK;
err:
  fall_detect_module_cleanup();
  return ret;
}
esp_err_t
fall_detect_module_del() {
  fall_detect_module_stop();
  fall_detect_module_cleanup();

  ESP_LOGI(TAG, "Deleted Fall Detect module");

  return ESP_OK;
}

esp_err_t
fall_detect_module_start() {
  xSemaphoreGive(fall_detect_adxl_sample_sync_semphr_handle);
  vTaskResume(task_adxl345_sampling_handle);

  ESP_LOGI(TAG, "Started Fall Detect module");

  return ESP_OK;
}
esp_err_t
fall_detect_module_stop() {
  xTaskNotifyGive(task_adxl345_sampling_handle);

  ESP_LOGI(TAG, "Stopped Fall Detect module");

  return ESP_OK;
}

static esp_err_t
init_adxl345(TwoWire *wire) {
  ESP_RETURN_ON_FALSE(adxl345.begin(ADXL345_I2C_ADDR_HIGH, *wire), ESP_FAIL, TAG, "Failed to initialize ADXL345 sensor");
  ESP_RETURN_ON_FALSE(adxl345.powerOn(), ESP_FAIL, TAG, "Failed to power on ADXL345 sensor");
  adxl345.setRangeSetting(16);
  adxl345.setFullResBit(true);
  adxl345.set_bw(0b00001000);
  adxl345.setLowPower(true);

  ESP_LOGI(TAG, "ADXL345 Initialized");

  return ESP_OK;
}

static void
task_adxl345_sampling(void *arg) {
  xSemaphoreGive(fall_detect_adxl_sample_sync_semphr_handle);
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, 0)) {
      xSemaphoreTake(fall_detect_adxl_sample_sync_semphr_handle, portMAX_DELAY);
      vTaskSuspend(NULL);
    }

    int x, y, z;
    adxl345.readAccel(&x, &y, &z);

    adxl345_data data = {
        .x = x,
        .y = y,
        .z = z,
        .timestamp = (uint64_t)esp_timer_get_time(),
    };

    xQueueSend(fall_detection_queue_handle, &data, 0U);

    vTaskDelay(pdMS_TO_TICKS(TASK_ADXL345_SAMPLING_PERIOD_MS));
  }
}

static void
task_fall_detection(void *arg) {
  sensor_payload_t payload;

  strncpy(payload.sensor, "alerts", SENSOR_NAME_MAX_LEN - 1);
  payload.sensor[SENSOR_NAME_MAX_LEN - 1] = '\0';

  strncpy(payload.fields[0].name, "fall", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[0].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[0].type = SENSOR_FIELD_DATATYPE_UINT;

  payload.field_count = 1U;

  uint64_t time_last_report_us = (uint64_t)esp_timer_get_time();

  bool fall_detect_status = false;

  adxl345_data sensor_data_window[TASK_FALL_DETECTION_DATA_WINDOW_SIZE];
  uint8_t idx = 0U;
  for (;;) {
    xSemaphoreTake(fall_detect_adxl_sample_sync_semphr_handle, portMAX_DELAY);

    adxl345_data new_sample;
    if (xQueueReceive(fall_detection_queue_handle, &new_sample, portMAX_DELAY) == pdTRUE) {
      sensor_data_window[idx] = new_sample;
      idx = (idx + 1) % TASK_FALL_DETECTION_DATA_WINDOW_SIZE;

      // process_window(sensor_data_window, TASK_FALL_DETECTION_DATA_WINDOW_SIZE, idx);
      uint64_t time_now_us = (uint64_t)esp_timer_get_time();
      if (time_now_us - time_last_report_us >= (TASK_FALL_DETECTION_REPORT_PERIOD_MS * 1000U)) {
        payload.timestamp = time_now_us + sntp_client_get_boot_posix_time_us(sntp_client);

        fall_detect_status = !fall_detect_status;

        payload.fields[0].value.u = (uint8_t)fall_detect_status;

        time_last_report_us = time_now_us;

        if (xQueueSend(out_queue, &payload, pdMS_TO_TICKS(TASK_QUEUE_SEND_TIMEOUT_MS)) != pdTRUE) {
          ESP_LOGE(TAG, "Failed sending from fall detection task");
        }
      }
    }

    xSemaphoreGive(fall_detect_adxl_sample_sync_semphr_handle);
  }
}

static bool
process_window(adxl345_data *sensor_data, uint8_t win_size, uint8_t offset) {
  adxl345_data ordered[TASK_FALL_DETECTION_DATA_WINDOW_SIZE];
  for (int i = 0; i < TASK_FALL_DETECTION_DATA_WINDOW_SIZE; i++) {
    int real_idx = (offset + i) % TASK_FALL_DETECTION_DATA_WINDOW_SIZE;
    ordered[i] = sensor_data[real_idx];
  }

  return false;
  // Process window
}

static void
fall_detect_module_cleanup() {
  if (task_fall_detection_handle)
    vTaskDelete(task_fall_detection_handle);
  if (task_adxl345_sampling_handle)
    vTaskDelete(task_adxl345_sampling_handle);

  vSemaphoreDelete(fall_detect_adxl_sample_sync_semphr_handle);
  vQueueDelete(fall_detection_queue_handle);

  task_fall_detection_handle = NULL;
  task_adxl345_sampling_handle = NULL;
  fall_detection_queue_handle = NULL;
  fall_detect_adxl_sample_sync_semphr_handle = NULL;
}
