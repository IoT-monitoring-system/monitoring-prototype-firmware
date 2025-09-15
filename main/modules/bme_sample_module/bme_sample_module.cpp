#include "float.h"
#include "math.h"
#include "stdio.h"

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "Arduino.h"
#include "Wire.h"

#include "cbor_sensor_encoder.h"

#include "FSManager.h"

#include "BME6xxManager.h"
#include "BME6xxManagerErrors.h"
#include "bme69xLibrary.h"

#include "bme_sample_module.h"
#include "config.h"

static BME69x bme690;
static BME6xxManager bme6xx_manager;

static TaskHandle_t task_bme6xx_sampling_handle;

/* Managed by main app*/
static sntp_client_handle sntp_client;
/* Managed by main app*/
static QueueHandle_t out_queue;

static const char *TAG = "bme_sample_module";

static esp_err_t
init_bme6xx(TwoWire *wire);

static esp_err_t
init_bme_manager(FSManager *fs_manager);

static void
task_bme6xx_sampling(void *arg);

static void
bme_sample_module_cleanup();

esp_err_t
bme_sample_module_init(bme_sample_module_config *module_cfg) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(module_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(module_cfg->out_queue, ESP_ERR_INVALID_ARG, TAG, "out_queue is invalid");
  ESP_RETURN_ON_FALSE(module_cfg->wire, ESP_ERR_INVALID_ARG, TAG, "wire is invalid");
  ESP_RETURN_ON_FALSE(module_cfg->sntp_client, ESP_ERR_INVALID_ARG, TAG, "sntp_client is invalid");

  out_queue = module_cfg->out_queue;
  sntp_client = module_cfg->sntp_client;

  ESP_GOTO_ON_ERROR(init_bme6xx(module_cfg->wire), err, TAG, "failed to initialize BME6XX sensor");
  ESP_GOTO_ON_ERROR(init_bme_manager(module_cfg->fs_manager), err, TAG, "failed to initialize BME Manager");

  ESP_GOTO_ON_FALSE(
      xTaskCreatePinnedToCore(task_bme6xx_sampling, "bme6xxTsk", 3072U, NULL, 6, &task_bme6xx_sampling_handle, 0U) == pdPASS,
      ESP_FAIL, err, TAG, "failed to create bme6xxTsk");

  return ESP_OK;

err:
  bme_sample_module_cleanup();
  return ret;
}
esp_err_t
bme_sample_module_del() {
  bme_sample_module_stop();
  bme_sample_module_cleanup();

  ESP_LOGI(TAG, "Deleted BME Sample module");

  return ESP_OK;
}

esp_err_t
bme_sample_module_start() {
  vTaskResume(task_bme6xx_sampling_handle);

  ESP_LOGI(TAG, "Started BME Sample module");

  return ESP_OK;
}
esp_err_t
bme_sample_module_stop() {
  bme6xx_manager.sleepAll();

  xTaskNotifyGive(task_bme6xx_sampling_handle);

  ESP_LOGI(TAG, "Stopped BME Sample module");

  return ESP_OK;
}

static esp_err_t
init_bme6xx(TwoWire *wire) {
  bme690.begin(BME690_I2C_ADDRESS, *wire);
  ESP_RETURN_ON_FALSE(static_cast<BME6xxStatus>(bme690.status) == BME6xxStatus::OK, ESP_FAIL, TAG,
                      "bme690.begin() failed, code: %i", bme690.status);

  ESP_LOGI(TAG, "Initialized BME690");

  return ESP_OK;
}
static esp_err_t
init_bme_manager(FSManager *fs_manager) {
  std::unique_ptr<FSFile> bmeCfg = std::unique_ptr<FSFile>(fs_manager->readFile("/x8default_1sens.bmeconfig"));
  ESP_RETURN_ON_FALSE(bmeCfg, ESP_FAIL, TAG, "Invalid BME config");

  ESP_RETURN_ON_ERROR(bme6xx_manager.addSensor(bme690, true), TAG, "Failed to add a bme690 sensor");

  ESP_RETURN_ON_ERROR(bme6xx_manager.configure(*bmeCfg), TAG, "Failed to configure BMEManager");
  ESP_RETURN_ON_ERROR(bme6xx_manager.begin(), TAG, "Failed to start BMEManager");

  ESP_LOGI(TAG, "bmeManager Initialized");

  return ESP_OK;
}
static void
task_bme6xx_sampling(void *arg) {
  sensor_payload_t payload;

  strncpy(payload.fields[0].name, "sens_id", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[0].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[0].type = SENSOR_FIELD_DATATYPE_UINT;

  strncpy(payload.fields[1].name, "gas_idx", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[1].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[1].type = SENSOR_FIELD_DATATYPE_UINT;

  strncpy(payload.fields[2].name, "gas_res", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[2].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[2].type = SENSOR_FIELD_DATATYPE_FLOAT;

  payload.field_count = 3U;

  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, 0)) {
      vTaskSuspend(NULL);
    }

    while (bme6xx_manager.scheduleSensor()) {
      BMESensorData sensor_data;
      esp_err_t ret = bme6xx_manager.collectData(sensor_data);
      if (ret != ESP_OK) {
        break;
      }

      strncpy(payload.sensor, sensor_data.type, SENSOR_NAME_MAX_LEN - 1); // Dynamic
      payload.sensor[SENSOR_NAME_MAX_LEN - 1] = '\0';

      for (uint8_t i = 0; i < sensor_data.dataLen; ++i) {
        payload.timestamp = (uint64_t)(sensor_data.data[i].meas_timestamp + sntp_client_get_boot_posix_time_us(sntp_client));

        payload.fields[0].value.u = sensor_data.sensorId;               /* sens_id */
        payload.fields[1].value.u = sensor_data.data[i].gas_index;      /* gas index */
        payload.fields[2].value.f = sensor_data.data[i].gas_resistance; /* gas res */

        if (xQueueSend(out_queue, &payload, pdMS_TO_TICKS(TASK_QUEUE_SEND_TIMEOUT_MS)) != pdTRUE) {
          ESP_LOGE(TAG, "Failed sending from BME sampling task");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(65));
  }
}

static void
bme_sample_module_cleanup() {
  if (task_bme6xx_sampling_handle)
    vTaskDelete(task_bme6xx_sampling_handle);

  task_bme6xx_sampling_handle = NULL;
}