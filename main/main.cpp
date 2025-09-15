#include "float.h"
#include "math.h"
#include "stdio.h"

#include "sdkconfig.h"

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "Arduino.h"
#include "LittleFS.h"
#include "WiFi.h"
#include "Wire.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "FSManager.h"

#include "sntp_client.h"

#include "peripheral_config.h"
#include "sntp_config.h"
#include "wifi_config.h"

#include "app_errors.h"

#include "bme_sample_module.h"
#include "fall_detect_module.h"
#include "gnss_position_module.h"
#include "mqtt_transmission_module.h"

static const char *TAG = "monitoring_node";

FSManager fs_manager;

sntp_client_handle sntp_client;

QueueHandle_t data_aggregation_queue_handle;

esp_err_t
init_i2c();
esp_err_t
init_file_system();
esp_err_t
init_wifi();
esp_err_t
init_sntp();

extern "C" void
app_main() {
  initArduino();
  vTaskDelay(pdMS_TO_TICKS(1000));

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());

  vTaskDelay(pdMS_TO_TICKS(1000));

  init_i2c();
  init_file_system();

  init_wifi();
  init_sntp();

  mqtt_transmission_module_init(&data_aggregation_queue_handle);

  bme_sample_module_config bme_cfg = {
      .out_queue = data_aggregation_queue_handle,
      .wire = &Wire,
      .fs_manager = &fs_manager,
      .sntp_client = sntp_client,
  };
  bme_sample_module_init(&bme_cfg);

  gnss_position_module_config gnss_cfg = {
      .out_queue = data_aggregation_queue_handle,
      .sntp_client = sntp_client,
  };
  gnss_position_module_init(&gnss_cfg);

  fall_detect_module_config fall_cfg = {
      .out_queue = data_aggregation_queue_handle,
      .wire = &Wire,
      .sntp_client = sntp_client,
  };
  fall_detect_module_init(&fall_cfg);

  vTaskDelay(pdMS_TO_TICKS(1000U));

  mqtt_transmission_module_start();
  bme_sample_module_start();
  gnss_position_module_start();
  fall_detect_module_start();
}

esp_err_t
init_i2c() {
  Wire.end();

  ESP_RETURN_ON_FALSE(Wire.begin(I2C_SDA_BUS0, I2C_SCL_BUS0, I2C_FREQ_BUS0), ESP_ERR_MAIN_APP_I2C_FAIL, TAG,
                      "Failed to initialize I2C bus 0");

  ESP_RETURN_ON_FALSE(Wire1.begin(I2C_SDA_BUS1, I2C_SCL_BUS1, I2C_FREQ_BUS1), ESP_ERR_MAIN_APP_I2C_FAIL, TAG,
                      "Failed to initialize I2C bus 1");

  ESP_LOGD(TAG, "Initialized I2C");
  return ESP_OK;
}
esp_err_t
init_file_system() {
  ESP_RETURN_ON_FALSE(LittleFS.begin(true, "/littlefs", 10, "littlefs"), ESP_ERR_MAIN_APP_LITTLEFS_FAIL, TAG,
                      "Failed to start LittleFS");

  ESP_RETURN_ON_ERROR(fs_manager.initializeFileSystem(LittleFS, FSManager::FS_MANAGER_LITTLE_FS), TAG,
                      "Failed to initialize FSManager");

  ESP_LOGD(TAG, "Initialized LittleFS");

  return ESP_OK;
}
esp_err_t
init_wifi() {
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();

    WiFi.mode(WIFI_STA);
    WiFi.enableSTA(true);
    WiFi.setSleep(WIFI_PS_MAX_MODEM);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    vTaskDelay(pdMS_TO_TICKS(5000));
  }

  ESP_LOGD(TAG, "Connected to a WiFi Access Point");

  return ESP_OK;
}
esp_err_t
init_sntp() {
  sntp_client_config sntp_cfg = {
      .server = SNTP_SERVER_1,
  };
  ESP_RETURN_ON_ERROR(sntp_client_init(&sntp_cfg, &sntp_client), TAG, "Failed to initialize SNTP client");
  ESP_RETURN_ON_ERROR(sntp_client_start(sntp_client), TAG, "Failed to start SNTP client");
  ESP_RETURN_ON_ERROR(sntp_client_sync_wait(sntp_client, SNTP_SYNC_WAIT_MS), TAG, "Failed to sync SNTP client");

  ESP_LOGD(TAG, "Initialized SNTP module");
  return ESP_OK;
}
