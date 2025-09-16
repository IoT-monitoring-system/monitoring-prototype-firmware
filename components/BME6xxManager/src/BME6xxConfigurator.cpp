#include "BME6xxConfigurator.h"
#include "esp_log.h"

const char *TAG_BME6XX_CFG_CONFIGURATOR = "BME6xxConfigurator";

esp_err_t
BME6xxConfigurator::checkStatus(BME6xxSensor &sensor) {
  BME6xxStatus status = sensor.CheckStatus();

  ESP_LOGD(TAG_BME6XX_CFG_CONFIGURATOR, "Sensor status: %i", static_cast<int8_t>(status));

  if (status != BME6xxStatus::OK)
    return ESP_FAIL;

  return ESP_OK;
}

BME6xxConfigurator::BME6xxConfigurator() {
}

esp_err_t
BME6xxConfigurator::setMode(BMEMngrSensor &sensor, BME6xxMode mode) {
  esp_err_t err = ESP_OK;

  sensor.device->SetOperationMode(mode);
  err = checkStatus(*sensor.device);
  if (err != ESP_OK)
    return ESP_FAIL;

  sensor.config.mode = mode;

  return ESP_OK;
}

esp_err_t
BME6xxConfigurator::initialize() {
  return ESP_OK;
}

esp_err_t
BME6xxConfigurator::configureSensor(BMEMngrSensor &sensor, BMESensorConfig &config) {
  esp_err_t err = ESP_OK;

  err = configureOS(sensor, config.bme6xxDevCfg.os);
  if (err != ESP_OK)
    return err;

  err = configureHeaterProfile(sensor, config.heaterProfile);
  if (err != ESP_OK)
    return err;

  sensor.config = config;

  return err;
}
esp_err_t
BME6xxConfigurator::resetSensor(BMEMngrSensor &sensor) {
  esp_err_t err = ESP_OK;

  sensor.device->SoftReset();

  err = checkStatus(*sensor.device);
  if (err != ESP_OK)
    return err;

  err = resetHeaterProfile(sensor);
  if (err != ESP_OK)
    return err;

  return err;
}

esp_err_t
BME6xxConfigurator::configureHeaterProfile(BMEMngrSensor &sensor, BMEHeaterProfile &profile) {
  uint16_t sharedHeatrDurMs =
      (uint16_t)((profile.timeBaseUs - sensor.device->GetMeasurementDuration(sensor.config.mode)) / 1000U);

  sensor.device->SetHeaterProfile(profile.temperature, profile.duration, sharedHeatrDurMs, profile.length);

  return checkStatus(*sensor.device);
}
esp_err_t
BME6xxConfigurator::resetHeaterProfile(BMEMngrSensor &sensor) {
  BMEDutyCycleProfile newBMEDutyCycleProf = BMEDutyCycleProfile{};
  BMEHeaterProfile newBMEHeaterProf = BMEHeaterProfile{};

  sensor.config.dutyCycleProfile = newBMEDutyCycleProf;
  sensor.config.heaterProfile = newBMEHeaterProf;

  return ESP_OK;
}

esp_err_t
BME6xxConfigurator::configureOS(BMEMngrSensor &sensor, BME6xxOS &os) {
  sensor.device->SetOversampling(os);

  return checkStatus(*sensor.device);
}
esp_err_t
BME6xxConfigurator::resetOS(BMEMngrSensor &sensor) {
  sensor.config.bme6xxDevCfg.os = BME6xxOS{};

  return ESP_OK;
}