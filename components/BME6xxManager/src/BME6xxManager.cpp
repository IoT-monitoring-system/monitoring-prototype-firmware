#include "BME6xxManager.h"
#include "BME6xxManagerErrors.h"

#include <utility>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG_BME6XX_MANAGER = "BME6xxManager";

#ifdef ESP_PLATFORM
static uint64_t
timestamp_millis() {
  return esp_timer_get_time() / 1000U;
}
#endif

/*!
 * @brief The constructor of the BME6xxManager class
 */
BME6xxManager::BME6xxManager() {
  if (configurator.initialize() != ESP_OK) {
    ESP_LOGE(TAG_BME6XX_MANAGER, "Configurator init failed");
    state = BMEMngrState::INVALID;
  }

  if (scheduler.initialize() != ESP_OK) {
    ESP_LOGE(TAG_BME6XX_MANAGER, "Scheduler init failed");
    state = BMEMngrState::INVALID;
  }

  state = BMEMngrState::INITIALIZED;
}

esp_err_t
BME6xxManager::addSensor(BME6xxSensor &sensorToAdd, bool performSelftest) {
  if (addedSensorsCounter >= MAX_BME6XX_UNITS) {
    return ESP_ERR_MAX_NUM_SENSORS;
  }

  uint32_t id = sensorToAdd.GetUniqueId();
  if (sensorToAdd.CheckStatus() != BME6xxStatus::OK)
    return ESP_ERR_BME6XX_DRIVER_ERROR;

  if (performSelftest) {
    ESP_LOGW(TAG_BME6XX_MANAGER, "Performing seltest check, sensor: %lu", id);
    sensorToAdd.SelftestCheck();
    if (sensorToAdd.CheckStatus() != BME6xxStatus::OK)
      return ESP_ERR_BME6XX_DRIVER_ERROR;
  }

  BMEMngrSensor sensor = BMEMngrSensor{.id = id,
                                       .device = &sensorToAdd,
                                       .config = BMESensorConfig{},
                                       .stateInfo = BMESensorStateInfo{},
                                       .scheduleInfo = BMESensorScheduleInfo{}};
  sensor.stateInfo.state = BMESensorState::INITIALIZED;

  sensors.push_back(std::move(sensor));

  return ESP_OK;
}

uint64_t
BME6xxManager::scheduleSensor() {
  return scheduler.scheduleSensor(sensors);
}

std::vector<BME6xxSensor *>
BME6xxManager::getSensors() const {
  std::vector<BME6xxSensor *> devices;
  for (const auto &sensor : sensors) {
    devices.push_back(sensor.device);
  }
  return devices;
}

esp_err_t
BME6xxManager::configure(FSFile &configFile) {
  if (state != BMEMngrState::INITIALIZED)
    return ESP_ERR_BME_MNGR_STATE_INVALID;

  if (!configFile.isValid())
    return ESP_ERR_CONFIG_FILE_ERROR;

  ESP_LOGD(TAG_BME6XX_MANAGER, "Loading configuration file: %s; File size: %lu", configFile.getFileName().c_str(),
           configFile.getFileData()->fileSize);

  esp_err_t err = ESP_OK;

  std::vector<BMESensorConfig> configurations(sensors.size());
  err = cfgSerializer.deserializeConfig(configFile, configurations);
  if (err != ESP_OK)
    return err;

  for (auto cfg : configurations) {
    ESP_LOGD(TAG_BME6XX_MANAGER, "Configuration for sensor:");
    ESP_LOGD(TAG_BME6XX_MANAGER, "Mode: %u", static_cast<uint8_t>(cfg.mode));

    ESP_LOGD(TAG_BME6XX_MANAGER,
             "Heater Profile: %s; Length: %u; Time Base: %lu; Scan Cycle Duration: "
             "%llu; ",
             cfg.heaterProfile.id.c_str(), cfg.heaterProfile.length, cfg.heaterProfile.timeBaseUs,
             cfg.heaterProfile.heatCycleDurationUs);

    for (uint8_t i = 0; i < cfg.heaterProfile.length; i++) {
      ESP_LOGD(TAG_BME6XX_MANAGER, "Step: %u; Temperature: %u; Time: %u", i, cfg.heaterProfile.temperature[i],
               cfg.heaterProfile.duration[i]);
    }

    ESP_LOGD(TAG_BME6XX_MANAGER,
             "Duty Cycle Profile: %s; Number Scan Cycles: %u; Number Sleep Cycles: "
             "%u; Total Sleep Duration: %llu",
             cfg.dutyCycleProfile.id.c_str(), cfg.dutyCycleProfile.numScans, cfg.dutyCycleProfile.numSleeps,
             cfg.dutyCycleProfile.sleepDurationUs);
  }

  for (uint8_t i = 0; i < sensors.size(); i++) {
    if (sensors[i].device == nullptr)
      return ESP_ERR_SENSOR_INDEX_ERROR;
    if (sensors[i].stateInfo.state == BMESensorState::INVALID)
      continue;

    err = resetSensor(sensors[i]);
    if (err != ESP_OK)
      return err;

    err = configurator.configureSensor(sensors[i], configurations[i]);
    if (err != ESP_OK)
      return err;

    err = configurator.setMode(sensors[i], BME6xxMode::SLEEP);
    if (err != ESP_OK)
      return err;

    sensors[i].stateInfo.state = BMESensorState::CONFIGURED;
  }

  state = BMEMngrState::CONFIGURED;

  return ESP_OK;
}

/*!
 * @brief This function configures the sensor manager using the provided config
 * file
 */
esp_err_t
BME6xxManager::begin() {
  if (state != BMEMngrState::CONFIGURED)
    return ESP_ERR_BME_MNGR_STATE_INVALID;

  for (uint8_t i = 0; i < sensors.size(); i++) {
    if (sensors[i].stateInfo.state != BMESensorState::CONFIGURED)
      continue;

    if (sensors[i].device->CheckStatus() != BME6xxStatus::OK)
      return ESP_ERR_BME6XX_DRIVER_ERROR;

    sensors[i].stateInfo.state = BMESensorState::RUNNING;
  }

  state = BMEMngrState::RUNNING;

  return ESP_OK;
}

esp_err_t
BME6xxManager::resetSensorState(BMEMngrSensor &sensor) {
  sensor.stateInfo.state = BMESensorState::INITIALIZED;
  return ESP_OK;
}
esp_err_t
BME6xxManager::resetSensorData(BMEMngrSensor &sensor) {
  memset(sensor.stateInfo.lastData, 0, sizeof(sensor.stateInfo.lastData));
  return ESP_OK;
}

esp_err_t
BME6xxManager::resetSensor(BME6xxSensor &sensor) {
  esp_err_t err = ESP_OK;

  auto it = std::find_if(sensors.begin(), sensors.end(), [&](const auto &s) { return s.id == sensor.GetUniqueId(); });

  if (it != sensors.end()) {
    err = configurator.setMode((*it), BME6xxMode::SLEEP);
    if (err != ESP_OK)
      return err;

    err = configurator.resetSensor((*it));
    if (err != ESP_OK)
      return err;

    err = scheduler.resetScheduleData((*it));
    if (err != ESP_OK)
      return err;

    err = resetSensorState((*it));
    if (err != ESP_OK)
      return err;
  } else {
    return ESP_ERR_SENSOR_NOT_FOUND;
  }

  return ESP_OK;
}
esp_err_t
BME6xxManager::resetSensor(BMEMngrSensor &sensor) {
  esp_err_t err = ESP_OK;

  err = configurator.resetSensor(sensor);
  if (err != ESP_OK)
    return err;

  err = scheduler.resetScheduleData(sensor);
  if (err != ESP_OK)
    return err;

  err = configurator.setMode(sensor, BME6xxMode::SLEEP);
  if (err != ESP_OK)
    return err;

  err = resetSensorState(sensor);
  if (err != ESP_OK)
    return err;

  return ESP_OK;
}

esp_err_t
BME6xxManager::sleepAll() {
  esp_err_t err = ESP_OK;
  if (sensors.empty())
    return ESP_ERR_NO_SENSORS;

  for (auto &sensor : sensors) {
    err = configurator.setMode(sensor, BME6xxMode::SLEEP);
    if (err != ESP_OK)
      return err;

    delay(100);

    uint8_t nFields = 0;
    nFields = sensor.device->FetchData();
    if (nFields) {
      sensor.device->GetAllData(sensor.stateInfo.lastData, nFields);
      ESP_LOGD(TAG_BME6XX_MANAGER, "Drained %u leftover measurements", nFields);
    }

    err = scheduler.resetScheduleData(sensor);
    if (err != ESP_OK)
      return err;

    err = resetSensorData(sensor);
    if (err != ESP_OK)
      return err;

    err = scheduler.updateWakeUp(sensor, esp_timer_get_time(), 0U);
    if (err != ESP_OK)
      return err;
  }
  return ESP_OK;
}

/*!
 * @brief This function retrieves the selected sensor data
 */
esp_err_t
BME6xxManager::collectData(BMESensorData &sensData) {
  if (state != BMEMngrState::RUNNING)
    return ESP_ERR_BME_MNGR_STATE_INVALID;

  esp_err_t err = ESP_OK;

  BMEMngrSensor *sensor;
  err = scheduler.getLastScheduledSensor(sensors, &sensor);
  if (err != ESP_OK)
    return err;

  if (NULL == sensor) {
    ESP_LOGW(TAG_BME6XX_MANAGER, "Sensor object is invalid");
    return ESP_FAIL;
  }

  if (sensor->stateInfo.state != BMESensorState::RUNNING)
    return ESP_ERR_SENSOR_STATE_INVALID;

  resetSensorData(*sensor);
  sensData.sensorId = sensor->id;

  if (sensor->config.mode == BME6xxMode::SLEEP) {
    err = configurator.setMode(*sensor, BME6xxMode::PARALLEL);
    if (err != ESP_OK)
      return err;
    sensor->last_meas_timestamp_us = (uint64_t)esp_timer_get_time();

    scheduler.scheduleWakeUp(*sensor, 0, (uint64_t)esp_timer_get_time(), configurator);

    return err;
  }

  uint8_t nFields, j = 0;
  nFields = sensor->device->FetchData();
  sensor->device->GetAllData(sensor->stateInfo.lastData, nFields);

  for (uint8_t i = 0; i < nFields; ++i) {
    auto &data = sensor->stateInfo.lastData[i];

    if (!(data.status & GASM_VALID_MSK))
      continue;

    uint8_t currentGasIdx = data.gas_index;

    if (currentGasIdx != sensor->scheduleInfo.heaterIndex) {
      ESP_LOGW(TAG_BME6XX_MANAGER, "Heater index mismatch: got %u, expected %u", currentGasIdx, sensor->scheduleInfo.heaterIndex);
      continue;
    }

    uint64_t expected_time_us = sensor->config.heaterProfile.duration[currentGasIdx] * sensor->config.heaterProfile.timeBaseUs;
    uint64_t actual_time_us = data.meas_timestamp - sensor->last_meas_timestamp_us;

    ESP_LOGD(TAG_BME6XX_MANAGER, "Measurement valid, idx:%u, expected:%llu us, actual:%llu us, diff:%llu", currentGasIdx,
             expected_time_us, actual_time_us,
             (actual_time_us > expected_time_us) ? (actual_time_us - expected_time_us) : (expected_time_us - actual_time_us));

    sensData.data[j++] = data;
    sensData.type = sensor->device->GetType();
    sensor->last_meas_timestamp_us = data.meas_timestamp;

    uint64_t time_now_us = esp_timer_get_time();

    scheduler.scheduleWakeUp(*sensor, currentGasIdx + 1, time_now_us, configurator);
  }

  sensData.dataLen = j;

  if (nFields == 0 || j == 0)
    err = ESP_ERR_SENSOR_NO_NEW_DATA;

  return err;
}
