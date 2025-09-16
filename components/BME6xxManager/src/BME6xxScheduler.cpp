#include "BME6xxScheduler.h"
#include "BME6xxManagerErrors.h"

#include "esp_log.h"

#ifdef ESP_PLATFORM
static uint64_t
timestamp_millis() {
  return esp_timer_get_time() / 1000U;
}
#endif

#define MAX_HEATER_DURATION 200U
#define GAS_WAIT_SHARED     120U

#define SENSOR_WAKE_UP_TIME_OFFSET_US 5000U

static const char *TAG_BME6XX_SCHEDULER = "BME6xxScheduler";

static inline uint64_t
calcWakeMargin(uint64_t durationUs);

esp_err_t
BME6xxScheduler::initialize() {
  return ESP_OK;
}

esp_err_t
BME6xxScheduler::resetScheduleData(BMEMngrSensor &sensor) {
  sensor.scheduleInfo.dutyCycleIndex = 0;
  sensor.scheduleInfo.heaterIndex = 0;
  sensor.scheduleInfo.wakeUpTimeUs = 0;

  return ESP_OK;
}

bool
BME6xxScheduler::selectNextSensor(std::vector<BMEMngrSensor> &sensors, BME6xxMode mode, uint64_t &wakeUpTime, uint8_t &num) {
  if (sensors.empty())
    return false;

  for (uint8_t i = 0; i < sensors.size(); ++i) {
    if ((sensors[i].config.mode == mode) && (sensors[i].scheduleInfo.wakeUpTimeUs < wakeUpTime)) {
      wakeUpTime = sensors[i].scheduleInfo.wakeUpTimeUs;
      num = i;
    }
  }

  if (num < sensors.size()) {
    return true;
  }
  return false;
}

esp_err_t
BME6xxScheduler::updateWakeUp(BMEMngrSensor &sensor, uint64_t wakeUpTimeUs, uint8_t nextHeaterIndex) {
  sensor.scheduleInfo.heaterIndex = nextHeaterIndex;
  sensor.scheduleInfo.wakeUpTimeUs = wakeUpTimeUs;
  return ESP_OK;
}

uint64_t
BME6xxScheduler::scheduleSensor(std::vector<BMEMngrSensor> &sensors) {
  if (sensors.empty())
    return 0U;

  uint64_t wakeUpTimeUs = sensors[lastScheduledSensor].scheduleInfo.wakeUpTimeUs;
  selectNextSensor(sensors, BME6xxMode::PARALLEL, wakeUpTimeUs, lastScheduledSensor);
  selectNextSensor(sensors, BME6xxMode::SLEEP, wakeUpTimeUs, lastScheduledSensor);

  return wakeUpTimeUs;
}

esp_err_t
BME6xxScheduler::scheduleWakeUp(BMEMngrSensor &sensor, uint8_t nextHeaterIndex, uint64_t lastSampleTimeUs,
                                  BME6xxConfigurator &configurator) {
  esp_err_t err = ESP_OK;

  uint64_t conversionDurationUs = sensor.config.heaterProfile.duration[nextHeaterIndex % sensor.config.heaterProfile.length] *
                                      sensor.config.heaterProfile.timeBaseUs +
                                  SENSOR_WAKE_UP_TIME_OFFSET_US;

  uint64_t marginUs = calcWakeMargin(conversionDurationUs);

  uint64_t nextWakeUpUs = lastSampleTimeUs + conversionDurationUs - marginUs;

  if (nextHeaterIndex == sensor.config.heaterProfile.length) {
    nextHeaterIndex = 0;
    sensor.scheduleInfo.dutyCycleIndex++;

    ESP_LOGD(TAG_BME6XX_SCHEDULER, "Sensor: %lu; Scan cycle finished, number scanning cycles %u/%u", sensor.id,
             sensor.scheduleInfo.dutyCycleIndex, sensor.config.dutyCycleProfile.numScans);

    if (sensor.scheduleInfo.dutyCycleIndex >= sensor.config.dutyCycleProfile.numScans) {
      sensor.scheduleInfo.dutyCycleIndex = 0;

      if (sensor.config.dutyCycleProfile.sleepDurationUs > 0U) {
        marginUs = calcWakeMargin(sensor.config.dutyCycleProfile.sleepDurationUs);
        nextWakeUpUs = lastSampleTimeUs + sensor.config.dutyCycleProfile.sleepDurationUs - marginUs;
        ESP_LOGD(TAG_BME6XX_SCHEDULER, "Sensor: %lu; Sleeping for %llu us; Wake up time: %llu", sensor.id,
                 sensor.config.dutyCycleProfile.sleepDurationUs, nextWakeUpUs);

        err = configurator.setMode(sensor, BME6xxMode::SLEEP);
        if (err != ESP_OK)
          return err;
      }
    }
  }
  err = updateWakeUp(sensor, nextWakeUpUs, nextHeaterIndex);
  return err;
}

esp_err_t
BME6xxScheduler::getLastScheduledSensor(std::vector<BMEMngrSensor> &sensors, BMEMngrSensor **sensOut) {
  if (sensors.empty())
    return ESP_ERR_INVALID_ARG;

  if (lastScheduledSensor >= sensors.size())
    return ESP_ERR_SENSOR_INDEX_ERROR;

  *sensOut = &sensors[lastScheduledSensor];

  if (NULL == sensOut)
    return ESP_ERR_SENSOR_INDEX_ERROR;

  return ESP_OK;
}

static inline uint64_t
calcWakeMargin(uint64_t durationUs) {
  uint64_t margin = durationUs * 0.1f;
  if (margin < 2000ULL)
    margin = 2000ULL;
  if (margin > 200000ULL)
    margin = 200000ULL;
  return margin;
}
