#pragma once
#ifndef BME6XXSCHEDULER_H
#define BME6XXSCHEDULER_H

#include <vector>

#include "esp_system.h"

#include "BME6xxConfigurator.h"
#include "BME6xxManagerTypes.h"

class BME6xxScheduler {
public:
  esp_err_t
  initialize();

  esp_err_t
  resetScheduleData(BMEMngrSensor &sensor);

  esp_err_t
  updateWakeUp(BMEMngrSensor &sensor, uint64_t wakeUpTime, uint8_t nextHeaterIndex);

  uint64_t
  scheduleSensor(std::vector<BMEMngrSensor> &sensors);
  esp_err_t
  scheduleWakeUp(BMEMngrSensor &sensor, uint8_t currentHeaterIndex, uint64_t lastMeasurementTimestampUs,
                   BME6xxConfigurator &configurator);

  esp_err_t
  getLastScheduledSensor(std::vector<BMEMngrSensor> &sensors, BMEMngrSensor **sensOut);

private:
  uint8_t lastScheduledSensor = 0;

  bool
  selectNextSensor(std::vector<BMEMngrSensor> &sensors, BME6xxMode mode, uint64_t &wakeUpTime, uint8_t &num);
};

#endif
