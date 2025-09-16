#pragma once
#ifndef BME6XXMANAGER_H
#define BME6XXMANAGER_H

#include "esp_system.h"

#include "Arduino.h"
#include "ArduinoJson.h"
#include "LittleFS.h"
#include "SPI.h"
#include "Wire.h"

#include "FS.h"

#include "BME6xxManagerTypes.h"

#include "BME6xxCfgSerializer.h"
#include "BME6xxConfigurator.h"
#include "BME6xxScheduler.h"

#include "FSFile.h"

class BME6xxManager {
private:
  BME6xxConfigurator configurator;
  BME6xxScheduler scheduler;
  BME6xxCfgSerializer cfgSerializer;

  BMEMngrState state = BMEMngrState::STATE_DEFAULT;

  std::vector<BMEMngrSensor> sensors;
  uint8_t addedSensorsCounter = 0;

  esp_err_t
  resetSensorState(BMEMngrSensor &sensor);
  esp_err_t
  resetSensorData(BMEMngrSensor &sensor);

public:
  BME6xxManager();

  BMEMngrState
  getState();

  esp_err_t
  addSensor(BME6xxSensor &sensor, bool performSelftest);

  esp_err_t
  configure(FSFile &configFile);
  esp_err_t
  begin();

  esp_err_t
  resetSensor(BME6xxSensor &sensor);
  esp_err_t
  resetSensor(BMEMngrSensor &sensor);

  uint64_t
  scheduleSensor();

  std::vector<BME6xxSensor *>
  getSensors() const;

  esp_err_t
  sleepAll();

  esp_err_t
  collectData(BMESensorData &data);
};

#endif
