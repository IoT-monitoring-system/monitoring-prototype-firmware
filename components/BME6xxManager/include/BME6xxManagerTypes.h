#pragma once
#ifndef BME6XXMANAGERTYPES_H
#define BME6XXMANAGERTYPES_H

#include <string>

#include "BME6xxSensor.h"

#define MAX_BME6XX_UNITS 8U

enum class BMEIntfType {
  BME_INTF_SPI,
  BME_INTF_I2C,
  BME_INTF_MUX,
};

enum class BMEMngrState {
  STATE_DEFAULT,
  INVALID,

  INITIALIZED,
  CONFIGURED,
  RUNNING,
};

enum class BMESensorState {
  STATE_DEFAULT,
  INVALID,

  INITIALIZED,
  CONFIGURED,
  RUNNING,
};

/*!
 * @brief Structure to hold heater profile data
 */
struct BMEHeaterProfile {
  std::string id;
  uint32_t timeBaseUs = 140000U;
  uint16_t temperature[10];
  uint16_t duration[10];
  uint64_t heatCycleDurationUs;
  uint8_t length;
};

struct BMEDutyCycleProfile {
  std::string id;
  uint8_t numScans;
  uint8_t numSleeps;
  uint64_t sleepDurationUs;
};

struct BMESensorConfig {
  BMEHeaterProfile heaterProfile;
  BMEDutyCycleProfile dutyCycleProfile;
  BME6xxConf bme6xxDevCfg;
  BME6xxMode mode = BME6xxMode::SLEEP;
};

struct BMESensorScheduleInfo {
  uint64_t wakeUpTimeUs;
  uint8_t dutyCycleIndex;
  uint8_t heaterIndex;
};

struct BMESensorStateInfo {
  BMESensorState state = BMESensorState::INVALID;
  BME6xxData lastData[3];
};

/*!
 * @brief Structure to hold sensor state information
 */
struct BMEMngrSensor {
  uint32_t id;
  BME6xxSensor *device;
  BMESensorConfig config;
  BMESensorStateInfo stateInfo;
  BMESensorScheduleInfo scheduleInfo;

  uint64_t last_meas_timestamp_us;
};

struct BMESensorData {
  uint32_t sensorId;
  const char *type = "";
  BME6xxData data[3];
  size_t dataLen;
};

#endif