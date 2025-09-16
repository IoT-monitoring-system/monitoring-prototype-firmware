#pragma once
#ifndef BME6XXSENSORDATATYPES_H
#define BME6XXSENSORDATATYPES_H

#include <cstdint>

// #define BME6XX_DONT_USE_FPU
#define BME6XX_USE_FPU

enum BME6xxMask {
  HEAT_STAB_MSK = 0x10,
  GASM_VALID_MSK = 0x20,
  NEW_DATA_MSK = 0x80,
};

enum class BME6xxStatus : int8_t {
  ERROR_SELF_TEST = -5,
  ERROR_INVALID_LENGTH = -4,
  ERROR_DEV_NOT_FOUND = -3,
  ERROR_COM_FAIL = -2,
  ERROR_NULL_PTR = -1,
  OK = 0,
  WARNING_DEFINE_OP_MODE = 1,
  WARNING_NO_NEW_DATA = 2,
  WARNING_DEFINE_SHD_HEATR_DUR = 3
};

enum class BME6xxMode : uint8_t {
  SLEEP = 0,
  FORCED,
  PARALLEL,
  SEQUENTIAL,
};

enum class BME6xxFilter : uint8_t {
  /* Switch off the filter */
  OFF,
  /* Filter coefficient of 2 */
  SIZE_1,
  /* Filter coefficient of 4 */
  SIZE_3,
  /* Filter coefficient of 8 */
  SIZE_7,
  /* Filter coefficient of 16 */
  SIZE_15,
  /* Filter coefficient of 32 */
  SIZE_31,
  /* Filter coefficient of 64 */
  SIZE_63,
  /* Filter coefficient of 128 */
  SIZE_127,
};

enum class BME6xxODR : uint8_t {
  /* Standby time of 0.59ms */
  ODR_0_59_MS = 0,
  /* Standby time of 62.5ms */
  ODR_62_5_MS,
  /* Standby time of 125ms */
  ODR_125_MS,
  /* Standby time of 250ms */
  ODR_250_MS,
  /* Standby time of 500ms */
  ODR_500_MS,
  /* Standby time of 1s */
  ODR_1000_MS,
  /* Standby time of 10ms */
  ODR_10_MS,
  /* Standby time of 20ms */
  ODR_20_MS,
  /* No standby time */
  ODR_NONE,
};

enum class BME6xxOversampling : uint8_t {
  /* Switch off measurement */
  NONE = 0,
  /* Perform 1 measurement */
  X1,
  /* Perform 2 measurements */
  X2,
  /* Perform 4 measurements */
  X4,
  /* Perform 8 measurements */
  X8,
  /* Perform 16 measurements */
  X16
};

enum class BME6xxIntf : uint8_t {
  /*! SPI interface */
  SPI,
  /*! I2C interface */
  I2C
};

struct BME6xxOS {
  BME6xxOversampling temp = BME6xxOversampling::X2;
  BME6xxOversampling pres = BME6xxOversampling::X16;
  BME6xxOversampling hum = BME6xxOversampling::X1;
};

struct BME6xxData {
  /*! Contains new_data, gasm_valid & heat_stab */
  uint8_t status;

  /*! The index of the heater profile used */
  uint8_t gas_index;

  /*! Measurement index to track order */
  uint8_t meas_index;

  /*! Heater resistance */
  uint8_t res_heat;

  /*! Current DAC */
  uint8_t idac;

  /*! Gas wait period */
  uint8_t gas_wait;
#ifndef BME6XX_USE_FPU

  /*! Temperature in degree celsius x100 */
  int16_t temperature;

  /*! Pressure in Pascal */
  uint32_t pressure;

  /*! Humidity in % relative humidity x1000 */
  uint32_t humidity;

  /*! Gas resistance in Ohms */
  uint32_t gas_resistance;
#else

  /*! Temperature in degree celsius */
  float temperature;

  /*! Pressure in Pascal */
  float pressure;

  /*! Humidity in % relative humidity x1000 */
  float humidity;

  /*! Gas resistance in Ohms */
  float gas_resistance;

#endif
  /*! A timestamp of a measurement, use
   * bme68x_set_timestamp_function(timestamp_function) to set the function used
   * to retrieve timestamps */
  uint64_t meas_timestamp;
};

struct BME6xxHeatrConf {
  /*! Enable gas measurement. Refer @ref en_dis */
  uint8_t enable;

  /*! Store the heater temperature for forced mode degree Celsius */
  uint16_t heatr_temp;

  /*! Store the heating duration for forced mode in milliseconds */
  uint16_t heatr_dur;

  /*! Store the heater temperature profile in degree Celsius */
  uint16_t *heatr_temp_prof;

  /*! Store the heating duration profile in milliseconds */
  uint16_t *heatr_dur_prof;

  /*! Variable to store the length of the heating profile */
  uint8_t profile_len;

  /*!
   * Variable to store heating duration for parallel mode
   * in milliseconds
   */
  uint16_t shared_heatr_dur;
};

struct BME6xxConf {
  /*! Oversampling struct */
  BME6xxOS os;
  /*! Filter coefficient. Refer @ref filter*/
  BME6xxFilter filter;
  /*!
   * Standby time between sequential mode measurement profiles.
   * Refer @ref odr
   */
  BME6xxODR odr;
};
#endif