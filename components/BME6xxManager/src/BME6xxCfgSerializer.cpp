#include "BME6xxCfgSerializer.h"
#include "BME6xxManagerErrors.h"

#include "esp_log.h"

const char *TAG_BME6XX_CFG_SERIALIZER = "BME6xxCfgSerializer";

esp_err_t
BME6xxCfgSerializer::serializeConfig() {
  return ESP_FAIL;
}

esp_err_t
BME6xxCfgSerializer::deserializeConfig(FSFile &cfg, std::vector<BMESensorConfig> &configurations) {
  JsonDocument configDoc;

  DeserializationError error = deserializeJson(configDoc, cfg);

  if (error) {
    return ESP_ERR_JSON_DESERIAL_ERROR;
  }

  JsonArray devCfgs = configDoc["configBody"]["sensorConfigurations"].as<JsonArray>();

  ESP_LOGD(TAG_BME6XX_CFG_SERIALIZER, "Loading %u sensor configurations", devCfgs.size());

  if (configurations.size() > devCfgs.size())
    return ESP_ERR_CONFIG_FILE_ERROR;

  // Get all heating profiles
  JsonArray heaterProfilesJson = configDoc["configBody"]["heaterProfiles"].as<JsonArray>();
  BMEHeaterProfile heaterProfiles[heaterProfilesJson.size()];

  uint16_t dur = 0;
  uint16_t temp = 0;
  for (uint8_t i = 0; i < heaterProfilesJson.size(); i++) {
    heaterProfiles[i].id = heaterProfilesJson[i]["id"].as<std::string>();

    heaterProfiles[i].length = heaterProfilesJson[i]["temperatureTimeVectors"].size();
    heaterProfiles[i].timeBaseUs = heaterProfilesJson[i]["timeBase"].as<uint32_t>() * 1000U;
    heaterProfiles[i].heatCycleDurationUs = 0;

    for (uint8_t j = 0; j < heaterProfiles[i].length; j++) {
      temp = heaterProfilesJson[i]["temperatureTimeVectors"][j][0].as<uint16_t>();
      dur = heaterProfilesJson[i]["temperatureTimeVectors"][j][1].as<uint16_t>();

      heaterProfiles[i].temperature[j] = temp;
      heaterProfiles[i].duration[j] = dur;

      heaterProfiles[i].heatCycleDurationUs += (dur * heaterProfiles[i].timeBaseUs);
    }
  }

  JsonArray dutyCycleProfilesJson = configDoc["configBody"]["dutyCycleProfiles"].as<JsonArray>();
  BMEDutyCycleProfile dutyCycleProfiles[dutyCycleProfilesJson.size()];

  for (uint8_t i = 0; i < dutyCycleProfilesJson.size(); i++) {
    dutyCycleProfiles[i].id = dutyCycleProfilesJson[i]["id"].as<std::string>();
    dutyCycleProfiles[i].numScans = dutyCycleProfilesJson[i]["numberScanningCycles"].as<uint8_t>();
    dutyCycleProfiles[i].numSleeps = dutyCycleProfilesJson[i]["numberSleepingCycles"].as<uint8_t>();
  }

  /* Number of duty cycle profiles, heater profiles and number of configured
    sensors may not be symmetrical, therefore three separate loops to avoid
    issues */
  for (uint8_t i = 0; i < devCfgs.size(); i++) {
    if (i >= configurations.size()) {
      break;
    }

    for (uint8_t j = 0; j < heaterProfilesJson.size(); j++) {
      if (devCfgs[i]["heaterProfile"].as<std::string>() == heaterProfiles[j].id) {
        configurations[i].heaterProfile = heaterProfiles[j];
      }
    }
    for (uint8_t j = 0; j < dutyCycleProfilesJson.size(); j++) {
      if (devCfgs[i]["dutyCycleProfile"].as<std::string>() == dutyCycleProfiles[j].id) {
        configurations[i].dutyCycleProfile = dutyCycleProfiles[j];
      }
    }

    configurations[i].dutyCycleProfile.sleepDurationUs =
        configurations[i].dutyCycleProfile.numSleeps * configurations[i].heaterProfile.heatCycleDurationUs;
  }

  return ESP_OK;
}