file(GLOB_RECURSE MQTT_CLIENT_SIMPLE_SRC
  "src/*.*"
)

list(APPEND INCLUDE_DIRS
  "include"
)

if(ESP_PLATFORM)
  idf_component_register(
    SRCS ${MQTT_CLIENT_SIMPLE_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES esp_event mqtt
  )
endif()

project(mqtt_client_simple)
