file(GLOB_RECURSE LOGGER_SRC
  "src/*.*"
)

list(APPEND INCLUDE_DIRS
  "include"
)

if(ESP_PLATFORM)
  idf_component_register(
    SRCS ${LOGGER_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES esp_netif
  )
endif()
