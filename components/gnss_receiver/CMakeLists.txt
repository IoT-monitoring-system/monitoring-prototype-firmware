file(GLOB_RECURSE GNSS_RECEIVER_SRC
  "src/*.*"
  "lib/*/*.*"
)

list(APPEND INCLUDE_DIRS
  "lib"
  "include"
)

if(ESP_PLATFORM)
  idf_component_register(
    SRCS ${GNSS_RECEIVER_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES esp_driver_uart
  )
endif()

project(gnss_receiver)
