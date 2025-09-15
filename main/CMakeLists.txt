file(GLOB_RECURSE APP_SRCS
  "*.*"
  "modules/*/*.c"
  "modules/*/*.cpp"
)

list(APPEND INCLUDE_DIRS
  "."
)

list(APPEND INCLUDE_DIRS
  "modules/mqtt_transmission_module"
  "modules/bme_sample_module"
  "modules/gnss_position_module"
  "modules/fall_detect_module"
)

idf_component_register(
  SRCS ${APP_SRCS}
  INCLUDE_DIRS ${INCLUDE_DIRS}
  REQUIRES
)

littlefs_create_partition_image(littlefs littlefs FLASH_IN_PROJECT)
