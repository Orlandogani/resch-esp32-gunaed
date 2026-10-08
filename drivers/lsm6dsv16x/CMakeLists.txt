# drivers/: hardware-facing. One I2C device on a bus the board owns, plus the INT1 input
# (DR10). Register access through ST's vendored driver (ADR-027); sleep-lock coupling to
# pm_policy and watchdog coupling to diag are the standard platform services.
# esp_driver_i2c is public: the config takes a bus handle.
idf_component_register(SRCS "lsm6dsv16x.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c
                       PRIV_REQUIRES lsm6dsv16x_pid esp_driver_gpio pm_policy diag freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
