# drivers/: hardware-facing. Owns one PCNT unit and two of its channels exclusively
# (DR10); watchdog coupling to diag is the standard platform service (SAD §3.3 D6).
idf_component_register(SRCS "encoder.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_pcnt esp_driver_gpio diag freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
