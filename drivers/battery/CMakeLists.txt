# drivers/: hardware-facing. Owns ADC1 (oneshot) and the power-good input exclusively
# (DR10); watchdog coupling to diag is the standard platform service (SAD §3.3 D6).
idf_component_register(SRCS "battery.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_adc esp_driver_gpio diag freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
