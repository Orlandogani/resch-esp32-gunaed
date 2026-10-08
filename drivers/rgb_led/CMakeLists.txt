# drivers/: hardware-facing. A control driver (SDD §15.3): one LEDC timer and three
# channels, owned exclusively (DR10); sleep-lock coupling to pm_policy (DR8).
idf_component_register(SRCS "rgb_led.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_ledc esp_driver_gpio esp_timer pm_policy freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
