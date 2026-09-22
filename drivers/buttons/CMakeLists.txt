# drivers/: hardware-facing. Owns its GPIO inputs exclusively (DR10).
# Sleep-lock coupling to pm_policy, watchdog coupling to diag, and deep-sleep wake
# registration through power are the standard platform services (SAD §3.3 D6).
idf_component_register(SRCS "buttons.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_gpio esp_timer pm_policy power diag)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
