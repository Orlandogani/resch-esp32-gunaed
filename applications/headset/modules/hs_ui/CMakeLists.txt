# Headset policy (ADR-013): board inputs to events, and the status LED patterns.
idf_component_register(SRCS "hs_ui.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES board buttons encoder rgb_led esp_timer freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../../../cmake/sdk_component.cmake)
sdk_component_strict()
