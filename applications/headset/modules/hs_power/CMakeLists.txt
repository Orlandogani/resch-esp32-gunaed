# Headset policy (ADR-013): the soft power latch, battery thresholds and power-off.
idf_component_register(SRCS "hs_power.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES board battery power_latch freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../../../cmake/sdk_component.cmake)
sdk_component_strict()
