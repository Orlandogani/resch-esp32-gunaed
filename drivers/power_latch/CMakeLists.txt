# drivers/: hardware-facing. A control driver (SDD §15.3): one GPIO, no task, no ISR.
idf_component_register(SRCS "power_latch.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_gpio freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
