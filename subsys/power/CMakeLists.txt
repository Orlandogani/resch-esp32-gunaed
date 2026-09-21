idf_component_register(SRCS "power.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_pm esp_hw_support esp_driver_gpio
                       PRIV_REQUIRES esp_system esp_driver_usb_serial_jtag)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
