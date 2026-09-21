idf_component_register(SRCS "usb_hid.c"
                       INCLUDE_DIRS "include"
                       REQUIRES usb_device
                       PRIV_REQUIRES esp_tinyusb)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
