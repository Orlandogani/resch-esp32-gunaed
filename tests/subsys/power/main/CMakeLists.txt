idf_component_register(SRCS "test_power.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity power esp_timer esp_driver_usb_serial_jtag)
