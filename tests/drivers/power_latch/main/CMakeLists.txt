idf_component_register(SRCS "test_power_latch.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity power_latch esp_driver_gpio)
