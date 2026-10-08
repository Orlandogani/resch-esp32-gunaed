idf_component_register(SRCS "test_battery.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity battery diag cfg power pm_policy esp_driver_gpio)
