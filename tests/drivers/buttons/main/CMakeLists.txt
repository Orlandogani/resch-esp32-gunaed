idf_component_register(SRCS "test_buttons.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity buttons power pm_policy diag cfg esp_driver_gpio)
