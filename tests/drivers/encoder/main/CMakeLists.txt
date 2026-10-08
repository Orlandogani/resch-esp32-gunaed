idf_component_register(SRCS "test_encoder.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity encoder diag cfg power pm_policy esp_driver_gpio)
