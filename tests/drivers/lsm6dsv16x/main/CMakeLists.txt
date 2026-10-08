idf_component_register(SRCS "test_lsm6dsv16x.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity lsm6dsv16x esp_driver_i2c pm_policy power cfg diag)
