idf_component_register(SRCS "test_rgb_led.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity rgb_led pm_policy power cfg diag esp_driver_ledc)
