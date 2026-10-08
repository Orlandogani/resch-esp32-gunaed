idf_component_register(SRCS "bringup.c"
                       INCLUDE_DIRS "."
                       REQUIRES console board cfg diag power pm_policy ringbuf
                                audio_capture audio_playback battery buttons encoder
                                lsm6dsv16x power_latch rgb_led tlv320aic3104 esp_driver_i2c)
