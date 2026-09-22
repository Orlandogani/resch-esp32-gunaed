idf_component_register(SRCS "test_audio_playback.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity audio_playback audio_capture ringbuf power pm_policy diag cfg esp_driver_gpio)
