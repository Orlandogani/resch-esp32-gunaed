idf_component_register(SRCS "test_audio_capture.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity audio_capture ringbuf power pm_policy diag cfg)
