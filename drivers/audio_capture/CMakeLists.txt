# drivers/: hardware-facing. Owns one I2S RX channel exclusively (DR10).
# Sleep-lock coupling to pm_policy and watchdog coupling to diag are the standard
# platform services (SAD §3.3 D6).
idf_component_register(SRCS "audio_capture.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ringbuf
                       PRIV_REQUIRES esp_driver_i2s pm_policy diag heap)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
