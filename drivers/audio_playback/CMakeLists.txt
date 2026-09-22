# drivers/: hardware-facing. Owns one I2S TX channel exclusively (DR10).
# Unlike audio_capture it does not own its ring — the caller does — so ringbuf is a
# public dependency of the header's API. Sleep-lock coupling to pm_policy and watchdog
# coupling to diag are the standard platform services (SAD §3.3 D6).
idf_component_register(SRCS "audio_playback.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ringbuf
                       PRIV_REQUIRES esp_driver_i2s pm_policy diag heap)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
