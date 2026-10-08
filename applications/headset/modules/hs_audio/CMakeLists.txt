# Headset policy (ADR-013): the board's audio path for whichever profile runs. Lives in
# modules/ beside profiles/ — product code, not SDK mechanism.
idf_component_register(SRCS "hs_audio.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ringbuf
                       PRIV_REQUIRES board audio_playback audio_capture tlv320aic3104 freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../../../cmake/sdk_component.cmake)
sdk_component_strict()
