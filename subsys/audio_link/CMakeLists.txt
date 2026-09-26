# Wireless duplex audio link (ADR-022). The codec is private — callers see PCM only
# (ADR-024) — and so is the BLE transport, which this module owns for the link's
# lifetime. ringbuf is public because the configuration carries ringbuf_t pointers.
idf_component_register(SRCS "audio_link.c" "audio_link_frame.c" "link_codec.c" "link_ble.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ringbuf
                       PRIV_REQUIRES opus ble esp_timer esp_pm)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
