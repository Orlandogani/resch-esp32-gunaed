# Reads audio through lib/ringbuf as an ordinary reader; no dependency on the
# capture driver (DES-AUD-001). Registers with usb_device (ADR-007).
idf_component_register(SRCS "usb_audio.c"
                       INCLUDE_DIRS "include"
                       REQUIRES usb_device ringbuf
                       PRIV_REQUIRES esp_tinyusb esp_timer)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
