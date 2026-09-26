# The dongle's host side: USB duplex audio + HID, or its console-safe emulation.
# Policy (the HID descriptor, what the emulation sends) — so not subsys/ (ADR-013).
# Both implementations' dependencies are named unconditionally: requirements are
# expanded before Kconfig is read, so they cannot follow DONGLE_ENABLE_USB.
idf_component_register(SRCS "host_port.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ringbuf
                       PRIV_REQUIRES usb_device usb_audio usb_hid esp_timer)

include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/sdk_component.cmake)
sdk_component_strict()
