cmake_minimum_required(VERSION 3.22)

# Zephyr-shaped SDK trees (ADR-001). Each is a normal ESP-IDF component directory.
set(SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(EXTRA_COMPONENT_DIRS
    "${SDK_ROOT}/subsys"
    "${SDK_ROOT}/drivers"
    "${SDK_ROOT}/lib"
    "${SDK_ROOT}/third_party"
    # Product-local policy components (ADR-013): what the backends do with the audio,
    # and what the host side looks like.
    "${CMAKE_CURRENT_LIST_DIR}/backends"
    "${CMAKE_CURRENT_LIST_DIR}/host_port"
    # Policy shared with the headset: the CONTROL-message vocabulary.
    "${SDK_ROOT}/applications/common"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Build only main and its transitive REQUIRES (SYS-BLD-004).
idf_build_set_property(MINIMAL_BUILD ON)

project(esp32-dongle)

# A fresh `idf.py build` with no target set configures for plain esp32, and the failure
# then surfaces deep in TinyUSB ("MCUs are only supported with CFG_TUH_MAX3421 enabled").
# Say what is actually wrong, before anything compiles.
if(NOT IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR
        "dongle: configured for '${IDF_TARGET}', but this product is ESP32-S3 only (CON-01). "
        "Run: idf.py -C applications/dongle set-target esp32s3 build")
endif()

# Same reason as the headset: the console-safe image and the one that severs the only
# host link are one project and one Kconfig value apart, and anything that regenerates
# sdkconfig can flip it silently. Say which one this is, every time.
if(CONFIG_DONGLE_ENABLE_USB)
    message(WARNING
        "dongle: CONFIG_DONGLE_ENABLE_USB=y - this image starts TinyUSB and SEVERS "
        "the USB-Serial/JTAG console about a second after every boot. Replugging "
        "alone will NOT bring the port back: recovery is holding BOOT while "
        "replugging to reach ROM download mode. Do not flash this unless you are at "
        "the board (CLAUDE.md hard rule 1, TBD-009).")
else()
    message(STATUS
        "dongle: CONFIG_DONGLE_ENABLE_USB=n - console-safe image; the PC is emulated "
        "(host_port) and the USB-Serial/JTAG console is kept.")
endif()
