cmake_minimum_required(VERSION 3.22)

# Zephyr-shaped SDK trees (ADR-001). Each is a normal ESP-IDF component directory.
set(SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(EXTRA_COMPONENT_DIRS
    "${SDK_ROOT}/subsys"
    "${SDK_ROOT}/drivers"
    "${SDK_ROOT}/lib"
    "${SDK_ROOT}/third_party"
    # Product-local policy components. They live here and not in subsys/ on
    # purpose: which profile runs, and how it behaves, is headset policy, not
    # reusable SDK mechanism (ADR-013).
    "${CMAKE_CURRENT_LIST_DIR}/profiles"
    # Policy shared with the dongle: the CONTROL-message vocabulary.
    "${SDK_ROOT}/applications/common"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Build only main and its transitive REQUIRES (SYS-BLD-004). A subsystem the
# application does not name in main/CMakeLists.txt is not compiled at all.
idf_build_set_property(MINIMAL_BUILD ON)

project(esp32-headset)

# A fresh `idf.py build` with no target set configures for plain esp32, and the failure
# then surfaces deep in TinyUSB ("MCUs are only supported with CFG_TUH_MAX3421 enabled").
# Say what is actually wrong, before anything compiles.
if(NOT IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR
        "headset: configured for '${IDF_TARGET}', but this product is ESP32-S3 only (CON-01). "
        "Run: idf.py -C applications/headset set-target esp32s3 build")
endif()

# The console-safety state of this image is not obvious from `idf.py build`, and the
# difference is one Kconfig value: the safe image and the one that severs the only
# host link are the same project, the same build directory and the same sdkconfig.
# Anything that regenerates sdkconfig - menuconfig, the VS Code configuration editor,
# a stray SDKCONFIG_DEFAULTS - flips it silently, and a plain `idf.py flash` then
# ships whichever one is armed. So say which one this is, every time, at the end of
# configuration where it is the last thing on screen before the build starts.
if(CONFIG_HEADSET_ENABLE_USB)
    message(WARNING
        "headset: CONFIG_HEADSET_ENABLE_USB=y - this image starts TinyUSB and SEVERS "
        "the USB-Serial/JTAG console about a second after every boot. Replugging "
        "alone will NOT bring the port back: recovery is holding BOOT while "
        "replugging to reach ROM download mode. Do not flash this unless you are at "
        "the board (CLAUDE.md hard rule 1, TBD-009).")
elseif(CONFIG_HEADSET_ENABLE_WIRELESS)
    message(STATUS
        "headset: CONFIG_HEADSET_ENABLE_USB=n - console-safe image; boots into the "
        "wireless (wlink) profile and advertises to the dongle.")
else()
    message(STATUS
        "headset: CONFIG_HEADSET_ENABLE_USB=n, CONFIG_HEADSET_ENABLE_WIRELESS=n - "
        "console-safe image, stays in IDLE.")
endif()
