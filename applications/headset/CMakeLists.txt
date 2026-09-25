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
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Build only main and its transitive REQUIRES (SYS-BLD-004). A subsystem the
# application does not name in main/CMakeLists.txt is not compiled at all.
idf_build_set_property(MINIMAL_BUILD ON)

project(esp32-headset)

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
else()
    message(STATUS
        "headset: CONFIG_HEADSET_ENABLE_USB=n - console-safe image, stays in IDLE.")
endif()
