cmake_minimum_required(VERSION 3.22)

# Zephyr-shaped SDK trees (ADR-001). Each is a normal ESP-IDF component directory.
set(SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(EXTRA_COMPONENT_DIRS
    "${SDK_ROOT}/subsys"
    "${SDK_ROOT}/drivers"
    "${SDK_ROOT}/lib"
    "${SDK_ROOT}/third_party"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Build only main and its transitive REQUIRES (SYS-BLD-004). A subsystem the
# application does not name in main/CMakeLists.txt is not compiled at all.
idf_build_set_property(MINIMAL_BUILD ON)

project(esp32-gunshot-aed)
