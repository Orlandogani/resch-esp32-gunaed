cmake_minimum_required(VERSION 3.22)
set(SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
# third_party for the vendored libopus (ADR-024).
set(EXTRA_COMPONENT_DIRS "${SDK_ROOT}/subsys" "${SDK_ROOT}/lib" "${SDK_ROOT}/third_party")
# Override from the command line with -D SDKCONFIG_DEFAULTS="a;b".
if(NOT DEFINED SDKCONFIG_DEFAULTS)
    set(SDKCONFIG_DEFAULTS "${SDK_ROOT}/tests/sdkconfig.common.defaults;${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(test_audio_link)
