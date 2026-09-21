# lib/: pure logic. REQUIRES is empty by rule (SAD §3.3 D5); esp_err.h comes from
# the always-available common components and is shimmed in host builds.
idf_component_register(SRCS "ringbuf.c"
                       INCLUDE_DIRS "include")

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
