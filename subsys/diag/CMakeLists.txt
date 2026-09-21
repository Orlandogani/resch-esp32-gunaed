# Sibling dependency diag -> cfg is on the SAD §3.3 D3 whitelist (persistent counters).
idf_component_register(SRCS "diag.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_timer esp_system
                       PRIV_REQUIRES cfg esp_hw_support espcoredump heap)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
