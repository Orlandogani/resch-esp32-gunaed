# Board: ESP32-S3-DevKitC-1 (N16R8) with provisional wiring (ADR-025).
idf_component_register(SRCS "board.c"
                       INCLUDE_DIRS "."
                       REQUIRES board_api
                       PRIV_REQUIRES freertos)

include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/sdk_component.cmake)
sdk_component_strict()
