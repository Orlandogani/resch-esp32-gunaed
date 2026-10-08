# Board: ORGA v1 headset main board (ADR-025). Wiring from the netlist, doc/index.md.
idf_component_register(SRCS "board.c"
                       INCLUDE_DIRS "."
                       REQUIRES board_api
                       PRIV_REQUIRES freertos esp_driver_gpio)

include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/sdk_component.cmake)
sdk_component_strict()
