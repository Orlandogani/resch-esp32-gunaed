# The board contract (ADR-025). Every boards/<name>/board component implements board.h;
# applications depend on `board`, which re-exports this. The one source here is the
# shared I2C-bus helper the board implementations call, so it exists once.
idf_component_register(SRCS "board_api.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c
                       PRIV_REQUIRES freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
