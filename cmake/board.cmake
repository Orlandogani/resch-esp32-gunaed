# Board selection (ADR-025). Included by an application's top-level CMakeLists.txt.
#
#     include(${SDK_ROOT}/cmake/board.cmake)
#     sdk_board_select(esp32s3_devkitc)      # before include(project.cmake)
#     include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#     sdk_board_bootloader()                 # after include(project.cmake)
#
# `-D BOARD=<name>` picks `boards/<name>/`; the default keeps existing builds unchanged.
# For the chosen board this:
#   - adds `boards/board_api` (the contract) and `boards/<name>/board` (the component named
#     `board`) to EXTRA_COMPONENT_DIRS;
#   - appends `boards/<name>/sdkconfig.defaults` and, if present, the application's own
#     `boards/<name>.defaults` (Zephyr's `boards/<board>.conf`) to SDKCONFIG_DEFAULTS;
#   - for a non-default board, writes its own `sdkconfig.<name>` unless SDKCONFIG is given,
#     so two boards never share one generated configuration (BACKLOG, 2026-09-22);
#   - adds `boards/<name>/bootloader_components/` to the bootloader build, if present.

macro(sdk_board_select default_board)
    if(NOT DEFINED BOARD OR "${BOARD}" STREQUAL "")
        set(BOARD "${default_board}")
    endif()
    set(SDK_BOARD_DIR "${SDK_ROOT}/boards/${BOARD}")
    if(NOT EXISTS "${SDK_BOARD_DIR}/board/CMakeLists.txt")
        message(FATAL_ERROR "BOARD=${BOARD}: no ${SDK_BOARD_DIR}/board/ component. "
                            "Boards: see ${SDK_ROOT}/boards/.")
    endif()
    list(APPEND EXTRA_COMPONENT_DIRS "${SDK_ROOT}/boards/board_api" "${SDK_BOARD_DIR}/board")

    if(NOT DEFINED SDKCONFIG_DEFAULTS)
        set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
    endif()
    if(EXISTS "${SDK_BOARD_DIR}/sdkconfig.defaults")
        list(APPEND SDKCONFIG_DEFAULTS "${SDK_BOARD_DIR}/sdkconfig.defaults")
    endif()
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}.defaults")
        list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}.defaults")
    endif()

    if(NOT "${BOARD}" STREQUAL "${default_board}" AND NOT DEFINED SDKCONFIG)
        set(SDKCONFIG "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.${BOARD}")
    endif()
    message(STATUS "board: ${BOARD}")
endmacro()

macro(sdk_board_bootloader)
    if(EXISTS "${SDK_BOARD_DIR}/bootloader_components")
        idf_build_set_property(BOOTLOADER_EXTRA_COMPONENT_DIRS "${SDK_BOARD_DIR}/bootloader_components" APPEND)
    endif()
endmacro()
