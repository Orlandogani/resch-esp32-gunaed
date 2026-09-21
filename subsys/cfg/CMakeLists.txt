idf_component_register(SRCS "cfg.c"
                       INCLUDE_DIRS "include"
                       REQUIRES nvs_flash)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
