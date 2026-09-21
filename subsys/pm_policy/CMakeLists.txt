idf_component_register(SRCS "pm_policy.c"
                       INCLUDE_DIRS "include"
                       REQUIRES power)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
