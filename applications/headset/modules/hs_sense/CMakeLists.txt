# Headset policy (ADR-013): wear heuristic and head pose from the board's IMU.
idf_component_register(SRCS "hs_sense.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES board lsm6dsv16x log)

include(${CMAKE_CURRENT_LIST_DIR}/../../../../cmake/sdk_component.cmake)
sdk_component_strict()
