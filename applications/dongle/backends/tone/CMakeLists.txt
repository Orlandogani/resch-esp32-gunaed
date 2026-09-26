# Diagnostic backend: both directions terminated on the dongle (no radio).
idf_component_register(SRCS "tone_backend.c"
                       INCLUDE_DIRS "include"
                       REQUIRES dongle_backend
                       PRIV_REQUIRES esp_timer)

include(${CMAKE_CURRENT_LIST_DIR}/../../../../cmake/sdk_component.cmake)
sdk_component_strict()
