# The dongle end of the wireless link: subsys/audio_link as BLE central. Policy only —
# formats, callbacks, what control messages mean (ADR-013).
idf_component_register(SRCS "wlink_central.c"
                       INCLUDE_DIRS "include"
                       REQUIRES dongle_backend
                       PRIV_REQUIRES audio_link headset_link_proto)

include(${CMAKE_CURRENT_LIST_DIR}/../../../../cmake/sdk_component.cmake)
sdk_component_strict()
