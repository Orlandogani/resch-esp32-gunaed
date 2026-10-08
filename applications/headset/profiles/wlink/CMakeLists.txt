# Phase 2 profile: the wireless link to the dongle. Product policy (ADR-013) — the
# link mechanism itself is subsys/audio_link; this profile decides formats, wires the
# rings main/ owns, and says what buttons mean.
#
# headset_profile is public (wlink.h re-exports its types); everything the profile
# drives is private — main/ reaches it only through the vtable.
idf_component_register(SRCS "wlink.c"
                       INCLUDE_DIRS "include"
                       REQUIRES headset_profile
                       PRIV_REQUIRES audio_link hs_audio headset_link_proto esp_timer)

include(${CMAKE_CURRENT_LIST_DIR}/../../../../cmake/sdk_component.cmake)
sdk_component_strict()
