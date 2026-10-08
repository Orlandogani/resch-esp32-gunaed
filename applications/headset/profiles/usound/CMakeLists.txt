# Phase 1 profile. Product policy, not SDK mechanism (ADR-013) — which is why it
# lives under applications/headset/profiles/ and not under subsys/.
#
# headset_profile is a public dependency: usound.h re-exports its types. Everything
# this profile drives is private — main/ talks to it only through the vtable.
idf_component_register(SRCS "usound.c"
                       INCLUDE_DIRS "include"
                       REQUIRES headset_profile
                       PRIV_REQUIRES hs_audio usb_device usb_audio usb_hid)

# Product code is held to the same flag set as the SDK. Nothing here is more
# forgiving than subsys/ just because it is a profile.
include(${CMAKE_CURRENT_LIST_DIR}/../../../../cmake/sdk_component.cmake)
sdk_component_strict()
