# The only SDK component that talks to esp_tinyusb (ADR-007). Sibling dependency
# on pm_policy is the standard sleep-lock coupling (SAD §3.3 D6).
idf_component_register(SRCS "usb_device.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_tinyusb pm_policy
                       PRIV_REQUIRES esp_hw_support)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
