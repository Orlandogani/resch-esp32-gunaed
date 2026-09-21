# Sibling dependency wifi_sta -> cfg (credential helpers, and the driver's NVS need)
# is on the SAD §3.3 D3 whitelist. pm_policy is the standard sleep-lock coupling.
idf_component_register(SRCS "wifi_link.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif esp_event
                       PRIV_REQUIRES cfg pm_policy)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
