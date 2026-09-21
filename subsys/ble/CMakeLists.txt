# Bluedroid host (ADR-018). Sibling dependency on cfg (NVS for bonds) and pm_policy
# (sleep lock) per SAD §3.3. bt is a public requirement because ble.h exposes
# esp_gatts_attr_db_t deliberately.
idf_component_register(SRCS "ble.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt
                       PRIV_REQUIRES cfg pm_policy)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
