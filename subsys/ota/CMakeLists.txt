idf_component_register(SRCS "ota.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_https_ota app_update esp_http_client
                       PRIV_REQUIRES pm_policy esp_partition mbedtls esp_app_format esp_netif)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
