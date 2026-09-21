idf_component_register(SRCS "test_ota.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity ota power pm_policy wifi_link)
