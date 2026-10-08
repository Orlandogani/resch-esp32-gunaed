# Bootloader-only component (ADR-025): added to the bootloader build by
# cmake/board.cmake when BOARD=orga_v1, never to the application.
idf_component_register(SRCS "hooks.c"
                       PRIV_REQUIRES esp_rom esp_hal_gpio soc)
# The bootloader's hook symbols are weak; force this archive into the link.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u bootloader_hooks_include")
