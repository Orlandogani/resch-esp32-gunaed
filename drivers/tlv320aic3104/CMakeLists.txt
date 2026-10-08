# drivers/: hardware-facing. A control driver (SDD §15.3): one I2C device on a bus the
# board owns, plus the codec's reset pin. The audio data is audio_playback's and
# audio_capture's (ADR-026). esp_driver_i2c is public: the config takes a bus handle.
idf_component_register(SRCS "tlv320aic3104.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c
                       PRIV_REQUIRES esp_driver_gpio esp_rom freertos log)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
sdk_component_strict()
