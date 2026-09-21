# Application wiring only — no business logic lives here (ADR-013).
# Under MINIMAL_BUILD every SDK subsystem the product uses must be named here;
# esp_psram and espcoredump must be listed explicitly for their Kconfig to apply.
idf_component_register(SRCS "esp32-gunshot-aed.c"
                       INCLUDE_DIRS "."
                       REQUIRES cfg diag power pm_policy
                                audio_capture ringbuf
                                usb_device usb_hid usb_audio
                                wifi_link ble ota
                       PRIV_REQUIRES esp_psram espcoredump)
