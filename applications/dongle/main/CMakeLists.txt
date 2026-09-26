# Wiring and backend arbitration only — no backend or host-side logic here
# (ADR-013). Under MINIMAL_BUILD every component the product uses must be named;
# esp_psram and espcoredump must be listed explicitly for their Kconfig to apply.
#
# main/ requires the backends it can switch between and the host side it wires to
# them, plus the platform services. It does NOT require audio_link or usb_audio:
# those are the backend's and the host side's business.
idf_component_register(SRCS "dongle.c"
                       INCLUDE_DIRS "."
                       REQUIRES cfg diag power pm_policy buttons ringbuf
                                dongle_backend tone wlink_central host_port
                       PRIV_REQUIRES esp_psram espcoredump)
