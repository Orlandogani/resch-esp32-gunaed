# Wiring, the mode state machine and device-level policy only — no profile logic here
# (ADR-013), and no pins: those come from the selected board (ADR-025).
# Under MINIMAL_BUILD every component the product uses must be named; esp_psram and
# espcoredump must be listed explicitly for their Kconfig to apply.
#
# main/ requires the profiles it can switch between and the modules that own the
# board's hardware on its behalf. It does NOT require audio_playback, usb_audio, usb_hid
# or audio_link: those are the active profile's business and reach main/ only through
# the headset_profile vtable.
#
# Both profiles are required unconditionally: requirements are expanded before
# Kconfig is read, so they cannot follow HEADSET_ENABLE_WIRELESS. A profile that is
# compiled but not enabled is simply never started.
idf_component_register(SRCS "headset.c"
                       INCLUDE_DIRS "."
                       REQUIRES cfg diag power pm_policy ringbuf board
                                headset_profile usound wlink
                                hs_audio hs_ui hs_power hs_sense
                       PRIV_REQUIRES esp_psram espcoredump esp_timer freertos)
