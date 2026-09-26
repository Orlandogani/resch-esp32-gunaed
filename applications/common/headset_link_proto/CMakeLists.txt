# The CONTROL-message vocabulary the headset and the dongle speak over subsys/audio_link.
# Product policy shared by two applications — not SDK mechanism (ADR-013), so it lives
# under applications/common/ and not under subsys/. Header-only.
idf_component_register(INCLUDE_DIRS "include")
