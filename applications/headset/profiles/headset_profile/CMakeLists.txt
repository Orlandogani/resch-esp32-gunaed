# The profile vtable. Header-only: there is no code here, only the contract that
# main/ and every profiles/<name>/ agree on (ADR-013).
#
# REQUIRES, not PRIV_REQUIRES: headset_profile.h includes ringbuf.h, pm_policy.h and
# buttons.h in its public types, so anything including it needs them too.
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES ringbuf pm_policy buttons)
