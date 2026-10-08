# The profile vtable. Header-only: there is no code here, only the contract that
# main/ and every profiles/<name>/ agree on (ADR-013).
#
# REQUIRES, not PRIV_REQUIRES: headset_profile.h includes ringbuf.h and pm_policy.h in
# its public types, so anything including it needs them too. Controls arrive as
# headset_ctrl_t, so the contract no longer depends on drivers/buttons (ADR-025).
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES ringbuf pm_policy)
