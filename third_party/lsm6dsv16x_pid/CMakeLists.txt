# Vendored ST lsm6dsv16x-pid (platform-independent register driver), unpatched (ADR-027).
# Third-party code is not held to sdk_component_strict(): we do not own it. The one
# suppression is for ST's own style (unused parameters in platform hooks it never calls).
idf_component_register(SRCS "lsm6dsv16x_reg.c"
                       INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-parameter)
