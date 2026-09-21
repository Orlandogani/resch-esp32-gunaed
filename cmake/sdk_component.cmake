# SDK component conventions - included by every component under subsys/, drivers/, lib/.
#
# Usage, at the end of a component's CMakeLists.txt after idf_component_register():
#
#     include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/sdk_component.cmake)
#     sdk_component_strict()
#
# Rationale (SYS-BLD-005, FW-SYS-053): warnings are errors for SDK code only.
# ESP-IDF's own components are not held to this flag set, because we do not own them.

function(sdk_component_strict)
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall
        -Wextra
        -Werror
        -Wshadow
        # -Wundef is deliberately absent: ESP-IDF's own headers (assert.h, esp_compiler.h)
        # test undefined CONFIG_* symbols with #if, and we do not own those headers.
        -Wdouble-promotion
        -Wformat=2
        -Wformat-truncation
        -Wnull-dereference
        -Wimplicit-fallthrough
        -Wstrict-prototypes
        -Wmissing-prototypes
        -Wold-style-definition
        -Wno-unused-parameter   # Callback signatures routinely have unused params
    )
endfunction()
