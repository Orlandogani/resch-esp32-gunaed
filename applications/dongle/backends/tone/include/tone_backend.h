/**
 * @file tone_backend.h
 * @brief Diagnostic backend: terminates both directions on the dongle itself.
 *
 * The speaker stream is consumed at the dongle's own crystal rate, exactly as a
 * render device would, and its level is measured; the microphone stream is a 440 Hz
 * tone generated at the same rate. With USB on, the PC sees a working duplex device
 * whose microphone is a tone and whose speaker is regulated by the real feedback
 * servo — the whole host side verified with no headset. With USB off, the emulated
 * host (host_port) drives it and both directions are verified over the console.
 */
#pragma once

#include "dongle_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief The singleton. Static storage; touches no hardware until `start()`. */
const dongle_backend_t *tone_backend(void);

#ifdef __cplusplus
}
#endif
