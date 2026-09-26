/**
 * @file dongle_backend.h
 * @brief The contract every dongle backend implements: what happens to the audio the
 *        host sends, and where the audio the host receives comes from.
 *
 * The dongle's host side is fixed — one duplex USB audio function plus HID, or its
 * console-safe emulation (`host_port`) — and the backend is the part that varies:
 * `wlink_central` bridges to the headset over `subsys/audio_link`; `tone` terminates
 * both directions locally so the host side can be exercised with no headset at all.
 * `main/` owns the rings and chooses the backend; it never looks inside one
 * (ADR-013, applications/dongle/docs/design.md).
 *
 * Header-only, and deliberately not in `subsys/`: this is product policy.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Ask the host side to send one HID consumer-control tap (headset_link_proto.h bits). */
typedef void (*dongle_hid_tap_fn)(uint8_t bits);

/** Formats of the two directions, as the host side presents them. 16-bit PCM. */
typedef struct {
    uint32_t speaker_rate_hz;       /**< host → dongle                    */
    uint8_t  speaker_channels;
    uint32_t mic_rate_hz;           /**< dongle → host; always mono        */
} dongle_format_t;

/**
 * @brief What one backend owns while it is active. Granted by `main/` at `start()`,
 *        revoked at `stop()`; nothing may be retained past `stop()` returning.
 */
typedef struct {
    ringbuf_t        *speaker_ring; /**< Written by the host side. The backend opens
                                         its own read cursor on it.                 */
    ringbuf_t        *mic_ring;     /**< The backend is its **single writer**; the
                                         host side reads it.                         */
    dongle_format_t   format;
    dongle_hid_tap_fn hid_tap;      /**< Never NULL.                                 */
} dongle_backend_resources_t;

typedef struct {
    /** Stable, log-friendly name. Static lifetime. */
    const char *name;

    /** Take ownership of `res` and start. On failure everything taken is released,
     *  because `main/` does not call `stop()` after a failed `start()`. */
    esp_err_t (*start)(const dongle_backend_resources_t *res, void *ctx);

    /** Release everything and stop. Idempotent; safe when `start()` never ran. */
    esp_err_t (*stop)(void *ctx);

    /**
     * @brief The speaker path's backlog in bytes of the speaker format: everything
     *        the host has sent that has not been rendered yet, wherever it is. The
     *        host side's rate servo (ADR-021) regulates it. Called every millisecond
     *        from the USB feeder task: cheap, non-blocking.
     */
    uint32_t (*speaker_backlog)(void *ctx);

    /** Is the far end carrying audio? For status and logging only. */
    bool (*is_linked)(void *ctx);

    /** Host changed mute/volume on a feature unit. Optional. */
    void (*on_host_volume)(bool mic, bool mute, int16_t volume_db256, void *ctx);

    /** One line of backend-specific counters for the periodic log. Optional. */
    void (*log_stats)(void *ctx);

    void *ctx;
} dongle_backend_t;

#ifdef __cplusplus
}
#endif
