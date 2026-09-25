/**
 * @file headset_profile.h
 * @brief The contract every headset profile implements.
 *
 * `main/` owns the mode state machine and nothing else: a mode switch is `main/`
 * revoking the four resources below from profile A and granting them to profile B
 * (`applications/headset/docs/design.md`, "Data path and ownership"). Keeping that
 * hand-off behind one vtable is what lets Phase 2 add `bt_music` as a new profile
 * rather than a redesign, and what lets each profile be tested on its own.
 *
 * This component is deliberately header-only and deliberately *not* in `subsys/`:
 * it is product policy, not reusable SDK mechanism (ADR-013).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "buttons.h"
#include "pm_policy.h"
#include "ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The four resources exactly one profile owns at a time.
 *
 * Granted by `main/` at `start()` and implicitly revoked at `stop()`. A profile
 * must not retain any of them past its `stop()` returning.
 *
 * The rings are owned by `main/` rather than by a profile so that their storage
 * survives a mode change: re-allocating a DMA-capable ring on every switch would
 * be a fragmentation source and would race the playback driver's reader.
 */
typedef struct {
    /** Playback ring. The profile is its **single writer**; `drivers/audio_playback`
     *  holds the one read cursor (`DES-APB-002`). Never NULL. */
    ringbuf_t *playback_ring;

    /** Microphone ring, written by `drivers/audio_capture`. The profile opens its
     *  own read cursor on it (`lib/ringbuf` is single-writer/multi-reader).
     *  NULL when no microphone came up — a profile must tolerate that. */
    ringbuf_t *mic_ring;

    /** Held by the profile for as long as its stream must stay alive. `main/`
     *  creates it once; the profile acquires on start and releases on stop, and
     *  every transition into `IDLE` therefore ends with it released. */
    pm_policy_lock_handle_t stream_lock;
} headset_profile_resources_t;

/**
 * @brief A profile: one way of moving audio between the user and the far end.
 *
 * Every function is called from the `main/` mode task, never from an ISR, and never
 * re-entrantly: `main/` serialises the whole state machine on that one task.
 */
typedef struct {
    /** Stable, log-friendly name — `"usound"`, `"bt_music"`. Static lifetime. */
    const char *name;

    /**
     * @brief Take ownership of `res` and begin streaming.
     *
     * On any non-`ESP_OK` return the profile must have released everything it
     * took, because `main/` will not call `stop()` after a failed `start()`.
     */
    esp_err_t (*start)(const headset_profile_resources_t *res, void *ctx);

    /**
     * @brief Release every resource and stop streaming. Must be idempotent.
     *
     * Called on every exit from the profile's mode, including a failure path, so
     * it must tolerate being called when `start()` never ran.
     */
    esp_err_t (*stop)(void *ctx);

    /**
     * @brief A button event, already debounced by `drivers/buttons`.
     *
     * The profile translates it into whatever its far end understands — for
     * `usound` a USB HID consumer-control report. Optional; NULL means the
     * profile ignores controls. `main/` consumes the long-press override before
     * this is called, so a profile never sees it.
     */
    void (*on_button)(buttons_event_t evt, const buttons_event_info_t *info, void *ctx);

    /**
     * @brief Is the far end actually carrying audio right now?
     *
     * Drives the `IDLE` transition: `main/` polls it and drops the mode when it
     * goes false. Optional; NULL means "assume live until stopped".
     */
    bool (*is_live)(void *ctx);

    /** Passed back to every callback above. */
    void *ctx;
} headset_profile_t;

#ifdef __cplusplus
}
#endif
