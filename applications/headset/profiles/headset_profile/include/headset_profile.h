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
 * Controls arrive as `headset_ctrl_t`, not as switch indices: the board decides what
 * hardware produces them (ADR-025).
 *
 * This component is deliberately header-only and deliberately *not* in `subsys/`:
 * it is product policy, not reusable SDK mechanism (ADR-013).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
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
 * @brief A user control, already resolved from whatever the board has — buttons, the
 *        thumbwheel encoder — by `main/` (`modules/hs_ui`). A profile never sees a switch
 *        index or a GPIO, so the same profile runs on every board (ADR-025).
 */
typedef enum {
    HEADSET_CTRL_VOL_UP = 0,
    HEADSET_CTRL_VOL_DOWN,
    HEADSET_CTRL_PLAY_PAUSE,
    HEADSET_CTRL_MIC_MUTE,
} headset_ctrl_t;

/** Something the headset learned about itself that the far end may want to know. */
typedef enum {
    HEADSET_STATUS_BATTERY = 0,   /**< `battery` is valid.                                */
    HEADSET_STATUS_WEAR,          /**< `worn` is valid.                                   */
    HEADSET_STATUS_HEAD_POSE,     /**< `pose` is valid. Only to profiles with
                                       HEADSET_PROFILE_CAP_HEAD_POSE, while worn.         */
} headset_status_kind_t;

typedef struct {
    headset_status_kind_t kind;
    union {
        struct {
            uint8_t percent;
            bool    ext_power;    /**< Charger input present.                             */
            bool    charging;     /**< ext_power and not yet full (inferred, TBD-015).    */
        } battery;
        bool worn;
        struct {
            float w, x, y, z;     /**< Unit quaternion, game rotation (no magnetometer).  */
        } pose;
    };
} headset_status_t;

/** Capabilities a profile declares; `main/` only does the work a profile can use. */
#define HEADSET_PROFILE_CAP_HEAD_POSE (1u << 0)

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
     * @brief A user control (one step: a held volume key or a fast wheel arrives as
     *        several calls).
     *
     * The profile translates it into whatever its far end understands — for `usound`
     * a USB HID consumer-control report, for `wlink` a CONTROL message to the dongle.
     * Optional; NULL means the profile ignores controls. `main/` consumes the mode and
     * power controls first, so a profile never sees them. Called on the main task.
     */
    void (*on_control)(headset_ctrl_t ctrl, void *ctx);

    /**
     * @brief Battery, wear and head-pose updates for the far end. Optional.
     *
     * Battery and wear arrive on the main task; head pose arrives on the IMU task at the
     * sensor's rate and must not block. Only called between `start()` and `stop()`.
     */
    void (*on_status)(const headset_status_t *st, void *ctx);

    /** HEADSET_PROFILE_CAP_* bits. */
    uint32_t caps;

    /**
     * @brief Is the far end actually carrying audio right now?
     *
     * Drives the `IDLE` transition: `main/` polls it and drops the mode when it
     * goes false. Optional; NULL means "assume live until stopped".
     */
    bool (*is_live)(void *ctx);

    /**
     * @brief Is the far end actually there — the dongle linked, the host enumerated —
     *        as opposed to merely waiting for it? Drives the status LED only. Optional;
     *        NULL means "same as is_live".
     */
    bool (*is_connected)(void *ctx);

    /** Passed back to every callback above. */
    void *ctx;
} headset_profile_t;

#ifdef __cplusplus
}
#endif
