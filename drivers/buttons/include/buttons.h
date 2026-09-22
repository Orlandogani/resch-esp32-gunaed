/**
 * @file buttons.h
 * @brief Momentary switches on GPIO inputs: debounce, press/release, long-press and
 *        auto-repeat events, delivered by callback on the driver's own task.
 *
 * Realises FW-BTN-001..014 and the driver contract of SDD §15.3 (DR1–DR10). The
 * driver knows nothing about what a button *means* — volume, power, mode — that is
 * application policy (ADR-013). It reports edges and durations; the application
 * translates them.
 *
 * ## Lifecycle (DR1)
 *
 *     buttons_init(cfg) ─► buttons_start() ─► buttons_stop() ─► buttons_deinit()
 *                                 ▲                 │
 *                                 └─── repeatable ──┘
 *
 * ## Data path
 *
 *     GPIO edge ─(ISR: notify only)─► buttons task ─► sample all pins ─► debounce ─► callback
 *
 * The ISR does nothing but wake the task (FW-BTN-007, DR5). The task then samples
 * every configured pin at `CONFIG_BUTTONS_SAMPLE_MS` until all buttons are released
 * and settled, and blocks again — waking once per `CONFIG_BUTTONS_IDLE_POLL_MS` to
 * re-sample and feed the watchdog, so a missed edge is recovered within one period
 * (FW-BTN-013). A level is accepted only after it has been stable for the debounce
 * time; reversals inside that window are discarded and counted (FW-BTN-002).
 *
 * ## Sleep
 *
 * A `pm_policy` lock named `"buttons"` is held from the first edge of a press until
 * every button is released and settled, and never while idle (FW-BTN-008). An idle
 * device is free to sleep; a press in progress is not cut in half by sleep entry.
 * A pin flagged `wake_from_deep_sleep` is registered with `power` as an EXT1 wake
 * source, so a press ends deep sleep (FW-BTN-009); the resulting boot's first sample
 * sees the button held and reports it as a normal press.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit/start/stop` are not safe concurrently with each other.
 * `set_event_cb()`, `is_pressed()` and `stats()` are safe from any task at any time.
 * Nothing is ISR-safe. The event callback runs on the buttons task at
 * `CONFIG_BUTTONS_TASK_PRIORITY`; it must not block for long, since every button's
 * timing is measured on that task. The callback may call `buttons_stop()` and
 * `buttons_set_event_cb()`; it must not call `buttons_deinit()`, which deletes the
 * task it is running on.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BUTTONS_EVENT_PRESSED = 0,  /**< Debounced active edge.                                         */
    BUTTONS_EVENT_RELEASED,     /**< Debounced inactive edge. `held_ms` is the press duration.       */
    BUTTONS_EVENT_LONG_PRESS,   /**< Held past `long_press_ms`. Once per press.                      */
    BUTTONS_EVENT_REPEAT,       /**< Every `repeat_ms` after LONG_PRESS while held, if `auto_repeat`.*/
} buttons_event_t;

typedef struct {
    uint8_t  index;    /**< Position in the `pins` array given at init.                          */
    int      gpio;     /**< That entry's GPIO, for convenience.                                  */
    uint32_t held_ms;  /**< Milliseconds since the debounced press. 0 for PRESSED.               */
} buttons_event_info_t;

/**
 * @brief Event sink. Runs on the buttons task, never in ISR context.
 * @param info  Valid only for the duration of the call.
 */
typedef void (*buttons_event_cb_t)(buttons_event_t evt, const buttons_event_info_t *info, void *ctx);

typedef struct {
    int  gpio;                  /**< Required. Any input-capable GPIO.                                 */
    bool active_low;            /**< true: pressed reads 0 (switch to ground, pull-up). false: reads 1.*/
    bool pull_enable;           /**< Enable the internal pull opposing the active level.               */
    bool wake_from_deep_sleep;  /**< Register as an EXT1 wake source. Needs an RTC-capable GPIO
                                     (ESP32-S3: 0..21) and `power_init()` beforehand.                  */
} buttons_pin_config_t;

typedef struct {
    const buttons_pin_config_t *pins;  /**< Copied at init; need not outlive the call.               */
    uint8_t  count;                    /**< 1..CONFIG_BUTTONS_MAX.                                    */
    uint16_t debounce_ms;              /**< 0 selects Kconfig (20).                                  */
    uint16_t long_press_ms;            /**< 0 selects Kconfig (800).                                 */
    uint16_t repeat_ms;                /**< 0 selects Kconfig (200).                                 */
    bool     auto_repeat;              /**< Emit REPEAT while held past the long-press threshold.    */
    buttons_event_cb_t cb;             /**< Optional; may also be set later with `buttons_set_event_cb()`. */
    void    *ctx;                      /**< Passed to every callback.                                */
} buttons_config_t;

typedef struct {
    uint32_t presses;             /**< PRESSED events emitted.                                       */
    uint32_t releases;            /**< RELEASED events emitted.                                      */
    uint32_t long_presses;        /**< LONG_PRESS events emitted.                                    */
    uint32_t repeats;             /**< REPEAT events emitted.                                        */
    uint32_t bounces_rejected;    /**< Raw reversals inside the debounce window, discarded.          */
    uint32_t events_dropped;      /**< Events that occurred with no callback installed.              */
    uint32_t isr_wakeups;         /**< Edge interrupts that woke the task.                           */
    uint32_t task_stack_free_min; /**< Words. From uxTaskGetStackHighWaterMark().                    */
    bool     running;
} buttons_stats_t;

/**
 * @brief Copy the pin table, configure every GPIO as an input with its pull and an
 *        any-edge interrupt (interrupts left disabled until `start()`), register
 *        deep-sleep wake pins with `power`, create the task and the `"buttons"`
 *        sleep lock.
 *
 * Requires `pm_policy_init()` first, and `power_init()` if any pin sets
 * `wake_from_deep_sleep` (FW-SYS-023). `diag_init()` is optional; without it the
 * task simply is not watchdog-supervised.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  already initialised, or `pm_policy`/`power` not initialised
 *   - ESP_ERR_INVALID_ARG    NULL cfg or pins, count 0 or > CONFIG_BUTTONS_MAX, a GPIO
 *                            that is not a valid input, a duplicate GPIO, or a
 *                            wake pin that is not RTC-capable
 *   - ESP_ERR_NO_MEM         task creation failed or the lock table is full
 *   - (propagated)           GPIO driver errors
 *
 * @note Thread-safety: not safe concurrently with any other lifecycle call.
 *       ISR-safety: no. Blocking: briefly (lock table mutex).
 */
esp_err_t buttons_init(const buttons_config_t *cfg);

/**
 * @brief Stop if running, remove ISR handlers, reset every pin to its default state,
 *        deregister wake sources, delete the task. The sleep lock is kept
 *        (`pm_policy` locks are never destroyed) and is released if held.
 *
 * @return ESP_OK, also when never initialised.
 * @note Thread-safety: not safe concurrently with any other lifecycle call.
 */
esp_err_t buttons_deinit(void);

/**
 * @brief Enable the edge interrupts and begin delivering events. Registers the task
 *        with the `diag` watchdog when `diag` is initialised.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not initialised or already running.
 * @note Thread-safety: not safe concurrently with any other lifecycle call.
 */
esp_err_t buttons_start(void);

/**
 * @brief Disable the interrupts; no further events. Releases the sleep lock if a
 *        press was in progress. Idempotent once initialised.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not initialised.
 * @note Thread-safety: not safe concurrently with any other lifecycle call.
 */
esp_err_t buttons_stop(void);

/**
 * @brief Install, replace, or (with NULL) remove the event callback. Takes effect for
 *        the next event; an event in flight completes with the previous callback.
 *        Events that occur while no callback is installed are dropped and counted
 *        (FW-BTN-006). This is the hand-off point when ownership of the buttons moves
 *        between application modes.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE before init.
 * @note Thread-safety: safe from any task. ISR-safety: no. Blocking: never.
 */
esp_err_t buttons_set_event_cb(buttons_event_cb_t cb, void *ctx);

/**
 * @brief Debounced state of one button.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG for NULL or an index ≥ count,
 *         ESP_ERR_INVALID_STATE before init.
 * @note Thread-safety: safe from any task. ISR-safety: no. Blocking: never.
 */
esp_err_t buttons_is_pressed(uint8_t index, bool *pressed);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t buttons_stats(buttons_stats_t *out);

/** @brief Zero the counters. */
esp_err_t buttons_stats_reset(void);

#ifdef __cplusplus
}
#endif
