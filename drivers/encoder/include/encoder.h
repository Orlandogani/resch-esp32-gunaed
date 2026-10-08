/**
 * @file encoder.h
 * @brief Quadrature rotary encoder (thumbwheel) on the PCNT peripheral: detent steps as events.
 *
 * Realises FW-ENC-001..009 and the driver contract of SDD §15.3.
 *
 * ## Counting
 *
 * Both edges of both phases count (×4 decoding): channel 0 counts A's edges with B as the
 * direction level, channel 1 counts B's edges with A as the level. The unit's limits are
 * ±`counts_per_detent` with accumulation on, so every detent reaches a limit; the limit is a
 * watch point, its interrupt only wakes the task (DR5), and the task turns the accumulated
 * count into signed detent steps. A wheel at rest between detents emits nothing.
 *
 * Contact bounce is rejected twice: PCNT's glitch filter (`glitch_ns`), and the fact that a
 * bounce which reverses the count before a limit never becomes a step.
 *
 * ## Sleep
 *
 * PCNT is clocked from APB and does not count in light sleep. The driver deliberately takes
 * **no** sleep lock (a deviation from DR8, recorded in DES-ENC-005): a thumbwheel is idle
 * nearly always, and a lock would forbid sleep for good. Turns made while asleep are lost;
 * an application that wants to wake on a turn registers a GPIO wake on a phase pin.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit/start/stop` are not safe concurrently with each other. `set_event_cb()`,
 * `position()` and `stats()` are safe from any task. The callback runs on the encoder task.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Step sink. Runs on the encoder task, never in ISR context.
 * @param steps  Signed detents since the last call: positive clockwise (A leads B, or the
 *               reverse when `reverse` is set). Several fast detents may arrive as one call.
 */
typedef void (*encoder_step_cb_t)(int32_t steps, void *ctx);

typedef struct {
    int      gpio_a;              /**< Phase A. Required, input-capable.                   */
    int      gpio_b;              /**< Phase B. Required, input-capable, not `gpio_a`.     */
    bool     pull_up;             /**< Enable internal pull-ups (common-to-ground parts
                                       with no external pull-ups).                         */
    bool     reverse;             /**< Swap the sense of rotation.                          */
    uint8_t  counts_per_detent;   /**< Edges per detent, ×4 decoding. 0 selects Kconfig (4:
                                       one full quadrature cycle per click).               */
    uint16_t glitch_ns;           /**< Pulses shorter than this are ignored. 0 selects
                                       Kconfig (10000 ns). Capped by hardware at ~12.7 µs. */
    encoder_step_cb_t cb;         /**< Optional; may be set later.                         */
    void    *ctx;
} encoder_config_t;

typedef struct {
    int32_t  position;            /**< Net detents since init.                              */
    uint32_t steps_cw;            /**< Detents clockwise.                                   */
    uint32_t steps_ccw;           /**< Detents counter-clockwise.                           */
    uint32_t isr_wakeups;         /**< Watch-point interrupts that woke the task.           */
    uint32_t events_dropped;      /**< Steps that occurred with no callback installed.      */
    uint32_t task_stack_free_min; /**< Words. From uxTaskGetStackHighWaterMark().           */
    bool     running;
} encoder_stats_t;

/**
 * @brief Create the PCNT unit and its two channels, configure the pins, create the task.
 *        The unit is not counting until `start()`.
 * @return ESP_OK; ESP_ERR_INVALID_STATE if already initialised; ESP_ERR_INVALID_ARG for NULL,
 *         a missing, invalid or duplicate pin, or `counts_per_detent` > 64; ESP_ERR_NO_MEM;
 *         propagated PCNT/GPIO errors (ESP_ERR_NOT_FOUND: no free PCNT unit).
 */
esp_err_t encoder_init(const encoder_config_t *cfg);

/** @brief Stop if running, delete the task, the channels and the unit. Idempotent. */
esp_err_t encoder_deinit(void);

/** @brief Enable and start counting. ESP_ERR_INVALID_STATE unless initialised and stopped. */
esp_err_t encoder_start(void);

/** @brief Stop counting; position is kept. Idempotent. */
esp_err_t encoder_stop(void);

/** @brief Replace the callback atomically. NULL detaches (steps are then counted as dropped). */
esp_err_t encoder_set_event_cb(encoder_step_cb_t cb, void *ctx);

/** @brief Net detents since init. 0 before init. */
int32_t encoder_position(void);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t encoder_stats(encoder_stats_t *out);

#ifdef __cplusplus
}
#endif
