/**
 * @file rgb_led.h
 * @brief One RGB status LED on three LEDC PWM channels: set, and hardware fade.
 *
 * Realises FW-LED-001..008. A *control driver* (SDD §15.3): no task, no ISR of its own;
 * its lifecycle is `init` / `deinit` / `stats`. What the colours *mean* is application
 * policy (ADR-013) — this module only renders them.
 *
 * ## Rendering
 *
 * A requested 8-bit level `v` on a channel becomes the duty
 * `gamma(v) × gain_permille / 1000` of the PWM period, where `gamma(v)` is `v` or, with
 * `CONFIG_RGB_LED_GAMMA`, `v² / 255` (a cheap perceptual curve). The per-channel gain is the
 * board's knob for unequal LED efficiencies and for colours whose forward voltage sits close
 * to the supply (ORGA v1: green and blue at Vf ≈ 3.2 V from 3.3 V). Common-anode wiring is
 * `active_low`: LEDC inverts the output in hardware, so 0 is dark on either wiring.
 *
 * ## Sleep
 *
 * LEDC runs from the APB clock, which stops in light sleep. The driver therefore holds a
 * `pm_policy` lock named `"rgb_led"` while any channel is lit (DR8), and releases it when the
 * LED goes dark — at the end of a fade to black, not at its start.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit` are not safe concurrently with anything. `set/fade/get/stats` are safe from
 * any task (internal mutex). Nothing is ISR-safe.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t r, g, b;
} rgb_led_color_t;

#define RGB_LED_OFF ((rgb_led_color_t){ 0, 0, 0 })

typedef struct {
    int      gpio_r;              /**< Red channel pin, or -1 if absent.               */
    int      gpio_g;              /**< Green channel pin, or -1 if absent.             */
    int      gpio_b;              /**< Blue channel pin, or -1 if absent.              */
    bool     active_low;          /**< Common anode to the supply: low lights the LED. */
    uint16_t gain_permille[3];    /**< R, G, B scale, 1..1000. 0 selects Kconfig.      */
    uint32_t pwm_hz;              /**< 0 selects Kconfig (5 kHz).                      */
} rgb_led_config_t;

typedef struct {
    uint32_t sets;                /**< `set()` calls that reached the hardware.        */
    uint32_t fades;               /**< `fade()` calls that started a hardware fade.    */
    uint32_t errors;              /**< LEDC calls that failed.                         */
    rgb_led_color_t color;        /**< Last requested colour (the fade target).        */
    bool     lit;                 /**< The sleep lock is held.                         */
} rgb_led_stats_t;

/**
 * @brief Configure one LEDC timer and up to three channels (all dark), install the fade
 *        service, create the `"rgb_led"` sleep lock.
 *
 * Uses LEDC timer `CONFIG_RGB_LED_LEDC_TIMER` and channels
 * `CONFIG_RGB_LED_LEDC_CHANNEL_BASE` .. +2 exclusively (DR10). Requires `pm_policy_init()`.
 *
 * @return ESP_OK; ESP_ERR_INVALID_STATE if already initialised or `pm_policy` is not;
 *         ESP_ERR_INVALID_ARG for NULL, no channel at all, a pin that cannot drive an output,
 *         or a gain over 1000; propagated LEDC errors.
 */
esp_err_t rgb_led_init(const rgb_led_config_t *cfg);

/** @brief Turn the LED off, release the pins and the LEDC channels. */
esp_err_t rgb_led_deinit(void);

/** @brief Show `c` now, cancelling any fade in progress. */
esp_err_t rgb_led_set(rgb_led_color_t c);

/**
 * @brief Fade from the current output to `c` over `ms` in hardware, without blocking.
 *        `ms` = 0 is `set()`.
 */
esp_err_t rgb_led_fade(rgb_led_color_t c, uint32_t ms);

/** @brief The last requested colour (a fade's target, not where it is now). */
esp_err_t rgb_led_get(rgb_led_color_t *out);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t rgb_led_stats(rgb_led_stats_t *out);

/**
 * @brief The duty a level renders to, for a channel's gain — the pure part of the
 *        driver, exposed so the curve can be tested and reasoned about.
 * @return Duty in LEDC counts at `CONFIG_RGB_LED_DUTY_BITS` resolution.
 */
uint32_t rgb_led_level_to_duty(uint8_t level, uint16_t gain_permille);

#ifdef __cplusplus
}
#endif
