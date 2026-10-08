/**
 * @file hs_ui.h
 * @brief The headset's user interface on whatever board it runs: inputs (switches by role,
 *        the jack, the thumbwheel) as events, and the status LED as named patterns.
 *
 * Product policy (ADR-013) over drivers/buttons, drivers/encoder and drivers/rgb_led,
 * configured from `board_desc()` (ADR-025). What an input *does* is main/'s decision; this
 * module only says what happened.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HS_IN_ACTION_PRESS = 0,  /**< Action (MFB) pressed.                                 */
    HS_IN_ACTION_LONG,       /**< Action held past the long-press time.                 */
    HS_IN_VOL_UP,            /**< One step: a button press/repeat or one wheel detent.  */
    HS_IN_VOL_DOWN,
    HS_IN_POWER_SHORT,       /**< Power pressed and released before the long press.     */
    HS_IN_POWER_HOLD,        /**< Power held for CONFIG_HS_UI_POWER_OFF_HOLD_MS. Once.  */
    HS_IN_JACK_IN,           /**< Boom microphone plugged (also reported at boot).      */
    HS_IN_JACK_OUT,
} hs_input_t;

/** Runs on the buttons or the encoder task: post, never act. */
typedef void (*hs_input_cb_t)(hs_input_t in, void *ctx);

typedef enum {
    HS_LED_OFF = 0,
    HS_LED_BOOT,             /**< White fade-in.                                         */
    HS_LED_IDLE,             /**< Dim green blip every few seconds.                      */
    HS_LED_USB,              /**< Dim white, steady: wired mode.                         */
    HS_LED_ADVERTISING,      /**< Blue blink: looking for the dongle.                    */
    HS_LED_LINKED,           /**< Dim blue, steady: dongle linked.                       */
    HS_LED_CHARGING,         /**< Amber breathing (with external power, not full).       */
    HS_LED_CHARGED,          /**< Green, steady (with external power, full).             */
    HS_LED_LOW_BATTERY,      /**< Red blink.                                             */
    HS_LED_LEVEL_HIGH,       /**< Battery level on a power short-press: green.           */
    HS_LED_LEVEL_MID,        /**< ... amber.                                             */
    HS_LED_LEVEL_LOW,        /**< ... red.                                               */
    HS_LED_POWER_OFF,        /**< Red fade-out.                                          */
    HS_LED_PATTERN_COUNT,
} hs_led_t;

/**
 * @brief Set up and start every input the board has, and the LED if it has one.
 *        Requires `pm_policy_init()` and `power_init()` (drivers/buttons).
 * @return ESP_OK, or the first input error (the LED is optional and never fails this).
 */
esp_err_t hs_ui_init(hs_input_cb_t cb, void *ctx);

/** @brief Is the power switch held right now? false on boards without one. */
bool hs_ui_power_pressed(void);

/** @brief The steady pattern the LED returns to. No-op without an LED. Any task. */
void hs_ui_led(hs_led_t pattern);

/** @brief Show `pattern` for `ms`, then return to the steady one. Any task. */
void hs_ui_led_flash(hs_led_t pattern, uint32_t ms);

#ifdef __cplusplus
}
#endif
