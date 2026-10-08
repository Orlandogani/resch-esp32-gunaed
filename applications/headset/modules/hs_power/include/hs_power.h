/**
 * @file hs_power.h
 * @brief The headset's power: keep the soft latch held, watch the battery, switch off.
 *
 * Product policy (ADR-013) over drivers/power_latch and drivers/battery, configured from
 * `board_desc()` (ADR-025). On a board without a latch or a battery (the devkit) every
 * call is a harmless no-op and the device is "always on, always on external power".
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t mv;
    uint8_t  percent;
    bool     ext_power;   /**< Charger input good (USB plugged).                         */
    bool     charging;    /**< ext_power and below CONFIG_HS_POWER_FULL_MV (TBD-015).    */
    bool     low;         /**< On battery and at or under CONFIG_HS_POWER_LOW_PERCENT.   */
    bool     critical;    /**< On battery and at or under CONFIG_HS_POWER_CRITICAL_MV.   */
    bool     valid;
} hs_battery_t;

/** Runs on the battery task: post, never act. */
typedef void (*hs_power_cb_t)(const hs_battery_t *b, void *ctx);

/**
 * @brief The very first call in `app_main`: assert and latch the board's power hold, so
 *        the user may let go of the power button (DES-LAT-003). No-op without a latch.
 */
esp_err_t hs_power_hold(void);

/** @brief Start the battery monitor. No-op (ESP_OK) without a battery. */
esp_err_t hs_power_init(hs_power_cb_t cb, void *ctx);

/** @brief The latest battery state; `valid` is false before the first sample or without
 *         a battery. */
void hs_power_get(hs_battery_t *out);

/** @brief Does this board switch itself off (has a latch)? */
bool hs_power_has_latch(void);

/**
 * @brief Cut power. The caller has already quiesced everything (audio muted, profile
 *        stopped, LED shown). Waits up to `CONFIG_HS_POWER_RELEASE_WAIT_MS` for
 *        `still_pressed()` to return false — the latch cannot drop while the power button
 *        itself holds the regulator on — then releases the latch.
 *
 * @return Does not return on a board whose latch works. ESP_ERR_NOT_SUPPORTED without a
 *         latch; ESP_ERR_TIMEOUT if power persisted (the caller carries on).
 */
esp_err_t hs_power_off(bool (*still_pressed)(void));

#ifdef __cplusplus
}
#endif
