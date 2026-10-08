/**
 * @file power_latch.h
 * @brief Soft power latch: hold the board's supply enable asserted, and drop it to power off.
 *
 * Realises FW-LAT-001..007. A *control driver* (SDD §15.3): no data path, no task, no ISR,
 * so its lifecycle is `init` / `deinit` / `stats` and DR5/DR6 do not apply.
 *
 * ## The circuit this serves
 *
 * A push button raises the regulator enable through one diode of a diode-OR; the MCU must
 * then drive the other input (the *hold* pin) before the button is released, or power
 * collapses. A pull-down on the hold pin means anything that floats it — any reset, a
 * crash, a watchdog — also cuts power. ORGA v1: `PWR_HOLD` on GPIO21, 100 kΩ pull-down,
 * BAT54C into the TPS63001 EN (`boards/orga_v1/doc/index.md`).
 *
 * ## Surviving resets
 *
 * `init()` drives the pin and enables the pad hold (`gpio_hold_en()`), which latches the level
 * through software resets, panics and watchdog resets, so an OTA reboot does not power the
 * board off (DES-LAT-002). A power-on reset clears the hold: the board's bootloader hook
 * asserts the pin within tens of milliseconds, and the user's button press covers the gap.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit/release` are not safe concurrently with each other. `is_held()` and
 * `stats()` are safe from any task. Nothing is ISR-safe.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      hold_gpio;           /**< Required. Output-capable; RTC-capable keeps the hold
                                       across the widest set of resets.                    */
    bool     active_low;          /**< false (default): driving high keeps power on.       */
    uint16_t release_timeout_ms;  /**< How long `release()` waits for power to collapse.
                                       0 selects Kconfig (2000 ms).                        */
} power_latch_config_t;

typedef struct {
    uint32_t release_attempts;    /**< `release()` calls that dropped the pin.             */
    uint32_t release_timeouts;    /**< ...after which power was still there.               */
    bool     held;                /**< The pin is asserted and the pad hold is on.         */
} power_latch_stats_t;

/**
 * @brief Assert the hold pin and latch it with the pad hold.
 *
 * Glitch-free when the pin is already asserted — by the bootloader hook or by a hold that
 * survived a reset: the output level is written before the pad is configured.
 *
 * @return ESP_OK; ESP_ERR_INVALID_STATE if already initialised; ESP_ERR_INVALID_ARG for
 *         NULL or a pin that cannot drive an output; propagated GPIO errors.
 */
esp_err_t power_latch_init(const power_latch_config_t *cfg);

/**
 * @brief Stop managing the pin. **Power stays on**: the pin is left asserted and held, so
 *        tearing the driver down can never be what switches the device off.
 */
esp_err_t power_latch_deinit(void);

/**
 * @brief Power off: release the pad hold, drive the pin inactive, and wait for the supply
 *        to collapse.
 *
 * On a board whose latch works, this does not return. If power is still present after the
 * timeout — the user is still holding the power button, or the board has no latch (a
 * devkit) — the pin is asserted and held again and `ESP_ERR_TIMEOUT` is returned, so the
 * caller can retry once the button is released.
 *
 * The caller is responsible for quiescing the system first (audio muted, flash writes
 * complete): this function is the last thing the firmware does.
 *
 * @return ESP_ERR_TIMEOUT (power persisted), ESP_ERR_INVALID_STATE (not initialised).
 */
esp_err_t power_latch_release(void);

/** @brief True while the pin is asserted and held. */
bool power_latch_is_held(void);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t power_latch_stats(power_latch_stats_t *out);

#ifdef __cplusplus
}
#endif
