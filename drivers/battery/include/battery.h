/**
 * @file battery.h
 * @brief Battery voltage through a resistive divider into ADC1, state of charge from an
 *        open-circuit-voltage table, and external-power (charger "power good") detection.
 *
 * Realises FW-BAT-001..010 and the driver contract of SDD §15.3. For boards without a fuel
 * gauge (ORGA v1: 1 MΩ/1 MΩ divider on GPIO1, BQ24074 PGOOD on GPIO47).
 *
 * ## Measurement
 *
 * Each sample is `CONFIG_BATTERY_ADC_MULTISAMPLE` oneshot conversions averaged, converted to
 * millivolts by ESP-IDF's eFuse calibration (curve fitting on the S3), scaled by the divider,
 * then smoothed by a moving average of `average` samples. ADC1 only: ADC2 is unusable while
 * the radio runs.
 *
 * ## State of charge
 *
 * Linear interpolation in a caller-supplied OCV table (`percent` against millivolts), or a
 * generic single-cell Li-ion curve by default. Voltage-based SoC is an estimate: it reads
 * high while charging (the charge current through the cell's resistance) and low under load.
 * The application knows which applies — `ext_power` is reported for that reason.
 *
 * ## Events
 *
 * The task samples every `sample_period_ms`, and at once when the power-good input changes.
 * The callback runs when the percentage or the external-power state changes.
 *
 * ## Sleep
 *
 * No sleep lock: a conversion is a few hundred microseconds inside a task that is otherwise
 * blocked, which light sleep does not disturb.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit/start/stop` are not safe concurrently with each other. `get()`,
 * `sample_now()` and `stats()` are safe from any task. The callback runs on the battery task.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One point of an open-circuit-voltage curve. Tables run from full to empty. */
typedef struct {
    uint16_t mv;
    uint8_t  percent;
} battery_ocv_point_t;

typedef struct {
    uint16_t mv;          /**< Smoothed battery voltage.                                 */
    uint8_t  percent;     /**< From the OCV table, 0..100.                               */
    bool     ext_power;   /**< External power present (charger input good).              */
    bool     valid;       /**< At least one sample has been taken.                       */
} battery_status_t;

typedef void (*battery_event_cb_t)(const battery_status_t *st, void *ctx);

typedef struct {
    int      adc_gpio;              /**< Required. Must map to ADC1.                       */
    uint16_t divider_num;           /**< V_bat = V_pin × num / den. 1 MΩ/1 MΩ is 2/1.    */
    uint16_t divider_den;           /**< 0 for either selects 1/1.                         */
    int      ext_power_gpio;        /**< Power-good input, or -1 for none.                 */
    bool     ext_power_active_low;  /**< Open-drain "good" pulling low (BQ2407x PGOOD).    */
    const battery_ocv_point_t *ocv; /**< Table, descending in mV; NULL selects the default
                                         Li-ion curve. Must outlive the driver.            */
    size_t   ocv_count;             /**< Points in `ocv`, >= 2 when `ocv` is set.          */
    uint32_t sample_period_ms;      /**< 0 selects Kconfig (5000).                         */
    uint8_t  average;               /**< Moving-average length, 1..16. 0 selects Kconfig.  */
    battery_event_cb_t cb;          /**< Optional.                                         */
    void    *ctx;
} battery_config_t;

typedef struct {
    uint32_t samples;             /**< Samples taken.                                       */
    uint32_t adc_errors;          /**< Conversions that failed.                             */
    uint32_t ext_power_edges;     /**< Power-good interrupts.                               */
    uint32_t events;              /**< Callbacks delivered.                                 */
    uint16_t mv_min;              /**< Lowest smoothed reading since start (0: none yet).   */
    uint16_t mv_max;
    uint32_t task_stack_free_min; /**< Words. From uxTaskGetStackHighWaterMark().           */
    bool     calibrated;          /**< eFuse calibration in use (else a nominal curve).     */
    bool     running;
} battery_stats_t;

/**
 * @brief Claim ADC1 and the channel, set up calibration and the power-good input, create
 *        the task. Sampling begins at `start()`.
 * @return ESP_OK; ESP_ERR_INVALID_STATE if already initialised; ESP_ERR_INVALID_ARG for NULL,
 *         a pin that is not on ADC1, a bad table, or an average over 16; propagated ADC/GPIO
 *         errors; ESP_ERR_NO_MEM.
 */
esp_err_t battery_init(const battery_config_t *cfg);

/** @brief Stop if running, delete the task, release the ADC and the input. Idempotent. */
esp_err_t battery_deinit(void);

/** @brief Take a first sample at once, then every period. */
esp_err_t battery_start(void);

/** @brief Stop sampling; the last status is kept. Idempotent. */
esp_err_t battery_stop(void);

/** @brief The latest status. `valid` is false until the first sample. */
esp_err_t battery_get(battery_status_t *out);

/**
 * @brief Take one sample on the calling task, outside the schedule, and return it.
 *        Works while stopped (e.g. a low-battery check at boot). Blocks ~1 ms.
 */
esp_err_t battery_sample_now(battery_status_t *out);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t battery_stats(battery_stats_t *out);

/**
 * @brief State of charge for `mv` by linear interpolation in `tbl` (descending mV),
 *        clamped to the table's ends. The default table is used when `tbl` is NULL.
 */
uint8_t battery_percent_from_mv(const battery_ocv_point_t *tbl, size_t n, uint16_t mv);

#ifdef __cplusplus
}
#endif
