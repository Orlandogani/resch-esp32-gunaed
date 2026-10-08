/**
 * @file lsm6dsv16x.h
 * @brief ST LSM6DSV16X 6-axis IMU over I2C: motion/stillness events and head orientation.
 *
 * Realises FW-IMU-001..011 and the driver contract of SDD §15.3. Register access goes
 * through ST's own platform-independent driver, vendored unpatched in
 * `third_party/lsm6dsv16x` (ADR-027); this module adds the ESP-IDF binding, the lifecycle,
 * the interrupt handling and the two features the headset uses.
 *
 * ## Motion (always on while started)
 *
 * The chip's activity/inactivity engine watches the accelerometer: after `still_time_s`
 * with no movement above `motion_threshold_mg` it reports *still* (and drops its own
 * accelerometer to 1.875 Hz, and the gyroscope off when orientation is not running); the
 * first movement reports *moving*. Both edges arrive on INT1 as a latched interrupt.
 * Whether "still" means "taken off" is application policy (ADR-013) — this module reports
 * motion, not wear.
 *
 * ## Orientation (on request)
 *
 * `pose_enable(true)` turns on the gyroscope and the chip's sensor-fusion block (SFLP), which
 * produces a *game rotation vector* — orientation relative to where it started, no
 * magnetometer, so heading drifts slowly — into the FIFO at `pose_rate_hz`. The FIFO
 * watermark raises INT1; the task drains it and delivers unit quaternions.
 *
 * ## Interrupt handling
 *
 * INT1 is latched by the chip and taken as a *level* interrupt here: the ISR disables it and
 * wakes the task (DR5), the task reads the sources (which clears the latch) and re-enables
 * it. An edge that happens during light sleep is therefore not lost — the level is still
 * there when the CPU wakes.
 *
 * ## Sleep
 *
 * A `pm_policy` lock `"imu"` is held while orientation streams (DR8). Motion detection alone
 * takes none: it is a once-a-minute event the level interrupt survives.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit/start/stop` are not safe concurrently with each other. `pose_enable()`,
 * `read_accel_mg()`, `is_moving()` and `stats()` are safe from any task. Callbacks run on
 * the IMU task.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LSM6DSV16X_I2C_ADDR_SA0_LOW  0x6A
#define LSM6DSV16X_I2C_ADDR_SA0_HIGH 0x6B
#define LSM6DSV16X_WHO_AM_I_VALUE    0x70

/** Unit quaternion, game rotation vector. */
typedef struct {
    float w, x, y, z;
} lsm6dsv16x_quat_t;

typedef void (*lsm6dsv16x_motion_cb_t)(bool moving, void *ctx);
typedef void (*lsm6dsv16x_pose_cb_t)(const lsm6dsv16x_quat_t *q, void *ctx);

typedef struct {
    i2c_master_bus_handle_t bus;     /**< Required. Shared bus, owned by the board.         */
    uint8_t  i2c_addr;               /**< 0 selects 0x6A (SA0 low).                         */
    uint32_t i2c_hz;                 /**< 0 selects Kconfig (400 kHz).                      */
    int      int1_gpio;              /**< Required. INT1, push-pull active high.            */
    uint16_t motion_threshold_mg;    /**< 0 selects Kconfig (62 mg). Rounded to 15.625 mg.  */
    uint16_t still_time_s;           /**< Stillness before "still". 0 selects Kconfig (60 s).
                                          Quantised to 512/60 s steps, at most 128 s; an
                                          application wanting longer runs its own timer.   */
    uint8_t  pose_rate_hz;           /**< 15, 30, 60 or 120. 0 selects Kconfig (30).        */
    lsm6dsv16x_motion_cb_t on_motion;
    lsm6dsv16x_pose_cb_t   on_pose;
    void    *ctx;                    /**< Passed to both callbacks.                         */
} lsm6dsv16x_config_t;

typedef struct {
    uint32_t i2c_errors;             /**< ST driver calls that failed.                      */
    uint32_t int_wakeups;            /**< INT1 interrupts handled.                          */
    uint32_t motion_events;          /**< moving/still transitions delivered.               */
    uint32_t pose_samples;           /**< Quaternions delivered.                            */
    uint32_t fifo_overruns;          /**< FIFO overflowed before the task drained it.       */
    uint32_t task_stack_free_min;    /**< Words.                                            */
    bool     moving;
    bool     pose_enabled;
    bool     running;
} lsm6dsv16x_stats_t;

/**
 * @brief Probe (WHO_AM_I), software-reset, and configure the chip; set up INT1; create the
 *        task. Nothing is sensed until `start()`.
 * @return ESP_OK; ESP_ERR_INVALID_STATE (already initialised, or `pm_policy` not);
 *         ESP_ERR_INVALID_ARG (NULL, no bus, bad INT1 pin, bad pose rate);
 *         ESP_ERR_NOT_FOUND (no ACK); ESP_ERR_INVALID_RESPONSE (wrong WHO_AM_I);
 *         ESP_ERR_NO_MEM; propagated I2C/GPIO errors.
 */
esp_err_t lsm6dsv16x_init(const lsm6dsv16x_config_t *cfg);

/** @brief Stop, power the sensor down, release INT1, the device and the task. Idempotent. */
esp_err_t lsm6dsv16x_deinit(void);

/** @brief Turn the accelerometer and the motion engine on; enable INT1. */
esp_err_t lsm6dsv16x_start(void);

/** @brief Stop orientation, power both sensors down, disable INT1. Idempotent. */
esp_err_t lsm6dsv16x_stop(void);

/** @brief Start or stop the orientation stream (gyroscope + SFLP + FIFO). Needs `start()`. */
esp_err_t lsm6dsv16x_pose_enable(bool enable);

/** @brief One accelerometer reading in mg (bring-up and orientation checks). */
esp_err_t lsm6dsv16x_read_accel_mg(float out_mg[3]);

/** @brief The last reported motion state. */
bool lsm6dsv16x_is_moving(void);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t lsm6dsv16x_stats(lsm6dsv16x_stats_t *out);

/**
 * @brief IEEE 754 half-precision to float. Pure; the SFLP output format.
 */
float lsm6dsv16x_half_to_float(uint16_t h);

/**
 * @brief Decode one SFLP game-rotation FIFO word (x, y, z as little-endian halves) into a
 *        unit quaternion, w ≥ 0 reconstructed. Pure; exposed for testing.
 */
void lsm6dsv16x_sflp_to_quat(const uint8_t raw[6], lsm6dsv16x_quat_t *q);

#ifdef __cplusplus
}
#endif
