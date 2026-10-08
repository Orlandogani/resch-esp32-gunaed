/**
 * @file hs_sense.h
 * @brief The headset's sensing: is it being worn, and which way is the head pointing.
 *
 * Product policy (ADR-013) over drivers/lsm6dsv16x, configured from `board_desc()`.
 *
 * ## Wear (a heuristic, said plainly)
 *
 * An IMU cannot see a head. It sees movement: a worn headset is never perfectly still for
 * long, one on a desk is. So "worn" here means "moved within the IMU's still time"
 * (`CONFIG_LSM6DSV16X_STILL_TIME_S`, 60 s by default), and "off head" means "still for
 * that long". That is enough for the two uses it has — auto power-off after
 * `CONFIG_HS_POWER_AUTO_OFF_MIN`, and not streaming head pose to nobody — and not enough
 * for pausing playback the moment the headset comes off (that wants a proximity sensor).
 *
 * ## Head pose
 *
 * On request, unit quaternions at `CONFIG_LSM6DSV16X_POSE_RATE_HZ`, delivered on the IMU
 * task (DES-HSS-002).
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*hs_sense_wear_cb_t)(bool worn, void *ctx);
typedef void (*hs_sense_pose_cb_t)(float w, float x, float y, float z, void *ctx);

/**
 * @brief Start the IMU if the board has one. Without one, the headset counts as always
 *        worn and pose never arrives.
 * @return ESP_OK, ESP_ERR_NOT_SUPPORTED (no IMU on this board), or the driver's error.
 */
esp_err_t hs_sense_init(hs_sense_wear_cb_t on_wear, hs_sense_pose_cb_t on_pose, void *ctx);

/** @brief Worn, as last reported (true without an IMU). */
bool hs_sense_worn(void);

/** @brief Stream head pose or stop. No-op without an IMU. */
esp_err_t hs_sense_pose(bool enable);

#ifdef __cplusplus
}
#endif
