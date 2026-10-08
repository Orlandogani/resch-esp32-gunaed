/**
 * Headset sensing: wear (motion heuristic) and head pose, from the board's IMU.
 */
#include "hs_sense.h"

#include "esp_log.h"

#include "board.h"
#include "lsm6dsv16x.h"

static const char *TAG = "hs_sense";

static struct {
    bool                present;
    volatile bool       worn;
    hs_sense_wear_cb_t  on_wear;
    hs_sense_pose_cb_t  on_pose;
    void               *ctx;
} s_s = { .worn = true };

static void on_motion(bool moving, void *ctx)
{
    (void)ctx;
    s_s.worn = moving;
    if (s_s.on_wear != NULL) {
        s_s.on_wear(moving, s_s.ctx);
    }
}

static void on_pose(const lsm6dsv16x_quat_t *q, void *ctx)
{
    (void)ctx;
    if (s_s.on_pose != NULL) {
        s_s.on_pose(q->w, q->x, q->y, q->z, s_s.ctx);
    }
}

esp_err_t hs_sense_init(hs_sense_wear_cb_t on_wear, hs_sense_pose_cb_t pose_cb, void *ctx)
{
    s_s.on_wear = on_wear;
    s_s.on_pose = pose_cb;
    s_s.ctx = ctx;
    const board_desc_t *b = board_desc();
    if (!b->imu.present) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    i2c_master_bus_handle_t bus;
    esp_err_t err = board_i2c_bus(&bus);
    if (err != ESP_OK) {
        return err;
    }
    const lsm6dsv16x_config_t c = {
        .bus = bus,
        .i2c_addr = b->imu.i2c_addr,
        .int1_gpio = b->imu.int1,
        .on_motion = on_motion,
        .on_pose = on_pose,
    };
    err = lsm6dsv16x_init(&c);
    if (err == ESP_OK) {
        err = lsm6dsv16x_start();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "IMU: %s — wear and head pose unavailable", esp_err_to_name(err));
        return err;
    }
    s_s.present = true;
    return ESP_OK;
}

bool hs_sense_worn(void)
{
    return !s_s.present || s_s.worn;
}

esp_err_t hs_sense_pose(bool enable)
{
    return s_s.present ? lsm6dsv16x_pose_enable(enable) : ESP_OK;
}
