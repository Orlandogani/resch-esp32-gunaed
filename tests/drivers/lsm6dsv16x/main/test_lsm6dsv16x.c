/**
 * On-target Unity tests for drivers/lsm6dsv16x (FW-IMU-001..011).
 *
 * On a devkit there is no IMU: the suite checks the pure SFLP decoding, argument validation
 * and the absent-chip path. [needs_orga] cases need the real board (CONFIG_TEST_ON_ORGA=y).
 * I2C on GPIO 8/9 and INT1 on GPIO 11 — the ORGA v1 wiring, free on a DevKitC.
 */
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "unity.h"

#include "cfg.h"
#include "diag.h"
#include "lsm6dsv16x.h"
#include "pm_policy.h"
#include "power.h"

#define PIN_SDA  8
#define PIN_SCL  9
#define PIN_INT1 11

static i2c_master_bus_handle_t s_bus;
static volatile uint32_t       s_poses;
static lsm6dsv16x_quat_t       s_last_q;

static i2c_master_bus_handle_t bus(void)
{
    if (s_bus == NULL) {
        const i2c_master_bus_config_t c = {
            .i2c_port = -1, .sda_io_num = PIN_SDA, .scl_io_num = PIN_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
            .flags = { .enable_internal_pullup = true },
        };
        TEST_ASSERT_EQUAL(ESP_OK, i2c_new_master_bus(&c, &s_bus));
    }
    return s_bus;
}

static void fresh(void)
{
    (void)lsm6dsv16x_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    TEST_ASSERT_EQUAL(ESP_OK, diag_init(NULL));
}

static void on_pose(const lsm6dsv16x_quat_t *q, void *ctx)
{
    (void)ctx;
    s_last_q = *q;
    s_poses++;
}

static lsm6dsv16x_config_t base_cfg(void)
{
    lsm6dsv16x_config_t c = { .bus = bus(), .int1_gpio = PIN_INT1, .on_pose = on_pose };
    return c;
}

TEST_CASE("half-float decoding", "[lsm6dsv16x]")
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, lsm6dsv16x_half_to_float(0x0000));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, lsm6dsv16x_half_to_float(0x3C00));
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, lsm6dsv16x_half_to_float(0xC000));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, lsm6dsv16x_half_to_float(0x3800));
    TEST_ASSERT_EQUAL_FLOAT(65504.0f, lsm6dsv16x_half_to_float(0x7BFF));
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, 5.9604645e-8f, lsm6dsv16x_half_to_float(0x0001));  /* subnormal */
    TEST_ASSERT_TRUE(isinf(lsm6dsv16x_half_to_float(0x7C00)));
}

TEST_CASE("SFLP word to unit quaternion", "[lsm6dsv16x]")
{
    lsm6dsv16x_quat_t q;
    const uint8_t identity[6] = { 0 };
    lsm6dsv16x_sflp_to_quat(identity, &q);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, q.w);

    /* x = 0.5, y = 0.5, z = 0.5 -> w = 0.5 */
    const uint8_t half[6] = { 0x00, 0x38, 0x00, 0x38, 0x00, 0x38 };
    lsm6dsv16x_sflp_to_quat(half, &q);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, q.w);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, q.x);

    /* Slightly outside the unit sphere: renormalised, w = 0. */
    const uint8_t over[6] = { 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00 };   /* x = 1.000977 */
    lsm6dsv16x_sflp_to_quat(over, &q);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, q.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.w);
}

TEST_CASE("init validates its arguments", "[lsm6dsv16x]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, lsm6dsv16x_init(NULL));
    lsm6dsv16x_config_t c = base_cfg(); c.bus = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, lsm6dsv16x_init(&c));
    c = base_cfg(); c.int1_gpio = -1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, lsm6dsv16x_init(&c));
    c = base_cfg(); c.pose_rate_hz = 50;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, lsm6dsv16x_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, lsm6dsv16x_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, lsm6dsv16x_pose_enable(true));
    float a[3];
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, lsm6dsv16x_read_accel_mg(a));
}

TEST_CASE("absent IMU: NOT_FOUND, nothing left initialised", "[lsm6dsv16x][no_imu]")
{
    fresh();
    lsm6dsv16x_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, lsm6dsv16x_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, lsm6dsv16x_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, lsm6dsv16x_start());
}

TEST_CASE("ORGA: gravity is about 1 g at rest", "[lsm6dsv16x][needs_orga]")
{
    fresh();
    lsm6dsv16x_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_start());
    vTaskDelay(pdMS_TO_TICKS(100));
    float a[3];
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_read_accel_mg(a));
    float g = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    printf("accel %.0f %.0f %.0f mg, |g| %.0f mg\n", (double)a[0], (double)a[1], (double)a[2], (double)g);
    TEST_ASSERT_FLOAT_WITHIN(150.0f, 1000.0f, g);
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_deinit());
}

TEST_CASE("ORGA: orientation streams unit quaternions at the set rate", "[lsm6dsv16x][needs_orga]")
{
    fresh();
    lsm6dsv16x_config_t c = base_cfg();
    c.pose_rate_hz = 30;
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_start());
    s_poses = 0;
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_pose_enable(true));
    vTaskDelay(pdMS_TO_TICKS(2000));
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_pose_enable(false));
    printf("poses in 2 s: %u, last w=%.3f x=%.3f y=%.3f z=%.3f\n", (unsigned)s_poses, (double)s_last_q.w,
           (double)s_last_q.x, (double)s_last_q.y, (double)s_last_q.z);
    TEST_ASSERT_UINT32_WITHIN(15, 60, s_poses);
    float n = s_last_q.w * s_last_q.w + s_last_q.x * s_last_q.x + s_last_q.y * s_last_q.y + s_last_q.z * s_last_q.z;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, n);
    lsm6dsv16x_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_stats(&st));
    TEST_ASSERT_EQUAL(0, st.i2c_errors);
    TEST_ASSERT_EQUAL(ESP_OK, lsm6dsv16x_deinit());
}

void app_main(void)
{
    /* Let a USB-Serial/JTAG capture attach first (.claude/BACKLOG.md). */
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
#if CONFIG_TEST_ON_ORGA
    unity_run_tests_by_tag("[no_imu]", true);
#else
    unity_run_tests_by_tag("[needs_orga]", true);
#endif
    UNITY_END();
}
