#include "lsm6dsv16x.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "diag.h"
#include "lsm6dsv16x_reg.h"
#include "pm_policy.h"

static const char *TAG = "lsm6dsv16x";

#define I2C_TIMEOUT_MS   50
#define XL_ODR_ACTIVE    LSM6DSV16X_ODR_AT_60Hz   /* motion engine; SLEEP_DUR counts 512/60 s */
#define XL_ODR_ACTIVE_HZ 60u
#define GY_ODR_POSE      LSM6DSV16X_ODR_AT_120Hz  /* SFLP needs the sensors at or above its rate */
#define XL_ODR_POSE      LSM6DSV16X_ODR_AT_120Hz

typedef enum { ST_UNINIT = 0, ST_READY, ST_RUNNING } state_t;

static volatile state_t        s_state;
static lsm6dsv16x_config_t     s_cfg;
static i2c_master_dev_handle_t s_dev;
static stmdev_ctx_t            s_ctx;
static SemaphoreHandle_t       s_mutex;
static TaskHandle_t            s_task;
static pm_policy_lock_handle_t s_lock;
static bool                    s_isr_added;
static lsm6dsv16x_stats_t      s_stats;

/* -------------------------------------------------------------------------- */
/* Pure                                                                        */
/* -------------------------------------------------------------------------- */

float lsm6dsv16x_half_to_float(uint16_t h)
{
    uint32_t sign = (uint32_t)(h >> 15) & 1u;
    uint32_t exp = (uint32_t)(h >> 10) & 0x1Fu;
    uint32_t man = (uint32_t)h & 0x3FFu;
    float v;
    if (exp == 0) {
        v = ldexpf((float)man, -24);                       /* subnormal: man x 2^-24 */
    } else if (exp == 31) {
        v = (man == 0) ? INFINITY : NAN;
    } else {
        v = ldexpf((float)(man | 0x400u), (int)exp - 25);  /* 1.man x 2^(exp-15) */
    }
    return sign ? -v : v;
}

void lsm6dsv16x_sflp_to_quat(const uint8_t raw[6], lsm6dsv16x_quat_t *q)
{
    float x = lsm6dsv16x_half_to_float((uint16_t)(raw[0] | (raw[1] << 8)));
    float y = lsm6dsv16x_half_to_float((uint16_t)(raw[2] | (raw[3] << 8)));
    float z = lsm6dsv16x_half_to_float((uint16_t)(raw[4] | (raw[5] << 8)));
    float s = x * x + y * y + z * z;
    if (s > 1.0f) {
        /* Half precision can overshoot the unit sphere by an LSB; renormalise. */
        float n = sqrtf(s);
        x /= n;
        y /= n;
        z /= n;
        s = 1.0f;
    }
    q->x = x;
    q->y = y;
    q->z = z;
    q->w = sqrtf(1.0f - s);
}

/* -------------------------------------------------------------------------- */
/* ST driver binding (DES-IMU-001)                                             */
/* -------------------------------------------------------------------------- */

static int32_t plat_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint8_t tmp[17];
    if (len > sizeof(tmp) - 1) {
        return -1;   /* the ST driver writes at most a few bytes at once */
    }
    tmp[0] = reg;
    memcpy(&tmp[1], buf, len);
    return i2c_master_transmit(s_dev, tmp, len + 1u, I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

static int32_t plat_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

static void plat_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
}

/* ST calls return 0 on success, a sum of failures otherwise. */
static esp_err_t st(int32_t rc)
{
    if (rc != 0) {
        s_stats.i2c_errors++;
        return ESP_FAIL;
    }
    return ESP_OK;
}

#define TRY(x) do { esp_err_t e_ = (x); if (e_ != ESP_OK) { return e_; } } while (0)

/* -------------------------------------------------------------------------- */
/* Configuration (DES-IMU-003, DES-IMU-004)                                    */
/* -------------------------------------------------------------------------- */

static lsm6dsv16x_sflp_data_rate_t sflp_rate(uint8_t hz)
{
    switch (hz) {
    case 15:  return LSM6DSV16X_SFLP_15Hz;
    case 60:  return LSM6DSV16X_SFLP_60Hz;
    case 120: return LSM6DSV16X_SFLP_120Hz;
    default:  return LSM6DSV16X_SFLP_30Hz;
    }
}

static esp_err_t configure_static(void)
{
    TRY(st(lsm6dsv16x_sw_reset(&s_ctx)));
    TRY(st(lsm6dsv16x_block_data_update_set(&s_ctx, 1)));
    TRY(st(lsm6dsv16x_xl_full_scale_set(&s_ctx, LSM6DSV16X_4g)));
    TRY(st(lsm6dsv16x_gy_full_scale_set(&s_ctx, LSM6DSV16X_2000dps)));

    /* Motion engine: wake/inactivity thresholds at 15.625 mg/LSB, sleep after SLEEP_DUR x
     * 512/ODR_XL, accelerometer to 1.875 Hz while still. */
    uint32_t ths = (s_cfg.motion_threshold_mg * 1000u + 7812u) / 15625u;
    if (ths < 1) {
        ths = 1;
    } else if (ths > 63) {
        ths = 63;
    }
    lsm6dsv16x_act_thresholds_t t = {
        .inactivity_cfg = { .wu_inact_ths_w = 1, .xl_inact_odr = 0, .inact_dur = 0, .sleep_status_on_int = 0 },
        .inactivity_ths = (uint8_t)ths,
        .threshold = (uint8_t)ths,
        .duration = 0,
    };
    TRY(st(lsm6dsv16x_act_thresholds_set(&s_ctx, &t)));
    uint32_t quiet = (s_cfg.still_time_s * XL_ODR_ACTIVE_HZ + 256u) / 512u;
    if (quiet < 1) {
        quiet = 1;
    } else if (quiet > 15) {
        quiet = 15;
    }
    lsm6dsv16x_act_wkup_time_windows_t w = { .shock = 0, .quiet = (uint8_t)quiet };
    TRY(st(lsm6dsv16x_act_wkup_time_windows_set(&s_ctx, w)));

    /* Latched interrupts: INT1 stays high until the sources are read. */
    lsm6dsv16x_interrupt_mode_t im = { .enable = 1, .lir = 1 };
    TRY(st(lsm6dsv16x_interrupt_enable_set(&s_ctx, im)));
    lsm6dsv16x_pin_int_route_t r = { 0 };
    r.sleep_change = 1;
    r.fifo_th = 1;
    TRY(st(lsm6dsv16x_pin_int1_route_set(&s_ctx, &r)));

    /* FIFO carries only SFLP game rotation, one word per watermark. */
    TRY(st(lsm6dsv16x_fifo_watermark_set(&s_ctx, 1)));
    TRY(st(lsm6dsv16x_fifo_mode_set(&s_ctx, LSM6DSV16X_BYPASS_MODE)));
    return ESP_OK;
}

static esp_err_t pose_apply(bool on)
{
    if (on) {
        TRY(st(lsm6dsv16x_act_mode_set(&s_ctx, LSM6DSV16X_XL_AND_GY_NOT_AFFECTED)));
        TRY(st(lsm6dsv16x_xl_data_rate_set(&s_ctx, XL_ODR_POSE)));
        TRY(st(lsm6dsv16x_gy_data_rate_set(&s_ctx, GY_ODR_POSE)));
        TRY(st(lsm6dsv16x_sflp_data_rate_set(&s_ctx, sflp_rate(s_cfg.pose_rate_hz))));
        TRY(st(lsm6dsv16x_sflp_game_rotation_set(&s_ctx, 1)));
        lsm6dsv16x_fifo_sflp_raw_t b = { .game_rotation = 1 };
        TRY(st(lsm6dsv16x_fifo_sflp_batch_set(&s_ctx, b)));
        TRY(st(lsm6dsv16x_fifo_mode_set(&s_ctx, LSM6DSV16X_STREAM_MODE)));
    } else {
        lsm6dsv16x_fifo_sflp_raw_t b = { 0 };
        TRY(st(lsm6dsv16x_fifo_mode_set(&s_ctx, LSM6DSV16X_BYPASS_MODE)));
        TRY(st(lsm6dsv16x_fifo_sflp_batch_set(&s_ctx, b)));
        TRY(st(lsm6dsv16x_sflp_game_rotation_set(&s_ctx, 0)));
        TRY(st(lsm6dsv16x_gy_data_rate_set(&s_ctx, LSM6DSV16X_ODR_OFF)));
        TRY(st(lsm6dsv16x_xl_data_rate_set(&s_ctx, XL_ODR_ACTIVE)));
        TRY(st(lsm6dsv16x_act_mode_set(&s_ctx, LSM6DSV16X_XL_LOW_POWER_GY_POWER_DOWN)));
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* ISR and task (DES-IMU-005)                                                  */
/* -------------------------------------------------------------------------- */

static void int1_isr(void *arg)
{
    /* Level interrupt: mask it until the task has read (and so cleared) the sources. */
    gpio_intr_disable(s_cfg.int1_gpio);
    BaseType_t woken = pdFALSE;
    if (s_task != NULL) {
        vTaskNotifyGiveFromISR(s_task, &woken);
    }
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void drain_fifo(void)
{
    lsm6dsv16x_fifo_status_t fs;
    if (st(lsm6dsv16x_fifo_status_get(&s_ctx, &fs)) != ESP_OK) {
        return;
    }
    if (fs.fifo_ovr || fs.fifo_full) {
        s_stats.fifo_overruns++;
    }
    for (uint16_t i = 0; i < fs.fifo_level; i++) {
        lsm6dsv16x_fifo_out_raw_t o;
        if (st(lsm6dsv16x_fifo_out_raw_get(&s_ctx, &o)) != ESP_OK) {
            break;
        }
        if (o.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG) {
            lsm6dsv16x_quat_t q;
            lsm6dsv16x_sflp_to_quat(o.data, &q);
            s_stats.pose_samples++;
            if (s_cfg.on_pose != NULL) {
                s_cfg.on_pose(&q, s_cfg.ctx);
            }
        }
    }
}

static void imu_task(void *arg)
{
    for (;;) {
        /* The timeout is a safety net for a missed interrupt while streaming, and the
         * watchdog feed while idle. */
        uint32_t n = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(s_stats.pose_enabled ? 100 : 1000));
        if (s_state != ST_RUNNING) {
            continue;
        }
        if (n > 0) {
            s_stats.int_wakeups++;
        }
        bool motion_changed = false;
        bool moving = s_stats.moving;
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        lsm6dsv16x_all_sources_t src;
        if (st(lsm6dsv16x_all_sources_get(&s_ctx, &src)) == ESP_OK) {   /* clears the latch */
            if (src.sleep_change) {
                moving = !src.sleep_state;
                motion_changed = (moving != s_stats.moving);
                s_stats.moving = moving;
            }
            if (s_stats.pose_enabled) {
                drain_fifo();
            }
        }
        xSemaphoreGive(s_mutex);
        gpio_intr_enable(s_cfg.int1_gpio);

        if (motion_changed) {
            s_stats.motion_events++;
            if (s_cfg.on_motion != NULL) {
                s_cfg.on_motion(moving, s_cfg.ctx);
            }
        }
        UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
        if (s_stats.task_stack_free_min == 0 || hw < s_stats.task_stack_free_min) {
            s_stats.task_stack_free_min = (uint32_t)hw;
        }
        diag_task_feed();
    }
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

static void release(void)
{
    if (s_isr_added) {
        (void)gpio_isr_handler_remove(s_cfg.int1_gpio);
        (void)gpio_reset_pin(s_cfg.int1_gpio);
        s_isr_added = false;
    }
    if (s_dev != NULL) {
        (void)i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
}

esp_err_t lsm6dsv16x_init(const lsm6dsv16x_config_t *cfg)
{
    if (s_state != ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || cfg->bus == NULL || !GPIO_IS_VALID_GPIO(cfg->int1_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->pose_rate_hz != 0 && cfg->pose_rate_hz != 15 && cfg->pose_rate_hz != 30 &&
        cfg->pose_rate_hz != 60 && cfg->pose_rate_hz != 120) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    if (s_cfg.i2c_addr == 0) {
        s_cfg.i2c_addr = LSM6DSV16X_I2C_ADDR_SA0_LOW;
    }
    if (s_cfg.i2c_hz == 0) {
        s_cfg.i2c_hz = CONFIG_LSM6DSV16X_I2C_HZ;
    }
    if (s_cfg.motion_threshold_mg == 0) {
        s_cfg.motion_threshold_mg = CONFIG_LSM6DSV16X_MOTION_THRESHOLD_MG;
    }
    if (s_cfg.still_time_s == 0) {
        s_cfg.still_time_s = CONFIG_LSM6DSV16X_STILL_TIME_S;
    }
    if (s_cfg.pose_rate_hz == 0) {
        s_cfg.pose_rate_hz = CONFIG_LSM6DSV16X_POSE_RATE_HZ;
    }

    esp_err_t err = pm_policy_lock_create("imu", &s_lock);
    if (err != ESP_OK) {
        return err;
    }
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();   /* Kept across deinit. */
        if (s_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (i2c_master_probe(s_cfg.bus, s_cfg.i2c_addr, I2C_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "no ACK at 0x%02x", s_cfg.i2c_addr);
        return ESP_ERR_NOT_FOUND;
    }
    const i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_cfg.i2c_addr,
        .scl_speed_hz = s_cfg.i2c_hz,
    };
    TRY(i2c_master_bus_add_device(s_cfg.bus, &dcfg, &s_dev));
    s_ctx = (stmdev_ctx_t){ .write_reg = plat_write, .read_reg = plat_read, .mdelay = plat_delay, .handle = NULL };

    memset(&s_stats, 0, sizeof(s_stats));
    uint8_t id = 0;
    if (st(lsm6dsv16x_device_id_get(&s_ctx, &id)) != ESP_OK || id != LSM6DSV16X_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "WHO_AM_I 0x%02x, want 0x%02x", id, LSM6DSV16X_WHO_AM_I_VALUE);
        release();
        return ESP_ERR_INVALID_RESPONSE;
    }
    err = configure_static();
    if (err == ESP_OK) {
        const gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_cfg.int1_gpio,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* INT1 is push-pull; this only covers a missing chip */
            .intr_type = GPIO_INTR_HIGH_LEVEL,
        };
        err = gpio_config(&io);
    }
    if (err == ESP_OK) {
        esp_err_t svc = gpio_install_isr_service(0);
        if (svc != ESP_OK && svc != ESP_ERR_INVALID_STATE) {
            err = svc;
        }
    }
    if (err == ESP_OK) {
        err = gpio_isr_handler_add(s_cfg.int1_gpio, int1_isr, NULL);
        s_isr_added = (err == ESP_OK);
    }
    if (err == ESP_OK) {
        err = gpio_intr_disable(s_cfg.int1_gpio);
    }
    if (err != ESP_OK) {
        release();
        return err;
    }

    s_state = ST_READY;
    BaseType_t ok = xTaskCreatePinnedToCore(imu_task, "imu", CONFIG_LSM6DSV16X_TASK_STACK, NULL,
                                            CONFIG_LSM6DSV16X_TASK_PRIORITY, &s_task, CONFIG_LSM6DSV16X_TASK_CORE);
    if (ok != pdPASS) {
        s_state = ST_UNINIT;
        s_task = NULL;
        release();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "init: 0x%02x, INT1 GPIO%d, motion > %u mg, still after %u s (x%u), pose %u Hz",
             s_cfg.i2c_addr, s_cfg.int1_gpio, s_cfg.motion_threshold_mg, s_cfg.still_time_s,
             (unsigned)XL_ODR_ACTIVE_HZ, s_cfg.pose_rate_hz);
    return ESP_OK;
}

esp_err_t lsm6dsv16x_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    (void)lsm6dsv16x_stop();
    if (s_task != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        TaskHandle_t t = s_task;
        s_task = NULL;
        vTaskDelete(t);
        xSemaphoreGive(s_mutex);
    }
    release();
    s_state = ST_UNINIT;
    return ESP_OK;
}

esp_err_t lsm6dsv16x_start(void)
{
    if (s_state != ST_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = st(lsm6dsv16x_xl_data_rate_set(&s_ctx, XL_ODR_ACTIVE));
    if (err == ESP_OK) {
        err = st(lsm6dsv16x_act_mode_set(&s_ctx, LSM6DSV16X_XL_LOW_POWER_GY_POWER_DOWN));
    }
    xSemaphoreGive(s_mutex);
    if (err != ESP_OK) {
        return err;
    }
    s_stats.moving = true;   /* assume worn-and-moving until the chip says otherwise */
    diag_task_register(s_task);
    s_stats.running = true;
    s_state = ST_RUNNING;
    (void)gpio_intr_enable(s_cfg.int1_gpio);
    return ESP_OK;
}

esp_err_t lsm6dsv16x_stop(void)
{
    if (s_state != ST_RUNNING) {
        return s_state == ST_UNINIT ? ESP_ERR_INVALID_STATE : ESP_OK;
    }
    (void)lsm6dsv16x_pose_enable(false);
    (void)gpio_intr_disable(s_cfg.int1_gpio);
    s_state = ST_READY;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    (void)st(lsm6dsv16x_act_mode_set(&s_ctx, LSM6DSV16X_XL_AND_GY_NOT_AFFECTED));
    (void)st(lsm6dsv16x_xl_data_rate_set(&s_ctx, LSM6DSV16X_ODR_OFF));
    (void)st(lsm6dsv16x_gy_data_rate_set(&s_ctx, LSM6DSV16X_ODR_OFF));
    xSemaphoreGive(s_mutex);
    diag_task_unregister(s_task);
    s_stats.running = false;
    return ESP_OK;
}

esp_err_t lsm6dsv16x_pose_enable(bool enable)
{
    if (s_state != ST_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }
    if (enable == s_stats.pose_enabled) {
        return ESP_OK;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = pose_apply(enable);
    if (err == ESP_OK) {
        s_stats.pose_enabled = enable;
    }
    xSemaphoreGive(s_mutex);
    if (err == ESP_OK) {
        (void)(enable ? pm_policy_lock_acquire(s_lock) : pm_policy_lock_release(s_lock));
    }
    return err;
}

esp_err_t lsm6dsv16x_read_accel_mg(float out_mg[3])
{
    if (out_mg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    int16_t raw[3];
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = st(lsm6dsv16x_acceleration_raw_get(&s_ctx, raw));
    xSemaphoreGive(s_mutex);
    if (err == ESP_OK) {
        for (int i = 0; i < 3; i++) {
            out_mg[i] = lsm6dsv16x_from_fs4_to_mg(raw[i]);
        }
    }
    return err;
}

bool lsm6dsv16x_is_moving(void)
{
    return s_stats.moving;
}

esp_err_t lsm6dsv16x_stats(lsm6dsv16x_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    out->running = (s_state == ST_RUNNING);
    return ESP_OK;
}
