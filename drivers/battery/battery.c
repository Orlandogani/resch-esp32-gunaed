#include "battery.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include "diag.h"

static const char *TAG = "battery";

#define AVG_MAX 16

/* Generic single-cell Li-ion/LiPo rest curve, 4.20 V full. A board with a characterised
 * cell supplies its own (TBD-016). */
static const battery_ocv_point_t k_default_ocv[] = {
    { 4200, 100 }, { 4150, 95 }, { 4110, 90 }, { 4080, 85 }, { 4020, 80 }, { 3980, 75 },
    { 3950, 70 },  { 3910, 65 }, { 3870, 60 }, { 3850, 55 }, { 3840, 50 }, { 3820, 45 },
    { 3800, 40 },  { 3790, 35 }, { 3770, 30 }, { 3750, 25 }, { 3730, 20 }, { 3710, 15 },
    { 3690, 10 },  { 3610, 5 },  { 3270, 0 },
};

typedef enum { ST_UNINIT = 0, ST_READY, ST_RUNNING } state_t;

static volatile state_t          s_state;
static battery_config_t          s_cfg;
static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t             s_chan;
static adc_cali_handle_t         s_cali;
static TaskHandle_t              s_task;
static SemaphoreHandle_t         s_mutex;      /* ADC and status access. */
static uint16_t                  s_hist[AVG_MAX];
static uint8_t                   s_hist_n, s_hist_pos;
static battery_status_t          s_status;
static battery_stats_t           s_stats;
static bool                      s_isr_added;

/* -------------------------------------------------------------------------- */
/* Pure helpers                                                                */
/* -------------------------------------------------------------------------- */

uint8_t battery_percent_from_mv(const battery_ocv_point_t *tbl, size_t n, uint16_t mv)
{
    if (tbl == NULL || n < 2) {
        tbl = k_default_ocv;
        n = sizeof(k_default_ocv) / sizeof(k_default_ocv[0]);
    }
    if (mv >= tbl[0].mv) {
        return tbl[0].percent;
    }
    if (mv <= tbl[n - 1].mv) {
        return tbl[n - 1].percent;
    }
    for (size_t i = 1; i < n; i++) {
        if (mv >= tbl[i].mv) {
            const battery_ocv_point_t *hi = &tbl[i - 1], *lo = &tbl[i];
            uint32_t span = (uint32_t)(hi->mv - lo->mv);
            uint32_t frac = (uint32_t)(mv - lo->mv) * (uint32_t)(hi->percent - lo->percent);
            return (uint8_t)(lo->percent + (frac + span / 2) / span);
        }
    }
    return tbl[n - 1].percent;
}

static bool table_ok(const battery_ocv_point_t *t, size_t n)
{
    if (n < 2) {
        return false;
    }
    for (size_t i = 0; i < n; i++) {
        if (t[i].percent > 100 || (i > 0 && (t[i].mv >= t[i - 1].mv || t[i].percent > t[i - 1].percent))) {
            return false;
        }
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/* Measurement (DES-BAT-002)                                                   */
/* -------------------------------------------------------------------------- */

static bool read_ext_power(void)
{
    if (s_cfg.ext_power_gpio < 0) {
        return false;
    }
    return (gpio_get_level(s_cfg.ext_power_gpio) == 0) == s_cfg.ext_power_active_low;
}

/* Called with s_mutex held. Returns true if the percentage or ext_power changed. */
static bool sample_locked(void)
{
    int32_t acc = 0;
    int ok = 0;
    for (int i = 0; i < CONFIG_BATTERY_ADC_MULTISAMPLE; i++) {
        int v = 0;
        esp_err_t err = (s_cali != NULL) ? adc_oneshot_get_calibrated_result(s_adc, s_cali, s_chan, &v)
                                         : adc_oneshot_read(s_adc, s_chan, &v);
        if (err == ESP_OK) {
            if (s_cali == NULL) {
                v = v * 3100 / 4095;   /* nominal 12 dB full scale without eFuse data */
            }
            acc += v;
            ok++;
        } else {
            s_stats.adc_errors++;
        }
    }
    if (ok == 0) {
        return false;
    }
    uint32_t pin_mv = (uint32_t)(acc / ok);
    uint32_t bat_mv = pin_mv * s_cfg.divider_num / s_cfg.divider_den;
    if (bat_mv > UINT16_MAX) {
        bat_mv = UINT16_MAX;
    }

    s_hist[s_hist_pos] = (uint16_t)bat_mv;
    s_hist_pos = (uint8_t)((s_hist_pos + 1) % s_cfg.average);
    if (s_hist_n < s_cfg.average) {
        s_hist_n++;
    }
    uint32_t sum = 0;
    for (uint8_t i = 0; i < s_hist_n; i++) {
        sum += s_hist[i];
    }
    uint16_t mv = (uint16_t)(sum / s_hist_n);

    battery_status_t next = {
        .mv = mv,
        .percent = battery_percent_from_mv(s_cfg.ocv, s_cfg.ocv_count, mv),
        .ext_power = read_ext_power(),
        .valid = true,
    };
    bool changed = !s_status.valid || next.percent != s_status.percent || next.ext_power != s_status.ext_power;
    s_status = next;
    s_stats.samples++;
    if (s_stats.mv_min == 0 || mv < s_stats.mv_min) {
        s_stats.mv_min = mv;
    }
    if (mv > s_stats.mv_max) {
        s_stats.mv_max = mv;
    }
    return changed;
}

/* -------------------------------------------------------------------------- */
/* ISR and task                                                                */
/* -------------------------------------------------------------------------- */

static void ext_power_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    s_stats.ext_power_edges++;
    if (s_task != NULL) {
        vTaskNotifyGiveFromISR(s_task, &woken);
    }
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void battery_task(void *arg)
{
    for (;;) {
        uint32_t edge = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(s_cfg.sample_period_ms));
        if (s_state != ST_RUNNING) {
            continue;
        }
        if (edge > 0) {
            /* Let a plugging connector settle before reading the level (DES-BAT-004). */
            vTaskDelay(pdMS_TO_TICKS(CONFIG_BATTERY_EXT_POWER_SETTLE_MS));
            (void)ulTaskNotifyTake(pdTRUE, 0);
        }
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        bool changed = sample_locked();
        battery_status_t snap = s_status;
        xSemaphoreGive(s_mutex);
        if (changed && s_cfg.cb != NULL) {
            s_stats.events++;
            s_cfg.cb(&snap, s_cfg.ctx);
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

static void release_hw(void)
{
    if (s_isr_added) {
        (void)gpio_isr_handler_remove(s_cfg.ext_power_gpio);
        s_isr_added = false;
    }
    if (s_cfg.ext_power_gpio >= 0) {
        (void)gpio_reset_pin(s_cfg.ext_power_gpio);
    }
    if (s_cali != NULL) {
        (void)adc_cali_delete_scheme_curve_fitting(s_cali);
        s_cali = NULL;
    }
    if (s_adc != NULL) {
        (void)adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
    }
}

esp_err_t battery_init(const battery_config_t *cfg)
{
    if (s_state != ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || cfg->average > AVG_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->ocv != NULL && !table_ok(cfg->ocv, cfg->ocv_count)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->ext_power_gpio >= 0 && !GPIO_IS_VALID_GPIO(cfg->ext_power_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    adc_unit_t unit;
    if (adc_oneshot_io_to_channel(cfg->adc_gpio, &unit, &s_chan) != ESP_OK || unit != ADC_UNIT_1) {
        return ESP_ERR_INVALID_ARG;   /* FW-BAT-002: ADC1 only */
    }

    s_cfg = *cfg;
    if (s_cfg.divider_num == 0 || s_cfg.divider_den == 0) {
        s_cfg.divider_num = 1;
        s_cfg.divider_den = 1;
    }
    if (s_cfg.sample_period_ms == 0) {
        s_cfg.sample_period_ms = CONFIG_BATTERY_SAMPLE_PERIOD_MS;
    }
    if (s_cfg.average == 0) {
        s_cfg.average = CONFIG_BATTERY_AVERAGE;
    }
    if (s_cfg.ocv == NULL) {
        s_cfg.ocv = k_default_ocv;
        s_cfg.ocv_count = sizeof(k_default_ocv) / sizeof(k_default_ocv[0]);
    }

    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();   /* Kept across deinit. */
        if (s_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    const adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    esp_err_t err = adc_oneshot_new_unit(&ucfg, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit: %s", esp_err_to_name(err));
        s_adc = NULL;
        return err;
    }
    const adc_oneshot_chan_cfg_t ccfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    err = adc_oneshot_config_channel(s_adc, s_chan, &ccfg);
    if (err != ESP_OK) {
        release_hw();
        return err;
    }
    const adc_cali_curve_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1, .chan = s_chan, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) != ESP_OK) {
        s_cali = NULL;
        ESP_LOGW(TAG, "no ADC calibration in eFuse; readings use a nominal curve");
    }

    if (s_cfg.ext_power_gpio >= 0) {
        const gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_cfg.ext_power_gpio,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,   /* the board pulls up; a duplicate is harmless but hides a missing one */
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };
        err = gpio_config(&io);
        if (err == ESP_OK) {
            /* The shared ISR service may already be installed by drivers/buttons. */
            esp_err_t svc = gpio_install_isr_service(0);
            if (svc != ESP_OK && svc != ESP_ERR_INVALID_STATE) {
                err = svc;
            }
        }
        if (err == ESP_OK) {
            err = gpio_isr_handler_add(s_cfg.ext_power_gpio, ext_power_isr, NULL);
            s_isr_added = (err == ESP_OK);
        }
        if (err == ESP_OK) {
            err = gpio_intr_disable(s_cfg.ext_power_gpio);   /* until start() */
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "power-good input GPIO%d: %s", s_cfg.ext_power_gpio, esp_err_to_name(err));
            release_hw();
            return err;
        }
    }

    memset(&s_stats, 0, sizeof(s_stats));
    memset(&s_status, 0, sizeof(s_status));
    s_stats.calibrated = (s_cali != NULL);
    s_hist_n = 0;
    s_hist_pos = 0;
    s_state = ST_READY;

    BaseType_t ok = xTaskCreatePinnedToCore(battery_task, "battery", CONFIG_BATTERY_TASK_STACK, NULL,
                                            CONFIG_BATTERY_TASK_PRIORITY, &s_task, CONFIG_BATTERY_TASK_CORE);
    if (ok != pdPASS) {
        s_state = ST_UNINIT;
        s_task = NULL;
        release_hw();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "init: ADC1 ch%d (GPIO%d) x%u/%u, power-good GPIO%d, %u ms, avg %u, %s",
             (int)s_chan, s_cfg.adc_gpio, s_cfg.divider_num, s_cfg.divider_den, s_cfg.ext_power_gpio,
             (unsigned)s_cfg.sample_period_ms, s_cfg.average, s_cali ? "calibrated" : "uncalibrated");
    return ESP_OK;
}

esp_err_t battery_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    (void)battery_stop();
    if (s_task != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);   /* never delete it inside a sample */
        TaskHandle_t t = s_task;
        s_task = NULL;
        vTaskDelete(t);
        xSemaphoreGive(s_mutex);
    }
    release_hw();
    s_state = ST_UNINIT;
    return ESP_OK;
}

esp_err_t battery_start(void)
{
    if (s_state != ST_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_isr_added) {
        (void)gpio_intr_enable(s_cfg.ext_power_gpio);
    }
    diag_task_register(s_task);
    s_stats.running = true;
    s_state = ST_RUNNING;
    xTaskNotifyGive(s_task);   /* first sample now, not one period from now */
    return ESP_OK;
}

esp_err_t battery_stop(void)
{
    if (s_state != ST_RUNNING) {
        return s_state == ST_UNINIT ? ESP_ERR_INVALID_STATE : ESP_OK;
    }
    s_state = ST_READY;
    if (s_isr_added) {
        (void)gpio_intr_disable(s_cfg.ext_power_gpio);
    }
    diag_task_unregister(s_task);
    s_stats.running = false;
    return ESP_OK;
}

esp_err_t battery_get(battery_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t battery_sample_now(battery_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    (void)sample_locked();
    *out = s_status;
    xSemaphoreGive(s_mutex);
    return out->valid ? ESP_OK : ESP_FAIL;
}

esp_err_t battery_stats(battery_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    out->running = (s_state == ST_RUNNING);
    return ESP_OK;
}
