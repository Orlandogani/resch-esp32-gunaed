#include "encoder.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

#include "diag.h"

static const char *TAG = "encoder";

typedef enum { ST_UNINIT = 0, ST_READY, ST_RUNNING } state_t;

static volatile state_t    s_state;
static encoder_config_t    s_cfg;
static pcnt_unit_handle_t  s_unit;
static pcnt_channel_handle_t s_chan[2];
static TaskHandle_t        s_task;
static int32_t             s_detent_seen;   /* count / counts_per_detent, last emitted. */
static encoder_stats_t     s_stats;
static portMUX_TYPE        s_cb_lock = portMUX_INITIALIZER_UNLOCKED;
static encoder_step_cb_t   s_cb;
static void               *s_cb_ctx;

/* -------------------------------------------------------------------------- */
/* ISR: notify only (DR5)                                                      */
/* -------------------------------------------------------------------------- */

static bool on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    if (s_task != NULL) {
        vTaskNotifyGiveFromISR(s_task, &woken);
    }
    return woken == pdTRUE;
}

/* -------------------------------------------------------------------------- */
/* Task (DES-ENC-003)                                                          */
/* -------------------------------------------------------------------------- */

static void emit(int32_t steps)
{
    encoder_step_cb_t cb;
    void *ctx;
    portENTER_CRITICAL(&s_cb_lock);
    cb = s_cb;
    ctx = s_cb_ctx;
    portEXIT_CRITICAL(&s_cb_lock);
    if (cb != NULL) {
        cb(steps, ctx);
    } else {
        s_stats.events_dropped += (uint32_t)(steps < 0 ? -steps : steps);
    }
}

static void encoder_task(void *arg)
{
    for (;;) {
        /* The idle timeout only lets the watchdog see the task alive while the wheel rests. */
        uint32_t n = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (s_state == ST_RUNNING) {
            if (n > 0) {
                s_stats.isr_wakeups += n;
            }
            int count = 0;
            if (pcnt_unit_get_count(s_unit, &count) == ESP_OK) {
                /* C division truncates toward zero, so a wheel resting a few edges either
                 * side of a detent maps to that detent (DES-ENC-002). */
                int32_t detent = (int32_t)count / (int32_t)s_cfg.counts_per_detent;
                int32_t steps = detent - s_detent_seen;
                if (steps != 0) {
                    s_detent_seen = detent;
                    if (s_cfg.reverse) {
                        steps = -steps;
                    }
                    s_stats.position += steps;
                    if (steps > 0) {
                        s_stats.steps_cw += (uint32_t)steps;
                    } else {
                        s_stats.steps_ccw += (uint32_t)(-steps);
                    }
                    emit(steps);
                }
            }
            UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
            if (s_stats.task_stack_free_min == 0 || hw < s_stats.task_stack_free_min) {
                s_stats.task_stack_free_min = (uint32_t)hw;
            }
            diag_task_feed();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

static void teardown(void)
{
    for (unsigned i = 0; i < 2; i++) {
        if (s_chan[i] != NULL) {
            (void)pcnt_del_channel(s_chan[i]);
            s_chan[i] = NULL;
        }
    }
    if (s_unit != NULL) {
        (void)pcnt_del_unit(s_unit);
        s_unit = NULL;
    }
}

esp_err_t encoder_init(const encoder_config_t *cfg)
{
    if (s_state != ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || !GPIO_IS_VALID_GPIO(cfg->gpio_a) || !GPIO_IS_VALID_GPIO(cfg->gpio_b) ||
        cfg->gpio_a == cfg->gpio_b || cfg->counts_per_detent > 64) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    if (s_cfg.counts_per_detent == 0) {
        s_cfg.counts_per_detent = CONFIG_ENCODER_COUNTS_PER_DETENT;
    }
    if (s_cfg.glitch_ns == 0) {
        s_cfg.glitch_ns = CONFIG_ENCODER_GLITCH_NS;
    }

    const int limit = s_cfg.counts_per_detent;
    const pcnt_unit_config_t ucfg = {
        .low_limit = -limit,
        .high_limit = limit,
        .flags = { .accum_count = 1 },
    };
    esp_err_t err = pcnt_new_unit(&ucfg, &s_unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_new_unit: %s", esp_err_to_name(err));
        s_unit = NULL;
        return err;
    }

    const pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = s_cfg.glitch_ns };
    err = pcnt_unit_set_glitch_filter(s_unit, &fcfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "glitch filter %u ns: %s; continuing without", s_cfg.glitch_ns, esp_err_to_name(err));
    }

    /* x4 quadrature (DES-ENC-001): each phase's edges count, the other phase gives the sense. */
    const pcnt_chan_config_t a = { .edge_gpio_num = s_cfg.gpio_a, .level_gpio_num = s_cfg.gpio_b };
    const pcnt_chan_config_t b = { .edge_gpio_num = s_cfg.gpio_b, .level_gpio_num = s_cfg.gpio_a };
    err = pcnt_new_channel(s_unit, &a, &s_chan[0]);
    if (err == ESP_OK) {
        err = pcnt_new_channel(s_unit, &b, &s_chan[1]);
    }
    if (err == ESP_OK) {
        err = pcnt_channel_set_edge_action(s_chan[0], PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                           PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    }
    if (err == ESP_OK) {
        err = pcnt_channel_set_level_action(s_chan[0], PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    }
    if (err == ESP_OK) {
        err = pcnt_channel_set_edge_action(s_chan[1], PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                           PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    }
    if (err == ESP_OK) {
        err = pcnt_channel_set_level_action(s_chan[1], PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    }
    /* With accumulation on, the limits must be watch points; they are also our detents. */
    if (err == ESP_OK) {
        err = pcnt_unit_add_watch_point(s_unit, limit);
    }
    if (err == ESP_OK) {
        err = pcnt_unit_add_watch_point(s_unit, -limit);
    }
    if (err == ESP_OK) {
        const pcnt_event_callbacks_t cbs = { .on_reach = on_reach };
        err = pcnt_unit_register_event_callbacks(s_unit, &cbs, NULL);
    }
    if (err == ESP_OK && s_cfg.pull_up) {
        err = gpio_pullup_en(s_cfg.gpio_a);
        if (err == ESP_OK) {
            err = gpio_pullup_en(s_cfg.gpio_b);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCNT setup: %s", esp_err_to_name(err));
        teardown();
        return err;
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_detent_seen = 0;
    s_cb = s_cfg.cb;
    s_cb_ctx = s_cfg.ctx;
    s_state = ST_READY;

    BaseType_t ok = xTaskCreatePinnedToCore(encoder_task, "encoder", CONFIG_ENCODER_TASK_STACK, NULL,
                                            CONFIG_ENCODER_TASK_PRIORITY, &s_task, CONFIG_ENCODER_TASK_CORE);
    if (ok != pdPASS) {
        s_state = ST_UNINIT;
        s_task = NULL;
        teardown();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "init: A=GPIO%d B=GPIO%d, %u counts/detent, glitch %u ns%s",
             s_cfg.gpio_a, s_cfg.gpio_b, s_cfg.counts_per_detent, s_cfg.glitch_ns,
             s_cfg.reverse ? ", reversed" : "");
    return ESP_OK;
}

esp_err_t encoder_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    (void)encoder_stop();
    if (s_task != NULL) {
        TaskHandle_t t = s_task;
        s_task = NULL;          /* The ISR checks this before notifying. */
        vTaskDelete(t);
    }
    teardown();
    if (s_cfg.pull_up) {
        (void)gpio_pullup_dis(s_cfg.gpio_a);
        (void)gpio_pullup_dis(s_cfg.gpio_b);
    }
    s_state = ST_UNINIT;
    return ESP_OK;
}

esp_err_t encoder_start(void)
{
    if (s_state != ST_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = pcnt_unit_enable(s_unit);
    if (err == ESP_OK) {
        err = pcnt_unit_start(s_unit);
        if (err != ESP_OK) {
            (void)pcnt_unit_disable(s_unit);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start: %s", esp_err_to_name(err));
        return err;
    }
    diag_task_register(s_task);
    s_stats.running = true;
    s_state = ST_RUNNING;
    return ESP_OK;
}

esp_err_t encoder_stop(void)
{
    if (s_state != ST_RUNNING) {
        return s_state == ST_UNINIT ? ESP_ERR_INVALID_STATE : ESP_OK;
    }
    s_state = ST_READY;
    (void)pcnt_unit_stop(s_unit);
    (void)pcnt_unit_disable(s_unit);
    diag_task_unregister(s_task);
    s_stats.running = false;
    return ESP_OK;
}

esp_err_t encoder_set_event_cb(encoder_step_cb_t cb, void *ctx)
{
    portENTER_CRITICAL(&s_cb_lock);
    s_cb = cb;
    s_cb_ctx = ctx;
    portEXIT_CRITICAL(&s_cb_lock);
    return ESP_OK;
}

int32_t encoder_position(void)
{
    return s_state == ST_UNINIT ? 0 : s_stats.position;
}

esp_err_t encoder_stats(encoder_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    out->running = (s_state == ST_RUNNING);
    return ESP_OK;
}
