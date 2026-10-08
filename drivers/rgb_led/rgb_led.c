#include "rgb_led.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "pm_policy.h"

static const char *TAG = "rgb_led";

#define DUTY_MAX ((1u << CONFIG_RGB_LED_DUTY_BITS) - 1u)

static bool                    s_init;
static rgb_led_config_t        s_cfg;
static int                     s_gpio[3];
static SemaphoreHandle_t       s_mutex;
static pm_policy_lock_handle_t s_lock;
static bool                    s_locked;
static esp_timer_handle_t      s_dark_timer;   /* Releases the lock when a fade to black ends. */
static rgb_led_stats_t         s_stats;

static ledc_channel_t chan(unsigned i)
{
    return (ledc_channel_t)(CONFIG_RGB_LED_LEDC_CHANNEL_BASE + i);
}

uint32_t rgb_led_level_to_duty(uint8_t level, uint16_t gain_permille)
{
    if (gain_permille == 0 || gain_permille > 1000) {
        gain_permille = 1000;
    }
#if CONFIG_RGB_LED_GAMMA
    uint32_t v = ((uint32_t)level * level + 127u) / 255u;   /* v^2 / 255, rounded */
#else
    uint32_t v = level;
#endif
    /* (v / 255) x (gain / 1000) x DUTY_MAX, rounded; fits 32 bits for DUTY_BITS <= 14. */
    return (v * gain_permille * DUTY_MAX + 127500u) / 255000u;
}

static void lock_set(bool lit)
{
    if (lit && !s_locked) {
        if (pm_policy_lock_acquire(s_lock) == ESP_OK) {
            s_locked = true;
        }
    } else if (!lit && s_locked) {
        if (pm_policy_lock_release(s_lock) == ESP_OK) {
            s_locked = false;
        }
    }
    s_stats.lit = s_locked;
}

static bool is_dark(rgb_led_color_t c)
{
    return c.r == 0 && c.g == 0 && c.b == 0;
}

static void on_dark_timer(void *arg)
{
    (void)arg;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_init && is_dark(s_stats.color)) {
        lock_set(false);
    }
    xSemaphoreGive(s_mutex);
}

static esp_err_t apply(rgb_led_color_t c, uint32_t ms)
{
    const uint8_t lv[3] = { c.r, c.g, c.b };
    esp_err_t first = ESP_OK;

    (void)esp_timer_stop(s_dark_timer);
    if (!is_dark(c) || ms > 0) {
        lock_set(true);   /* A fade to black is still lit until it ends. */
    }
    for (unsigned i = 0; i < 3; i++) {
        if (s_gpio[i] < 0) {
            continue;
        }
        uint32_t duty = rgb_led_level_to_duty(lv[i], s_cfg.gain_permille[i]);
        esp_err_t err;
        if (ms == 0) {
            (void)ledc_fade_stop(LEDC_LOW_SPEED_MODE, chan(i));
            err = ledc_set_duty(LEDC_LOW_SPEED_MODE, chan(i), duty);
            if (err == ESP_OK) {
                err = ledc_update_duty(LEDC_LOW_SPEED_MODE, chan(i));
            }
        } else {
            err = ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, chan(i), duty, (int)ms);
            if (err == ESP_OK) {
                err = ledc_fade_start(LEDC_LOW_SPEED_MODE, chan(i), LEDC_FADE_NO_WAIT);
            }
        }
        if (err != ESP_OK) {
            s_stats.errors++;
            if (first == ESP_OK) {
                first = err;
            }
        }
    }
    s_stats.color = c;
    if (is_dark(c)) {
        if (ms == 0) {
            lock_set(false);
        } else {
            (void)esp_timer_start_once(s_dark_timer, (uint64_t)ms * 1000u + 1000u);
        }
    }
    return first;
}

esp_err_t rgb_led_init(const rgb_led_config_t *cfg)
{
    if (s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const int pins[3] = { cfg->gpio_r, cfg->gpio_g, cfg->gpio_b };
    bool any = false;
    for (unsigned i = 0; i < 3; i++) {
        if (pins[i] >= 0) {
            if (!GPIO_IS_VALID_OUTPUT_GPIO(pins[i])) {
                return ESP_ERR_INVALID_ARG;
            }
            any = true;
        }
        if (cfg->gain_permille[i] > 1000) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (!any) {
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *cfg;
    static const uint16_t kdefault[3] = { CONFIG_RGB_LED_GAIN_R, CONFIG_RGB_LED_GAIN_G, CONFIG_RGB_LED_GAIN_B };
    for (unsigned i = 0; i < 3; i++) {
        s_gpio[i] = pins[i];
        if (s_cfg.gain_permille[i] == 0) {
            s_cfg.gain_permille[i] = kdefault[i];
        }
    }
    if (s_cfg.pwm_hz == 0) {
        s_cfg.pwm_hz = CONFIG_RGB_LED_PWM_HZ;
    }

    esp_err_t err = pm_policy_lock_create("rgb_led", &s_lock);
    if (err != ESP_OK) {
        return err;   /* ESP_ERR_INVALID_STATE without pm_policy_init() */
    }
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();   /* Kept across deinit, like the lock. */
        if (s_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_dark_timer == NULL) {
        const esp_timer_create_args_t targs = { .callback = on_dark_timer, .name = "rgb_led_dark" };
        err = esp_timer_create(&targs, &s_dark_timer);
        if (err != ESP_OK) {
            return err;
        }
    }

    const ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)CONFIG_RGB_LED_DUTY_BITS,
        .timer_num = (ledc_timer_t)CONFIG_RGB_LED_LEDC_TIMER,
        .freq_hz = s_cfg.pwm_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config: %s", esp_err_to_name(err));
        return err;
    }
    for (unsigned i = 0; i < 3; i++) {
        if (s_gpio[i] < 0) {
            continue;
        }
        const ledc_channel_config_t ccfg = {
            .gpio_num = s_gpio[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = chan(i),
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = (ledc_timer_t)CONFIG_RGB_LED_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
            .flags = { .output_invert = s_cfg.active_low ? 1u : 0u },
        };
        err = ledc_channel_config(&ccfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config(%u): %s", i, esp_err_to_name(err));
            return err;
        }
    }
    err = ledc_fade_func_install(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   /* already installed is fine */
        ESP_LOGE(TAG, "ledc_fade_func_install: %s", esp_err_to_name(err));
        return err;
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_locked = false;
    s_init = true;
    ESP_LOGI(TAG, "init: R%d G%d B%d, active %s, %u Hz, gains %u/%u/%u",
             s_gpio[0], s_gpio[1], s_gpio[2], s_cfg.active_low ? "low" : "high", (unsigned)s_cfg.pwm_hz,
             s_cfg.gain_permille[0], s_cfg.gain_permille[1], s_cfg.gain_permille[2]);
    return ESP_OK;
}

esp_err_t rgb_led_deinit(void)
{
    if (!s_init) {
        return ESP_OK;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    (void)esp_timer_stop(s_dark_timer);
    for (unsigned i = 0; i < 3; i++) {
        if (s_gpio[i] >= 0) {
            (void)ledc_fade_stop(LEDC_LOW_SPEED_MODE, chan(i));
            /* Idle level = dark on either wiring: invert maps idle 0 to the anode level. */
            (void)ledc_stop(LEDC_LOW_SPEED_MODE, chan(i), s_cfg.active_low ? 1u : 0u);
            (void)gpio_reset_pin(s_gpio[i]);
        }
    }
    lock_set(false);
    s_init = false;
    xSemaphoreGive(s_mutex);
    /* The fade service stays installed: other LEDC users may share it. */
    return ESP_OK;
}

esp_err_t rgb_led_set(rgb_led_color_t c)
{
    return rgb_led_fade(c, 0);
}

esp_err_t rgb_led_fade(rgb_led_color_t c, uint32_t ms)
{
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = apply(c, ms);
    if (err == ESP_OK) {
        if (ms == 0) {
            s_stats.sets++;
        } else {
            s_stats.fades++;
        }
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t rgb_led_get(rgb_led_color_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    *out = s_stats.color;
    return ESP_OK;
}

esp_err_t rgb_led_stats(rgb_led_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    return ESP_OK;
}
