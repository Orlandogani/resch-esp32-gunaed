#include "power_latch.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "power_latch";

static bool                 s_init;
static power_latch_config_t s_cfg;
static power_latch_stats_t  s_stats;

static int level(bool asserted)
{
    return (asserted != s_cfg.active_low) ? 1 : 0;
}

/* Write the level first, then make the pad an output, then latch it (DES-LAT-001). While a
 * hold from before the reset is still on, none of this reaches the pad, so there is no
 * window in which the pin floats. */
static esp_err_t assert_and_hold(void)
{
    esp_err_t err = gpio_set_level(s_cfg.hold_gpio, level(true));
    if (err != ESP_OK) {
        return err;
    }
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_cfg.hold_gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT,   /* readable back, for diagnostics and tests */
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&io);
    if (err == ESP_OK) {
        err = gpio_set_level(s_cfg.hold_gpio, level(true));
    }
    if (err == ESP_OK) {
        err = gpio_hold_en(s_cfg.hold_gpio);
    }
    s_stats.held = (err == ESP_OK);
    return err;
}

esp_err_t power_latch_init(const power_latch_config_t *cfg)
{
    if (s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || !GPIO_IS_VALID_OUTPUT_GPIO(cfg->hold_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    if (s_cfg.release_timeout_ms == 0) {
        s_cfg.release_timeout_ms = CONFIG_POWER_LATCH_RELEASE_TIMEOUT_MS;
    }
    memset(&s_stats, 0, sizeof(s_stats));

    esp_err_t err = assert_and_hold();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d: %s", s_cfg.hold_gpio, esp_err_to_name(err));
        return err;
    }
    s_init = true;
    ESP_LOGI(TAG, "holding power on GPIO%d (active %s)", s_cfg.hold_gpio, s_cfg.active_low ? "low" : "high");
    return ESP_OK;
}

esp_err_t power_latch_deinit(void)
{
    /* Deliberately leaves the pin asserted and held (FW-LAT-004). */
    s_init = false;
    return ESP_OK;
}

esp_err_t power_latch_release(void)
{
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    s_stats.release_attempts++;
    ESP_LOGW(TAG, "releasing power");
    /* The level goes into the output register while the hold still pins the pad, so
     * dropping the hold moves the pad straight to inactive. */
    (void)gpio_set_level(s_cfg.hold_gpio, level(false));
    (void)gpio_hold_dis(s_cfg.hold_gpio);
    (void)gpio_set_level(s_cfg.hold_gpio, level(false));
    s_stats.held = false;

    /* On a working latch the rail collapses within milliseconds and nothing below runs. */
    vTaskDelay(pdMS_TO_TICKS(s_cfg.release_timeout_ms));

    s_stats.release_timeouts++;
    ESP_LOGW(TAG, "still powered after %u ms (button held, or no latch); holding again",
             (unsigned)s_cfg.release_timeout_ms);
    (void)assert_and_hold();
    return ESP_ERR_TIMEOUT;
}

bool power_latch_is_held(void)
{
    return s_init && s_stats.held;
}

esp_err_t power_latch_stats(power_latch_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    return ESP_OK;
}
