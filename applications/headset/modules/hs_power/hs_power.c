/**
 * Headset power policy: latch, battery thresholds, power-off sequence.
 */
#include "hs_power.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "battery.h"
#include "board.h"
#include "power_latch.h"

static const char *TAG = "hs_power";

static struct {
    bool           latch;
    bool           battery;
    hs_power_cb_t  cb;
    void          *ctx;
    hs_battery_t   last;
    portMUX_TYPE   lock;
} s_p = { .lock = portMUX_INITIALIZER_UNLOCKED };

static hs_battery_t classify(const battery_status_t *st)
{
    hs_battery_t b = {
        .mv = st->mv,
        .percent = st->percent,
        .ext_power = st->ext_power,
        .valid = st->valid,
    };
    /* The charger's CHG pin is not wired to the S3 (TBD-015): "charging" is inferred. */
    b.charging = b.ext_power && b.mv < CONFIG_HS_POWER_FULL_MV;
    b.low = !b.ext_power && b.percent <= CONFIG_HS_POWER_LOW_PERCENT;
    b.critical = !b.ext_power && b.mv <= CONFIG_HS_POWER_CRITICAL_MV;
    return b;
}

static void on_battery(const battery_status_t *st, void *ctx)
{
    (void)ctx;
    hs_battery_t b = classify(st);
    portENTER_CRITICAL(&s_p.lock);
    s_p.last = b;
    portEXIT_CRITICAL(&s_p.lock);
    if (s_p.cb != NULL) {
        s_p.cb(&b, s_p.ctx);
    }
}

esp_err_t hs_power_hold(void)
{
    const board_desc_t *b = board_desc();
    if (b->power_latch.hold == BOARD_PIN_NONE) {
        return ESP_OK;
    }
    const power_latch_config_t c = { .hold_gpio = b->power_latch.hold, .active_low = b->power_latch.active_low };
    esp_err_t err = power_latch_init(&c);
    s_p.latch = (err == ESP_OK);
    return err;
}

esp_err_t hs_power_init(hs_power_cb_t cb, void *ctx)
{
    s_p.cb = cb;
    s_p.ctx = ctx;
    const board_desc_t *b = board_desc();
    if (b->battery.adc_gpio == BOARD_PIN_NONE) {
        return ESP_OK;
    }
    const battery_config_t c = {
        .adc_gpio = b->battery.adc_gpio,
        .divider_num = b->battery.divider_num,
        .divider_den = b->battery.divider_den,
        .ext_power_gpio = b->battery.ext_power_gpio,
        .ext_power_active_low = b->battery.ext_power_active_low,
        .cb = on_battery,
    };
    esp_err_t err = battery_init(&c);
    if (err == ESP_OK) {
        /* One synchronous sample, so boot can refuse to run on a flat cell. */
        battery_status_t st;
        if (battery_sample_now(&st) == ESP_OK) {
            hs_battery_t hb = classify(&st);
            portENTER_CRITICAL(&s_p.lock);
            s_p.last = hb;
            portEXIT_CRITICAL(&s_p.lock);
            ESP_LOGI(TAG, "battery %u mV, %u%%%s", hb.mv, hb.percent, hb.ext_power ? ", external power" : "");
        }
        err = battery_start();
    }
    s_p.battery = (err == ESP_OK);
    return err;
}

void hs_power_get(hs_battery_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_p.lock);
    *out = s_p.last;
    portEXIT_CRITICAL(&s_p.lock);
}

bool hs_power_has_latch(void)
{
    return s_p.latch;
}

esp_err_t hs_power_off(bool (*still_pressed)(void))
{
    if (!s_p.latch) {
        ESP_LOGW(TAG, "power off requested, but this board has no latch");
        return ESP_ERR_NOT_SUPPORTED;
    }
    /* The power button feeds the regulator enable too: dropping the hold while it is
     * pressed changes nothing. Wait for the release, bounded (DES-HSP-002). */
    TickType_t t0 = xTaskGetTickCount();
    while (still_pressed != NULL && still_pressed() &&
           xTaskGetTickCount() - t0 < pdMS_TO_TICKS(CONFIG_HS_POWER_RELEASE_WAIT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_p.battery) {
        (void)battery_stop();
    }
    ESP_LOGW(TAG, "powering off");
    vTaskDelay(pdMS_TO_TICKS(50));   /* let the log line out of the UART */
    esp_err_t err = power_latch_release();
    /* Still here: the button is held, or the latch does not work. */
    if (s_p.battery) {
        (void)battery_start();
    }
    return err;
}
