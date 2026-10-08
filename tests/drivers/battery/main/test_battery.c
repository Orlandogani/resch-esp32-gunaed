/**
 * On-target Unity tests for drivers/battery (FW-BAT-001..010).
 *
 * The state-of-charge arithmetic is pure and tested exactly. The ADC path is tested for
 * plumbing only: on a devkit GPIO1 floats, so a reading is checked for range, not value
 * ([needs_orga]: compare against a bench supply on the real divider). The power-good input
 * is simulated by driving GPIO47 in GPIO_MODE_INPUT_OUTPUT (as the buttons test does).
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "unity.h"

#include "battery.h"
#include "cfg.h"
#include "diag.h"
#include "pm_policy.h"
#include "power.h"

#define PIN_ADC   1
#define PIN_PGOOD 47

static volatile uint32_t s_events;
static battery_status_t  s_last;

static void on_event(const battery_status_t *st, void *ctx)
{
    (void)ctx;
    s_last = *st;
    s_events++;
}

static void fresh(void)
{
    (void)battery_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    TEST_ASSERT_EQUAL(ESP_OK, diag_init(NULL));
    s_events = 0;
}

static battery_config_t base_cfg(void)
{
    battery_config_t c = {
        .adc_gpio = PIN_ADC, .divider_num = 2, .divider_den = 1,
        .ext_power_gpio = PIN_PGOOD, .ext_power_active_low = true,
        .sample_period_ms = 200, .cb = on_event,
    };
    return c;
}

TEST_CASE("percent from mV: ends clamp, interpolation, custom table", "[battery]")
{
    TEST_ASSERT_EQUAL(100, battery_percent_from_mv(NULL, 0, 4300));
    TEST_ASSERT_EQUAL(100, battery_percent_from_mv(NULL, 0, 4200));
    TEST_ASSERT_EQUAL(0, battery_percent_from_mv(NULL, 0, 3000));
    TEST_ASSERT_EQUAL(50, battery_percent_from_mv(NULL, 0, 3840));
    static const battery_ocv_point_t t[] = { { 4000, 100 }, { 3000, 0 } };
    TEST_ASSERT_EQUAL(50, battery_percent_from_mv(t, 2, 3500));
    TEST_ASSERT_EQUAL(25, battery_percent_from_mv(t, 2, 3250));
    uint8_t prev = 0;
    for (uint16_t mv = 3000; mv <= 4300; mv += 5) {
        uint8_t p = battery_percent_from_mv(NULL, 0, mv);
        TEST_ASSERT_GREATER_OR_EQUAL(prev, p);
        prev = p;
    }
}

TEST_CASE("init validates its arguments", "[battery]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, battery_init(NULL));
    battery_config_t c = base_cfg(); c.adc_gpio = 11;      /* ADC2 on the S3 */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, battery_init(&c));
    c = base_cfg(); c.adc_gpio = 40;                       /* no ADC at all */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, battery_init(&c));
    c = base_cfg(); c.average = 17;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, battery_init(&c));
    static const battery_ocv_point_t rising[] = { { 3000, 0 }, { 4000, 100 } };
    c = base_cfg(); c.ocv = rising; c.ocv_count = 2;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, battery_init(&c));
    battery_status_t st;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, battery_get(&st));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, battery_start());
}

TEST_CASE("sample_now works while stopped and stays in range", "[battery]")
{
    fresh();
    battery_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, battery_init(&c));
    battery_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, battery_get(&st));
    TEST_ASSERT_FALSE(st.valid);
    TEST_ASSERT_EQUAL(ESP_OK, battery_sample_now(&st));
    TEST_ASSERT_TRUE(st.valid);
    TEST_ASSERT_LESS_OR_EQUAL(2 * 3300, st.mv);
    TEST_ASSERT_LESS_OR_EQUAL(100, st.percent);
    battery_stats_t bs;
    TEST_ASSERT_EQUAL(ESP_OK, battery_stats(&bs));
    TEST_ASSERT_EQUAL(1, bs.samples);
    TEST_ASSERT_EQUAL(0, bs.adc_errors);
    printf("floating GPIO1: %u mV, calibrated %d\n", st.mv, bs.calibrated);
    TEST_ASSERT_EQUAL(ESP_OK, battery_deinit());
}

TEST_CASE("start samples at once and periodically", "[battery]")
{
    fresh();
    battery_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, battery_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, battery_start());
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_GREATER_OR_EQUAL(1, s_events);   /* the first sample always reports */
    vTaskDelay(pdMS_TO_TICKS(1000));
    battery_stats_t bs;
    TEST_ASSERT_EQUAL(ESP_OK, battery_stats(&bs));
    TEST_ASSERT_INT_WITHIN(2, 6, (int)bs.samples);
    TEST_ASSERT_TRUE(bs.running);
    TEST_ASSERT_EQUAL(ESP_OK, battery_stop());
    TEST_ASSERT_EQUAL(ESP_OK, battery_stop());
    TEST_ASSERT_EQUAL(ESP_OK, battery_deinit());
}

TEST_CASE("a power-good edge reports external power promptly", "[battery]")
{
    fresh();
    battery_config_t c = base_cfg();
    c.sample_period_ms = 60000;   /* only the edge can cause a sample within the test */
    TEST_ASSERT_EQUAL(ESP_OK, battery_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_level(PIN_PGOOD, 1));
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_direction(PIN_PGOOD, GPIO_MODE_INPUT_OUTPUT));
    TEST_ASSERT_EQUAL(ESP_OK, battery_start());
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_FALSE(s_last.ext_power);
    uint32_t before = s_events;

    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_level(PIN_PGOOD, 0));   /* charger: power good */
    vTaskDelay(pdMS_TO_TICKS(CONFIG_BATTERY_EXT_POWER_SETTLE_MS + 100));
    TEST_ASSERT_GREATER_THAN(before, s_events);
    TEST_ASSERT_TRUE(s_last.ext_power);

    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_level(PIN_PGOOD, 1));
    vTaskDelay(pdMS_TO_TICKS(CONFIG_BATTERY_EXT_POWER_SETTLE_MS + 100));
    TEST_ASSERT_FALSE(s_last.ext_power);

    battery_stats_t bs;
    TEST_ASSERT_EQUAL(ESP_OK, battery_stats(&bs));
    TEST_ASSERT_GREATER_OR_EQUAL(2, bs.ext_power_edges);
    TEST_ASSERT_EQUAL(ESP_OK, battery_deinit());
}

void app_main(void)
{
    /* Let a USB-Serial/JTAG capture attach first (.claude/BACKLOG.md). */
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
