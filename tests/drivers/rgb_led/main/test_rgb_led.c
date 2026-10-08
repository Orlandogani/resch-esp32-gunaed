/**
 * On-target Unity tests for drivers/rgb_led (FW-LED-001..008).
 *
 * No LED is needed: the LEDC duty registers are read back with ledc_get_duty(), and the
 * sleep lock is observed through pm_policy. Pins default to GPIO 38/39/40 — the ORGA v1
 * wiring, and free (or the on-board addressable LED's data line, which a PWM cannot harm)
 * on an ESP32-S3-DevKitC.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "unity.h"

#include "cfg.h"
#include "diag.h"
#include "pm_policy.h"
#include "power.h"
#include "rgb_led.h"

#define DUTY_MAX ((1u << CONFIG_RGB_LED_DUTY_BITS) - 1u)
#define CH(i)    ((ledc_channel_t)(CONFIG_RGB_LED_LEDC_CHANNEL_BASE + (i)))

static pm_policy_lock_handle_t s_probe;

static void fresh(void)
{
    (void)rgb_led_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    TEST_ASSERT_EQUAL(ESP_OK, diag_init(NULL));
    /* Same name as the driver's lock: create returns the existing handle (FW-PMP-012). */
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("rgb_led", &s_probe));
}

static rgb_led_config_t base_cfg(void)
{
    rgb_led_config_t c = { .gpio_r = 38, .gpio_g = 39, .gpio_b = 40, .active_low = true };
    return c;
}

TEST_CASE("level-to-duty curve: ends, gain, monotonic", "[rgb_led]")
{
    TEST_ASSERT_EQUAL(0, rgb_led_level_to_duty(0, 1000));
    TEST_ASSERT_EQUAL(DUTY_MAX, rgb_led_level_to_duty(255, 1000));
    TEST_ASSERT_UINT32_WITHIN(1, DUTY_MAX / 2, rgb_led_level_to_duty(255, 500));
    TEST_ASSERT_EQUAL(DUTY_MAX, rgb_led_level_to_duty(255, 0));   /* 0 = full */
    uint32_t prev = 0;
    for (unsigned v = 0; v <= 255; v++) {
        uint32_t d = rgb_led_level_to_duty((uint8_t)v, 1000);
        TEST_ASSERT_GREATER_OR_EQUAL(prev, d);
        prev = d;
    }
#if CONFIG_RGB_LED_GAMMA
    /* Half level is about a quarter of full duty on the square law. */
    TEST_ASSERT_UINT32_WITHIN(DUTY_MAX / 50, DUTY_MAX / 4, rgb_led_level_to_duty(128, 1000));
#endif
}

TEST_CASE("init validates and leaves the LED dark", "[rgb_led]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rgb_led_init(NULL));
    rgb_led_config_t c = { .gpio_r = -1, .gpio_g = -1, .gpio_b = -1 };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rgb_led_init(&c));
    c = base_cfg(); c.gpio_g = 46;                       /* input-only */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rgb_led_init(&c));
    c = base_cfg(); c.gain_permille[1] = 1001;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rgb_led_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, rgb_led_set(RGB_LED_OFF));

    c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, rgb_led_init(&c));
    for (unsigned i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL(0, ledc_get_duty(LEDC_LOW_SPEED_MODE, CH(i)));
    }
    TEST_ASSERT_EQUAL(0, pm_policy_lock_count(s_probe));
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_deinit());
}

TEST_CASE("set renders per-channel duty with gains; lock while lit", "[rgb_led]")
{
    fresh();
    rgb_led_config_t c = base_cfg();
    c.gain_permille[0] = 1000;
    c.gain_permille[1] = 500;
    c.gain_permille[2] = 250;
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_init(&c));

    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_set((rgb_led_color_t){ 255, 255, 255 }));
    TEST_ASSERT_EQUAL(rgb_led_level_to_duty(255, 1000), ledc_get_duty(LEDC_LOW_SPEED_MODE, CH(0)));
    TEST_ASSERT_EQUAL(rgb_led_level_to_duty(255, 500), ledc_get_duty(LEDC_LOW_SPEED_MODE, CH(1)));
    TEST_ASSERT_EQUAL(rgb_led_level_to_duty(255, 250), ledc_get_duty(LEDC_LOW_SPEED_MODE, CH(2)));
    TEST_ASSERT_EQUAL(1, pm_policy_lock_count(s_probe));

    /* Repeated lit sets do not stack the lock. */
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_set((rgb_led_color_t){ 10, 0, 0 }));
    TEST_ASSERT_EQUAL(1, pm_policy_lock_count(s_probe));

    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_set(RGB_LED_OFF));
    TEST_ASSERT_EQUAL(0, pm_policy_lock_count(s_probe));
    rgb_led_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_stats(&st));
    TEST_ASSERT_EQUAL(3, st.sets);
    TEST_ASSERT_FALSE(st.lit);
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_deinit());
}

TEST_CASE("fade reaches its target; lock released only after a fade to black", "[rgb_led]")
{
    fresh();
    rgb_led_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_init(&c));

    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_fade((rgb_led_color_t){ 0, 200, 0 }, 200));
    rgb_led_color_t got;
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_get(&got));
    TEST_ASSERT_EQUAL(200, got.g);                       /* the target, at once */
    vTaskDelay(pdMS_TO_TICKS(300));
    TEST_ASSERT_UINT32_WITHIN(2, rgb_led_level_to_duty(200, CONFIG_RGB_LED_GAIN_G),
                              ledc_get_duty(LEDC_LOW_SPEED_MODE, CH(1)));

    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_fade(RGB_LED_OFF, 200));
    TEST_ASSERT_EQUAL(1, pm_policy_lock_count(s_probe));   /* still lit mid-fade */
    vTaskDelay(pdMS_TO_TICKS(400));
    TEST_ASSERT_EQUAL(0, ledc_get_duty(LEDC_LOW_SPEED_MODE, CH(1)));
    TEST_ASSERT_EQUAL(0, pm_policy_lock_count(s_probe));

    /* deinit while lit releases the lock. */
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_set((rgb_led_color_t){ 1, 1, 1 }));
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_deinit());
    TEST_ASSERT_EQUAL(0, pm_policy_lock_count(s_probe));
}

TEST_CASE("a board with only some channels works", "[rgb_led]")
{
    fresh();
    rgb_led_config_t c = { .gpio_r = 38, .gpio_g = -1, .gpio_b = -1 };
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_set((rgb_led_color_t){ 255, 255, 255 }));
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_set(RGB_LED_OFF));
    TEST_ASSERT_EQUAL(ESP_OK, rgb_led_deinit());
}

void app_main(void)
{
    /* Let a USB-Serial/JTAG capture attach first (.claude/BACKLOG.md). */
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
