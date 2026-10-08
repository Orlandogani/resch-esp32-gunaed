/**
 * On-target Unity tests for drivers/power_latch (FW-LAT-001..007).
 *
 * A devkit has no latch, so power never collapses: that is exactly the failure path
 * `release()` must handle (re-assert, ESP_ERR_TIMEOUT), and it is safe to exercise here.
 * Default pin GPIO21, which is free on an ESP32-S3-DevKitC and is the ORGA v1 PWR_HOLD.
 * The pad hold itself is a first-board check ([needs_orga]: survive an esp_restart()).
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "unity.h"

#include "power_latch.h"

#define PIN_HOLD 21

static void fresh(void)
{
    (void)power_latch_deinit();
    (void)gpio_hold_dis(PIN_HOLD);
}

TEST_CASE("init validates its arguments", "[power_latch]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, power_latch_init(NULL));
    power_latch_config_t c = { .hold_gpio = 46 };   /* input-only on the S3 */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, power_latch_init(&c));
    c.hold_gpio = -1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, power_latch_init(&c));
    TEST_ASSERT_FALSE(power_latch_is_held());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, power_latch_release());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, power_latch_stats(NULL));
}

TEST_CASE("init asserts and holds; a second init is refused", "[power_latch]")
{
    fresh();
    power_latch_config_t c = { .hold_gpio = PIN_HOLD };
    TEST_ASSERT_EQUAL(ESP_OK, power_latch_init(&c));
    TEST_ASSERT_TRUE(power_latch_is_held());
    TEST_ASSERT_EQUAL(1, gpio_get_level(PIN_HOLD));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, power_latch_init(&c));

    /* The pad hold pins the level: writing the register does not reach the pad. */
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_level(PIN_HOLD, 0));
    TEST_ASSERT_EQUAL(1, gpio_get_level(PIN_HOLD));
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_level(PIN_HOLD, 1));
}

TEST_CASE("release without a latch times out and re-asserts", "[power_latch]")
{
    fresh();
    power_latch_config_t c = { .hold_gpio = PIN_HOLD, .release_timeout_ms = 100 };
    TEST_ASSERT_EQUAL(ESP_OK, power_latch_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, power_latch_release());
    TEST_ASSERT_TRUE(power_latch_is_held());
    TEST_ASSERT_EQUAL(1, gpio_get_level(PIN_HOLD));

    power_latch_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, power_latch_stats(&st));
    TEST_ASSERT_EQUAL(1, st.release_attempts);
    TEST_ASSERT_EQUAL(1, st.release_timeouts);
    TEST_ASSERT_TRUE(st.held);
}

TEST_CASE("deinit leaves power on", "[power_latch]")
{
    fresh();
    power_latch_config_t c = { .hold_gpio = PIN_HOLD };
    TEST_ASSERT_EQUAL(ESP_OK, power_latch_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, power_latch_deinit());
    TEST_ASSERT_FALSE(power_latch_is_held());          /* no longer managed ...      */
    TEST_ASSERT_EQUAL(1, gpio_get_level(PIN_HOLD));    /* ... but still asserted     */
}

TEST_CASE("active-low latch drives low to hold", "[power_latch]")
{
    fresh();
    power_latch_config_t c = { .hold_gpio = PIN_HOLD, .active_low = true, .release_timeout_ms = 100 };
    TEST_ASSERT_EQUAL(ESP_OK, power_latch_init(&c));
    TEST_ASSERT_EQUAL(0, gpio_get_level(PIN_HOLD));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, power_latch_release());
    TEST_ASSERT_EQUAL(0, gpio_get_level(PIN_HOLD));
    fresh();
}

void app_main(void)
{
    /* Let a USB-Serial/JTAG capture attach first (.claude/BACKLOG.md). */
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
