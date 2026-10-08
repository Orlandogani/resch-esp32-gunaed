/**
 * On-target Unity tests for drivers/encoder (FW-ENC-001..009).
 *
 * No wheel needed: after encoder_init() the test switches both phase pins to
 * GPIO_MODE_INPUT_OUTPUT and drives quadrature itself. PCNT reads the pad through the
 * input path, which output-enabling does not unroute (the same trick as the buttons test).
 * Pins GPIO 13/14 — the ORGA v1 wiring, free on an ESP32-S3-DevKitC.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "unity.h"

#include "cfg.h"
#include "diag.h"
#include "encoder.h"
#include "pm_policy.h"
#include "power.h"

#define PIN_A 13
#define PIN_B 14

static volatile int32_t  s_sum;
static volatile uint32_t s_calls;

static void on_step(int32_t steps, void *ctx)
{
    (void)ctx;
    s_sum += steps;
    s_calls++;
}

static void fresh(void)
{
    (void)encoder_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    TEST_ASSERT_EQUAL(ESP_OK, diag_init(NULL));
    s_sum = 0;
    s_calls = 0;
}

static void drive_init(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_level(PIN_A, 0));
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_level(PIN_B, 0));
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_direction(PIN_A, GPIO_MODE_INPUT_OUTPUT));
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_direction(PIN_B, GPIO_MODE_INPUT_OUTPUT));
}

/* One quadrature edge at a time, slower than the glitch filter. */
static void edge(int pin, int level)
{
    gpio_set_level(pin, level);
    esp_rom_delay_us(200);
}

/* `n` full cycles, A leading B (clockwise) when n > 0. */
static void turn(int n)
{
    for (int i = 0; i < (n < 0 ? -n : n); i++) {
        if (n > 0) {
            edge(PIN_A, 1); edge(PIN_B, 1); edge(PIN_A, 0); edge(PIN_B, 0);
        } else {
            edge(PIN_B, 1); edge(PIN_A, 1); edge(PIN_B, 0); edge(PIN_A, 0);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(20));   /* let the task drain */
}

static encoder_config_t base_cfg(void)
{
    encoder_config_t c = { .gpio_a = PIN_A, .gpio_b = PIN_B, .cb = on_step };
    return c;
}

TEST_CASE("init validates its arguments", "[encoder]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, encoder_init(NULL));
    encoder_config_t c = base_cfg(); c.gpio_b = PIN_A;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, encoder_init(&c));
    c = base_cfg(); c.gpio_a = -1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, encoder_init(&c));
    c = base_cfg(); c.counts_per_detent = 65;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, encoder_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, encoder_start());
    TEST_ASSERT_EQUAL(0, encoder_position());
}

TEST_CASE("lifecycle is clean and repeatable", "[encoder]")
{
    fresh();
    encoder_config_t c = base_cfg();
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, encoder_init(&c));
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, encoder_init(&c));
        TEST_ASSERT_EQUAL(ESP_OK, encoder_start());
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, encoder_start());
        TEST_ASSERT_EQUAL(ESP_OK, encoder_stop());
        TEST_ASSERT_EQUAL(ESP_OK, encoder_stop());
        TEST_ASSERT_EQUAL(ESP_OK, encoder_deinit());
    }
}

TEST_CASE("one detent per quadrature cycle, signed by direction", "[encoder]")
{
    fresh();
    encoder_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, encoder_init(&c));
    drive_init();
    TEST_ASSERT_EQUAL(ESP_OK, encoder_start());

    turn(3);
    TEST_ASSERT_EQUAL(3, s_sum);
    TEST_ASSERT_EQUAL(3, encoder_position());
    turn(-5);
    TEST_ASSERT_EQUAL(-2, s_sum);
    TEST_ASSERT_EQUAL(-2, encoder_position());

    encoder_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, encoder_stats(&st));
    TEST_ASSERT_EQUAL(3, st.steps_cw);
    TEST_ASSERT_EQUAL(5, st.steps_ccw);
    TEST_ASSERT_GREATER_OR_EQUAL(1, st.isr_wakeups);
    TEST_ASSERT_TRUE(st.running);
    TEST_ASSERT_EQUAL(ESP_OK, encoder_deinit());
}

TEST_CASE("a half turn and back emits nothing (bounce between detents)", "[encoder]")
{
    fresh();
    encoder_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, encoder_init(&c));
    drive_init();
    TEST_ASSERT_EQUAL(ESP_OK, encoder_start());

    for (int i = 0; i < 10; i++) {          /* rock two edges forward and back */
        edge(PIN_A, 1); edge(PIN_B, 1);
        edge(PIN_B, 0); edge(PIN_A, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    TEST_ASSERT_EQUAL(0, s_sum);
    TEST_ASSERT_EQUAL(0, s_calls);
    TEST_ASSERT_EQUAL(ESP_OK, encoder_deinit());
}

TEST_CASE("reverse flips the sense; no callback counts as dropped", "[encoder]")
{
    fresh();
    encoder_config_t c = base_cfg();
    c.reverse = true;
    TEST_ASSERT_EQUAL(ESP_OK, encoder_init(&c));
    drive_init();
    TEST_ASSERT_EQUAL(ESP_OK, encoder_start());
    turn(2);
    TEST_ASSERT_EQUAL(-2, s_sum);

    TEST_ASSERT_EQUAL(ESP_OK, encoder_set_event_cb(NULL, NULL));
    turn(1);
    encoder_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, encoder_stats(&st));
    TEST_ASSERT_EQUAL(1, st.events_dropped);
    TEST_ASSERT_EQUAL(-3, st.position);
    TEST_ASSERT_EQUAL(ESP_OK, encoder_deinit());
}

TEST_CASE("stopped encoder does not count", "[encoder]")
{
    fresh();
    encoder_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, encoder_init(&c));
    drive_init();
    TEST_ASSERT_EQUAL(ESP_OK, encoder_start());
    turn(1);
    TEST_ASSERT_EQUAL(ESP_OK, encoder_stop());
    turn(4);
    TEST_ASSERT_EQUAL(1, s_sum);
    TEST_ASSERT_EQUAL(ESP_OK, encoder_deinit());
}

void app_main(void)
{
    /* Let a USB-Serial/JTAG capture attach first (.claude/BACKLOG.md). */
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
