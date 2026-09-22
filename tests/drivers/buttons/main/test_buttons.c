/**
 * On-target Unity tests for drivers/buttons.
 *
 * No switch is needed: after buttons_init() has configured a pin as an input, the
 * test switches that pin to GPIO_MODE_INPUT_OUTPUT and drives it. The input buffer
 * reads the pad back and the edge interrupt fires exactly as it would for a real
 * switch to ground. Nothing here sleeps or touches USB, so the project is safe to
 * run over the USB-Serial/JTAG console.
 *
 * Pins default to GPIO 4 and 5 (RTC-capable, free on most ESP32-S3 dev boards);
 * GPIO 38 is used only as an example of a non-RTC pin and is never driven.
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "cfg.h"
#include "diag.h"
#include "pm_policy.h"
#include "power.h"
#include "buttons.h"

#ifndef TEST_PIN_A
#define TEST_PIN_A 4
#endif
#ifndef TEST_PIN_B
#define TEST_PIN_B 5
#endif
#define TEST_PIN_NON_RTC 38

#define DEBOUNCE_MS 20
#define LONG_MS     300
#define REPEAT_MS   100
/* Sampling quantum plus scheduling: what any timing assertion must tolerate. */
#define SLOP_MS     (CONFIG_BUTTONS_SAMPLE_MS * 2 + 10)

/* ------------------------------------------------------------------------- */
/* Event recorder                                                             */
/* ------------------------------------------------------------------------- */

typedef struct {
    buttons_event_t evt;
    uint8_t  index;
    int      gpio;
    uint32_t held_ms;
    uint32_t at_ms;
} rec_t;

static rec_t   s_rec[32];
static size_t  s_rec_n;
static int     s_ctx_seen;

static void recorder(buttons_event_t evt, const buttons_event_info_t *info, void *ctx)
{
    if (ctx != NULL) {
        s_ctx_seen = *(int *)ctx;
    }
    if (s_rec_n < sizeof(s_rec) / sizeof(s_rec[0])) {
        s_rec[s_rec_n++] = (rec_t){
            .evt = evt, .index = info->index, .gpio = info->gpio,
            .held_ms = info->held_ms, .at_ms = (uint32_t)(esp_timer_get_time() / 1000),
        };
    }
}

static void rec_clear(void)
{
    s_rec_n = 0;
    memset(s_rec, 0, sizeof(s_rec));
}

static size_t rec_count(buttons_event_t evt)
{
    size_t n = 0;
    for (size_t i = 0; i < s_rec_n; i++) {
        if (s_rec[i].evt == evt) {
            n++;
        }
    }
    return n;
}

/* ------------------------------------------------------------------------- */
/* Pin driving                                                                */
/* ------------------------------------------------------------------------- */

static void pin_loopback(int gpio)
{
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_level(gpio, 1)); /* released, active-low */
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_direction(gpio, GPIO_MODE_INPUT_OUTPUT));
}

static void press(int gpio)   { gpio_set_level(gpio, 0); }
static void release(int gpio) { gpio_set_level(gpio, 1); }

static const buttons_pin_config_t s_pins[2] = {
    { .gpio = TEST_PIN_A, .active_low = true, .pull_enable = true },
    { .gpio = TEST_PIN_B, .active_low = true, .pull_enable = true },
};

static buttons_config_t base_cfg(void)
{
    buttons_config_t c = {
        .pins = s_pins,
        .count = 2,
        .debounce_ms = DEBOUNCE_MS,
        .long_press_ms = LONG_MS,
        .repeat_ms = REPEAT_MS,
        .auto_repeat = false,
        .cb = recorder,
        .ctx = NULL,
    };
    return c;
}

static void fresh(void)
{
    buttons_deinit();
    rec_clear();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    TEST_ASSERT_EQUAL(ESP_OK, diag_init(NULL));
}

/* init + loopback + start, ready to press. */
static void up(const buttons_config_t *c)
{
    TEST_ASSERT_EQUAL(ESP_OK, buttons_init(c));
    for (uint8_t i = 0; i < c->count; i++) {
        pin_loopback(c->pins[i].gpio);
    }
    TEST_ASSERT_EQUAL(ESP_OK, buttons_start());
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS)); /* let the first sample settle */
    rec_clear();
}

/* ------------------------------------------------------------------------- */
/* Lifecycle and validation                                                   */
/* ------------------------------------------------------------------------- */

TEST_CASE("before init: queries fail cleanly", "[buttons]")
{
    buttons_deinit();
    bool p;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, buttons_is_pressed(0, &p));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, buttons_set_event_cb(recorder, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, buttons_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, buttons_stop());
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit()); /* idempotent */
}

TEST_CASE("init rejects bad configurations without side effects", "[buttons]")
{
    fresh();
    buttons_config_t c;
    buttons_pin_config_t pins[CONFIG_BUTTONS_MAX + 1];

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, buttons_init(NULL));
    c = base_cfg(); c.pins = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, buttons_init(&c));
    c = base_cfg(); c.count = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, buttons_init(&c));

    memset(pins, 0, sizeof(pins));
    for (int i = 0; i < CONFIG_BUTTONS_MAX + 1; i++) {
        pins[i].gpio = i + 1;
    }
    c = base_cfg(); c.pins = pins; c.count = CONFIG_BUTTONS_MAX + 1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, buttons_init(&c));

    buttons_pin_config_t bad[2] = { { .gpio = -1 }, { .gpio = TEST_PIN_B } };
    c = base_cfg(); c.pins = bad;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, buttons_init(&c));

    buttons_pin_config_t dup[2] = { { .gpio = TEST_PIN_A }, { .gpio = TEST_PIN_A } };
    c = base_cfg(); c.pins = dup;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, buttons_init(&c));

    buttons_pin_config_t nonrtc[1] = { { .gpio = TEST_PIN_NON_RTC, .active_low = true, .wake_from_deep_sleep = true } };
    c = base_cfg(); c.pins = nonrtc; c.count = 1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, buttons_init(&c));

    /* Still uninitialised, and a good init now succeeds. */
    bool p;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, buttons_is_pressed(0, &p));
    c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, buttons_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
}

TEST_CASE("init/deinit cycle is clean and repeatable", "[buttons]")
{
    fresh();
    buttons_config_t c = base_cfg();
    size_t heap_before = esp_get_free_heap_size();
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, buttons_init(&c));
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, buttons_init(&c)); /* singleton */
        TEST_ASSERT_EQUAL(ESP_OK, buttons_start());
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, buttons_start());
        TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
    }
    size_t heap_after = esp_get_free_heap_size();
    TEST_ASSERT_TRUE_MESSAGE(heap_before - heap_after < 512, "heap leak across init/deinit cycles");
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
}

TEST_CASE("a deep-sleep wake pin registers with power and deregisters on deinit", "[buttons]")
{
    fresh();
    buttons_pin_config_t wake[1] = {
        { .gpio = TEST_PIN_A, .active_low = true, .pull_enable = true, .wake_from_deep_sleep = true },
    };
    buttons_config_t c = base_cfg();
    c.pins = wake;
    c.count = 1;
    TEST_ASSERT_EQUAL(ESP_OK, buttons_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
    /* Re-registering after deinit proves the mask was cleared, not left stale. */
    TEST_ASSERT_EQUAL(ESP_OK, buttons_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
}

/* ------------------------------------------------------------------------- */
/* Events                                                                     */
/* ------------------------------------------------------------------------- */

TEST_CASE("press and release produce one event each with the held time", "[buttons]")
{
    fresh();
    buttons_config_t c = base_cfg();
    up(&c);

    bool p = true;
    TEST_ASSERT_EQUAL(ESP_OK, buttons_is_pressed(0, &p));
    TEST_ASSERT_FALSE(p);

    press(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(150));
    TEST_ASSERT_EQUAL(ESP_OK, buttons_is_pressed(0, &p));
    TEST_ASSERT_TRUE(p);
    release(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));

    TEST_ASSERT_EQUAL(2, s_rec_n);
    TEST_ASSERT_EQUAL(BUTTONS_EVENT_PRESSED, s_rec[0].evt);
    TEST_ASSERT_EQUAL(0, s_rec[0].index);
    TEST_ASSERT_EQUAL(TEST_PIN_A, s_rec[0].gpio);
    TEST_ASSERT_EQUAL(0, s_rec[0].held_ms);
    TEST_ASSERT_EQUAL(BUTTONS_EVENT_RELEASED, s_rec[1].evt);
    /* Held ≈ 150 ms: the press was accepted DEBOUNCE after the edge and so was the release. */
    TEST_ASSERT_UINT32_WITHIN(SLOP_MS + DEBOUNCE_MS, 150, s_rec[1].held_ms);

    buttons_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, buttons_stats(&st));
    TEST_ASSERT_TRUE(st.running);
    TEST_ASSERT_EQUAL(1, st.presses);
    TEST_ASSERT_EQUAL(1, st.releases);
    TEST_ASSERT_EQUAL(0, st.long_presses);
    TEST_ASSERT_EQUAL(0, st.events_dropped);
    TEST_ASSERT_TRUE(st.isr_wakeups >= 2);
    TEST_ASSERT_TRUE(st.task_stack_free_min > 256);
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
}

TEST_CASE("contact bounce inside the debounce window is rejected", "[buttons]")
{
    fresh();
    buttons_config_t c = base_cfg();
    up(&c);

    /* Five 8 ms half-periods: each is observed (> sample period) and none is
     * accepted (< debounce). Ends pressed. */
    for (int i = 0; i < 5; i++) {
        gpio_set_level(TEST_PIN_A, i & 1);
        vTaskDelay(pdMS_TO_TICKS(8));
    }
    press(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    release(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));

    TEST_ASSERT_EQUAL(1, rec_count(BUTTONS_EVENT_PRESSED));
    TEST_ASSERT_EQUAL(1, rec_count(BUTTONS_EVENT_RELEASED));
    buttons_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, buttons_stats(&st));
    TEST_ASSERT_TRUE_MESSAGE(st.bounces_rejected >= 2, "bounces not counted");
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
}

TEST_CASE("long press fires once, auto-repeat follows at the configured period", "[buttons]")
{
    fresh();
    buttons_config_t c = base_cfg();
    c.auto_repeat = true;
    up(&c);

    press(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(LONG_MS + 2 * REPEAT_MS + REPEAT_MS / 2)); /* 550 ms */
    release(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));

    TEST_ASSERT_EQUAL(1, rec_count(BUTTONS_EVENT_PRESSED));
    TEST_ASSERT_EQUAL(1, rec_count(BUTTONS_EVENT_LONG_PRESS));
    TEST_ASSERT_EQUAL(1, rec_count(BUTTONS_EVENT_RELEASED));
    size_t repeats = rec_count(BUTTONS_EVENT_REPEAT);
    TEST_ASSERT_TRUE_MESSAGE(repeats >= 1 && repeats <= 3, "repeat count off nominal (2)");

    /* Order and timing: PRESSED, LONG_PRESS at ≈ LONG_MS, REPEATs, RELEASED last. */
    TEST_ASSERT_EQUAL(BUTTONS_EVENT_PRESSED, s_rec[0].evt);
    TEST_ASSERT_EQUAL(BUTTONS_EVENT_LONG_PRESS, s_rec[1].evt);
    TEST_ASSERT_UINT32_WITHIN(SLOP_MS, LONG_MS, s_rec[1].held_ms);
    TEST_ASSERT_UINT32_WITHIN(SLOP_MS, LONG_MS, s_rec[1].at_ms - s_rec[0].at_ms);
    TEST_ASSERT_EQUAL(BUTTONS_EVENT_RELEASED, s_rec[s_rec_n - 1].evt);
    TEST_ASSERT_TRUE(s_rec[s_rec_n - 1].held_ms >= LONG_MS);
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());

    /* Without auto_repeat: no REPEAT, still exactly one LONG_PRESS. */
    fresh();
    c.auto_repeat = false;
    up(&c);
    press(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(LONG_MS + 2 * REPEAT_MS));
    release(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    TEST_ASSERT_EQUAL(1, rec_count(BUTTONS_EVENT_LONG_PRESS));
    TEST_ASSERT_EQUAL(0, rec_count(BUTTONS_EVENT_REPEAT));
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
}

TEST_CASE("two buttons report their own index and gpio", "[buttons]")
{
    fresh();
    buttons_config_t c = base_cfg();
    up(&c);

    press(TEST_PIN_B);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    bool a, b;
    TEST_ASSERT_EQUAL(ESP_OK, buttons_is_pressed(0, &a));
    TEST_ASSERT_EQUAL(ESP_OK, buttons_is_pressed(1, &b));
    TEST_ASSERT_FALSE(a);
    TEST_ASSERT_TRUE(b);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, buttons_is_pressed(2, &a));
    release(TEST_PIN_B);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));

    TEST_ASSERT_EQUAL(2, s_rec_n);
    TEST_ASSERT_EQUAL(1, s_rec[0].index);
    TEST_ASSERT_EQUAL(TEST_PIN_B, s_rec[0].gpio);
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
}

/* ------------------------------------------------------------------------- */
/* Sleep lock, hand-off, stop/start                                           */
/* ------------------------------------------------------------------------- */

TEST_CASE("the buttons lock is held only while a press is in progress", "[buttons]")
{
    fresh();
    buttons_config_t c = base_cfg();
    up(&c);
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());

    press(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(SLOP_MS));               /* first sample after the edge */
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep());     /* held before debounce completes */
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep());

    release(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());      /* released once settled */

    /* Balanced across many presses: the count never drifts. */
    for (int i = 0; i < 10; i++) {
        press(TEST_PIN_A);
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS));
        release(TEST_PIN_A);
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(SLOP_MS));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
}

TEST_CASE("event callback can be removed and replaced at runtime", "[buttons]")
{
    fresh();
    buttons_config_t c = base_cfg();
    up(&c);

    TEST_ASSERT_EQUAL(ESP_OK, buttons_set_event_cb(NULL, NULL));
    press(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    release(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    TEST_ASSERT_EQUAL(0, s_rec_n);
    buttons_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, buttons_stats(&st));
    TEST_ASSERT_EQUAL(2, st.events_dropped);
    TEST_ASSERT_EQUAL(1, st.presses);   /* still counted: the driver kept working */

    static int token = 42;
    s_ctx_seen = 0;
    TEST_ASSERT_EQUAL(ESP_OK, buttons_set_event_cb(recorder, &token));
    press(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    release(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    TEST_ASSERT_EQUAL(2, s_rec_n);
    TEST_ASSERT_EQUAL(42, s_ctx_seen);
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
}

TEST_CASE("stop halts events and drops the lock; start resumes without re-init", "[buttons]")
{
    fresh();
    buttons_config_t c = base_cfg();
    up(&c);

    TEST_ASSERT_EQUAL(ESP_OK, buttons_stop());
    TEST_ASSERT_EQUAL(ESP_OK, buttons_stop());         /* idempotent */
    press(TEST_PIN_A);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    TEST_ASSERT_EQUAL(0, s_rec_n);
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());      /* held pin, but not running */

    /* Start with the button already held — the wake-from-sleep case (DR1). */
    TEST_ASSERT_EQUAL(ESP_OK, buttons_start());
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS + SLOP_MS + 20));
    TEST_ASSERT_EQUAL(1, s_rec_n);
    TEST_ASSERT_EQUAL(BUTTONS_EVENT_PRESSED, s_rec[0].evt);
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep());

    /* Stop mid-press: lock released, no RELEASED event fabricated. */
    TEST_ASSERT_EQUAL(ESP_OK, buttons_stop());
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
    TEST_ASSERT_EQUAL(1, s_rec_n);
    release(TEST_PIN_A);
    TEST_ASSERT_EQUAL(ESP_OK, buttons_deinit());
}

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
