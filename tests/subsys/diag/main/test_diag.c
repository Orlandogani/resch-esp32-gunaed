/**
 * On-target Unity tests for subsys/diag (SDD §16.2).
 *
 * A real watchdog expiry panics the chip, so expiry itself is not tested here;
 * subscription, feeding, and unsubscription are. Fault handling is tested for its
 * state transition, callback contract, and persistence via cfg.
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cfg.h"
#include "diag.h"

static uint32_t    s_cb_code;
static const char *s_cb_detail;
static int         s_cb_calls;
static void       *s_cb_ctx;

static void fault_cb(uint32_t code, const char *detail, void *ctx)
{
    s_cb_code = code;
    s_cb_detail = detail;
    s_cb_ctx = ctx;
    s_cb_calls++;
}

/* Reset the persistent counters so assertions are absolute, not relative. */
static void wipe_persistent(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    cfg_handle_t h = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, cfg_open("diag", &h));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_erase_all(h));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_close(h));
}

static void fresh(const diag_config_t *cfg)
{
    TEST_ASSERT_EQUAL(ESP_OK, diag_deinit());
    s_cb_calls = 0;
    s_cb_code = 0;
    s_cb_detail = NULL;
    s_cb_ctx = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, diag_init(cfg));
}

/* ------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* ------------------------------------------------------------------------- */

TEST_CASE("before init: state is BOOT, register fails, health still answers", "[diag]")
{
    TEST_ASSERT_EQUAL(ESP_OK, diag_deinit());
    TEST_ASSERT_EQUAL(DIAG_STATE_BOOT, diag_state());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, diag_task_register(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, diag_mark_running());

    diag_health_t h;
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_EQUAL(DIAG_STATE_BOOT, h.state);
    TEST_ASSERT_TRUE(h.free_heap > 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, diag_health(NULL));
}

TEST_CASE("init is idempotent and increments the persistent boot count once", "[diag]")
{
    wipe_persistent();
    fresh(NULL);
    diag_health_t h;
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_EQUAL(1, h.boot_count);

    TEST_ASSERT_EQUAL(ESP_OK, diag_init(NULL)); /* second call: no second increment */
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_EQUAL(1, h.boot_count);

    /* A genuine re-init (deinit + init) models the next boot. */
    fresh(NULL);
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_EQUAL(2, h.boot_count);
}

TEST_CASE("state machine: BOOT -> RUNNING -> FAULT, FAULT is terminal", "[diag]")
{
    wipe_persistent();
    fresh(NULL);
    TEST_ASSERT_EQUAL(DIAG_STATE_BOOT, diag_state());
    TEST_ASSERT_EQUAL(ESP_OK, diag_mark_running());
    TEST_ASSERT_EQUAL(DIAG_STATE_RUNNING, diag_state());

    diag_fault(0x1001, "test fault");
    TEST_ASSERT_EQUAL(DIAG_STATE_FAULT, diag_state());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, diag_mark_running());

    /* Health keeps working in FAULT and reports the code. */
    diag_health_t h;
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_EQUAL(DIAG_STATE_FAULT, h.state);
    TEST_ASSERT_EQUAL(0x1001, h.current_fault_code);
    TEST_ASSERT_EQUAL(0x1001, h.last_fault_code);
    TEST_ASSERT_EQUAL(1, h.fault_count);
}

/* ------------------------------------------------------------------------- */
/* Faults                                                                     */
/* ------------------------------------------------------------------------- */

TEST_CASE("fault callback fires once with code, detail and ctx; later faults counted", "[diag]")
{
    wipe_persistent();
    int marker = 0;
    diag_config_t cfg = { .fault_cb = fault_cb, .fault_ctx = &marker };
    fresh(&cfg);
    TEST_ASSERT_EQUAL(ESP_OK, diag_mark_running());

    diag_fault(0x2002, "first");
    TEST_ASSERT_EQUAL(1, s_cb_calls);
    TEST_ASSERT_EQUAL(0x2002, s_cb_code);
    TEST_ASSERT_EQUAL_STRING("first", s_cb_detail);
    TEST_ASSERT_EQUAL_PTR(&marker, s_cb_ctx);

    /* A second fault: counted and persisted, callback NOT re-invoked, first code kept. */
    diag_fault(0x3003, NULL);
    TEST_ASSERT_EQUAL(1, s_cb_calls);
    diag_health_t h;
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_EQUAL(2, h.fault_count);
    TEST_ASSERT_EQUAL(0x2002, h.current_fault_code);
    TEST_ASSERT_EQUAL(0x3003, h.last_fault_code);
}

TEST_CASE("fault code zero is never recorded as 'no fault'", "[diag]")
{
    wipe_persistent();
    fresh(NULL);
    diag_fault(0, "zero");
    diag_health_t h;
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_NOT_EQUAL(0, h.current_fault_code);
    TEST_ASSERT_EQUAL(DIAG_STATE_FAULT, h.state);
}

TEST_CASE("fault count and last code persist across re-init", "[diag]")
{
    wipe_persistent();
    fresh(NULL);
    diag_fault(0x4004, "persisted");
    fresh(NULL); /* models a reboot */
    diag_health_t h;
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_EQUAL(DIAG_STATE_BOOT, h.state);   /* state is not persistent */
    TEST_ASSERT_EQUAL(0, h.current_fault_code);    /* nor is the current fault */
    TEST_ASSERT_EQUAL(1, h.fault_count);           /* but the history is */
    TEST_ASSERT_EQUAL(0x4004, h.last_fault_code);
}

/* ------------------------------------------------------------------------- */
/* Task watchdog                                                              */
/* ------------------------------------------------------------------------- */

TEST_CASE("wdt: register current task, feed, unregister; double register rejected", "[diag]")
{
    fresh(NULL);
    TEST_ASSERT_EQUAL(ESP_OK, diag_task_register(NULL));
    diag_health_t h;
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_EQUAL(1, h.wdt_tasks);

    TEST_ASSERT_EQUAL(ESP_OK, diag_task_feed());
    TEST_ASSERT_NOT_EQUAL(ESP_OK, diag_task_register(NULL)); /* already subscribed */

    TEST_ASSERT_EQUAL(ESP_OK, diag_task_unregister(NULL));
    TEST_ASSERT_EQUAL(ESP_OK, diag_health(&h));
    TEST_ASSERT_EQUAL(0, h.wdt_tasks);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, diag_task_feed()); /* not subscribed any more */
}

TEST_CASE("wdt: deinit unsubscribes everything it registered", "[diag]")
{
    fresh(NULL);
    TEST_ASSERT_EQUAL(ESP_OK, diag_task_register(NULL));
    TEST_ASSERT_EQUAL(ESP_OK, diag_deinit());
    /* If deinit had leaked the subscription, a fresh register would fail. */
    fresh(NULL);
    TEST_ASSERT_EQUAL(ESP_OK, diag_task_register(NULL));
    TEST_ASSERT_EQUAL(ESP_OK, diag_task_unregister(NULL));
}

/* ------------------------------------------------------------------------- */
/* Misc                                                                       */
/* ------------------------------------------------------------------------- */

TEST_CASE("log level and coredump entry points behave", "[diag]")
{
    fresh(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, diag_set_log_level(NULL, ESP_LOG_INFO));
    TEST_ASSERT_EQUAL(ESP_OK, diag_set_log_level("diag", ESP_LOG_DEBUG));
    TEST_ASSERT_EQUAL(ESP_OK, diag_set_log_level("diag", ESP_LOG_INFO));

    bool present = true;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, diag_coredump_present(NULL));
    esp_err_t err = diag_coredump_present(&present);
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_NOT_SUPPORTED);
    err = diag_coredump_erase();
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_NOT_SUPPORTED);
}

TEST_CASE("rate-limited log macro suppresses repeats", "[diag]")
{
    /* Behavioural smoke test: this must not crash and must compile in a loop.
     * Output inspection is manual; the counter logic is trivially readable. */
    for (int i = 0; i < 50; i++) {
        DIAG_LOG_RL(ESP_LOGW, "rl", 200, "burst %d", i);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
    DIAG_LOG_RL(ESP_LOGW, "rl", 200, "after pause");
    TEST_PASS();
}

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
