/**
 * On-target Unity tests for subsys/pm_policy (SDD §16.2).
 *
 * The one thing deliberately not tested here is a successful
 * pm_policy_try_deep_sleep(): it resets the chip, which would end the test run.
 * The veto path is fully exercised; sleep entry itself is covered by the power
 * subsystem's manual verification (SYS-TST-003).
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pm_policy.h"
#include "power.h"

/* Each test starts from a clean table. */
static void reset_subsystem(void)
{
    pm_policy_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
}

/* ------------------------------------------------------------------------- */
/* Lifecycle and argument validation                                          */
/* ------------------------------------------------------------------------- */

TEST_CASE("every call before init returns ESP_ERR_INVALID_STATE", "[pm_policy]")
{
    pm_policy_deinit();
    pm_policy_lock_handle_t h = NULL;
    size_t n = 0;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, pm_policy_lock_create("x", &h));
    TEST_ASSERT_NULL(h);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, pm_policy_lock_acquire((pm_policy_lock_handle_t)1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, pm_policy_lock_release((pm_policy_lock_handle_t)1));
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, pm_policy_try_deep_sleep(1000));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, pm_policy_holders(NULL, 0, &n));
    TEST_ASSERT_EQUAL(0, pm_policy_lock_count((pm_policy_lock_handle_t)1));
}

TEST_CASE("init is idempotent and preserves locks", "[pm_policy]")
{
    reset_subsystem();
    pm_policy_lock_handle_t h = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("keep", &h));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_acquire(h));

    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init()); /* second init */
    TEST_ASSERT_EQUAL(1, pm_policy_lock_count(h));
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep());
}

TEST_CASE("lock_create validates arguments", "[pm_policy]")
{
    reset_subsystem();
    pm_policy_lock_handle_t h = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, pm_policy_lock_create(NULL, &h));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, pm_policy_lock_create("", &h));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, pm_policy_lock_create("ok", NULL));
    TEST_ASSERT_NULL(h);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, pm_policy_lock_acquire(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, pm_policy_lock_release(NULL));
    size_t n;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, pm_policy_holders(NULL, 0, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, pm_policy_holders(NULL, 4, &n));
}

/* ------------------------------------------------------------------------- */
/* Table semantics                                                            */
/* ------------------------------------------------------------------------- */

TEST_CASE("table exhaustion returns ESP_ERR_NO_MEM without side effect", "[pm_policy]")
{
    reset_subsystem();
    /* Names must have static lifetime: a static array of literals qualifies. */
    static const char *const names[] = {
        "l0", "l1", "l2", "l3", "l4", "l5", "l6", "l7",
        "l8", "l9", "l10", "l11", "l12", "l13", "l14", "l15",
        "l16", "l17", "l18", "l19", "l20", "l21", "l22", "l23",
        "l24", "l25", "l26", "l27", "l28", "l29", "l30", "l31",
        "l32", "l33", "l34", "l35", "l36", "l37", "l38", "l39",
        "l40", "l41", "l42", "l43", "l44", "l45", "l46", "l47",
        "l48", "l49", "l50", "l51", "l52", "l53", "l54", "l55",
        "l56", "l57", "l58", "l59", "l60", "l61", "l62", "l63",
    };
    TEST_ASSERT_TRUE(CONFIG_PM_POLICY_MAX_LOCKS <= 64);

    pm_policy_lock_handle_t h;
    for (int i = 0; i < CONFIG_PM_POLICY_MAX_LOCKS; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create(names[i], &h));
    }
    pm_policy_lock_handle_t extra = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, pm_policy_lock_create("one-too-many", &extra));
    TEST_ASSERT_NULL(extra);

    size_t n = 0;
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_holders(NULL, 0, &n));
    TEST_ASSERT_EQUAL(CONFIG_PM_POLICY_MAX_LOCKS, n);

    /* An existing name still resolves even when the table is full. */
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create(names[0], &extra));
    TEST_ASSERT_NOT_NULL(extra);
}

TEST_CASE("same name returns the same handle and shares a count", "[pm_policy]")
{
    reset_subsystem();
    pm_policy_lock_handle_t a = NULL, b = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("shared", &a));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("shared", &b));
    TEST_ASSERT_EQUAL_PTR(a, b);

    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_acquire(a));
    TEST_ASSERT_EQUAL(1, pm_policy_lock_count(b));

    size_t n = 0;
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_holders(NULL, 0, &n));
    TEST_ASSERT_EQUAL(1, n); /* one slot, not two */
}

TEST_CASE("acquire and release balance; nested acquisition", "[pm_policy]")
{
    reset_subsystem();
    pm_policy_lock_handle_t h = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("nest", &h));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());

    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_acquire(h));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_acquire(h));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_acquire(h));
    TEST_ASSERT_EQUAL(3, pm_policy_lock_count(h));
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep());

    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_release(h));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_release(h));
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep()); /* still one holder */
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_release(h));
    TEST_ASSERT_EQUAL(0, pm_policy_lock_count(h));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
}

TEST_CASE("release at zero saturates and never wraps", "[pm_policy]")
{
    reset_subsystem();
    pm_policy_lock_handle_t h = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("sat", &h));

    /* Three unbalanced releases: logged at W, count stays 0, sleep still allowed. */
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_release(h));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_release(h));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_release(h));
    TEST_ASSERT_EQUAL(0, pm_policy_lock_count(h));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
}

/* ------------------------------------------------------------------------- */
/* Arbitration                                                                */
/* ------------------------------------------------------------------------- */

TEST_CASE("try_deep_sleep is vetoed by any held lock and reports all holders", "[pm_policy]")
{
    reset_subsystem();
    pm_policy_lock_handle_t usb = NULL, audio = NULL, idle = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("usb", &usb));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("audio", &audio));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("idle", &idle));

    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_acquire(usb));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_acquire(audio));
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_acquire(audio));

    /* Vetoed: must return, must not sleep. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, pm_policy_try_deep_sleep(1000 * 1000));

    pm_policy_holder_t rows[4];
    size_t n = 0;
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_holders(rows, 4, &n));
    TEST_ASSERT_EQUAL(3, n);

    uint32_t usb_count = 0, audio_count = 0, idle_count = 99;
    for (size_t i = 0; i < n; i++) {
        if (strcmp(rows[i].name, "usb") == 0)   { usb_count = rows[i].count; }
        if (strcmp(rows[i].name, "audio") == 0) { audio_count = rows[i].count; }
        if (strcmp(rows[i].name, "idle") == 0)  { idle_count = rows[i].count; }
    }
    TEST_ASSERT_EQUAL(1, usb_count);
    TEST_ASSERT_EQUAL(2, audio_count);
    TEST_ASSERT_EQUAL(0, idle_count);

    /* holders() truncates to `max` but still reports the full count. */
    pm_policy_holder_t one[1];
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_holders(one, 1, &n));
    TEST_ASSERT_EQUAL(3, n);

    /* Release usb: still vetoed by audio. */
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_release(usb));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, pm_policy_try_deep_sleep(1000 * 1000));
}

/* ------------------------------------------------------------------------- */
/* Concurrency                                                                */
/* ------------------------------------------------------------------------- */

#define HAMMER_ITERS 20000

typedef struct {
    pm_policy_lock_handle_t lock;
    volatile bool done;
} hammer_ctx_t;

static void hammer_task(void *arg)
{
    hammer_ctx_t *ctx = (hammer_ctx_t *)arg;
    for (int i = 0; i < HAMMER_ITERS; i++) {
        pm_policy_lock_acquire(ctx->lock);
        pm_policy_lock_release(ctx->lock);
        if ((i & 0x3FF) == 0) {
            taskYIELD();
        }
    }
    ctx->done = true;
    vTaskDelete(NULL);
}

TEST_CASE("concurrent acquire/release from both cores leaves the count exact", "[pm_policy][smp]")
{
    reset_subsystem();
    pm_policy_lock_handle_t h = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_create("hammer", &h));

    hammer_ctx_t c0 = { .lock = h, .done = false };
    hammer_ctx_t c1 = { .lock = h, .done = false };
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreatePinnedToCore(hammer_task, "h0", 2048, &c0,
                                                      tskIDLE_PRIORITY + 2, NULL, 0));
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreatePinnedToCore(hammer_task, "h1", 2048, &c1,
                                                      tskIDLE_PRIORITY + 2, NULL, 1));

    /* This task also hammers, from whichever core it is on. */
    for (int i = 0; i < HAMMER_ITERS; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_acquire(h));
        TEST_ASSERT_EQUAL(ESP_OK, pm_policy_lock_release(h));
    }
    while (!c0.done || !c1.done) {
        vTaskDelay(1);
    }

    TEST_ASSERT_EQUAL(0, pm_policy_lock_count(h));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
}

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
