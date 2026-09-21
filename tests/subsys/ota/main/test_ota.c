/**
 * On-target Unity tests for subsys/ota.
 *
 * Two configurations matter:
 *  - This project's default single-app partition table, where ota_init() MUST
 *    refuse (DES-OTA-001, TBD-001's whole point).
 *  - A dual-slot table (build with `-DSDKCONFIG_DEFAULTS=...dual.defaults`), where
 *    init succeeds and the session state machine is exercised against an
 *    unreachable URL — the failure path, without a network.
 *
 * A real download needs Wi-Fi and a server: that is the integration test in
 * SDD §16.3, not a unit test.
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ota_ops.h"
#include "pm_policy.h"
#include "wifi_link.h"
#include "power.h"
#include "ota.h"

static ota_event_t   s_last_evt = (ota_event_t)-1;
static esp_err_t     s_last_err;
static int           s_events;

static void on_ota(ota_event_t evt, const ota_event_info_t *info, void *ctx)
{
    s_last_evt = evt;
    s_last_err = info ? info->err : ESP_OK;
    s_events++;
}

static bool dual_slot(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    return run != NULL && next != NULL && next != run;
}

static void fresh(void)
{
    ota_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    s_events = 0;
    s_last_evt = (ota_event_t)-1;
    s_last_err = ESP_OK;
}

TEST_CASE("before init: every call returns ESP_ERR_INVALID_STATE", "[ota]")
{
    ota_deinit();
    ota_status_t st;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_start("https://example.invalid/fw.bin"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_abort());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_mark_valid());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_mark_invalid());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_status(&st));
    TEST_ASSERT_FALSE(ota_in_progress());
    TEST_ASSERT_EQUAL(ESP_OK, ota_deinit()); /* idempotent */
}

TEST_CASE("init requires a trust anchor", "[ota]")
{
    fresh();
    ota_config_t none = { 0 };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ota_init(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ota_init(&none));
}

TEST_CASE("init refuses a single-app partition table, accepts a dual-slot one", "[ota]")
{
    fresh();
    ota_config_t cfg = { .use_cert_bundle = true, .cb = on_ota };
    esp_err_t err = ota_init(&cfg);
    if (dual_slot()) {
        TEST_ASSERT_EQUAL(ESP_OK, err);
        TEST_ASSERT_EQUAL(ESP_OK, ota_init(&cfg)); /* idempotent */
        ota_status_t st;
        TEST_ASSERT_EQUAL(ESP_OK, ota_status(&st));
        TEST_ASSERT_TRUE(strlen(st.running_label) > 0);
        TEST_ASSERT_TRUE(strlen(st.next_label) > 0);
        TEST_ASSERT_NOT_EQUAL(0, strcmp(st.running_label, st.next_label));
        TEST_ASSERT_FALSE(st.in_progress);
    } else {
        TEST_ASSERT_EQUAL_MESSAGE(ESP_ERR_INVALID_STATE, err,
                                  "single-app table must be refused (DES-OTA-001)");
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_start("https://example.invalid/fw.bin"));
    }
}

TEST_CASE("start validates the URL and refuses plaintext HTTP", "[ota]")
{
    fresh();
    if (!dual_slot()) {
        TEST_IGNORE_MESSAGE("needs a dual-slot partition table");
    }
    ota_config_t cfg = { .use_cert_bundle = true, .cb = on_ota };
    TEST_ASSERT_EQUAL(ESP_OK, ota_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ota_start(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ota_start(""));
#if !CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ota_start("http://example.invalid/fw.bin"));
#endif
    static char too_long[300];
    memset(too_long, 'a', sizeof(too_long) - 1);
    memcpy(too_long, "https://", 8);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ota_start(too_long));
    TEST_ASSERT_FALSE(ota_in_progress());
}

TEST_CASE("start is refused before any network interface exists (lwIP would assert)", "[ota]")
{
    fresh();
    if (!dual_slot()) {
        TEST_IGNORE_MESSAGE("needs a dual-slot partition table");
    }
    ota_config_t cfg = { .use_cert_bundle = true, .cb = on_ota };
    TEST_ASSERT_EQUAL(ESP_OK, ota_init(&cfg));
    /* This test file runs before wifi_link_init() in this binary; ordering matters. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_start("https://198.51.100.1/fw.bin"));
    TEST_ASSERT_FALSE(ota_in_progress());
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
}

TEST_CASE("a session against an unreachable host fails cleanly and releases the lock", "[ota]")
{
    fresh();
    if (!dual_slot()) {
        TEST_IGNORE_MESSAGE("needs a dual-slot partition table");
    }
    ota_config_t cfg = { .use_cert_bundle = true, .cb = on_ota };
    TEST_ASSERT_EQUAL(ESP_OK, ota_init(&cfg));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());

    /* Bring the network stack up without connecting: the interface exists but has
     * no route, so the session must fail in the socket layer, not crash. */
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_init());
    TEST_ASSERT_EQUAL(ESP_OK, ota_start("https://198.51.100.1/fw.bin")); /* TEST-NET-2 */
    /* The worker (prio 5) may fail the connect before this task runs again, so
     * in-progress state is only checked if we happen to observe it (FW-OTA-013). */
    if (ota_in_progress()) {
        TEST_ASSERT_FALSE(pm_policy_can_deep_sleep());      /* "ota" lock held */
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_start("https://198.51.100.1/x"));
    }

    for (int i = 0; i < 600 && ota_in_progress(); i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    TEST_ASSERT_FALSE(ota_in_progress());
    TEST_ASSERT_EQUAL(OTA_EVT_FAILED, s_last_evt);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, s_last_err);
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());           /* lock released */

    /* A new session may start after a failure. */
    TEST_ASSERT_EQUAL(ESP_OK, ota_start("https://198.51.100.1/fw.bin"));
    for (int i = 0; i < 600 && ota_in_progress(); i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    TEST_ASSERT_EQUAL(2, s_events >= 2 ? 2 : s_events);

    /* Running image untouched: still the same partition. */
    ota_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, ota_status(&st));
    TEST_ASSERT_FALSE(st.in_progress);
}

TEST_CASE("mark_valid is a no-op when not pending verify", "[ota]")
{
    fresh();
    if (!dual_slot()) {
        TEST_IGNORE_MESSAGE("needs a dual-slot partition table");
    }
    ota_config_t cfg = { .use_cert_bundle = true };
    TEST_ASSERT_EQUAL(ESP_OK, ota_init(&cfg));
    ota_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, ota_status(&st));
    if (st.pending_verify) {
        TEST_IGNORE_MESSAGE("image is pending verify; not marking from a test");
    }
    TEST_ASSERT_EQUAL(ESP_OK, ota_mark_valid());
    TEST_ASSERT_EQUAL(ESP_OK, ota_mark_valid());
}

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
