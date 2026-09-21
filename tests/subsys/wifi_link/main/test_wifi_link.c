/**
 * On-target Unity tests for subsys/wifi_link.
 *
 * Runs without an access point: verifies lifecycle, argument validation, the
 * credential helpers, and that a connect attempt to a non-existent SSID exhausts
 * its retry budget, fires GAVE_UP, and releases the sleep lock. A real
 * association needs credentials and an AP: set TEST_WIFI_SSID/TEST_WIFI_PASS via
 * -D on the build to enable the [needs_ap] case.
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cfg.h"
#include "pm_policy.h"
#include "power.h"
#include "wifi_link.h"

static wifi_link_event_t s_last;
static int              s_count;
static int              s_gave_up;

static void on_evt(wifi_link_event_t evt, const wifi_link_event_info_t *info, void *ctx)
{
    s_last = evt;
    s_count++;
    if (evt == WIFI_LINK_EVT_GAVE_UP) {
        s_gave_up++;
    }
}

static void fresh(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    s_count = 0;
    s_gave_up = 0;
}

TEST_CASE("before init: calls fail cleanly", "[wifi_sta]")
{
    wifi_link_deinit();
    wifi_link_session_t c = { .ssid = "x" };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, wifi_link_connect(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, wifi_link_disconnect());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, wifi_link_wait_ip(10));
    TEST_ASSERT_FALSE(wifi_link_is_connected());
    wifi_link_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_status(&st));
    TEST_ASSERT_FALSE(st.connected);
}

TEST_CASE("init is idempotent; connect validates arguments", "[wifi_sta]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_init());
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_init());

    wifi_link_session_t c = { 0 };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wifi_link_connect(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wifi_link_connect(&c));            /* NULL ssid */
    c.ssid = "";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wifi_link_connect(&c));
    c.ssid = "123456789012345678901234567890123";                              /* 33 chars */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wifi_link_connect(&c));
    static char long_pass[66];
    memset(long_pass, 'p', 65);
    c.ssid = "ok";
    c.password = long_pass;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wifi_link_connect(&c));
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_disconnect()); /* idle: no-op */
}

TEST_CASE("credential helpers: save, load, clear", "[wifi_sta]")
{
    fresh();
    char ssid[WIFI_LINK_SSID_MAX + 1], pass[WIFI_LINK_PASS_MAX + 1];

    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_credentials_clear());
    TEST_ASSERT_EQUAL(ESP_ERR_NVS_NOT_FOUND, wifi_link_credentials_load(ssid, sizeof(ssid), pass, sizeof(pass)));
    TEST_ASSERT_EQUAL_STRING("", ssid);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wifi_link_credentials_save(NULL, "x"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wifi_link_credentials_save("", "x"));
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_credentials_save("MyNet", "s3cret"));
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_credentials_load(ssid, sizeof(ssid), pass, sizeof(pass)));
    TEST_ASSERT_EQUAL_STRING("MyNet", ssid);
    TEST_ASSERT_EQUAL_STRING("s3cret", pass);

    /* Open network: NULL password stores empty. */
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_credentials_save("Open", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_credentials_load(ssid, sizeof(ssid), pass, sizeof(pass)));
    TEST_ASSERT_EQUAL_STRING("Open", ssid);
    TEST_ASSERT_EQUAL_STRING("", pass);

    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_credentials_clear());
    TEST_ASSERT_EQUAL(ESP_ERR_NVS_NOT_FOUND, wifi_link_credentials_load(ssid, sizeof(ssid), pass, sizeof(pass)));
}

TEST_CASE("bounded retries: a non-existent SSID ends in GAVE_UP with the lock released", "[wifi_sta]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_init());
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());

    wifi_link_session_t c = {
        .ssid = "sdk-test-no-such-network-7f3a",
        .password = "irrelevant",
        .max_attempts = 2,
        .reconnect_on_loss = false,
        .cb = on_evt,
    };
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_connect(&c));
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep());                  /* "wifi" held */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, wifi_link_connect(&c)); /* session active */

    esp_err_t err = wifi_link_wait_ip(30000);
    TEST_ASSERT_EQUAL(ESP_ERR_WIFI_CONN, err);
    TEST_ASSERT_EQUAL(1, s_gave_up);
    TEST_ASSERT_FALSE(wifi_link_is_connected());
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());                   /* released */

    wifi_link_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_status(&st));
    TEST_ASSERT_EQUAL(2, st.connect_attempts);

    /* A new session may start after GAVE_UP. */
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_connect(&c));
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_disconnect());
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_deinit());
}

#if defined(TEST_WIFI_SSID) && defined(TEST_WIFI_PASS)
TEST_CASE("associates with the configured access point and gets an IP", "[wifi_sta][needs_ap]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_init());
    wifi_link_session_t c = { .ssid = TEST_WIFI_SSID, .password = TEST_WIFI_PASS, .max_attempts = 5, .cb = on_evt };
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_connect(&c));
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_wait_ip(20000));
    TEST_ASSERT_TRUE(wifi_link_is_connected());
    wifi_link_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_status(&st));
    TEST_ASSERT_NOT_EQUAL(0, st.ip.addr);
    TEST_ASSERT_TRUE(st.rssi < 0);
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_disconnect());
    TEST_ASSERT_EQUAL(ESP_OK, wifi_link_deinit());
}
#endif

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
