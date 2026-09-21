/**
 * On-target Unity tests for subsys/ble (Bluedroid).
 *
 * Without a central these verify lifecycle, argument validation, service
 * registration, that the asynchronous start sequence completes with real attribute
 * handles, that advertising starts and stops with the sleep lock following it, and
 * that stop/deinit unwind cleanly. Connection, MTU, pairing and notifications need a
 * phone or a second board and are bench items (SDD §16.3).
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cfg.h"
#include "pm_policy.h"
#include "power.h"
#include "ble.h"

/* A minimal custom service: one read/write/notify characteristic with a CCCD. */
static const uint16_t UUID_PRIMARY_SERVICE = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t UUID_CHAR_DECLARE    = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t UUID_CCCD            = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t SVC_UUID  = 0xFFF0;
static const uint16_t CHAR_UUID = 0xFFF1;
static const uint8_t  char_props = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t  s_char_value[20] = "hello";
static uint8_t  s_cccd[2] = {0};

enum { IDX_SVC, IDX_CHAR_DECL, IDX_CHAR_VAL, IDX_CCCD, IDX_COUNT };

static const esp_gatts_attr_db_t s_db[IDX_COUNT] = {
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&UUID_PRIMARY_SERVICE, ESP_GATT_PERM_READ,
                 sizeof(SVC_UUID), sizeof(SVC_UUID), (uint8_t *)&SVC_UUID}},
    [IDX_CHAR_DECL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECLARE, ESP_GATT_PERM_READ,
                 sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_props}},
    [IDX_CHAR_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&CHAR_UUID,
                 ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(s_char_value), 5, s_char_value}},
    [IDX_CCCD] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&UUID_CCCD,
                 ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(s_cccd), sizeof(s_cccd), s_cccd}},
};
static uint16_t s_handles[IDX_COUNT];
static int s_writes;

static void on_write(uint16_t h, const uint8_t *d, size_t l, void *ctx) { s_writes++; }

static ble_gatts_service_t s_svc = {
    .db = s_db, .count = IDX_COUNT, .handles = s_handles, .on_write = on_write,
};

static ble_event_t s_last_evt = (ble_event_t)-1;
static int s_events;
static void on_evt(ble_event_t evt, const ble_event_info_t *info, void *ctx) { s_last_evt = evt; s_events++; }

static void fresh(void)
{
    ble_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    memset(s_handles, 0, sizeof(s_handles));
    s_events = 0;
}

TEST_CASE("before init: calls fail cleanly", "[ble]")
{
    ble_deinit();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ble_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ble_adv_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ble_gatts_add_service(&s_svc));
    TEST_ASSERT_FALSE(ble_is_connected());
    ble_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, ble_status(&st));
    TEST_ASSERT_FALSE(st.initialised);
    TEST_ASSERT_EQUAL(ESP_OK, ble_deinit());
}

TEST_CASE("init validates arguments and is a singleton", "[ble]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_init(NULL));
    ble_config_t bad = { .device_name = "" };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_init(&bad));
    bad.device_name = "this-device-name-is-far-too-long-for-an-adv-pdu";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_init(&bad));

    ble_config_t ok = { .device_name = "sdk-test", .cb = on_evt };
    TEST_ASSERT_EQUAL(ESP_OK, ble_init(&ok));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ble_init(&ok));
    ble_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, ble_status(&st));
    TEST_ASSERT_TRUE(st.initialised);
    TEST_ASSERT_FALSE(st.started);
    TEST_ASSERT_EQUAL(ESP_OK, ble_deinit());
}

TEST_CASE("service registration validates and is bounded", "[ble]")
{
    fresh();
    ble_config_t c = { .device_name = "sdk-test" };
    TEST_ASSERT_EQUAL(ESP_OK, ble_init(&c));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_gatts_add_service(NULL));
    ble_gatts_service_t bad = s_svc; bad.db = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_gatts_add_service(&bad));
    bad = s_svc; bad.count = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_gatts_add_service(&bad));
    bad = s_svc; bad.handles = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_gatts_add_service(&bad));

    static ble_gatts_service_t many[CONFIG_BLE_MAX_SERVICES + 1];
    static uint16_t many_handles[CONFIG_BLE_MAX_SERVICES + 1][IDX_COUNT];
    for (int i = 0; i <= CONFIG_BLE_MAX_SERVICES; i++) {
        many[i] = s_svc;
        many[i].handles = many_handles[i];
        esp_err_t err = ble_gatts_add_service(&many[i]);
        if (i < CONFIG_BLE_MAX_SERVICES) {
            TEST_ASSERT_EQUAL(ESP_OK, err);
        } else {
            TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, err);
        }
    }
    TEST_ASSERT_EQUAL(ESP_OK, ble_deinit());
}

TEST_CASE("start creates the attribute table, advertising toggles the sleep lock, stop unwinds", "[ble]")
{
    fresh();
    ble_config_t c = { .device_name = "sdk-test", .cb = on_evt };
    TEST_ASSERT_EQUAL(ESP_OK, ble_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, ble_gatts_add_service(&s_svc));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());

    TEST_ASSERT_EQUAL(ESP_OK, ble_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ble_start());                 /* already */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ble_gatts_add_service(&s_svc)); /* frozen */
    TEST_ASSERT_EQUAL(BLE_EVT_READY, s_last_evt);

    /* Real handles, ascending, service first. */
    TEST_ASSERT_NOT_EQUAL(0, s_handles[IDX_SVC]);
    for (int i = 1; i < IDX_COUNT; i++) {
        TEST_ASSERT_TRUE(s_handles[i] > s_handles[i - 1]);
    }

    /* Value update on a started service. */
    TEST_ASSERT_EQUAL(ESP_OK, ble_gatts_set_value(s_handles[IDX_CHAR_VAL], "abc", 3));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_gatts_set_value(0, "abc", 3));
    /* Notify without a central is refused, not attempted. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ble_gatts_notify(s_handles[IDX_CHAR_VAL], "x", 1, false));

    TEST_ASSERT_EQUAL(ESP_OK, ble_adv_start());
    TEST_ASSERT_EQUAL(ESP_OK, ble_adv_start()); /* idempotent while advertising */
    ble_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, ble_status(&st));
    TEST_ASSERT_TRUE(st.advertising);
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep()); /* "ble" held while advertising */

    vTaskDelay(pdMS_TO_TICKS(300));

    TEST_ASSERT_EQUAL(ESP_OK, ble_adv_stop());
    TEST_ASSERT_EQUAL(ESP_OK, ble_status(&st));
    TEST_ASSERT_FALSE(st.advertising);
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
    TEST_ASSERT_EQUAL(BLE_EVT_ADV_STOPPED, s_last_evt);

    TEST_ASSERT_EQUAL(ESP_OK, ble_stop());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ble_adv_start());
    TEST_ASSERT_EQUAL(ESP_OK, ble_status(&st));
    TEST_ASSERT_FALSE(st.started);
    TEST_ASSERT_EQUAL(1, st.services); /* still registered */

    /* Restart without re-init. */
    TEST_ASSERT_EQUAL(ESP_OK, ble_start());
    TEST_ASSERT_EQUAL(ESP_OK, ble_stop());
    TEST_ASSERT_EQUAL(ESP_OK, ble_deinit());
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());
}

TEST_CASE("init/deinit cycle is repeatable without leaking", "[ble]")
{
    fresh();
    ble_config_t c = { .device_name = "sdk-test" };
    size_t before = esp_get_free_heap_size();
    for (int i = 0; i < 2; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, ble_init(&c));
        TEST_ASSERT_EQUAL(ESP_OK, ble_deinit());
    }
    size_t after = esp_get_free_heap_size();
    /* Bluedroid's own allocator keeps some pools; tolerate fragmentation, not a leak per cycle. */
    TEST_ASSERT_TRUE_MESSAGE(before - after < 4096, "heap shrinks per init/deinit cycle");
}

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
