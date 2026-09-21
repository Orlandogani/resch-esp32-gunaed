#include "ble.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "ble";

#if !(CONFIG_BT_ENABLED && CONFIG_BT_BLUEDROID_ENABLED)

/* -------------------------------------------------------------------------- */
/* Bluetooth compiled out: honest stubs (FW-SYS-004, FW-BLE-011).              */
/* -------------------------------------------------------------------------- */

esp_err_t ble_init(const ble_config_t *cfg)
{
    ESP_LOGW(TAG, "Bluetooth is not enabled in this build (CONFIG_BT_ENABLED/CONFIG_BT_BLUEDROID_ENABLED)");
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t ble_deinit(void)                                       { return ESP_OK; }
esp_err_t ble_gatts_add_service(ble_gatts_service_t *svc)        { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_start(void)                                        { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_stop(void)                                         { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_adv_start(void)                                    { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_adv_stop(void)                                     { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_disconnect(void)                                   { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gatts_notify(uint16_t h, const void *d, size_t l, bool i) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gatts_set_value(uint16_t h, const void *d, size_t l)      { return ESP_ERR_NOT_SUPPORTED; }
bool      ble_is_connected(void)                                 { return false; }
esp_err_t ble_status(ble_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    return ESP_OK;
}

#else /* Bluetooth enabled */

#if !CONFIG_BT_BLE_42_FEATURES_SUPPORTED || !CONFIG_BT_BLE_42_ADV_EN
#error "subsys/ble needs Bluedroid 4.2 legacy advertising: CONFIG_BT_BLE_42_FEATURES_SUPPORTED=y, CONFIG_BT_BLE_42_ADV_EN=y, CONFIG_BT_BLE_50_FEATURES_SUPPORTED=n (ADR-018)"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "cfg.h"
#include "pm_policy.h"

#define BLE_APP_ID          0x55
#define BLE_NAME_MAX        29     /* Fits a 31-byte advertising PDU with flags. */
#define BLE_MTU_LOCAL       CONFIG_BLE_LOCAL_MTU

/* Event bits used to make the asynchronous start sequence synchronous. */
#define BIT_GATTS_REG       BIT0
#define BIT_ATTR_TAB        BIT1
#define BIT_ADV_DATA        BIT2
#define BIT_ADV_START       BIT3
#define BIT_ADV_STOP        BIT4
#define BIT_FAIL            BIT5

typedef enum { ST_UNINIT = 0, ST_INIT, ST_STARTED } state_t;

typedef struct {
    ble_gatts_service_t *svc;
    uint16_t             first_handle;
    uint16_t             last_handle;
} svc_slot_t;

static state_t                 s_state;
static ble_config_t            s_cfg;
static char                    s_name[BLE_NAME_MAX + 1];
static svc_slot_t              s_services[CONFIG_BLE_MAX_SERVICES];
static uint8_t                 s_service_count;
static uint8_t                 s_tables_done;
static esp_gatt_if_t           s_gatts_if = ESP_GATT_IF_NONE;
static EventGroupHandle_t      s_events;
static StaticEventGroup_t      s_events_storage;
static pm_policy_lock_handle_t s_lock;
static bool                    s_lock_held;
static ble_status_t            s_status;
static esp_bd_addr_t           s_peer;
static bool                    s_adv_wanted;   /* Application asked for advertising. */

static esp_ble_adv_params_t s_adv_params = {
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void lock_set(bool held)
{
    if (held && !s_lock_held) {
        pm_policy_lock_acquire(s_lock);
        s_lock_held = true;
    } else if (!held && s_lock_held) {
        pm_policy_lock_release(s_lock);
        s_lock_held = false;
    }
}

static void emit(ble_event_t evt, const ble_event_info_t *info)
{
    if (s_cfg.cb != NULL) {
        static const ble_event_info_t empty = {0};
        s_cfg.cb(evt, info ? info : &empty, s_cfg.ctx);
    }
}

static svc_slot_t *slot_for_handle(uint16_t handle)
{
    for (uint8_t i = 0; i < s_service_count; i++) {
        if (handle >= s_services[i].first_handle && handle <= s_services[i].last_handle) {
            return &s_services[i];
        }
    }
    return NULL;
}

/* -------------------------------------------------------------------------- */
/* GAP callback — BTC task                                                     */
/* -------------------------------------------------------------------------- */

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        xEventGroupSetBits(s_events, BIT_ADV_DATA);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            s_status.advertising = true;
            lock_set(true);
            ESP_LOGI(TAG, "advertising as '%s'", s_name);
            emit(BLE_EVT_ADV_STARTED, NULL);
            xEventGroupSetBits(s_events, BIT_ADV_START);
        } else {
            ESP_LOGE(TAG, "advertising start failed: %d", param->adv_start_cmpl.status);
            xEventGroupSetBits(s_events, BIT_FAIL);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        s_status.advertising = false;
        if (!s_status.connected) {
            lock_set(false);
        }
        emit(BLE_EVT_ADV_STOPPED, NULL);
        xEventGroupSetBits(s_events, BIT_ADV_STOP);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        ble_event_info_t info = {0};
        memcpy(info.peer, param->ble_security.auth_cmpl.bd_addr, 6);
        if (param->ble_security.auth_cmpl.success) {
            s_status.encrypted = true;
            info.bonded = true; /* Bonding is requested whenever pairing happens. */
            ESP_LOGI(TAG, "paired (auth mode %d)", param->ble_security.auth_cmpl.auth_mode);
            emit(BLE_EVT_PAIRED, &info);
        } else {
            info.reason = param->ble_security.auth_cmpl.fail_reason;
            ESP_LOGW(TAG, "pairing failed: 0x%02x", info.reason);
            emit(BLE_EVT_PAIR_FAILED, &info);
        }
        break;
    }

    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* Peer asks for security: accept, letting the SM negotiate LE SC. */
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        s_status.conn_interval_ms = (uint16_t)((param->update_conn_params.conn_int * 5) / 4);
        ESP_LOGD(TAG, "conn params: interval %u ms latency %u timeout %u ms",
                 s_status.conn_interval_ms, param->update_conn_params.latency,
                 param->update_conn_params.timeout * 10);
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* GATTS callback — BTC task                                                   */
/* -------------------------------------------------------------------------- */

static void gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            s_gatts_if = gatts_if;
            xEventGroupSetBits(s_events, BIT_GATTS_REG);
        } else {
            ESP_LOGE(TAG, "gatts app register failed: %d", param->reg.status);
            xEventGroupSetBits(s_events, BIT_FAIL);
        }
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
        uint8_t idx = param->add_attr_tab.svc_inst_id;
        if (param->add_attr_tab.status != ESP_GATT_OK || idx >= s_service_count) {
            ESP_LOGE(TAG, "attribute table %u failed: %d", idx, param->add_attr_tab.status);
            xEventGroupSetBits(s_events, BIT_FAIL);
            break;
        }
        svc_slot_t *slot = &s_services[idx];
        uint16_t n = param->add_attr_tab.num_handle;
        if (n != slot->svc->count) {
            ESP_LOGE(TAG, "service %u: expected %u handles, got %u", idx, slot->svc->count, n);
            xEventGroupSetBits(s_events, BIT_FAIL);
            break;
        }
        memcpy(slot->svc->handles, param->add_attr_tab.handles, n * sizeof(uint16_t));
        slot->first_handle = slot->svc->handles[0];
        slot->last_handle = slot->svc->handles[n - 1];
        esp_ble_gatts_start_service(slot->first_handle);
        ESP_LOGI(TAG, "service %u started: handles 0x%04x..0x%04x", idx, slot->first_handle, slot->last_handle);
        if (++s_tables_done == s_service_count) {
            xEventGroupSetBits(s_events, BIT_ATTR_TAB);
        }
        break;
    }

    case ESP_GATTS_CONNECT_EVT: {
        s_status.connected = true;
        s_status.encrypted = false;
        s_status.conn_id = param->connect.conn_id;
        s_status.connections++;
        s_status.conn_interval_ms = (uint16_t)((param->connect.conn_params.interval * 5) / 4);
        memcpy(s_peer, param->connect.remote_bda, sizeof(s_peer));
        memcpy(s_status.peer, param->connect.remote_bda, sizeof(s_status.peer));
        s_status.advertising = false; /* Legacy advertising stops on connection. */
        lock_set(true);
        ESP_LOGI(TAG, "connected: conn_id %u peer %02x:%02x:%02x:%02x:%02x:%02x interval %u ms",
                 s_status.conn_id, s_peer[0], s_peer[1], s_peer[2], s_peer[3], s_peer[4], s_peer[5],
                 s_status.conn_interval_ms);

        if (s_cfg.encrypt_on_connect && !s_cfg.no_bonding) {
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
        }
        ble_event_info_t info = {
            .conn_id = s_status.conn_id,
            .conn_interval_ms = s_status.conn_interval_ms,
            .latency = param->connect.conn_params.latency,
            .timeout_ms = (uint16_t)(param->connect.conn_params.timeout * 10),
        };
        memcpy(info.peer, s_peer, 6);
        emit(BLE_EVT_CONNECTED, &info);
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT: {
        s_status.connected = false;
        s_status.encrypted = false;
        s_status.disconnections++;
        s_status.mtu = 0;
        ble_event_info_t info = { .conn_id = param->disconnect.conn_id, .reason = param->disconnect.reason };
        memcpy(info.peer, param->disconnect.remote_bda, 6);
        ESP_LOGI(TAG, "disconnected: reason 0x%02x", param->disconnect.reason);
        emit(BLE_EVT_DISCONNECTED, &info);

        /* DES-BLE-005: a supervision timeout must not leave us unreachable. */
        if (s_state == ST_STARTED && s_adv_wanted && !s_cfg.no_auto_readvertise) {
            esp_ble_gap_start_advertising(&s_adv_params);
        } else {
            lock_set(false);
        }
        break;
    }

    case ESP_GATTS_MTU_EVT: {
        s_status.mtu = param->mtu.mtu;
        ble_event_info_t info = { .conn_id = param->mtu.conn_id, .mtu = param->mtu.mtu };
        ESP_LOGI(TAG, "mtu %u", param->mtu.mtu);
        emit(BLE_EVT_MTU_CHANGED, &info);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        /* Attribute tables with ESP_GATT_AUTO_RSP have the value stored by the stack
         * already; we respond if asked and then tell the owning service. */
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                        ESP_GATT_OK, NULL);
        }
        svc_slot_t *slot = slot_for_handle(param->write.handle);
        if (slot != NULL && slot->svc->on_write != NULL && !param->write.is_prep) {
            slot->svc->on_write(param->write.handle, param->write.value, param->write.len, slot->svc->ctx);
        }
        break;
    }

    case ESP_GATTS_EXEC_WRITE_EVT:
        esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id, param->exec_write.trans_id,
                                    ESP_GATT_OK, NULL);
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

static esp_err_t configure_security(void)
{
    if (s_cfg.no_bonding) {
        return ESP_OK;
    }
    esp_ble_auth_req_t auth = ESP_LE_AUTH_REQ_SC_BOND;   /* LE SC + bonding; SYS-SEC-005 */
    esp_ble_io_cap_t   iocap = ESP_IO_CAP_NONE;          /* Just Works */
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t only_accept_specified_auth = 0;
    uint8_t oob = ESP_BLE_OOB_DISABLE;

    esp_err_t err = ESP_OK;
    err |= esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth, sizeof(auth));
    err |= esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    err |= esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    err |= esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    err |= esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
    err |= esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
                                          &only_accept_specified_auth, sizeof(only_accept_specified_auth));
    err |= esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob, sizeof(oob));
    return err == ESP_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t ble_init(const ble_config_t *cfg)
{
    if (s_state != ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const char *name = cfg->device_name ? cfg->device_name : CONFIG_BLE_DEVICE_NAME;
    if (name[0] == '\0' || strlen(name) > BLE_NAME_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Bluedroid keeps bonds in NVS. */
    esp_err_t err = cfg_init();
    if (err != ESP_OK) {
        return err;
    }

    s_cfg = *cfg;
    strlcpy(s_name, name, sizeof(s_name));
    s_cfg.device_name = s_name;

    uint16_t min_ms = cfg->adv_interval_min_ms ? cfg->adv_interval_min_ms : CONFIG_BLE_ADV_INTERVAL_MS;
    uint16_t max_ms = cfg->adv_interval_max_ms ? cfg->adv_interval_max_ms : (uint16_t)(min_ms + min_ms / 4);
    if (max_ms < min_ms) {
        max_ms = min_ms;
    }
    s_adv_params.adv_int_min = (uint16_t)((min_ms * 8) / 5); /* 0.625 ms units */
    s_adv_params.adv_int_max = (uint16_t)((max_ms * 8) / 5);

    /* The ESP32-S3 has no Classic radio; give its memory back (CON-04). */
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "controller init: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "controller enable: %s", esp_err_to_name(err));
        goto fail_ctrl_deinit;
    }

    esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    err = esp_bluedroid_init_with_cfg(&bd_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid init: %s", esp_err_to_name(err));
        goto fail_ctrl_disable;
    }
    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid enable: %s", esp_err_to_name(err));
        goto fail_bd_deinit;
    }

    err = esp_ble_gap_register_callback(gap_cb);
    if (err == ESP_OK) {
        err = esp_ble_gatts_register_callback(gatts_cb);
    }
    if (err == ESP_OK) {
        err = configure_security();
    }
    if (err == ESP_OK) {
        err = esp_ble_gatt_set_local_mtu(BLE_MTU_LOCAL);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gap/gatts setup: %s", esp_err_to_name(err));
        goto fail_bd_disable;
    }

    err = pm_policy_lock_create("ble", &s_lock);
    if (err != ESP_OK) {
        goto fail_bd_disable;
    }

    s_events = xEventGroupCreateStatic(&s_events_storage);
    memset(s_services, 0, sizeof(s_services));
    s_service_count = 0;
    memset(&s_status, 0, sizeof(s_status));
    s_status.initialised = true;
    s_gatts_if = ESP_GATT_IF_NONE;
    s_adv_wanted = false;
    s_state = ST_INIT;

    ESP_LOGI(TAG, "init: Bluedroid, LE only, name '%s', adv %u-%u ms, bonding %s",
             s_name, min_ms, max_ms, s_cfg.no_bonding ? "off" : "LE SC");
    return ESP_OK;

fail_bd_disable:
    esp_bluedroid_disable();
fail_bd_deinit:
    esp_bluedroid_deinit();
fail_ctrl_disable:
    esp_bt_controller_disable();
fail_ctrl_deinit:
    esp_bt_controller_deinit();
    return err;
}

esp_err_t ble_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    if (s_state == ST_STARTED) {
        ble_stop();
    }
    lock_set(false);
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    vEventGroupDelete(s_events);
    s_events = NULL;
    memset(&s_status, 0, sizeof(s_status));
    s_service_count = 0;
    s_state = ST_UNINIT;
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

esp_err_t ble_gatts_add_service(ble_gatts_service_t *svc)
{
    if (s_state != ST_INIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (svc == NULL || svc->db == NULL || svc->count == 0 || svc->handles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_service_count >= CONFIG_BLE_MAX_SERVICES) {
        ESP_LOGE(TAG, "service table full (max %d); raise CONFIG_BLE_MAX_SERVICES", CONFIG_BLE_MAX_SERVICES);
        return ESP_ERR_NO_MEM;
    }
    s_services[s_service_count].svc = svc;
    s_services[s_service_count].first_handle = 0;
    s_services[s_service_count].last_handle = 0;
    s_service_count++;
    s_status.services = s_service_count;
    return ESP_OK;
}

static esp_err_t wait_bits(EventBits_t want)
{
    EventBits_t got = xEventGroupWaitBits(s_events, want | BIT_FAIL, pdTRUE, pdFALSE,
                                          pdMS_TO_TICKS(CONFIG_BLE_START_TIMEOUT_MS));
    if (got & BIT_FAIL) {
        return ESP_FAIL;
    }
    return (got & want) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t ble_start(void)
{
    if (s_state != ST_INIT) {
        return ESP_ERR_INVALID_STATE;
    }
    xEventGroupClearBits(s_events, 0xFF);
    s_tables_done = 0;

    esp_err_t err = esp_ble_gatts_app_register(BLE_APP_ID);
    if (err != ESP_OK) {
        return err;
    }
    err = wait_bits(BIT_GATTS_REG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gatts app register: %s", esp_err_to_name(err));
        return err;
    }

    for (uint8_t i = 0; i < s_service_count; i++) {
        err = esp_ble_gatts_create_attr_tab(s_services[i].svc->db, s_gatts_if, s_services[i].svc->count, i);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "create attr tab %u: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    if (s_service_count > 0) {
        err = wait_bits(BIT_ATTR_TAB);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "attribute tables: %s", esp_err_to_name(err));
            return err;
        }
    }

    err = esp_ble_gap_set_device_name(s_name);
    if (err != ESP_OK) {
        return err;
    }
    esp_ble_adv_data_t adv = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0006,
        .max_interval = 0x0010,
        .appearance = 0x00,
        .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
    };
    err = esp_ble_gap_config_adv_data(&adv);
    if (err != ESP_OK) {
        return err;
    }
    err = wait_bits(BIT_ADV_DATA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "advertising data: %s", esp_err_to_name(err));
        return err;
    }

    s_state = ST_STARTED;
    s_status.started = true;
    ESP_LOGI(TAG, "started: %u service(s)", s_service_count);
    emit(BLE_EVT_READY, NULL);
    return ESP_OK;
}

esp_err_t ble_stop(void)
{
    if (s_state != ST_STARTED) {
        return ESP_ERR_INVALID_STATE;
    }
    s_adv_wanted = false;
    if (s_status.advertising) {
        xEventGroupClearBits(s_events, BIT_ADV_STOP);
        esp_ble_gap_stop_advertising();
        wait_bits(BIT_ADV_STOP);
    }
    if (s_status.connected) {
        esp_ble_gatts_close(s_gatts_if, s_status.conn_id);
    }
    for (uint8_t i = 0; i < s_service_count; i++) {
        if (s_services[i].first_handle) {
            esp_ble_gatts_stop_service(s_services[i].first_handle);
            esp_ble_gatts_delete_service(s_services[i].first_handle);
            s_services[i].first_handle = 0;
            s_services[i].last_handle = 0;
        }
    }
    if (s_gatts_if != ESP_GATT_IF_NONE) {
        esp_ble_gatts_app_unregister(s_gatts_if);
        s_gatts_if = ESP_GATT_IF_NONE;
    }
    lock_set(false);
    s_status.started = false;
    s_status.advertising = false;
    s_status.connected = false;
    s_state = ST_INIT;
    ESP_LOGI(TAG, "stopped");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Advertising and connection                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t ble_adv_start(void)
{
    if (s_state != ST_STARTED || s_status.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    s_adv_wanted = true;
    if (s_status.advertising) {
        return ESP_OK;
    }
    xEventGroupClearBits(s_events, BIT_ADV_START | BIT_FAIL);
    esp_err_t err = esp_ble_gap_start_advertising(&s_adv_params);
    if (err != ESP_OK) {
        return err;
    }
    return wait_bits(BIT_ADV_START);
}

esp_err_t ble_adv_stop(void)
{
    if (s_state != ST_STARTED) {
        return ESP_ERR_INVALID_STATE;
    }
    s_adv_wanted = false;
    if (!s_status.advertising) {
        return ESP_OK;
    }
    xEventGroupClearBits(s_events, BIT_ADV_STOP);
    esp_err_t err = esp_ble_gap_stop_advertising();
    if (err != ESP_OK) {
        return err;
    }
    return wait_bits(BIT_ADV_STOP);
}

esp_err_t ble_disconnect(void)
{
    if (s_state != ST_STARTED || !s_status.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_ble_gap_disconnect(s_peer);
}

esp_err_t ble_gatts_notify(uint16_t attr_handle, const void *data, size_t len, bool indicate)
{
    if (s_state != ST_STARTED || !s_status.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || len == 0 || attr_handle == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t mtu = s_status.mtu ? s_status.mtu : 23;
    if (len > (size_t)(mtu - 3)) {
        return ESP_ERR_INVALID_SIZE;
    }
    return esp_ble_gatts_send_indicate(s_gatts_if, s_status.conn_id, attr_handle,
                                       (uint16_t)len, (uint8_t *)data, indicate);
}

esp_err_t ble_gatts_set_value(uint16_t attr_handle, const void *data, size_t len)
{
    if (s_state != ST_STARTED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (attr_handle == 0 || (data == NULL && len > 0) || len > UINT16_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_ble_gatts_set_attr_value(attr_handle, (uint16_t)len, (const uint8_t *)data);
}

bool ble_is_connected(void)
{
    return s_state == ST_STARTED && s_status.connected;
}

esp_err_t ble_status(ble_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_status;
    return ESP_OK;
}

#endif /* Bluetooth enabled */
