#include "ble.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "ble";

/* Pure helper: needs no stack, so it exists in every build (FW-BLE-028). AD
 * structures are [len][type][data...], len covering type + data. */
bool ble_adv_has_uuid128(const uint8_t *adv, size_t len, const uint8_t uuid128[16])
{
    if (adv == NULL || uuid128 == NULL) {
        return false;
    }
    size_t i = 0;
    while (i + 1 < len) {
        uint8_t field_len = adv[i];
        if (field_len == 0 || i + 1 + field_len > len) {
            return false;   /* Truncated or padding: stop, never read past the end. */
        }
        uint8_t type = adv[i + 1];
        if (type == 0x06 || type == 0x07) {   /* Incomplete / complete list of 128-bit UUIDs */
            for (size_t off = 2; off + 16 <= (size_t)field_len + 1; off += 16) {
                if (memcmp(&adv[i + off], uuid128, 16) == 0) {
                    return true;
                }
            }
        }
        i += 1u + field_len;
    }
    return false;
}

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
esp_err_t ble_conn_params_request(const ble_conn_params_t *p)    { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_bond_remove(const uint8_t peer[6])                 { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_scan_start(uint32_t duration_ms)                   { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_scan_stop(void)                                    { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_connect(const uint8_t peer[6], uint8_t t, const ble_conn_params_t *p) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gattc_discover(const uint8_t svc_uuid128[16])      { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gattc_find_char(const uint8_t u[16], uint16_t *h)  { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gattc_subscribe(uint16_t char_handle)              { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gattc_write(uint16_t h, const void *d, size_t l, bool r) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_status(ble_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    return ESP_OK;
}

#else /* Bluetooth enabled */

#if !CONFIG_BT_BLE_50_FEATURES_SUPPORTED || !CONFIG_BT_BLE_50_EXTEND_ADV_EN || CONFIG_BT_BLE_42_FEATURES_SUPPORTED
#error "subsys/ble needs Bluedroid's BLE 5.0 API set with extended advertising, and not the 4.2 set: CONFIG_BT_BLE_50_FEATURES_SUPPORTED=y, CONFIG_BT_BLE_50_EXTEND_ADV_EN=y, CONFIG_BT_BLE_42_FEATURES_SUPPORTED=n (ADR-023)"
#endif

/* 0/1, not an expression over CONFIG_* symbols: those are undefined when off, and
 * BLE_CENTRAL is also used in C expressions. */
#if CONFIG_BT_GATTC_ENABLE && CONFIG_BT_BLE_50_EXTEND_SCAN_EN
#define BLE_CENTRAL 1
#else
#define BLE_CENTRAL 0
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#if BLE_CENTRAL
#include "esp_gattc_api.h"
#endif
#include "cfg.h"
#include "pm_policy.h"

#define BLE_APP_ID          0x55
#define BLE_GATTC_APP_ID    0x56
#define BLE_NAME_MAX        29     /* Fits a 31-byte legacy PDU with flags. */
#define BLE_MTU_LOCAL       CONFIG_BLE_LOCAL_MTU
#define BLE_ADV_INSTANCE    0
#define LEGACY_PDU_MAX      31

/* Event bits used to make the asynchronous start sequence synchronous. */
#define BIT_GATTS_REG       BIT0
#define BIT_ATTR_TAB        BIT1
#define BIT_ADV_DATA        BIT2
#define BIT_ADV_START       BIT3
#define BIT_ADV_STOP        BIT4
#define BIT_FAIL            BIT5
#define BIT_ADV_PARAMS      BIT6
#define BIT_SCAN_RSP        BIT7
#define BIT_GATTC_REG       BIT8
#define BIT_SCAN_PARAMS     BIT9
#define BIT_SCAN_START      BIT10
#define BIT_SCAN_STOP       BIT11

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
static uint32_t                s_adv_min_units;
static uint32_t                s_adv_max_units;

#if BLE_CENTRAL
static esp_gatt_if_t           s_gattc_if = ESP_GATT_IF_NONE;
static bool                    s_connecting;
static uint16_t                s_svc_start;
static uint16_t                s_svc_end;
static bool                    s_svc_found;
static uint16_t                s_pending_sub;  /* Char handle whose CCCD write is in flight. */
#endif

/* The sleep lock follows the radio: held while advertising, scanning or connected. */
static void lock_update(void)
{
    bool want = s_status.advertising || s_status.scanning || s_status.connected
#if BLE_CENTRAL
                || s_connecting
#endif
                ;
    if (want && !s_lock_held) {
        pm_policy_lock_acquire(s_lock);
        s_lock_held = true;
    } else if (!want && s_lock_held) {
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

static esp_err_t resolve_params(const ble_conn_params_t *p, uint16_t *min, uint16_t *max,
                                uint16_t *latency, uint16_t *timeout_10ms)
{
    ble_conn_params_t d = { 0 };
    if (p != NULL) {
        d = *p;
    }
    *min = d.interval_min_1250us ? d.interval_min_1250us : CONFIG_BLE_CONN_INTERVAL_MIN;
    *max = d.interval_max_1250us ? d.interval_max_1250us : CONFIG_BLE_CONN_INTERVAL_MAX;
    uint32_t to_ms = d.timeout_ms ? d.timeout_ms : CONFIG_BLE_SUPERVISION_TIMEOUT_MS;
    *latency = d.latency;
    if (*min < 6 || *max > 3200 || *min > *max || *latency > 499 || to_ms < 100 || to_ms > 32000) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Core spec: timeout > (1 + latency) * interval_max * 2. */
    if (to_ms * 4u <= (1u + *latency) * (uint32_t)*max * 5u * 2u) {
        return ESP_ERR_INVALID_ARG;
    }
    *timeout_10ms = (uint16_t)(to_ms / 10u);
    return ESP_OK;
}

/* Requested once per new link, in either role (FW-BLE-024, FW-BLE-025). */
static void tune_link(const uint8_t peer[6])
{
#if CONFIG_BLE_PREFER_2M_PHY
    esp_err_t err = esp_ble_gap_set_preferred_phy((uint8_t *)peer, 0,
                                                  ESP_BLE_GAP_PHY_2M_PREF_MASK,
                                                  ESP_BLE_GAP_PHY_2M_PREF_MASK,
                                                  ESP_BLE_GAP_PHY_OPTIONS_NO_PREF);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "2M PHY request: %s", esp_err_to_name(err));
    }
#endif
#if CONFIG_BLE_DATA_LEN > 27
    esp_err_t e2 = esp_ble_gap_set_pkt_data_len((uint8_t *)peer, CONFIG_BLE_DATA_LEN);
    if (e2 != ESP_OK) {
        ESP_LOGW(TAG, "data length request: %s", esp_err_to_name(e2));
    }
#endif
}

/* -------------------------------------------------------------------------- */
/* GAP callback — BTC task                                                     */
/* -------------------------------------------------------------------------- */

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        xEventGroupSetBits(s_events, param->ext_adv_set_params.status == ESP_BT_STATUS_SUCCESS
                                     ? BIT_ADV_PARAMS : BIT_FAIL);
        break;

    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        xEventGroupSetBits(s_events, param->ext_adv_data_set.status == ESP_BT_STATUS_SUCCESS
                                     ? BIT_ADV_DATA : BIT_FAIL);
        break;

    case ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        xEventGroupSetBits(s_events, param->scan_rsp_set.status == ESP_BT_STATUS_SUCCESS
                                     ? BIT_SCAN_RSP : BIT_FAIL);
        break;

    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        if (param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS) {
            s_status.advertising = true;
            lock_update();
            ESP_LOGI(TAG, "advertising as '%s'", s_name);
            emit(BLE_EVT_ADV_STARTED, NULL);
            xEventGroupSetBits(s_events, BIT_ADV_START);
        } else {
            ESP_LOGE(TAG, "advertising start failed: %d", param->ext_adv_start.status);
            xEventGroupSetBits(s_events, BIT_FAIL);
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
        s_status.advertising = false;
        lock_update();
        emit(BLE_EVT_ADV_STOPPED, NULL);
        xEventGroupSetBits(s_events, BIT_ADV_STOP);
        break;

    case ESP_GAP_BLE_ADV_TERMINATED_EVT:
        /* Status 0: advertising ended because a central connected. */
        s_status.advertising = false;
        lock_update();
        break;

#if BLE_CENTRAL
    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
        xEventGroupSetBits(s_events, param->set_ext_scan_params.status == ESP_BT_STATUS_SUCCESS
                                     ? BIT_SCAN_PARAMS : BIT_FAIL);
        break;

    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        if (param->ext_scan_start.status == ESP_BT_STATUS_SUCCESS) {
            s_status.scanning = true;
            lock_update();
            xEventGroupSetBits(s_events, BIT_SCAN_START);
        } else {
            ESP_LOGE(TAG, "scan start failed: %d", param->ext_scan_start.status);
            xEventGroupSetBits(s_events, BIT_FAIL);
        }
        break;

    case ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT:
    case ESP_GAP_BLE_SCAN_TIMEOUT_EVT:
        if (s_status.scanning) {
            s_status.scanning = false;
            lock_update();
            emit(BLE_EVT_SCAN_STOPPED, NULL);
        }
        xEventGroupSetBits(s_events, BIT_SCAN_STOP);
        break;

    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
        const esp_ble_gap_ext_adv_report_t *r = &param->ext_adv_report.params;
        ble_event_info_t info = {
            .addr_type = r->addr_type,
            .rssi      = r->rssi,
            .adv_data  = r->adv_data,
            .adv_len   = r->adv_data_len,
        };
        memcpy(info.peer, r->addr, 6);
        emit(BLE_EVT_SCAN_RESULT, &info);
        break;
    }
#endif

    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT: {
        if (param->phy_update.status == ESP_BT_STATUS_SUCCESS) {
            s_status.tx_phy = param->phy_update.tx_phy;
            s_status.rx_phy = param->phy_update.rx_phy;
        }
        ble_event_info_t info = {
            .tx_phy = param->phy_update.tx_phy,
            .rx_phy = param->phy_update.rx_phy,
            .status = (uint8_t)param->phy_update.status,
        };
        ESP_LOGI(TAG, "phy: tx %u rx %u (status %d)", info.tx_phy, info.rx_phy, param->phy_update.status);
        emit(BLE_EVT_PHY_UPDATED, &info);
        break;
    }

    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT: {
        ble_event_info_t info = {
            .data_len = param->pkt_data_length_cmpl.params.tx_len,
            .status   = (uint8_t)param->pkt_data_length_cmpl.status,
        };
        if (param->pkt_data_length_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            s_status.data_len = info.data_len;
        }
        ESP_LOGI(TAG, "data length: tx %u rx %u", param->pkt_data_length_cmpl.params.tx_len,
                 param->pkt_data_length_cmpl.params.rx_len);
        emit(BLE_EVT_DATA_LEN_CHANGED, &info);
        break;
    }

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

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: {
        s_status.conn_interval_ms = (uint16_t)((param->update_conn_params.conn_int * 5) / 4);
        ble_event_info_t info = {
            .conn_interval_ms = s_status.conn_interval_ms,
            .latency    = param->update_conn_params.latency,
            .timeout_ms = (uint16_t)(param->update_conn_params.timeout * 10),
            .status     = (uint8_t)param->update_conn_params.status,
        };
        ESP_LOGI(TAG, "conn params: interval %u ms latency %u timeout %u ms",
                 info.conn_interval_ms, info.latency, info.timeout_ms);
        emit(BLE_EVT_CONN_PARAMS_UPDATED, &info);
        break;
    }

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* GATTS callback — BTC task. Connection state lives here for both roles:      */
/* every registered GATT application hears every physical link.                */
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
        s_status.discovered = false;
        s_status.tx_phy = 0;
        s_status.rx_phy = 0;
        s_status.data_len = 0;
        s_status.conn_id = param->connect.conn_id;
        s_status.connections++;
        s_status.role = param->connect.link_role == 0 ? BLE_ROLE_CENTRAL : BLE_ROLE_PERIPHERAL;
        s_status.conn_interval_ms = (uint16_t)((param->connect.conn_params.interval * 5) / 4);
        memcpy(s_peer, param->connect.remote_bda, sizeof(s_peer));
        memcpy(s_status.peer, param->connect.remote_bda, sizeof(s_status.peer));
        s_status.advertising = false; /* Advertising stops on connection. */
#if BLE_CENTRAL
        s_connecting = false;
#endif
        lock_update();
        ESP_LOGI(TAG, "connected as %s: conn_id %u peer %02x:%02x:%02x:%02x:%02x:%02x interval %u ms",
                 s_status.role == BLE_ROLE_CENTRAL ? "central" : "peripheral",
                 s_status.conn_id, s_peer[0], s_peer[1], s_peer[2], s_peer[3], s_peer[4], s_peer[5],
                 s_status.conn_interval_ms);

        tune_link(s_peer);
        if (s_cfg.encrypt_on_connect && !s_cfg.no_bonding) {
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
        }
        ble_event_info_t info = {
            .conn_id = s_status.conn_id,
            .conn_interval_ms = s_status.conn_interval_ms,
            .latency = param->connect.conn_params.latency,
            .timeout_ms = (uint16_t)(param->connect.conn_params.timeout * 10),
            .role = s_status.role,
            .addr_type = param->connect.ble_addr_type,
        };
        memcpy(info.peer, s_peer, 6);
        emit(BLE_EVT_CONNECTED, &info);
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT: {
        ble_role_t was = s_status.role;
        s_status.connected = false;
        s_status.encrypted = false;
        s_status.discovered = false;
        s_status.role = BLE_ROLE_NONE;
        s_status.disconnections++;
        s_status.mtu = 0;
        ble_event_info_t info = { .conn_id = param->disconnect.conn_id, .reason = param->disconnect.reason,
                                  .role = was };
        memcpy(info.peer, param->disconnect.remote_bda, 6);
        ESP_LOGI(TAG, "disconnected: reason 0x%02x", param->disconnect.reason);
        emit(BLE_EVT_DISCONNECTED, &info);

        /* DES-BLE-005: a supervision timeout must not leave a peripheral unreachable. */
        if (was == BLE_ROLE_PERIPHERAL && s_state == ST_STARTED && s_adv_wanted && !s_cfg.no_auto_readvertise) {
            const esp_ble_gap_ext_adv_t adv = { .instance = BLE_ADV_INSTANCE, .duration = 0, .max_events = 0 };
            esp_ble_gap_ext_adv_start(1, &adv);
        }
        lock_update();
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
/* GATTC callback — BTC task                                                   */
/* -------------------------------------------------------------------------- */

#if BLE_CENTRAL
static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            s_gattc_if = gattc_if;
            xEventGroupSetBits(s_events, BIT_GATTC_REG);
        } else {
            ESP_LOGE(TAG, "gattc app register failed: %d", param->reg.status);
            xEventGroupSetBits(s_events, BIT_FAIL);
        }
        break;

    case ESP_GATTC_CONNECT_EVT:
        /* The MTU exchange is the client's to start. */
        esp_ble_gattc_send_mtu_req(gattc_if, param->connect.conn_id);
        break;

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            s_connecting = false;
            lock_update();
            ble_event_info_t info = { .status = (uint8_t)param->open.status };
            memcpy(info.peer, param->open.remote_bda, 6);
            ESP_LOGW(TAG, "connect failed: %d", param->open.status);
            emit(BLE_EVT_CONNECT_FAILED, &info);
        }
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status == ESP_GATT_OK) {
            s_status.mtu = param->cfg_mtu.mtu;
            ble_event_info_t info = { .conn_id = param->cfg_mtu.conn_id, .mtu = param->cfg_mtu.mtu };
            ESP_LOGI(TAG, "mtu %u", param->cfg_mtu.mtu);
            emit(BLE_EVT_MTU_CHANGED, &info);
        }
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        s_svc_start = param->search_res.start_handle;
        s_svc_end = param->search_res.end_handle;
        s_svc_found = true;
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        bool ok = param->search_cmpl.status == ESP_GATT_OK && s_svc_found;
        s_status.discovered = ok;
        ble_event_info_t info = { .conn_id = param->search_cmpl.conn_id,
                                  .status = ok ? 0 : (param->search_cmpl.status ? (uint8_t)param->search_cmpl.status : 0x0A) };
        ESP_LOGI(TAG, "discovery %s: handles 0x%04x..0x%04x", ok ? "done" : "failed", s_svc_start, s_svc_end);
        emit(BLE_EVT_DISCOVERY_DONE, &info);
        break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT: {
        ble_event_info_t info = { .conn_id = param->write.conn_id, .handle = s_pending_sub,
                                  .status = (uint8_t)param->write.status };
        s_pending_sub = 0;
        emit(BLE_EVT_SUBSCRIBED, &info);
        break;
    }

    case ESP_GATTC_NOTIFY_EVT:
        if (s_cfg.on_notify != NULL) {
            s_cfg.on_notify(param->notify.handle, param->notify.value, param->notify.value_len, s_cfg.ctx);
        }
        break;

    default:
        break;
    }
}
#endif

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
    s_adv_min_units = ((uint32_t)min_ms * 8u) / 5u; /* 0.625 ms units */
    s_adv_max_units = ((uint32_t)max_ms * 8u) / 5u;

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
#if BLE_CENTRAL
    if (err == ESP_OK) {
        err = esp_ble_gattc_register_callback(gattc_cb);
    }
#endif
    if (err == ESP_OK) {
        err = configure_security();
    }
    if (err == ESP_OK) {
        err = esp_ble_gatt_set_local_mtu(BLE_MTU_LOCAL);
    }
#if CONFIG_BLE_PREFER_2M_PHY
    if (err == ESP_OK) {
        /* Default for every future link, so a peer that updates the PHY itself gets 2M too. */
        err = esp_ble_gap_set_preferred_default_phy(ESP_BLE_GAP_PHY_1M_PREF_MASK | ESP_BLE_GAP_PHY_2M_PREF_MASK,
                                                    ESP_BLE_GAP_PHY_1M_PREF_MASK | ESP_BLE_GAP_PHY_2M_PREF_MASK);
    }
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gap/gatt setup: %s", esp_err_to_name(err));
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
#if BLE_CENTRAL
    s_gattc_if = ESP_GATT_IF_NONE;
    s_connecting = false;
#endif
    s_state = ST_INIT;

    ESP_LOGI(TAG, "init: Bluedroid 5.0 API, LE only, name '%s', adv %u-%u ms, bonding %s, central %s",
             s_name, min_ms, max_ms, s_cfg.no_bonding ? "off" : "LE SC", BLE_CENTRAL ? "yes" : "no");
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
    s_status.advertising = false;
    s_status.scanning = false;
    s_status.connected = false;
    lock_update();
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

static size_t ad_append(uint8_t *buf, size_t at, uint8_t type, const void *data, size_t len)
{
    buf[at] = (uint8_t)(len + 1);
    buf[at + 1] = type;
    memcpy(&buf[at + 2], data, len);
    return at + 2 + len;
}

/* Legacy PDUs: 31 bytes each. Flags + optional UUID + as much name as fits in the
 * advertisement; the complete name in the scan response (FW-BLE-021). */
static esp_err_t configure_advertising(void)
{
    const esp_ble_gap_ext_adv_params_t params = {
        .type          = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND,
        .interval_min  = s_adv_min_units,
        .interval_max  = s_adv_max_units,
        .channel_map   = ADV_CHNL_ALL,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        .tx_power      = EXT_ADV_TX_PWR_NO_PREFERENCE,
        .primary_phy   = ESP_BLE_GAP_PRI_PHY_1M,
        .max_skip      = 0,
        .secondary_phy = ESP_BLE_GAP_PHY_1M,
        .sid           = 0,
        .scan_req_notif = false,
    };
    esp_err_t err = esp_ble_gap_ext_adv_set_params(BLE_ADV_INSTANCE, &params);
    if (err == ESP_OK) {
        err = wait_bits(BIT_ADV_PARAMS);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "advertising params: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t adv[LEGACY_PDU_MAX];
    const uint8_t flags = 0x06;   /* LE General Discoverable, BR/EDR not supported */
    size_t n = ad_append(adv, 0, 0x01, &flags, 1);
    if (s_cfg.adv_uuid128 != NULL) {
        n = ad_append(adv, n, 0x07, s_cfg.adv_uuid128, 16);
    }
    size_t name_len = strlen(s_name);
    size_t room = LEGACY_PDU_MAX - n;
    if (room > 2) {
        size_t fit = name_len <= room - 2 ? name_len : room - 2;
        n = ad_append(adv, n, fit == name_len ? 0x09 : 0x08, s_name, fit);
    }
    err = esp_ble_gap_config_ext_adv_data_raw(BLE_ADV_INSTANCE, (uint16_t)n, adv);
    if (err == ESP_OK) {
        err = wait_bits(BIT_ADV_DATA);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "advertising data: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t rsp[LEGACY_PDU_MAX];
    size_t m = ad_append(rsp, 0, 0x09, s_name, name_len);
    err = esp_ble_gap_config_ext_scan_rsp_data_raw(BLE_ADV_INSTANCE, (uint16_t)m, rsp);
    if (err == ESP_OK) {
        err = wait_bits(BIT_SCAN_RSP);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scan response: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ble_start(void)
{
    if (s_state != ST_INIT) {
        return ESP_ERR_INVALID_STATE;
    }
    xEventGroupClearBits(s_events, 0xFFFF);
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

#if BLE_CENTRAL
    err = esp_ble_gattc_app_register(BLE_GATTC_APP_ID);
    if (err == ESP_OK) {
        err = wait_bits(BIT_GATTC_REG);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gattc app register: %s", esp_err_to_name(err));
        return err;
    }
#endif

    err = esp_ble_gap_set_device_name(s_name);
    if (err != ESP_OK) {
        return err;
    }
    err = configure_advertising();
    if (err != ESP_OK) {
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
        const uint8_t inst = BLE_ADV_INSTANCE;
        esp_ble_gap_ext_adv_stop(1, &inst);
        wait_bits(BIT_ADV_STOP);
    }
#if BLE_CENTRAL
    if (s_status.scanning) {
        xEventGroupClearBits(s_events, BIT_SCAN_STOP);
        esp_ble_gap_stop_ext_scan();
        wait_bits(BIT_SCAN_STOP);
    }
#endif
    if (s_status.connected) {
        esp_ble_gap_disconnect(s_peer);
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
#if BLE_CENTRAL
    if (s_gattc_if != ESP_GATT_IF_NONE) {
        esp_ble_gattc_app_unregister(s_gattc_if);
        s_gattc_if = ESP_GATT_IF_NONE;
    }
    s_connecting = false;
#endif
    s_status.started = false;
    s_status.advertising = false;
    s_status.scanning = false;
    s_status.connected = false;
    s_status.discovered = false;
    s_status.role = BLE_ROLE_NONE;
    lock_update();
    s_state = ST_INIT;
    ESP_LOGI(TAG, "stopped");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Advertising and connection                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t ble_adv_start(void)
{
    if (s_state != ST_STARTED || s_status.connected || s_status.scanning) {
        return ESP_ERR_INVALID_STATE;
    }
    s_adv_wanted = true;
    if (s_status.advertising) {
        return ESP_OK;
    }
    xEventGroupClearBits(s_events, BIT_ADV_START | BIT_FAIL);
    const esp_ble_gap_ext_adv_t adv = { .instance = BLE_ADV_INSTANCE, .duration = 0, .max_events = 0 };
    esp_err_t err = esp_ble_gap_ext_adv_start(1, &adv);
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
    const uint8_t inst = BLE_ADV_INSTANCE;
    esp_err_t err = esp_ble_gap_ext_adv_stop(1, &inst);
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

/* No free ACL buffer means the controller is already holding a backlog: refuse
 * rather than queue without bound (FW-BLE-027). */
static bool have_credit(void)
{
    if (esp_ble_get_cur_sendable_packets_num(s_status.conn_id) == 0) {
        s_status.tx_refused_no_buf++;
        return false;
    }
    return true;
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
    if (!indicate && !have_credit()) {
        return ESP_ERR_NO_MEM;
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

esp_err_t ble_conn_params_request(const ble_conn_params_t *p)
{
    if (s_state != ST_STARTED || !s_status.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_ble_conn_update_params_t u = { 0 };
    esp_err_t err = resolve_params(p, &u.min_int, &u.max_int, &u.latency, &u.timeout);
    if (err != ESP_OK) {
        return err;
    }
    memcpy(u.bda, s_peer, sizeof(u.bda));
    return esp_ble_gap_update_conn_params(&u);
}

esp_err_t ble_bond_remove(const uint8_t peer[6])
{
    if (peer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_bd_addr_t a;
    memcpy(a, peer, sizeof(a));
    return esp_ble_remove_bond_device(a);
}

/* -------------------------------------------------------------------------- */
/* Central role                                                                */
/* -------------------------------------------------------------------------- */

#if BLE_CENTRAL

esp_err_t ble_scan_start(uint32_t duration_ms)
{
    if (s_state != ST_STARTED || s_status.connected || s_status.advertising || s_connecting) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_status.scanning) {
        return ESP_OK;
    }
    const esp_ble_ext_scan_params_t params = {
        .own_addr_type  = BLE_ADDR_TYPE_PUBLIC,
        .filter_policy  = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
        .cfg_mask       = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK,
        .uncoded_cfg    = { BLE_SCAN_TYPE_ACTIVE,
                            (uint16_t)((CONFIG_BLE_SCAN_INTERVAL_MS * 8) / 5),
                            (uint16_t)((CONFIG_BLE_SCAN_WINDOW_MS * 8) / 5) },
    };
    xEventGroupClearBits(s_events, BIT_SCAN_PARAMS | BIT_SCAN_START | BIT_FAIL);
    esp_err_t err = esp_ble_gap_set_ext_scan_params(&params);
    if (err == ESP_OK) {
        err = wait_bits(BIT_SCAN_PARAMS);
    }
    if (err != ESP_OK) {
        return err;
    }
    uint32_t units = duration_ms / 10u;   /* HCI: N × 10 ms, 0 = until stopped */
    if (units > 0xFFFF) {
        units = 0xFFFF;
    }
    err = esp_ble_gap_start_ext_scan(units, 0);
    if (err != ESP_OK) {
        return err;
    }
    return wait_bits(BIT_SCAN_START);
}

esp_err_t ble_scan_stop(void)
{
    if (s_state != ST_STARTED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_status.scanning) {
        return ESP_OK;
    }
    xEventGroupClearBits(s_events, BIT_SCAN_STOP);
    esp_err_t err = esp_ble_gap_stop_ext_scan();
    if (err != ESP_OK) {
        return err;
    }
    return wait_bits(BIT_SCAN_STOP);
}

esp_err_t ble_connect(const uint8_t peer[6], uint8_t addr_type, const ble_conn_params_t *p)
{
    if (peer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state != ST_STARTED || s_status.connected || s_status.advertising || s_connecting) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_ble_conn_params_t cp = {
        .scan_interval = (uint16_t)((CONFIG_BLE_SCAN_INTERVAL_MS * 8) / 5),
        .scan_window   = (uint16_t)((CONFIG_BLE_SCAN_WINDOW_MS * 8) / 5),
    };
    esp_err_t err = resolve_params(p, &cp.interval_min, &cp.interval_max, &cp.latency, &cp.supervision_timeout);
    if (err != ESP_OK) {
        return err;
    }
    err = ble_scan_stop();
    if (err != ESP_OK) {
        return err;
    }
    esp_ble_gatt_creat_conn_params_t c = {
        .remote_addr_type   = (esp_ble_addr_type_t)addr_type,
        .is_direct          = true,
        .is_aux             = true,     /* 5.0 create-connection; reaches legacy advertisers too */
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .phy_mask           = ESP_BLE_PHY_1M_PREF_MASK,
        .phy_1m_conn_params = &cp,
    };
    memcpy(c.remote_bda, peer, 6);
    s_connecting = true;
    s_status.discovered = false;
    s_svc_found = false;
    lock_update();
    err = esp_ble_gattc_enh_open(s_gattc_if, &c);
    if (err != ESP_OK) {
        s_connecting = false;
        lock_update();
    }
    return err;
}

esp_err_t ble_gattc_discover(const uint8_t svc_uuid128[16])
{
    if (svc_uuid128 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state != ST_STARTED || !s_status.connected || s_status.role != BLE_ROLE_CENTRAL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_bt_uuid_t u = { .len = ESP_UUID_LEN_128 };
    memcpy(u.uuid.uuid128, svc_uuid128, 16);
    s_svc_found = false;
    s_status.discovered = false;
    return esp_ble_gattc_search_service(s_gattc_if, s_status.conn_id, &u);
}

esp_err_t ble_gattc_find_char(const uint8_t char_uuid128[16], uint16_t *out_handle)
{
    if (char_uuid128 == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state != ST_STARTED || !s_status.connected || !s_status.discovered) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_bt_uuid_t u = { .len = ESP_UUID_LEN_128 };
    memcpy(u.uuid.uuid128, char_uuid128, 16);
    esp_gattc_char_elem_t elem;
    uint16_t count = 1;
    esp_gatt_status_t st = esp_ble_gattc_get_char_by_uuid(s_gattc_if, s_status.conn_id, s_svc_start,
                                                          s_svc_end, u, &elem, &count);
    if (st != ESP_GATT_OK || count == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    *out_handle = elem.char_handle;
    return ESP_OK;
}

esp_err_t ble_gattc_subscribe(uint16_t char_handle)
{
    if (char_handle == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state != ST_STARTED || !s_status.connected || !s_status.discovered) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_bt_uuid_t cccd = { .len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG };
    esp_gattc_descr_elem_t d;
    uint16_t count = 1;
    esp_gatt_status_t st = esp_ble_gattc_get_descr_by_char_handle(s_gattc_if, s_status.conn_id,
                                                                  char_handle, cccd, &d, &count);
    if (st != ESP_GATT_OK || count == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t err = esp_ble_gattc_register_for_notify(s_gattc_if, s_peer, char_handle);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t enable[2] = { 0x01, 0x00 };   /* Notifications on, indications off. */
    s_pending_sub = char_handle;
    return esp_ble_gattc_write_char_descr(s_gattc_if, s_status.conn_id, d.handle, sizeof(enable), enable,
                                          ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
}

esp_err_t ble_gattc_write(uint16_t handle, const void *data, size_t len, bool need_rsp)
{
    if (handle == 0 || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state != ST_STARTED || !s_status.connected || s_status.role != BLE_ROLE_CENTRAL) {
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t mtu = s_status.mtu ? s_status.mtu : 23;
    if (len > (size_t)(mtu - 3)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (!need_rsp && !have_credit()) {
        return ESP_ERR_NO_MEM;
    }
    return esp_ble_gattc_write_char(s_gattc_if, s_status.conn_id, handle, (uint16_t)len, (uint8_t *)data,
                                    need_rsp ? ESP_GATT_WRITE_TYPE_RSP : ESP_GATT_WRITE_TYPE_NO_RSP,
                                    ESP_GATT_AUTH_REQ_NONE);
}

#else /* !BLE_CENTRAL */

esp_err_t ble_scan_start(uint32_t duration_ms)                   { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_scan_stop(void)                                    { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_connect(const uint8_t peer[6], uint8_t t, const ble_conn_params_t *p) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gattc_discover(const uint8_t svc_uuid128[16])      { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gattc_find_char(const uint8_t u[16], uint16_t *h)  { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gattc_subscribe(uint16_t char_handle)              { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ble_gattc_write(uint16_t h, const void *d, size_t l, bool r) { return ESP_ERR_NOT_SUPPORTED; }

#endif /* BLE_CENTRAL */

#endif /* Bluetooth enabled */
