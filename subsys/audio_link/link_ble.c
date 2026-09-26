/* BLE transport for audio_link (DES-LNK-006, ADR-022).
 *
 * One GATT service, two characteristics:
 *
 *     Audio Link Service   6a3f0001-5e7c-4b2d-9d41-8c2f3b7a0e51
 *       downlink  ...0002  Write Without Response   central → peripheral frames
 *       uplink    ...0003  Notify (+ CCCD)          peripheral → central frames
 *
 * Every Bluedroid callback here runs on the BTC task and does nothing but enqueue:
 * frames and events are handled on the link task, which is the only place that may
 * call back into subsys/ble (its callbacks must not re-enter it). */
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "ble.h"
#include "link_priv.h"

static const char *TAG = "audio_link";

#if !(CONFIG_BT_ENABLED && CONFIG_BT_BLUEDROID_ENABLED)

esp_err_t link_ble_start(const audio_link_config_t *cfg)
{
    ESP_LOGE(TAG, "BLE transport needs CONFIG_BT_ENABLED and Bluedroid");
    return ESP_ERR_NOT_SUPPORTED;
}
void      link_ble_stop(void) {}
esp_err_t link_ble_send(const uint8_t *frame, size_t len) { return ESP_ERR_INVALID_STATE; }
void      link_ble_handle_event(const link_item_t *it) {}
void      link_ble_poll(void) {}

#else

/* Little-endian, as Bluedroid stores 128-bit UUIDs. */
static const uint8_t SVC_UUID[16] = { 0x51, 0x0e, 0x7a, 0x3b, 0x2f, 0x8c, 0x41, 0x9d,
                                      0x2d, 0x4b, 0x7c, 0x5e, 0x01, 0x00, 0x3f, 0x6a };
static const uint8_t DL_UUID[16]  = { 0x51, 0x0e, 0x7a, 0x3b, 0x2f, 0x8c, 0x41, 0x9d,
                                      0x2d, 0x4b, 0x7c, 0x5e, 0x02, 0x00, 0x3f, 0x6a };
static const uint8_t UL_UUID[16]  = { 0x51, 0x0e, 0x7a, 0x3b, 0x2f, 0x8c, 0x41, 0x9d,
                                      0x2d, 0x4b, 0x7c, 0x5e, 0x03, 0x00, 0x3f, 0x6a };

#if CONFIG_AUDIO_LINK_REQUIRE_ENCRYPTION
#define PERM_R  ESP_GATT_PERM_READ_ENCRYPTED
#define PERM_W  ESP_GATT_PERM_WRITE_ENCRYPTED
#else
#define PERM_R  ESP_GATT_PERM_READ
#define PERM_W  ESP_GATT_PERM_WRITE
#endif

enum { IDX_SVC, IDX_DL_DECL, IDX_DL_VAL, IDX_UL_DECL, IDX_UL_VAL, IDX_UL_CCCD, IDX_COUNT };

static const uint16_t UUID_PRI   = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t UUID_DECL  = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t UUID_CCCD  = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t  DL_PROPS   = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t  UL_PROPS   = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t        s_dl_val[AUDIO_LINK_FRAME_MAX_LEN];
static uint8_t        s_ul_val[AUDIO_LINK_FRAME_MAX_LEN];
static uint8_t        s_cccd[2];

static const esp_gatts_attr_db_t s_db[IDX_COUNT] = {
    [IDX_SVC]     = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&UUID_PRI, ESP_GATT_PERM_READ,
                      sizeof(SVC_UUID), sizeof(SVC_UUID), (uint8_t *)SVC_UUID}},
    [IDX_DL_DECL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&UUID_DECL, ESP_GATT_PERM_READ,
                      1, 1, (uint8_t *)&DL_PROPS}},
    [IDX_DL_VAL]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)DL_UUID, PERM_W,
                      sizeof(s_dl_val), 0, s_dl_val}},
    [IDX_UL_DECL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&UUID_DECL, ESP_GATT_PERM_READ,
                      1, 1, (uint8_t *)&UL_PROPS}},
    [IDX_UL_VAL]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)UL_UUID, PERM_R,
                      sizeof(s_ul_val), 0, s_ul_val}},
    [IDX_UL_CCCD] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&UUID_CCCD, PERM_R | PERM_W,
                      sizeof(s_cccd), sizeof(s_cccd), s_cccd}},
};
static uint16_t s_handles[IDX_COUNT];

static void on_write(uint16_t handle, const uint8_t *data, size_t len, void *ctx);

static ble_gatts_service_t s_svc = {
    .db = s_db, .count = IDX_COUNT, .handles = s_handles, .on_write = on_write,
};

static struct {
    bool              started;
    audio_link_role_t role;
    const uint8_t    *peer_filter;
    bool              connected;
    bool              connecting;
    bool              encrypted;
    bool              discovering;
    bool              subscribed;
    bool              mtu_warned;
    uint16_t          mtu;
    uint16_t          dl_handle;      /* Central: the peer's characteristic handles. */
    volatile uint16_t ul_handle;
    uint8_t           peer[6];
    int64_t           rescan_at_us;
} b;

/* -------------------------------------------------------------------------- */
/* BTC task: enqueue only                                                      */
/* -------------------------------------------------------------------------- */

static void on_write(uint16_t handle, const uint8_t *data, size_t len, void *ctx)
{
    if (handle == s_handles[IDX_DL_VAL]) {
        link_core_post_frame(data, len);
    } else if (handle == s_handles[IDX_UL_CCCD] && len >= 1) {
        link_core_post_event((data[0] & 0x01u) ? LINK_EVT_PEER_SUBSCRIBED : LINK_EVT_PEER_UNSUBSCRIBED,
                             0, NULL, 0, 0);
    }
}

static void on_notify(uint16_t handle, const uint8_t *data, size_t len, void *ctx)
{
    if (handle != 0 && handle == b.ul_handle) {
        link_core_post_frame(data, len);
    }
}

static void on_ble_event(ble_event_t evt, const ble_event_info_t *info, void *ctx)
{
    switch (evt) {
    case BLE_EVT_SCAN_RESULT:
        /* Filter here, cheaply, so a busy 2.4 GHz band does not flood the queue. */
        if (!ble_adv_has_uuid128(info->adv_data, info->adv_len, SVC_UUID)) {
            return;
        }
        if (b.peer_filter != NULL && memcmp(b.peer_filter, info->peer, 6) != 0) {
            return;
        }
        link_core_post_event((uint8_t)evt, 0, info->peer, info->addr_type, 0);
        break;
    case BLE_EVT_MTU_CHANGED:
        link_core_post_event((uint8_t)evt, 0, info->peer, 0, info->mtu);
        break;
    case BLE_EVT_DISCONNECTED:
        link_core_post_event((uint8_t)evt, info->reason, info->peer, 0, 0);
        break;
    case BLE_EVT_CONNECTED:
    case BLE_EVT_PAIRED:
    case BLE_EVT_PAIR_FAILED:
    case BLE_EVT_CONNECT_FAILED:
    case BLE_EVT_DISCOVERY_DONE:
    case BLE_EVT_SUBSCRIBED:
    case BLE_EVT_SCAN_STOPPED:
        link_core_post_event((uint8_t)evt, evt == BLE_EVT_PAIR_FAILED ? info->reason : info->status,
                             info->peer, info->addr_type, info->handle);
        break;
    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Link task                                                                   */
/* -------------------------------------------------------------------------- */

static void schedule_rescan(void)
{
    if (b.role == AUDIO_LINK_ROLE_CENTRAL) {
        b.rescan_at_us = esp_timer_get_time() + (int64_t)CONFIG_AUDIO_LINK_RECONNECT_DELAY_MS * 1000;
    }
}

static void maybe_up(void)
{
    if (!b.connected || !b.subscribed) {
        return;
    }
#if CONFIG_AUDIO_LINK_REQUIRE_ENCRYPTION
    if (!b.encrypted) {
        return;
    }
#endif
    size_t need = link_core_max_frame_len() + 3u;
    if (b.mtu < need) {
        if (!b.mtu_warned) {
            b.mtu_warned = true;
            ESP_LOGW(TAG, "waiting for an MTU of %u (have %u)", (unsigned)need, b.mtu);
        }
        return;
    }
    link_core_set_up(true);
}

static void start_discovery(void)
{
    if (b.role != AUDIO_LINK_ROLE_CENTRAL || b.discovering || !b.connected) {
        return;
    }
    b.discovering = true;
    esp_err_t err = ble_gattc_discover(SVC_UUID);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "discover: %s", esp_err_to_name(err));
        ble_disconnect();
    }
}

static void link_lost(void)
{
    b.connected = false;
    b.connecting = false;
    b.encrypted = false;
    b.discovering = false;
    b.subscribed = false;
    b.mtu_warned = false;
    b.mtu = 23;
    b.ul_handle = 0;
    b.dl_handle = 0;
    link_core_set_up(false);
    schedule_rescan();
}

void link_ble_handle_event(const link_item_t *it)
{
    if (!b.started) {
        return;
    }
    switch (it->evt) {
    case BLE_EVT_SCAN_RESULT: {
        if (b.role != AUDIO_LINK_ROLE_CENTRAL || b.connected || b.connecting) {
            break;
        }
        const ble_conn_params_t p = {
            .interval_min_1250us = CONFIG_AUDIO_LINK_CONN_INTERVAL,
            .interval_max_1250us = CONFIG_AUDIO_LINK_CONN_INTERVAL,
            .latency = 0,
            .timeout_ms = CONFIG_AUDIO_LINK_SUPERVISION_TIMEOUT_MS,
        };
        ESP_LOGI(TAG, "found %02x:%02x:%02x:%02x:%02x:%02x, connecting",
                 it->peer[0], it->peer[1], it->peer[2], it->peer[3], it->peer[4], it->peer[5]);
        esp_err_t err = ble_connect(it->peer, it->addr_type, &p);
        if (err == ESP_OK) {
            b.connecting = true;
        } else {
            ESP_LOGW(TAG, "connect: %s", esp_err_to_name(err));
            schedule_rescan();
        }
        break;
    }
    case BLE_EVT_CONNECT_FAILED:
        b.connecting = false;
        schedule_rescan();
        break;
    case BLE_EVT_CONNECTED:
        b.connecting = false;
        b.connected = true;
        b.encrypted = false;
        b.subscribed = false;
        b.discovering = false;
        b.mtu = 23;
        memcpy(b.peer, it->peer, 6);
#if !CONFIG_AUDIO_LINK_REQUIRE_ENCRYPTION
        start_discovery();
#endif
        break;
    case BLE_EVT_MTU_CHANGED:
        b.mtu = it->handle;
        maybe_up();
        break;
    case BLE_EVT_PAIRED:
        b.encrypted = true;
        start_discovery();
        maybe_up();
        break;
    case BLE_EVT_PAIR_FAILED:
        /* Usually a peer that lost its keys: forget ours so the next attempt pairs
         * afresh, rather than failing on the stale bond forever. */
        ESP_LOGW(TAG, "pairing failed (0x%02x): forgetting the bond and reconnecting", it->status);
        ble_bond_remove(b.peer);
        ble_disconnect();
        break;
    case BLE_EVT_DISCOVERY_DONE: {
        uint16_t dl = 0, ul = 0;
        if (it->status != 0 || ble_gattc_find_char(DL_UUID, &dl) != ESP_OK ||
            ble_gattc_find_char(UL_UUID, &ul) != ESP_OK) {
            ESP_LOGE(TAG, "peer does not offer the audio link service");
            ble_disconnect();
            break;
        }
        b.dl_handle = dl;
        b.ul_handle = ul;
        esp_err_t err = ble_gattc_subscribe(ul);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "subscribe: %s", esp_err_to_name(err));
            ble_disconnect();
        }
        break;
    }
    case BLE_EVT_SUBSCRIBED:
        if (it->status == 0) {
            b.subscribed = true;
            maybe_up();
        } else {
            ESP_LOGE(TAG, "uplink CCCD write failed: 0x%02x", it->status);
            ble_disconnect();
        }
        break;
    case LINK_EVT_PEER_SUBSCRIBED:
        b.subscribed = true;
        maybe_up();
        break;
    case LINK_EVT_PEER_UNSUBSCRIBED:
        b.subscribed = false;
        link_core_set_up(false);
        break;
    case BLE_EVT_DISCONNECTED:
        link_lost();
        break;
    case BLE_EVT_SCAN_STOPPED:
        if (!b.connected && !b.connecting) {
            schedule_rescan();
        }
        break;
    default:
        break;
    }
}

void link_ble_poll(void)
{
    if (!b.started || b.role != AUDIO_LINK_ROLE_CENTRAL || b.rescan_at_us == 0) {
        return;
    }
    if (b.connected || b.connecting || esp_timer_get_time() < b.rescan_at_us) {
        return;
    }
    b.rescan_at_us = 0;
    esp_err_t err = ble_scan_start(0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "rescan: %s", esp_err_to_name(err));
        schedule_rescan();
    }
}

esp_err_t link_ble_start(const audio_link_config_t *cfg)
{
    memset(&b, 0, sizeof(b));
    b.role = cfg->role;
    b.peer_filter = cfg->peer_addr;
    b.mtu = 23;
    bool central = cfg->role == AUDIO_LINK_ROLE_CENTRAL;

    const ble_config_t bc = {
        .device_name = cfg->device_name,
        .cb = on_ble_event,
        .on_notify = on_notify,
        .adv_uuid128 = central ? NULL : SVC_UUID,
        /* The central initiates security; the peripheral's CCCD and downlink
         * permissions refuse an unencrypted client (FW-LNK-027). */
        .encrypt_on_connect = central && CONFIG_AUDIO_LINK_REQUIRE_ENCRYPTION,
    };
    esp_err_t err = ble_init(&bc);
    if (err != ESP_OK) {
        return err;
    }
    if (!central) {
        memset(s_handles, 0, sizeof(s_handles));
        err = ble_gatts_add_service(&s_svc);
        if (err != ESP_OK) {
            goto fail_deinit;
        }
    }
    err = ble_start();
    if (err != ESP_OK) {
        goto fail_deinit;
    }
    b.started = true;
    err = central ? ble_scan_start(0) : ble_adv_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: %s", central ? "scan" : "advertise", esp_err_to_name(err));
        b.started = false;
        ble_stop();
        goto fail_deinit;
    }
    ESP_LOGI(TAG, "BLE %s", central ? "scanning for the audio link service" : "advertising the audio link service");
    return ESP_OK;

fail_deinit:
    ble_deinit();
    return err;
}

void link_ble_stop(void)
{
    if (!b.started) {
        return;
    }
    b.started = false;
    ble_stop();
    ble_deinit();
}

esp_err_t link_ble_send(const uint8_t *frame, size_t len)
{
    if (!b.started || !b.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    if (b.role == AUDIO_LINK_ROLE_PERIPHERAL) {
        return ble_gatts_notify(s_handles[IDX_UL_VAL], frame, len, false);
    }
    return ble_gattc_write(b.dl_handle, frame, len, false);
}

#endif
