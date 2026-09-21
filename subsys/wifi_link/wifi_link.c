#include "wifi_link.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "cfg.h"
#include "pm_policy.h"

static const char *TAG = "wifi_link";

#define BIT_GOT_IP   BIT0
#define BIT_GAVE_UP  BIT1

#define NS_WIFI   "wifi_link"
#define KEY_SSID  "ssid"
#define KEY_PASS  "pass"

typedef enum { ST_UNINIT = 0, ST_IDLE, ST_CONNECTING, ST_CONNECTED, ST_GAVE_UP } state_t;

static state_t                 s_state;
static esp_netif_t            *s_netif;
static esp_event_handler_instance_t s_h_wifi, s_h_ip;
static EventGroupHandle_t      s_events;
static StaticEventGroup_t      s_events_storage;
static pm_policy_lock_handle_t s_lock;
static bool                    s_lock_held;

static wifi_link_session_t       s_cfg;          /* cb/ctx/policy; strings are copied below. */
static char                    s_ssid[WIFI_LINK_SSID_MAX + 1];
static char                    s_pass[WIFI_LINK_PASS_MAX + 1];
static uint8_t                 s_attempt;
static bool                    s_had_ip;
static wifi_link_status_t       s_status;

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

static void emit(wifi_link_event_t evt, const wifi_link_event_info_t *info)
{
    if (s_cfg.cb != NULL) {
        s_cfg.cb(evt, info, s_cfg.ctx);
    }
}

/* -------------------------------------------------------------------------- */
/* Event handlers — default event loop task                                    */
/* -------------------------------------------------------------------------- */

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case WIFI_EVENT_STA_START:
        if (s_state == ST_CONNECTING) {
            s_attempt++;
            s_status.connect_attempts++;
            esp_wifi_connect();
        }
        break;

    case WIFI_EVENT_STA_CONNECTED: {
        wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)data;
        s_status.connected = true;
        s_status.channel = e->channel;
        memcpy(s_status.bssid, e->bssid, sizeof(s_status.bssid));
        wifi_ap_record_t ap;
        s_status.rssi = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
        ESP_LOGI(TAG, "associated to '%s' ch %u rssi %d (attempt %u)", s_ssid, e->channel, s_status.rssi, s_attempt);
        wifi_link_event_info_t info = { .attempt = s_attempt, .rssi = s_status.rssi };
        emit(WIFI_LINK_EVT_CONNECTED, &info);
        break;
    }

    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        bool was_connected = s_status.connected;
        s_status.connected = false;
        s_status.has_ip = false;
        s_status.rssi = 0;
        s_status.last_reason = e->reason;
        s_status.disconnects++;

        wifi_link_event_info_t info = { .reason = e->reason, .attempt = s_attempt };
        ESP_LOGW(TAG, "disconnected from '%s' reason %u (attempt %u)", s_ssid, e->reason, s_attempt);
        emit(WIFI_LINK_EVT_DISCONNECTED, &info);

        if (s_state != ST_CONNECTING && s_state != ST_CONNECTED) {
            break; /* Session ended by the application; do not retry. */
        }

        /* Retry policy: bounded before first IP; after an IP, only if asked. */
        bool retry;
        if (s_had_ip) {
            retry = s_cfg.reconnect_on_loss;
        } else {
            retry = (s_cfg.max_attempts == 0) || (s_attempt < s_cfg.max_attempts);
        }
        if (retry) {
            s_state = ST_CONNECTING;
            s_attempt++;
            s_status.connect_attempts++;
            esp_wifi_connect();
        } else {
            s_state = ST_GAVE_UP;
            lock_set(false);
            ESP_LOGE(TAG, "giving up on '%s' after %u attempt(s)%s", s_ssid, s_attempt,
                     was_connected ? " (link lost)" : "");
            emit(WIFI_LINK_EVT_GAVE_UP, &info);
            xEventGroupSetBits(s_events, BIT_GAVE_UP);
        }
        break;
    }

    default:
        break;
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id != IP_EVENT_STA_GOT_IP) {
        return;
    }
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    s_status.has_ip = true;
    s_status.ip = e->ip_info.ip;
    s_had_ip = true;
    s_state = ST_CONNECTED;
    ESP_LOGI(TAG, "got ip " IPSTR " gw " IPSTR, IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
    wifi_link_event_info_t info = {
        .ip = e->ip_info.ip, .gateway = e->ip_info.gw, .attempt = s_attempt, .rssi = s_status.rssi,
    };
    emit(WIFI_LINK_EVT_GOT_IP, &info);
    xEventGroupSetBits(s_events, BIT_GOT_IP);
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t wifi_link_init(void)
{
    if (s_state != ST_UNINIT) {
        return ESP_OK;
    }

    /* The Wi-Fi driver keeps calibration data in NVS. */
    esp_err_t err = cfg_init();
    if (err != ESP_OK) {
        return err;
    }
    err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err; /* INVALID_STATE: the application already created it. Fine. */
    }

    s_netif = esp_netif_create_default_wifi_sta();
    if (s_netif == NULL) {
        return ESP_FAIL;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(err));
        goto fail_netif;
    }
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM); /* Credentials come from us, not the driver's NVS. */
    if (err == ESP_OK) {
        err = esp_wifi_set_mode(WIFI_MODE_STA);
    }
    if (err == ESP_OK) {
        err = esp_wifi_set_ps(CONFIG_WIFI_LINK_POWER_SAVE_MIN_MODEM ? WIFI_PS_MIN_MODEM : WIFI_PS_NONE);
    }
    if (err != ESP_OK) {
        goto fail_wifi;
    }

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL, &s_h_wifi);
    if (err == ESP_OK) {
        err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_ip_event, NULL, &s_h_ip);
    }
    if (err != ESP_OK) {
        goto fail_handlers;
    }

    err = pm_policy_lock_create("wifi", &s_lock);
    if (err != ESP_OK) {
        goto fail_handlers;
    }

    s_events = xEventGroupCreateStatic(&s_events_storage);
    memset(&s_status, 0, sizeof(s_status));
    s_state = ST_IDLE;
    ESP_LOGI(TAG, "init");
    return ESP_OK;

fail_handlers:
    if (s_h_wifi) { esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_h_wifi); s_h_wifi = NULL; }
    if (s_h_ip)   { esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_h_ip); s_h_ip = NULL; }
fail_wifi:
    esp_wifi_deinit();
fail_netif:
    esp_netif_destroy_default_wifi(s_netif);
    s_netif = NULL;
    return err;
}

esp_err_t wifi_link_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    wifi_link_disconnect();
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_h_wifi);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_h_ip);
    s_h_wifi = NULL;
    s_h_ip = NULL;
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_netif_destroy_default_wifi(s_netif);
    s_netif = NULL;
    vEventGroupDelete(s_events);
    s_events = NULL;
    s_state = ST_UNINIT;
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

esp_err_t wifi_link_connect(const wifi_link_session_t *cfg)
{
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state == ST_CONNECTING || s_state == ST_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || cfg->ssid == NULL || cfg->ssid[0] == '\0' ||
        strlen(cfg->ssid) > WIFI_LINK_SSID_MAX ||
        (cfg->password != NULL && strlen(cfg->password) > WIFI_LINK_PASS_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *cfg;
    strlcpy(s_ssid, cfg->ssid, sizeof(s_ssid));
    strlcpy(s_pass, cfg->password ? cfg->password : "", sizeof(s_pass));
    s_cfg.ssid = s_ssid;
    s_cfg.password = s_pass;

    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid, s_ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, s_pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = (s_pass[0] == '\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    wc.sta.pmf_cfg.capable = true;
    wc.sta.pmf_cfg.required = false;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config: %s", esp_err_to_name(err));
        return err;
    }

    xEventGroupClearBits(s_events, BIT_GOT_IP | BIT_GAVE_UP);
    s_attempt = 0;
    s_had_ip = false;
    s_state = ST_CONNECTING;
    lock_set(true);

    /* First connect happens in STA_START; if already started, connect now. */
    err = esp_wifi_start();
    if (err == ESP_ERR_WIFI_NOT_STOPPED || err == ESP_ERR_INVALID_STATE) {
        s_attempt++;
        s_status.connect_attempts++;
        err = esp_wifi_connect();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start/connect: %s", esp_err_to_name(err));
        s_state = ST_IDLE;
        lock_set(false);
        return err;
    }
    ESP_LOGI(TAG, "connecting to '%s' (max attempts %u, reconnect %s)",
             s_ssid, s_cfg.max_attempts, s_cfg.reconnect_on_loss ? "on" : "off");
    return ESP_OK;
}

esp_err_t wifi_link_wait_ip(uint32_t timeout_ms)
{
    if (s_state == ST_UNINIT || s_state == ST_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state == ST_CONNECTED && s_status.has_ip) {
        return ESP_OK;
    }
    EventBits_t bits = xEventGroupWaitBits(s_events, BIT_GOT_IP | BIT_GAVE_UP, pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));
    if (bits & BIT_GOT_IP) {
        return ESP_OK;
    }
    if (bits & BIT_GAVE_UP) {
        return ESP_ERR_WIFI_CONN;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t wifi_link_disconnect(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state == ST_IDLE) {
        return ESP_OK;
    }
    s_state = ST_IDLE; /* Set first so the DISCONNECTED handler does not retry. */
    esp_wifi_disconnect();
    lock_set(false);
    s_status.connected = false;
    s_status.has_ip = false;
    ESP_LOGI(TAG, "session ended");
    return ESP_OK;
}

bool wifi_link_is_connected(void)
{
    return s_state == ST_CONNECTED && s_status.has_ip;
}

esp_err_t wifi_link_status(wifi_link_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_status;
    if (s_status.connected) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            out->rssi = ap.rssi;
        }
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Credential persistence                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t wifi_link_credentials_save(const char *ssid, const char *password)
{
    if (ssid == NULL || ssid[0] == '\0' || strlen(ssid) > WIFI_LINK_SSID_MAX ||
        (password != NULL && strlen(password) > WIFI_LINK_PASS_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }
    cfg_handle_t h = NULL;
    esp_err_t err = cfg_open(NS_WIFI, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = cfg_set_str(h, KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = cfg_set_str(h, KEY_PASS, password ? password : "");
    }
    cfg_close(h);
    return err;
}

esp_err_t wifi_link_credentials_load(char *ssid, size_t ssid_len, char *password, size_t pass_len)
{
    if (ssid == NULL || password == NULL || ssid_len < 2 || pass_len < 1) {
        return ESP_ERR_INVALID_ARG;
    }
    cfg_handle_t h = NULL;
    esp_err_t err = cfg_open(NS_WIFI, &h);
    if (err != ESP_OK) {
        return err;
    }
    if (!cfg_exists(h, KEY_SSID)) {
        cfg_close(h);
        ssid[0] = '\0';
        password[0] = '\0';
        return ESP_ERR_NVS_NOT_FOUND;
    }
    err = cfg_get_str(h, KEY_SSID, ssid, ssid_len, "");
    if (err == ESP_OK) {
        err = cfg_get_str(h, KEY_PASS, password, pass_len, "");
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK; /* SSID present, no password stored: open network. */
        }
    }
    cfg_close(h);
    return err;
}

esp_err_t wifi_link_credentials_clear(void)
{
    cfg_handle_t h = NULL;
    esp_err_t err = cfg_open(NS_WIFI, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = cfg_erase_all(h);
    cfg_close(h);
    return err;
}
