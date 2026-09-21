/**
 * @file wifi_link.h
 * @brief Wi-Fi station link manager: bring the interface up, connect with a bounded retry
 *        policy, report link events, and hold a sleep lock while the radio is in use.
 *
 * Mechanism only (ADR-013): credentials, when to connect, and what to do on loss
 * are the application's decisions. The optional NVS helpers store credentials in
 * the `wifi_sta` namespace via `subsys/cfg`; using them is the application's choice.
 * Added to the SDK so that `ota` can be exercised end-to-end (SAD §3.1, ADR-019).
 *
 * Named `wifi_link`, not `wifi_sta`: Espressif's closed-source Wi-Fi library exports
 * `wifi_sta_disconnect()` and other `wifi_sta_*` symbols, so that prefix collides at
 * link time. Do not rename it back.
 *
 * ## Lifecycle
 *
 *     wifi_link_init() ─► wifi_link_connect(cfg) ─► [events] ─► wifi_link_disconnect() ─► wifi_link_deinit()
 *
 * `connect()` is asynchronous; `wifi_link_wait_ip()` blocks for callers that want
 * synchronous semantics. Events arrive on the ESP-IDF default event loop task.
 *
 * ## Radio sharing
 *
 * Wi-Fi and BLE time-share the single 2.4 GHz radio (CON-05). Both stacks live on
 * core 0; the coexistence arbiter handles airtime. Expect throughput to fall when
 * both are active — that is physics, not a defect.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit` are not safe concurrently with anything. All other functions are
 * safe from any task. Nothing is ISR-safe. The event callback runs on the event
 * loop task and must not block.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"   /* ESP_ERR_WIFI_* codes are part of this API's contract */

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum SSID / passphrase lengths per 802.11 and WPA2. */
#define WIFI_LINK_SSID_MAX 32
#define WIFI_LINK_PASS_MAX 64

typedef enum {
    WIFI_LINK_EVT_CONNECTED = 0,   /**< Associated and authenticated; no IP yet.       */
    WIFI_LINK_EVT_GOT_IP,          /**< DHCP (or static) address assigned; usable now. */
    WIFI_LINK_EVT_DISCONNECTED,    /**< Link lost or connect attempt failed. See `reason`. */
    WIFI_LINK_EVT_GAVE_UP,         /**< Retry budget exhausted; no further attempts.   */
} wifi_link_event_t;

typedef struct {
    esp_ip4_addr_t ip;            /**< Valid for GOT_IP.                              */
    esp_ip4_addr_t gateway;       /**< Valid for GOT_IP.                              */
    uint8_t        reason;        /**< `wifi_err_reason_t` for DISCONNECTED/GAVE_UP.  */
    uint8_t        attempt;       /**< Connect attempts so far in this session.       */
    int8_t         rssi;          /**< Valid for CONNECTED/GOT_IP.                    */
} wifi_link_event_info_t;

typedef void (*wifi_link_event_cb_t)(wifi_link_event_t evt, const wifi_link_event_info_t *info, void *ctx);

typedef struct {
    const char *ssid;             /**< Required, 1..32 bytes. Copied.                  */
    const char *password;         /**< NULL or "" for an open network. Copied.         */
    uint8_t     max_attempts;     /**< Connect attempts before GAVE_UP. 0 = unlimited. */
    bool        reconnect_on_loss;/**< After GOT_IP, reconnect automatically on loss.  */
    wifi_link_event_cb_t cb;       /**< Optional.                                       */
    void       *ctx;
} wifi_link_session_t;

typedef struct {
    bool     connected;           /**< Associated.                                     */
    bool     has_ip;
    esp_ip4_addr_t ip;
    int8_t   rssi;                /**< Last known; 0 if not connected.                 */
    uint8_t  channel;
    uint32_t connect_attempts;    /**< Since init.                                     */
    uint32_t disconnects;         /**< Since init.                                     */
    uint8_t  last_reason;
    uint8_t  bssid[6];
} wifi_link_status_t;

/**
 * @brief Initialise netif, the default event loop, and the Wi-Fi driver in station
 *        mode. Does not connect. Requires NVS (`cfg_init()` is called if needed).
 *
 * Idempotent.
 *
 * @return ESP_OK, or a propagated `esp_netif`/`esp_wifi` error.
 * @note Blocking: yes (driver init, tens of ms).
 */
esp_err_t wifi_link_init(void);

/** @brief Disconnect if needed, stop and deinitialise the driver. ESP_OK if never initialised. */
esp_err_t wifi_link_deinit(void);

/**
 * @brief Start connecting. Returns immediately; outcome arrives via `cfg->cb`.
 *        Acquires the `"wifi"` sleep lock until `wifi_link_disconnect()` or GAVE_UP.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before init, or a session is already active
 *   - ESP_ERR_INVALID_ARG    NULL cfg/ssid, ssid > 32 bytes, password > 64 bytes
 *   - (propagated)           `esp_wifi_*` errors
 */
esp_err_t wifi_link_connect(const wifi_link_session_t *cfg);

/**
 * @brief Block until an IP is obtained, the retry budget is exhausted, or `timeout_ms`.
 *
 * @return ESP_OK on IP, ESP_ERR_TIMEOUT, ESP_ERR_WIFI_CONN if GAVE_UP fired, or
 *         ESP_ERR_INVALID_STATE if no session is active.
 */
esp_err_t wifi_link_wait_ip(uint32_t timeout_ms);

/** @brief End the session: disconnect, stop retrying, release the sleep lock. Idempotent. */
esp_err_t wifi_link_disconnect(void);

/** @brief true once GOT_IP has fired and the link has not since dropped. */
bool wifi_link_is_connected(void);

/** @brief Snapshot of link status. ESP_ERR_INVALID_ARG on NULL; works before init (zeros). */
esp_err_t wifi_link_status(wifi_link_status_t *out);

/* -------------------------------------------------------------------------- */
/* Optional credential persistence (subsys/cfg, namespace "wifi_link")           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Store credentials. Empty `password` is stored as an open network.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, or a propagated cfg error.
 */
esp_err_t wifi_link_credentials_save(const char *ssid, const char *password);

/**
 * @brief Load credentials into caller buffers of at least WIFI_LINK_SSID_MAX+1 and
 *        WIFI_LINK_PASS_MAX+1 bytes.
 * @return ESP_OK, ESP_ERR_NVS_NOT_FOUND if none stored, ESP_ERR_INVALID_ARG, or a
 *         propagated cfg error.
 */
esp_err_t wifi_link_credentials_load(char *ssid, size_t ssid_len, char *password, size_t pass_len);

/** @brief Forget stored credentials. ESP_OK also if none were stored. */
esp_err_t wifi_link_credentials_clear(void);

#ifdef __cplusplus
}
#endif
