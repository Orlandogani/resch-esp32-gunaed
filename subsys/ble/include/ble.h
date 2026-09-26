/**
 * @file ble.h
 * @brief Bluetooth Low Energy on Bluedroid: controller and host lifecycle,
 *        application-owned GATT services, legacy-PDU advertising, an optional central
 *        role with a GATT client, LE Secure Connections pairing with bonding, link
 *        tuning (2M PHY, data length, connection parameters), and a sleep lock while
 *        the radio is used.
 *
 * Realises FW-BLE-001..011 and FW-BLE-020..031 with Bluedroid per ADR-018, on the
 * BLE 5.0 feature API set per ADR-023. The ESP32-S3 is LE-only (CON-04); Classic
 * memory is released at init.
 *
 * ## Lifecycle
 *
 *     ble_init(cfg) ─► ble_gatts_add_service(svc)… ─► ble_start() ─┬─► ble_adv_start()      (peripheral)
 *                                                                  └─► ble_scan_start() ─►
 *                                                                      ble_connect()         (central)
 *     ble_deinit()  ◄──────────────────────────────────────────────  ble_stop()
 *
 * Services are registered between `ble_init()` and `ble_start()`; `ble_start()`
 * creates their attribute tables and starts them. After that the GATT database is
 * fixed until `ble_stop()`.
 *
 * **One connection at a time**, in either role. The role is whichever side the link
 * was made from: advertising gives a peripheral link, `ble_connect()` a central one.
 *
 * ## Advertising uses legacy PDUs on the 5.0 API (ADR-023)
 *
 * The 2M PHY is only reachable through Bluedroid's BLE 5.0 feature API set, and the
 * 4.2 and 5.0 sets cannot both be enabled. Advertising therefore goes through the
 * extended-advertising calls with legacy PDU properties: every central — including a
 * 4.x phone — still sees it. Advertising data carries the flags, the optional 128-bit
 * service UUID (`ble_config_t.adv_uuid128`) and as much of the name as fits; the scan
 * response carries the complete name.
 *
 * ## Link tuning
 *
 * On every new connection the module requests the LE 2M PHY
 * (`CONFIG_BLE_PREFER_2M_PHY`) and the LL data length `CONFIG_BLE_DATA_LEN`. Either
 * may be refused by the peer; the outcome arrives as `BLE_EVT_PHY_UPDATED` and
 * `BLE_EVT_DATA_LEN_CHANGED`. Connection parameters are the application's to ask
 * for (`ble_conn_params_request()`), because only it knows its latency budget.
 *
 * ## GATT database is application policy (ADR-013)
 *
 * The application supplies Bluedroid attribute tables (`esp_gatts_attr_db_t`). That
 * type is deliberately exposed rather than re-abstracted: it is the industry-shaped
 * description of a GATT service and wrapping it would add a layer without removing a
 * decision. Reads are answered by the stack from the attribute value when the table
 * uses `ESP_GATT_AUTO_RSP`; writes are delivered to the service's `on_write`.
 *
 * ## Central role and GATT client
 *
 * Built only when `CONFIG_BT_GATTC_ENABLE` is set; otherwise every central call
 * returns `ESP_ERR_NOT_SUPPORTED`. Scanning reports every advertiser through
 * `BLE_EVT_SCAN_RESULT`; which one to connect to is the application's decision, and
 * `ble_adv_has_uuid128()` is the helper for the common filter. After
 * `BLE_EVT_CONNECTED`, `ble_gattc_discover()` caches the peer's database and
 * `ble_gattc_find_char()` / `ble_gattc_subscribe()` / `ble_gattc_write()` work on it.
 *
 * ## Flow control
 *
 * `ble_gatts_notify()` and `ble_gattc_write()` without response are refused with
 * `ESP_ERR_NO_MEM` when the link has no free ACL buffer, rather than being queued
 * without bound. A streaming caller drops the frame and carries on (FW-BLE-027).
 *
 * ## Security
 *
 * Unless `no_bonding` is set, pairing uses LE Secure Connections with bonding and
 * Just Works (no I/O). Bonds persist in NVS via Bluedroid's own store
 * (`CONFIG_BT_BLE_SMP_BOND_NVS_FLASH`). Legacy pairing is not offered (SYS-SEC-005).
 *
 * ## Radio sharing
 *
 * BLE time-shares the 2.4 GHz radio with Wi-Fi (CON-05). The host task runs on core 0
 * alongside Wi-Fi (`CONFIG_BT_BLUEDROID_PINNED_TO_CORE_0`), away from USB and audio.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit/start/stop` are not safe concurrently with each other. Everything
 * else is safe from any task. Nothing is ISR-safe. Callbacks run on Bluedroid's BTC
 * task and must not block or call back into this module's lifecycle functions.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "sdkconfig.h"

#if CONFIG_BT_ENABLED && CONFIG_BT_BLUEDROID_ENABLED
#include "esp_gatts_api.h"
#else
/* Keep the header compilable so an application can be built with BLE disabled;
 * every call then returns ESP_ERR_NOT_SUPPORTED (FW-BLE-001). */
typedef struct { uint8_t opaque; } esp_gatts_attr_db_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Which side of the current link this device is. */
typedef enum {
    BLE_ROLE_NONE = 0,
    BLE_ROLE_PERIPHERAL,          /**< Connected to while advertising.            */
    BLE_ROLE_CENTRAL,             /**< Connected out with `ble_connect()`.         */
} ble_role_t;

typedef enum {
    BLE_EVT_READY = 0,      /**< Stack up, services started; may advertise or scan. */
    BLE_EVT_ADV_STARTED,
    BLE_EVT_ADV_STOPPED,    /**< By request or because a connection was made.       */
    BLE_EVT_CONNECTED,      /**< `peer`, `conn_id`, `role`, `conn_interval_ms` valid. */
    BLE_EVT_DISCONNECTED,   /**< `reason` valid. Re-advertising if configured.      */
    BLE_EVT_MTU_CHANGED,    /**< `mtu` valid.                                       */
    BLE_EVT_PAIRED,         /**< Encryption established; `bonded` valid.            */
    BLE_EVT_PAIR_FAILED,    /**< `reason` holds the SMP failure code.               */
    /* Added with ADR-023. Appended so existing values keep their meaning. */
    BLE_EVT_SCAN_RESULT,    /**< `peer`, `addr_type`, `rssi`, `adv_data`/`adv_len`.  */
    BLE_EVT_SCAN_STOPPED,   /**< By request, by duration, or by `ble_connect()`.    */
    BLE_EVT_CONNECT_FAILED, /**< Central connection attempt ended; `status` valid.  */
    BLE_EVT_PHY_UPDATED,    /**< `tx_phy`, `rx_phy` valid (1 = 1M, 2 = 2M, 3 = Coded). */
    BLE_EVT_DATA_LEN_CHANGED, /**< `data_len` = LL TX octets now in effect.         */
    BLE_EVT_CONN_PARAMS_UPDATED, /**< `conn_interval_ms`, `latency`, `timeout_ms`.  */
    BLE_EVT_DISCOVERY_DONE, /**< GATT client cache ready; `status` 0 on success.     */
    BLE_EVT_SUBSCRIBED,     /**< CCCD written for `handle`; `status` 0 on success.   */
} ble_event_t;

typedef struct {
    uint16_t conn_id;
    uint8_t  peer[6];
    uint16_t mtu;
    uint16_t conn_interval_ms;   /**< Rounded from 1.25 ms units.                 */
    uint16_t latency;
    uint16_t timeout_ms;
    uint8_t  reason;
    bool     bonded;
    /* Added with ADR-023. */
    ble_role_t role;
    uint8_t  addr_type;          /**< SCAN_RESULT: pass to `ble_connect()`.       */
    int8_t   rssi;
    uint8_t  tx_phy;
    uint8_t  rx_phy;
    uint16_t data_len;
    uint16_t handle;             /**< SUBSCRIBED.                                 */
    uint8_t  status;             /**< 0 = success for *_DONE / *_FAILED / SUBSCRIBED. */
    const uint8_t *adv_data;     /**< SCAN_RESULT: valid only during the callback. */
    uint8_t  adv_len;
} ble_event_info_t;

typedef void (*ble_event_cb_t)(ble_event_t evt, const ble_event_info_t *info, void *ctx);

/**
 * @brief Notification or indication received by the GATT client. BTC task context;
 *        `data` is valid only during the call. Must not block.
 */
typedef void (*ble_gattc_notify_cb_t)(uint16_t handle, const uint8_t *data, size_t len, void *ctx);

typedef struct {
    const char *device_name;        /**< NULL → CONFIG_BLE_DEVICE_NAME. Copied.        */
    uint16_t adv_interval_min_ms;   /**< 0 → CONFIG_BLE_ADV_INTERVAL_MS.               */
    uint16_t adv_interval_max_ms;   /**< 0 → min + 25 % .                               */
    /* Negative flags so that a zero-initialised config is the secure, resilient default. */
    bool     no_bonding;            /**< Disable LE SC pairing/bonding (not recommended). */
    bool     no_auto_readvertise;   /**< Do not restore advertising after a disconnect.   */
    bool     encrypt_on_connect;    /**< Initiate encryption as soon as a link is up.  */
    ble_event_cb_t cb;              /**< Optional.                                     */
    void    *ctx;
    /* Added with ADR-023. */
    const uint8_t *adv_uuid128;     /**< Optional 128-bit service UUID to advertise,
                                         little-endian as Bluedroid stores it. Static
                                         lifetime. Pushes the name to the scan response. */
    ble_gattc_notify_cb_t on_notify; /**< Central: notifications from the peer.        */
} ble_config_t;

/**
 * @brief Host → device write on one of this service's characteristics or descriptors.
 *        BTC task context. If `need_rsp` is true the SDK has already sent the
 *        response; the callback is informational.
 */
typedef void (*ble_gatts_write_cb_t)(uint16_t attr_handle, const uint8_t *data, size_t len, void *ctx);

typedef struct {
    const esp_gatts_attr_db_t *db;      /**< Attribute table. Static lifetime.         */
    uint16_t  count;                    /**< Entries in `db`.                          */
    uint16_t *handles;                  /**< Caller array of `count`; filled by ble_start(). */
    ble_gatts_write_cb_t on_write;      /**< Optional.                                 */
    void     *ctx;
} ble_gatts_service_t;

/** Connection parameters. Every zero selects the Kconfig default named. */
typedef struct {
    uint16_t interval_min_1250us;   /**< 1.25 ms units, 6..3200. 0 → CONFIG_BLE_CONN_INTERVAL_MIN. */
    uint16_t interval_max_1250us;   /**< 1.25 ms units, ≥ min.   0 → CONFIG_BLE_CONN_INTERVAL_MAX. */
    uint16_t latency;               /**< Peripheral latency, connection events. 0..499.           */
    uint16_t timeout_ms;            /**< Supervision timeout, 100..32000. 0 → CONFIG_BLE_SUPERVISION_TIMEOUT_MS. */
} ble_conn_params_t;

typedef struct {
    bool     initialised;
    bool     started;
    bool     advertising;
    bool     connected;
    bool     encrypted;
    uint16_t conn_id;
    uint8_t  peer[6];
    uint16_t mtu;
    uint16_t conn_interval_ms;
    uint32_t connections;        /**< Since init.                                     */
    uint32_t disconnections;
    uint8_t  services;
    /* Added with ADR-023. */
    ble_role_t role;
    bool     scanning;
    bool     discovered;         /**< Central: GATT client cache is ready.            */
    uint8_t  tx_phy;             /**< 0 until reported.                               */
    uint8_t  rx_phy;
    uint16_t data_len;           /**< LL TX octets; 0 until reported.                 */
    uint32_t tx_refused_no_buf;  /**< notify/write refused for lack of an ACL buffer.  */
} ble_status_t;

/**
 * @brief Initialise the controller (BLE mode) and Bluedroid host, register GAP/GATT
 *        callbacks, configure security. Does not advertise or scan.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_NOT_SUPPORTED  Bluetooth or Bluedroid not enabled in the build. Never
 *                            ESP_OK while doing nothing (FW-BLE-011).
 *   - ESP_ERR_INVALID_STATE  already initialised
 *   - ESP_ERR_INVALID_ARG    NULL cfg, or device name longer than 29 bytes
 *   - (propagated)           controller/host errors
 *
 * @note Blocking: yes (hundreds of ms for controller enable).
 */
esp_err_t ble_init(const ble_config_t *cfg);

/** @brief Stop if started, disable and deinitialise host and controller, free memory. */
esp_err_t ble_deinit(void);

/**
 * @brief Register a GATT service. Must precede `ble_start()`.
 *
 * @param[in,out] svc  Static lifetime; `svc->handles` is written by `ble_start()`.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not initialised, or already started),
 *         ESP_ERR_INVALID_ARG (NULL fields or count 0), or ESP_ERR_NO_MEM if
 *         `CONFIG_BLE_MAX_SERVICES` is reached.
 */
esp_err_t ble_gatts_add_service(ble_gatts_service_t *svc);

/**
 * @brief Register the GATT applications, create every service's attribute table,
 *        start the services, and set the device name, advertising and scan-response
 *        data. Blocks until the stack reports readiness or `CONFIG_BLE_START_TIMEOUT_MS`.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_TIMEOUT, or a propagated error.
 */
esp_err_t ble_start(void);

/** @brief Stop advertising and scanning, drop any connection, stop services. Services stay registered. */
esp_err_t ble_stop(void);

/**
 * @brief Begin connectable undirected advertising. Acquires the `"ble"` sleep lock.
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not started, already connected, or scanning.
 */
esp_err_t ble_adv_start(void);

/** @brief Stop advertising. Releases the sleep lock if not connected or scanning. */
esp_err_t ble_adv_stop(void);

/** @brief Drop the current connection. ESP_ERR_INVALID_STATE if none. */
esp_err_t ble_disconnect(void);

/**
 * @brief Send a notification or indication on a characteristic value handle to the
 *        connected peer. The peer must have enabled it via the CCCD.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not connected), ESP_ERR_INVALID_ARG,
 *         ESP_ERR_INVALID_SIZE (len > MTU − 3), ESP_ERR_NO_MEM (no free ACL
 *         buffer: drop and retry later), or a propagated error.
 */
esp_err_t ble_gatts_notify(uint16_t attr_handle, const void *data, size_t len, bool indicate);

/** @brief Update a stored attribute value (what reads return). */
esp_err_t ble_gatts_set_value(uint16_t attr_handle, const void *data, size_t len);

/** @brief Whether a link is up, in either role. */
bool ble_is_connected(void);

/** @brief Snapshot. ESP_ERR_INVALID_ARG on NULL; works before init (zeros). */
esp_err_t ble_status(ble_status_t *out);

/**
 * @brief Ask for new connection parameters on the current link, in either role. The
 *        peer decides; the result arrives as `BLE_EVT_CONN_PARAMS_UPDATED`.
 * @param[in] p  NULL selects every Kconfig default.
 * @return ESP_OK, ESP_ERR_INVALID_STATE (no link), ESP_ERR_INVALID_ARG (out of range).
 */
esp_err_t ble_conn_params_request(const ble_conn_params_t *p);

/**
 * @brief Whether `adv` (an advertising or scan-response payload, as delivered by
 *        `BLE_EVT_SCAN_RESULT`) lists `uuid128` among its complete or incomplete
 *        128-bit service UUIDs. Pure function; usable before init.
 */
bool ble_adv_has_uuid128(const uint8_t *adv, size_t len, const uint8_t uuid128[16]);

/**
 * @brief Forget the bond with `peer`, so the next connection pairs afresh. For the
 *        case where the peer lost its keys (re-flashed, factory reset) and pairing
 *        with the stale bond keeps failing.
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not initialised), ESP_ERR_INVALID_ARG,
 *         or a propagated error (including no such bond).
 */
esp_err_t ble_bond_remove(const uint8_t peer[6]);

/* -------------------------------------------------------------------------- */
/* Central role — needs CONFIG_BT_GATTC_ENABLE, else ESP_ERR_NOT_SUPPORTED.    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Start an active scan. Each advertising report is delivered as
 *        `BLE_EVT_SCAN_RESULT`, duplicates filtered by the controller. Holds the
 *        `"ble"` sleep lock while scanning.
 * @param[in] duration_ms  0 scans until `ble_scan_stop()` or `ble_connect()`.
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not started, connected, or advertising),
 *         ESP_ERR_NOT_SUPPORTED, or a propagated error.
 */
esp_err_t ble_scan_start(uint32_t duration_ms);

/** @brief Stop scanning. ESP_OK if not scanning. */
esp_err_t ble_scan_stop(void);

/**
 * @brief Connect to `peer` as central, stopping any scan first. Asynchronous: the
 *        result is `BLE_EVT_CONNECTED` or `BLE_EVT_CONNECT_FAILED`.
 * @param[in] peer       Address from `BLE_EVT_SCAN_RESULT`.
 * @param[in] addr_type  Address type from the same event.
 * @param[in] p          Initial connection parameters; NULL selects the Kconfig defaults.
 * @return ESP_OK (attempt started), ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_ARG,
 *         ESP_ERR_NOT_SUPPORTED, or a propagated error.
 */
esp_err_t ble_connect(const uint8_t peer[6], uint8_t addr_type, const ble_conn_params_t *p);

/**
 * @brief Discover the peer's GATT database, restricted to one primary service.
 *        Result: `BLE_EVT_DISCOVERY_DONE`. Also requests the local MTU.
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not a central link), ESP_ERR_INVALID_ARG,
 *         ESP_ERR_NOT_SUPPORTED.
 */
esp_err_t ble_gattc_discover(const uint8_t svc_uuid128[16]);

/**
 * @brief Look up a characteristic value handle by 128-bit UUID in the discovered
 *        service. Synchronous (reads the client cache).
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not discovered), ESP_ERR_NOT_FOUND,
 *         ESP_ERR_INVALID_ARG, ESP_ERR_NOT_SUPPORTED.
 */
esp_err_t ble_gattc_find_char(const uint8_t char_uuid128[16], uint16_t *out_handle);

/**
 * @brief Enable notifications on a characteristic: registers for them and writes
 *        its CCCD. Result: `BLE_EVT_SUBSCRIBED`. Notifications then arrive at
 *        `ble_config_t.on_notify`.
 * @return ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_NOT_FOUND (no CCCD),
 *         ESP_ERR_INVALID_ARG, ESP_ERR_NOT_SUPPORTED.
 */
esp_err_t ble_gattc_subscribe(uint16_t char_handle);

/**
 * @brief Write a characteristic value on the peer.
 * @param[in] need_rsp  false → Write Command (no response), for streaming.
 * @return ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_SIZE
 *         (len > MTU − 3), ESP_ERR_NO_MEM (write without response and no free ACL
 *         buffer), ESP_ERR_NOT_SUPPORTED, or a propagated error.
 */
esp_err_t ble_gattc_write(uint16_t handle, const void *data, size_t len, bool need_rsp);

#ifdef __cplusplus
}
#endif
