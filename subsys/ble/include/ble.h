/**
 * @file ble.h
 * @brief Bluetooth Low Energy peripheral on Bluedroid: controller and host
 *        lifecycle, application-owned GATT services, legacy advertising, LE Secure
 *        Connections pairing with bonding, and a sleep lock while the radio is used.
 *
 * Realises FW-BLE-001..011 with Bluedroid per ADR-018 (supersedes ADR-015). The
 * ESP32-S3 is LE-only (CON-04); Classic memory is released at init.
 *
 * ## Lifecycle
 *
 *     ble_init(cfg) ─► ble_gatts_add_service(svc)… ─► ble_start() ─► ble_adv_start()
 *                                                                       ▲        │
 *                                        ble_adv_stop() / connection ◄──┘        │
 *                                                                                ▼
 *     ble_deinit()  ◄────────────────────────────────────────────────  ble_stop()
 *
 * Services are registered between `ble_init()` and `ble_start()`; `ble_start()`
 * creates their attribute tables and starts them. After that the GATT database is
 * fixed until `ble_stop()`.
 *
 * ## GATT database is application policy (ADR-013)
 *
 * The application supplies Bluedroid attribute tables (`esp_gatts_attr_db_t`). That
 * type is deliberately exposed rather than re-abstracted: it is the industry-shaped
 * description of a GATT service and wrapping it would add a layer without removing a
 * decision. Reads are answered by the stack from the attribute value when the table
 * uses `ESP_GATT_AUTO_RSP`; writes are delivered to the service's `on_write`.
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

typedef enum {
    BLE_EVT_READY = 0,      /**< Stack up, services started; may advertise.       */
    BLE_EVT_ADV_STARTED,
    BLE_EVT_ADV_STOPPED,    /**< By request or because a connection was made.     */
    BLE_EVT_CONNECTED,      /**< `peer`, `conn_id`, `conn_interval_ms` valid.     */
    BLE_EVT_DISCONNECTED,   /**< `reason` valid. Re-advertising if configured.    */
    BLE_EVT_MTU_CHANGED,    /**< `mtu` valid.                                     */
    BLE_EVT_PAIRED,         /**< Encryption established; `bonded` valid.          */
    BLE_EVT_PAIR_FAILED,    /**< `reason` holds the SMP failure code.             */
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
} ble_event_info_t;

typedef void (*ble_event_cb_t)(ble_event_t evt, const ble_event_info_t *info, void *ctx);

typedef struct {
    const char *device_name;        /**< NULL → CONFIG_BLE_DEVICE_NAME. Copied.        */
    uint16_t adv_interval_min_ms;   /**< 0 → CONFIG_BLE_ADV_INTERVAL_MS.               */
    uint16_t adv_interval_max_ms;   /**< 0 → min + 25 % .                               */
    /* Negative flags so that a zero-initialised config is the secure, resilient default. */
    bool     no_bonding;            /**< Disable LE SC pairing/bonding (not recommended). */
    bool     no_auto_readvertise;   /**< Do not restore advertising after a disconnect.   */
    bool     encrypt_on_connect;    /**< Initiate encryption as soon as a central connects. */
    ble_event_cb_t cb;              /**< Optional.                                     */
    void    *ctx;
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
} ble_status_t;

/**
 * @brief Initialise the controller (BLE mode) and Bluedroid host, register GAP/GATTS
 *        callbacks, configure security. Does not advertise.
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
 * @brief Register the GATT application, create every service's attribute table,
 *        start the services, and set the device name and advertising data. Blocks
 *        until the stack reports readiness or `CONFIG_BLE_START_TIMEOUT_MS`.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_TIMEOUT, or a propagated error.
 */
esp_err_t ble_start(void);

/** @brief Stop advertising, drop any connection, stop services. Services stay registered. */
esp_err_t ble_stop(void);

/**
 * @brief Begin connectable undirected advertising. Acquires the `"ble"` sleep lock.
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not started or already connected.
 */
esp_err_t ble_adv_start(void);

/** @brief Stop advertising. Releases the sleep lock if not connected. */
esp_err_t ble_adv_stop(void);

/** @brief Drop the current connection. ESP_ERR_INVALID_STATE if none. */
esp_err_t ble_disconnect(void);

/**
 * @brief Send a notification or indication on a characteristic value handle to the
 *        connected central. The central must have enabled it via the CCCD.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not connected), ESP_ERR_INVALID_ARG,
 *         ESP_ERR_INVALID_SIZE (len > MTU − 3), or a propagated error.
 */
esp_err_t ble_gatts_notify(uint16_t attr_handle, const void *data, size_t len, bool indicate);

/** @brief Update a stored attribute value (what reads return). */
esp_err_t ble_gatts_set_value(uint16_t attr_handle, const void *data, size_t len);

/** @brief Whether a central is connected. */
bool ble_is_connected(void);

/** @brief Snapshot. ESP_ERR_INVALID_ARG on NULL; works before init (zeros). */
esp_err_t ble_status(ble_status_t *out);

#ifdef __cplusplus
}
#endif
