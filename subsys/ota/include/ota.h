/**
 * @file ota.h
 * @brief Over-the-air firmware update: HTTPS download into the inactive slot,
 *        verification, two-phase commit, and application-decided rollback.
 *
 * Realises FW-OTA-001..013. Built on `esp_https_ota` streaming, so the image is
 * never buffered whole. The running slot is never written; boot selection changes
 * only after the new image is completely written and verified (FW-OTA-005). Every
 * failure path leaves the previously working image bootable (SYS-OTA-003).
 *
 * ## Session
 *
 *     ota_start(url) ─► worker task ─► STARTED ─► PROGRESS… ─► VERIFIED ─► COMMITTED
 *                                                      │            │
 *                                                      └─ FAILED / ABORTED ◄─ ota_abort()
 *
 * `ota_start()` returns immediately; the outcome arrives through `ota_config_t::cb`
 * on the worker task. After COMMITTED the SDK does **not** reboot — the application
 * decides when (DES-OTA-006).
 *
 * ## Rollback (FW-OTA-006, FW-OTA-007)
 *
 * With `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` a freshly installed image boots in
 * the pending-verify state. The application must call `ota_mark_valid()` once it
 * has satisfied itself the image is healthy; if it resets first, the bootloader
 * returns to the previous slot on its own. `ota_mark_invalid()` requests that
 * rollback explicitly, again without rebooting.
 *
 * ## Network
 *
 * A network interface must be up before `ota_start()`; `subsys/wifi_sta` provides
 * one. This module does not manage connectivity.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit` are not safe concurrently with anything. `start/abort/mark_*` and
 * the queries are safe from any task. Nothing is ISR-safe. The callback runs on
 * the OTA worker task (core `CONFIG_OTA_TASK_CORE`, priority `CONFIG_OTA_TASK_PRIORITY`).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OTA_EVT_STARTED = 0,   /**< Connected; image header validated; download beginning. */
    OTA_EVT_PROGRESS,      /**< Periodic; see `bytes_read` / `image_size`.              */
    OTA_EVT_VERIFIED,      /**< Download complete and image verified; about to commit. */
    OTA_EVT_COMMITTED,     /**< Boot partition switched. Reboot when convenient.       */
    OTA_EVT_FAILED,        /**< See `err`. Running image untouched.                    */
    OTA_EVT_ABORTED,       /**< `ota_abort()` honoured. Running image untouched.       */
} ota_event_t;

typedef struct {
    size_t    bytes_read;   /**< Cumulative bytes written to the inactive slot.       */
    int       image_size;   /**< Total, or -1 if the server did not say.              */
    esp_err_t err;          /**< For FAILED.                                          */
    char      new_version[32]; /**< From the downloaded image header, once known.     */
} ota_event_info_t;

typedef void (*ota_event_cb_t)(ota_event_t evt, const ota_event_info_t *info, void *ctx);

typedef struct {
    const char *cert_pem;      /**< PEM trust anchor, static lifetime. NULL → use the
                                    certificate bundle if `use_cert_bundle`.          */
    bool  use_cert_bundle;     /**< Trust ESP-IDF's built-in CA bundle
                                    (`CONFIG_MBEDTLS_CERTIFICATE_BUNDLE`).           */
    bool  reject_same_version; /**< Fail if the new image's version string equals the
                                    running one's. Off by default.                    */
    ota_event_cb_t cb;         /**< Optional.                                         */
    void *ctx;
} ota_config_t;

typedef struct {
    char running_label[17];     /**< e.g. "ota_0".                                    */
    char next_label[17];        /**< Slot an update would go to.                      */
    char running_version[32];   /**< From the running image's app descriptor.         */
    bool pending_verify;        /**< Running image has not yet been marked valid.     */
    bool in_progress;
    size_t bytes_read;          /**< Of the session in progress, if any.              */
    int    image_size;
} ota_status_t;

/**
 * @brief Validate the partition layout and record the running slot's state.
 *
 * Idempotent.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  the partition table has no second application slot
 *                            (single-app layout); logged at ERROR. See ADR-008.
 *   - ESP_ERR_INVALID_ARG    `use_cert_bundle` without bundle support compiled in,
 *                            or neither `cert_pem` nor `use_cert_bundle` set
 *   - (propagated)           `pm_policy_lock_create()` failure
 */
esp_err_t ota_init(const ota_config_t *cfg);

/** @brief Abort any session and forget the configuration. ESP_OK if never initialised. */
esp_err_t ota_deinit(void);

/**
 * @brief Begin downloading `url` into the inactive slot on the worker task.
 *
 * @param[in] url  `https://…`. Copied. `http://` is refused unless the build sets
 *                 `CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP`, which a release must not (ADR-016).
 *
 * @return
 *   - ESP_OK                 session started
 *   - ESP_ERR_INVALID_STATE  before init, a session already in progress, or **no
 *                            network interface exists** (nothing has called
 *                            `esp_netif_init()`; lwIP would assert otherwise)
 *   - ESP_ERR_INVALID_ARG    NULL/empty URL, too long, or plaintext HTTP refused
 *   - ESP_ERR_NO_MEM         worker task could not be created
 */
esp_err_t ota_start(const char *url);

/**
 * @brief Request cancellation of the session in progress. Returns immediately; the
 *        worker stops at the next chunk boundary and emits ABORTED. The inactive
 *        slot is left without a valid image (FW-OTA-009).
 *
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if nothing is in progress.
 */
esp_err_t ota_abort(void);

/** @brief Whether a session is in progress. */
bool ota_in_progress(void);

/**
 * @brief Confirm the running image so the bootloader stops considering rollback.
 *        No-op with ESP_OK if the image was already valid.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE before init, or a propagated `esp_ota` error.
 */
esp_err_t ota_mark_valid(void);

/**
 * @brief Mark the running image invalid so the **next** boot returns to the previous
 *        slot. Does not reboot (DES-OTA-006).
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE before init or if no previous slot exists,
 *         or a propagated `esp_ota` error.
 */
esp_err_t ota_mark_invalid(void);

/** @brief Snapshot. ESP_ERR_INVALID_ARG on NULL; ESP_ERR_INVALID_STATE before init. */
esp_err_t ota_status(ota_status_t *out);

#ifdef __cplusplus
}
#endif
