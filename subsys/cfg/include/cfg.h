/**
 * @file cfg.h
 * @brief Persistent, namespaced configuration on NVS with self-healing init and
 *        schema-versioned blobs.
 *
 * Realises FW-CFG-001..005. Each subsystem opens a namespace equal to its own name
 * (`cfg_open("ble", ...)`) and cannot reach another's keys (FW-CFG-002). A missing
 * key never fails a boot: getters write the caller's default and report
 * `ESP_ERR_NVS_NOT_FOUND`, which callers treat as "using default", not as an error
 * to propagate (FW-CFG-003).
 *
 * ## Lifecycle
 *
 *     cfg_init() ──► cfg_open(ns) ──► get/set ──► cfg_close() ──► cfg_deinit()
 *
 * ## Storage semantics
 *
 * - Every setter commits immediately; there is no batching. `cfg_commit()` exists
 *   for API completeness and is a no-op after a setter.
 * - Blobs carry a header (magic, schema version, length). A read whose stored schema
 *   differs from the one requested returns `ESP_ERR_INVALID_VERSION` and tells the
 *   caller what was stored, so migration is the caller's deliberate act, never a
 *   misinterpreted struct (FW-CFG-005).
 * - Names: NVS limits namespace and key names to 15 characters.
 *
 * ## Thread and ISR safety
 *
 * All functions except `cfg_init()`/`cfg_deinit()` are safe from any task on any
 * core; NVS serialises internally. Nothing here is ISR-safe. Getters and setters
 * block on flash I/O — never call them on a deadline path.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "nvs.h"   /* ESP_ERR_NVS_NOT_FOUND is part of this API's contract */

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle to an open namespace. */
typedef struct cfg_ns *cfg_handle_t;

/** Maximum length of a namespace or key name, excluding the terminator (NVS limit). */
#define CFG_NAME_MAX 15

/**
 * @brief Initialise NVS, erasing and re-initialising the partition if it is corrupt,
 *        full of stale pages, or written by a newer NVS version (DES-CFG-002).
 *
 * Idempotent.
 *
 * @return
 *   - ESP_OK
 *   - (propagated)  `nvs_flash_init()` failure that erase-and-retry did not cure,
 *                   e.g. no `nvs` partition in the table
 *
 * @note Thread-safety: not safe concurrently with itself or `cfg_deinit()`.
 * @note ISR-safety: no. Blocking: yes (flash).
 */
esp_err_t cfg_init(void);

/**
 * @brief Close every open handle and deinitialise NVS.
 *
 * @return ESP_OK, also if never initialised.
 * @note Thread-safety: not safe concurrently with any `cfg_*` call. ISR-safety: no.
 */
esp_err_t cfg_deinit(void);

/**
 * @brief Open a namespace for read/write, creating it on first use.
 *
 * Opening the same namespace twice returns two independent handles; both are valid.
 *
 * @param[in]  ns   Namespace, 1..CFG_NAME_MAX chars. By convention the subsystem name.
 * @param[out] out  Receives the handle. Not written on failure.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `cfg_init()`
 *   - ESP_ERR_INVALID_ARG    NULL/empty/too-long `ns`, or NULL `out`
 *   - ESP_ERR_NO_MEM         all `CONFIG_CFG_MAX_HANDLES` slots are in use
 *   - (propagated)           `nvs_open()` failure
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: yes (flash).
 */
esp_err_t cfg_open(const char *ns, cfg_handle_t *out);

/**
 * @brief Close a handle and release its slot. Safe to call with NULL (no-op).
 *
 * @return ESP_OK, or ESP_ERR_INVALID_STATE before `cfg_init()`.
 * @note Thread-safety: not safe concurrently with other use of the same handle.
 */
esp_err_t cfg_close(cfg_handle_t h);

/* -------------------------------------------------------------------------- */
/* Scalars                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read a u32, or store and return `dflt` if the key is absent.
 *
 * @return
 *   - ESP_OK                  `*out` holds the stored value
 *   - ESP_ERR_NVS_NOT_FOUND   key was absent; `*out` = `dflt`, which was also written.
 *                             This is the "using default" outcome, not a failure.
 *   - ESP_ERR_INVALID_STATE   before `cfg_init()`
 *   - ESP_ERR_INVALID_ARG     NULL handle/key/out, or key too long
 *   - (propagated)            other NVS errors; `*out` = `dflt`
 */
esp_err_t cfg_get_u32(cfg_handle_t h, const char *key, uint32_t *out, uint32_t dflt);
esp_err_t cfg_set_u32(cfg_handle_t h, const char *key, uint32_t value);

/** Signed variants with identical semantics to the u32 pair. */
esp_err_t cfg_get_i32(cfg_handle_t h, const char *key, int32_t *out, int32_t dflt);
esp_err_t cfg_set_i32(cfg_handle_t h, const char *key, int32_t value);

/* -------------------------------------------------------------------------- */
/* Strings                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read a NUL-terminated string into `buf`, or store and return `dflt`.
 *
 * @param[out] buf      Destination, always NUL-terminated on return when `buf_len` > 0.
 * @param[in]  buf_len  Capacity of `buf` including the terminator.
 * @param[in]  dflt     Default; may be NULL, meaning empty string.
 *
 * @return
 *   - ESP_OK / ESP_ERR_NVS_NOT_FOUND   as for `cfg_get_u32()`
 *   - ESP_ERR_INVALID_SIZE             stored string does not fit; `buf` holds the
 *                                      truncated default and nothing was written
 *   - ESP_ERR_INVALID_STATE / ESP_ERR_INVALID_ARG / (propagated)
 */
esp_err_t cfg_get_str(cfg_handle_t h, const char *key, char *buf, size_t buf_len, const char *dflt);
esp_err_t cfg_set_str(cfg_handle_t h, const char *key, const char *value);

/* -------------------------------------------------------------------------- */
/* Versioned blobs (FW-CFG-005, DES-CFG-003)                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read a blob written by `cfg_set_blob()` with the same `schema`.
 *
 * @param[in]  schema          Schema version the caller understands.
 * @param[out] buf             Destination for the payload.
 * @param[in]  buf_len         Capacity of `buf`.
 * @param[out] out_len         Payload bytes copied (0 on any failure). Required.
 * @param[out] stored_schema   Optional. On ESP_ERR_INVALID_VERSION, the schema that is
 *                             actually stored, so the caller can migrate.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_NVS_NOT_FOUND     key absent. Nothing is written — a blob has no
 *                               meaningful scalar default.
 *   - ESP_ERR_INVALID_VERSION   stored schema != `schema`; see `stored_schema`
 *   - ESP_ERR_INVALID_SIZE      payload larger than `buf_len`
 *   - ESP_ERR_INVALID_CRC       stored data is not a blob this module wrote
 *   - ESP_ERR_INVALID_STATE / ESP_ERR_INVALID_ARG / (propagated)
 */
esp_err_t cfg_get_blob(cfg_handle_t h, const char *key, uint16_t schema,
                       void *buf, size_t buf_len, size_t *out_len, uint16_t *stored_schema);

/**
 * @brief Write a blob with a schema header. `len` may be 0.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_SIZE
 *         (len > CONFIG_CFG_MAX_BLOB_BYTES), or a propagated NVS error.
 */
esp_err_t cfg_set_blob(cfg_handle_t h, const char *key, uint16_t schema, const void *buf, size_t len);

/* -------------------------------------------------------------------------- */
/* Maintenance                                                                 */
/* -------------------------------------------------------------------------- */

/** @brief Remove one key. ESP_OK also if it did not exist. */
esp_err_t cfg_erase_key(cfg_handle_t h, const char *key);

/** @brief Remove every key in the namespace. */
esp_err_t cfg_erase_all(cfg_handle_t h);

/** @brief Flush pending writes. Setters already commit; provided for completeness. */
esp_err_t cfg_commit(cfg_handle_t h);

/** @brief Whether `key` exists in the namespace. false on any error or before init. */
bool cfg_exists(cfg_handle_t h, const char *key);

#ifdef __cplusplus
}
#endif
