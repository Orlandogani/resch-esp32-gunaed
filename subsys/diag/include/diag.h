/**
 * @file diag.h
 * @brief Diagnostics: task watchdog registration, health snapshot, boot and fault
 *        accounting, coredump access, and the single entry point for declaring an
 *        unrecoverable fault.
 *
 * Realises FW-DIAG-001..007. This module owns the *mechanism* of fault handling; what
 * to do about a fault (reboot, indicate, enter safe mode) is the application's
 * decision, delivered through `diag_config_t::fault_cb` (ADR-013).
 *
 * ## System state
 *
 *     BOOT ──diag_mark_running()──► RUNNING ──diag_fault()──► FAULT (terminal)
 *       └────────────diag_fault()──────────────────────────────►┘
 *
 * FAULT is terminal until reset. Logging and `diag_health()` keep working in FAULT;
 * the device must never go silently dark (SYS-DIAG-006).
 *
 * ## Persistence
 *
 * Boot count, fault count and last fault code live in the `diag` NVS namespace via
 * `subsys/cfg`. If `cfg` is not initialised when `diag_init()` runs, diag still works
 * but these counters read as zero and are not persisted.
 *
 * ## Thread and ISR safety
 *
 * All functions are safe from any task. `diag_task_feed()` is the only one that may
 * be called from an ISR. `diag_fault()` may be called from any task; it blocks on
 * flash briefly if persistence is available.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DIAG_STATE_BOOT = 0,   /**< From reset until the application declares readiness. */
    DIAG_STATE_RUNNING,    /**< Normal operation. */
    DIAG_STATE_FAULT,      /**< Terminal degraded state; observable, not dark. */
} diag_state_t;

/**
 * @brief Application hook invoked from `diag_fault()` after the fault is logged and
 *        persisted. Runs in the caller's task context. Must not return to "healthy":
 *        it may reboot, assert an indicator, or simply return to let the system
 *        continue in FAULT.
 */
typedef void (*diag_fault_cb_t)(uint32_t code, const char *detail, void *ctx);

typedef struct {
    diag_fault_cb_t fault_cb;   /**< Optional. */
    void           *fault_ctx;  /**< Passed to `fault_cb`. */
} diag_config_t;

typedef struct {
    diag_state_t       state;
    uint64_t           uptime_us;           /**< Since boot, from esp_timer. */
    esp_reset_reason_t reset_reason;
    uint32_t           wakeup_causes;       /**< Bitmask, 0 on a non-sleep boot. */
    uint32_t           boot_count;          /**< Persistent; 0 if cfg unavailable. */
    uint32_t           fault_count;         /**< Persistent; 0 if cfg unavailable. */
    uint32_t           last_fault_code;     /**< Persistent; 0 if none recorded. */
    uint32_t           current_fault_code;  /**< Non-zero only in DIAG_STATE_FAULT. */
    size_t             free_heap;
    size_t             min_free_heap_ever;
    size_t             free_internal_heap;
    size_t             largest_free_block;
    uint8_t            wdt_tasks;           /**< Tasks registered via diag_task_register(). */
    bool               coredump_present;    /**< A retrievable coredump exists in flash. */
} diag_health_t;

/**
 * @brief Initialise diagnostics: read reset/wake context, bump and persist the boot
 *        counter, and make the task watchdog available for registration.
 *
 * Idempotent. Call after `cfg_init()` if persistence is wanted.
 *
 * @param[in] cfg  May be NULL (no fault callback).
 *
 * @return
 *   - ESP_OK
 *
 * @note Thread-safety: not safe concurrently with itself or `diag_deinit()`.
 * @note ISR-safety: no. Blocking: yes (flash, if cfg is available).
 */
esp_err_t diag_init(const diag_config_t *cfg);

/**
 * @brief Unregister every watchdog subscription made through this module and reset
 *        to the uninitialised state. Persistent counters are untouched.
 *
 * @return ESP_OK, also if never initialised.
 */
esp_err_t diag_deinit(void);

/**
 * @brief Declare that initialisation is complete: BOOT → RUNNING.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `diag_init()`, or already in FAULT
 */
esp_err_t diag_mark_running(void);

/** @brief Current system state. DIAG_STATE_BOOT before init. */
diag_state_t diag_state(void);

/* -------------------------------------------------------------------------- */
/* Task watchdog (FW-DIAG-001, FW-DIAG-002)                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Subscribe a task to the task watchdog. It must then call `diag_task_feed()`
 *        at least once per watchdog period (`CONFIG_ESP_TASK_WDT_TIMEOUT_S`, or
 *        `CONFIG_DIAG_WDT_TIMEOUT_S` when diag initialises the WDT itself).
 *
 * If the task WDT was not initialised by the system (`CONFIG_ESP_TASK_WDT_INIT` off),
 * this initialises it with the configured timeout, panicking on expiry.
 *
 * @param[in] task  Task handle, or NULL for the calling task.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `diag_init()`
 *   - ESP_ERR_NOT_SUPPORTED  task WDT compiled out (`CONFIG_ESP_TASK_WDT_EN` off)
 *   - (propagated)           `esp_task_wdt_add()` failure, e.g. already subscribed
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: never.
 */
esp_err_t diag_task_register(TaskHandle_t task);

/** @brief Undo `diag_task_register()`. Same return codes. */
esp_err_t diag_task_unregister(TaskHandle_t task);

/**
 * @brief Feed the watchdog for the calling task.
 *
 * @return ESP_OK, ESP_ERR_NOT_FOUND if the calling task is not subscribed, or
 *         ESP_ERR_NOT_SUPPORTED if the WDT is compiled out.
 *
 * @note Thread-safety: safe. ISR-safety: yes. Blocking: never.
 */
esp_err_t diag_task_feed(void);

/* -------------------------------------------------------------------------- */
/* Health and faults (FW-DIAG-003, FW-DIAG-005, FW-DIAG-006)                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Fill a health snapshot. Works in every state including FAULT.
 *
 * @return ESP_OK, or ESP_ERR_INVALID_ARG if `out` is NULL. Usable before init
 *         (persistent fields read as zero).
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: never.
 */
esp_err_t diag_health(diag_health_t *out);

/**
 * @brief Declare an unrecoverable fault. The single entry point (DES-DIAG-003).
 *
 * Logs at ERROR, persists the code and increments the fault counter (if cfg is
 * available), transitions to FAULT, then invokes the application's `fault_cb`.
 * Never reboots on its own. A second call while already in FAULT is logged and
 * counted but does not overwrite the first fault's code.
 *
 * @param[in] code    Non-zero, subsystem-defined. Zero is reserved for "no fault".
 * @param[in] detail  Optional human-readable context for the log. May be NULL.
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: yes (flash).
 */
void diag_fault(uint32_t code, const char *detail);

/**
 * @brief Change the runtime log level for one tag, or for all tags with `"*"`.
 *
 * Bounded above by `CONFIG_LOG_MAXIMUM_LEVEL`: a level compiled out cannot be enabled.
 *
 * @return ESP_OK, or ESP_ERR_INVALID_ARG on NULL `tag`.
 */
esp_err_t diag_set_log_level(const char *tag, esp_log_level_t level);

/* -------------------------------------------------------------------------- */
/* Coredump (FW-DIAG-004)                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Whether a valid coredump image is stored in the coredump partition.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG on NULL, or ESP_ERR_NOT_SUPPORTED if
 *         coredump-to-flash is not enabled in the build.
 */
esp_err_t diag_coredump_present(bool *present);

/** @brief Erase the stored coredump. ESP_OK also if none was present. */
esp_err_t diag_coredump_erase(void);

/* -------------------------------------------------------------------------- */
/* Rate-limited logging (DES-DIAG-004)                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Log at most once per `interval_ms` from this call site, counting suppressed
 *        repeats and reporting them when the next line is allowed through.
 *
 * Use on any path that can fail in a storm (DMA overrun, queue full). Never use in
 * an ISR — it still calls `ESP_LOGx`.
 *
 *     DIAG_LOG_RL(ESP_LOGW, TAG, 1000, "dma overrun (total %u)", (unsigned)count);
 */
#define DIAG_LOG_RL(LOG_MACRO, tag, interval_ms, fmt, ...)                                   \
    do {                                                                                     \
        static int64_t  _diag_rl_last;                                                       \
        static uint32_t _diag_rl_suppressed;                                                 \
        int64_t _diag_rl_now = esp_timer_get_time();                                         \
        if (_diag_rl_now - _diag_rl_last >= (int64_t)(interval_ms) * 1000) {                 \
            _diag_rl_last = _diag_rl_now;                                                    \
            if (_diag_rl_suppressed) {                                                       \
                LOG_MACRO(tag, fmt " [+%u suppressed]", ##__VA_ARGS__,                       \
                          (unsigned)_diag_rl_suppressed);                                    \
                _diag_rl_suppressed = 0;                                                     \
            } else {                                                                         \
                LOG_MACRO(tag, fmt, ##__VA_ARGS__);                                          \
            }                                                                                \
        } else {                                                                             \
            _diag_rl_suppressed++;                                                           \
        }                                                                                    \
    } while (0)

#ifdef __cplusplus
}
#endif
