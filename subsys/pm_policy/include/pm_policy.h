/**
 * @file pm_policy.h
 * @brief Deep-sleep arbitration between subsystems via named, reference-counted locks.
 *
 * Structurally Zephyr's `pm_policy_state_lock_get()` / `_put()`: any subsystem that
 * needs the CPU to stay powered creates a named lock at init and holds it while it
 * is active. Deep sleep proceeds only when every lock's count is zero. Because the
 * arbiter never needs to know its clients, adding a subsystem requires no change
 * here (ADR-006, SYS-SYS-022). Realises FW-PMP-001..012, FW-PMP-020..023.
 *
 * ## Invariants — read these before using the API
 *
 * 1. **Names are borrowed, not copied.** The `name` passed to
 *    `pm_policy_lock_create()` must have static storage duration (a string
 *    literal, or a `static const char[]`). It is stored by pointer and read for the
 *    lifetime of the process (FW-PMP-020).
 * 2. **Locks are never destroyed.** A slot, once allocated, lives until reset. This
 *    is deliberate: it removes use-after-free on a handle entirely. Sizing is by
 *    `CONFIG_PM_POLICY_MAX_LOCKS`; exhaustion fails loudly at init time, not
 *    randomly under load (ADR-005, FW-PMP-021).
 * 3. **Nothing here is ISR-safe.** Every function takes a mutex (FW-PMP-022).
 * 4. **`pm_policy_try_deep_sleep()` returns only on failure.** Success enters deep
 *    sleep, which is a reset. Treat any return as "did not sleep".
 * 5. **Creating a lock with a name already in use returns the existing handle**
 *    rather than consuming a second slot (FW-PMP-012). Two subsystems sharing a
 *    name therefore share a count — choose names that are unique per subsystem.
 *
 * ## Thread safety
 *
 * All functions are safe to call concurrently from any task on any core once
 * `pm_policy_init()` has returned. `pm_policy_init()` and `pm_policy_deinit()`
 * themselves must not race with anything.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle to a named lock. Valid until reset once obtained. */
typedef struct pm_policy_lock *pm_policy_lock_handle_t;

/** One row of the diagnostic holder listing. */
typedef struct {
    const char *name;   /**< The name given at creation (borrowed pointer). */
    uint32_t    count;  /**< Current reference count; > 0 means "vetoing sleep". */
} pm_policy_holder_t;

/**
 * @brief Initialise the lock table and its mutex.
 *
 * Idempotent: a second call returns ESP_OK and preserves existing locks and counts
 * (FW-PMP-010). Call once from `app_main` before any subsystem that creates a lock.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_NO_MEM   the FreeRTOS mutex could not be allocated
 *
 * @note Thread-safety: not safe concurrently with any other `pm_policy_*` call.
 * @note ISR-safety: no. Blocking: never.
 */
esp_err_t pm_policy_init(void);

/**
 * @brief Tear down the table and free the mutex. **Invalidates every handle.**
 *
 * Exists for tests and for controlled full-system teardown (SYS-SYS-007). A
 * production application has no reason to call it; locks are meant to live
 * forever. Any handle obtained before this call must not be used afterwards.
 *
 * @return
 *   - ESP_OK   also if never initialised
 *
 * @note Thread-safety: not safe concurrently with any other `pm_policy_*` call.
 * @note ISR-safety: no. Blocking: never.
 */
esp_err_t pm_policy_deinit(void);

/**
 * @brief Obtain a named lock, allocating a table slot on first use of the name.
 *
 * @param[in]  name        Static-lifetime, NUL-terminated name. See invariant 1.
 * @param[out] out_handle  Receives the handle. Not written on failure.
 *
 * @return
 *   - ESP_OK                 new lock created, or existing lock with this name returned
 *   - ESP_ERR_INVALID_STATE  before `pm_policy_init()`
 *   - ESP_ERR_INVALID_ARG    NULL `name`, empty `name`, or NULL `out_handle`
 *   - ESP_ERR_NO_MEM         table full; logged at ERROR with the symbol to raise
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: briefly, on the table mutex.
 */
esp_err_t pm_policy_lock_create(const char *name, pm_policy_lock_handle_t *out_handle);

/**
 * @brief Increment the lock's count. Nested acquisition is supported.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `pm_policy_init()`
 *   - ESP_ERR_INVALID_ARG    NULL handle
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: briefly, on the table mutex.
 */
esp_err_t pm_policy_lock_acquire(pm_policy_lock_handle_t handle);

/**
 * @brief Decrement the lock's count.
 *
 * Releasing at zero is a caller bug; it is logged at WARN and the count stays at
 * zero. It never wraps — a wrapped count would be a permanent sleep veto, the
 * worst possible failure for a battery device (DES-PMP-004).
 *
 * @return
 *   - ESP_OK                 also on the release-at-zero case, which is logged
 *   - ESP_ERR_INVALID_STATE  before `pm_policy_init()`
 *   - ESP_ERR_INVALID_ARG    NULL handle
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: briefly, on the table mutex.
 */
esp_err_t pm_policy_lock_release(pm_policy_lock_handle_t handle);

/**
 * @brief Current count of a lock, for callers that want to assert their own balance.
 *
 * @return count, or 0 if `handle` is NULL or the subsystem is not initialised.
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: briefly, on the table mutex.
 */
uint32_t pm_policy_lock_count(pm_policy_lock_handle_t handle);

/**
 * @brief Whether deep sleep would be permitted right now.
 *
 * Advisory only: another task may acquire a lock between this call and any action
 * taken on its answer. Use `pm_policy_try_deep_sleep()` to act atomically.
 *
 * @return true if no in-use lock has a non-zero count. false before init.
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: briefly, on the table mutex.
 */
bool pm_policy_can_deep_sleep(void);

/**
 * @brief Enter deep sleep if and only if no lock is held. **Returns only on veto.**
 *
 * On veto, every held lock's name and count is logged at INFO — not just the first —
 * so that "why won't it sleep" is answerable from one log line (FW-PMP-008).
 * On success the call does not return; see `power_enter_deep_sleep()`.
 *
 * @param[in] sleep_us  Timer wake in microseconds, or 0 for GPIO-only wake.
 *
 * @return
 *   - ESP_ERR_INVALID_STATE  vetoed by at least one held lock, or before init
 *   - (does not return)      on success
 *
 * @note Thread-safety: safe. ISR-safety: no.
 * @note Blocking: briefly on the mutex, then forever on success.
 */
esp_err_t pm_policy_try_deep_sleep(uint64_t sleep_us);

/**
 * @brief Snapshot of every in-use lock, held or not, for diagnostics (FW-PMP-011).
 *
 * @param[out] out    Array of at least `max` entries. May be NULL if `max` is 0,
 *                    to query the count alone.
 * @param[in]  max    Capacity of `out`.
 * @param[out] count  Total in-use locks (may exceed `max`; only `max` are written).
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `pm_policy_init()`
 *   - ESP_ERR_INVALID_ARG    NULL `count`, or NULL `out` with `max` > 0
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: briefly, on the table mutex.
 */
esp_err_t pm_policy_holders(pm_policy_holder_t *out, size_t max, size_t *count);

#ifdef __cplusplus
}
#endif
