/**
 * @file power.h
 * @brief Physical power state: CPU frequency scaling, sleep entry, wake sources,
 *        and a CRC-protected block of state that survives deep sleep.
 *
 * This module holds **no policy** about when to sleep — that is `pm_policy`'s job
 * (SAD §3.4). It only knows how. Realises FW-PWR-001..011, FW-PWR-020..023.
 *
 * ## Lifecycle
 *
 *     power_init()  ──►  [any of the calls below]  ──►  power_deinit()
 *
 * `power_init()` is idempotent. Every other function returns ESP_ERR_INVALID_STATE
 * before it.
 *
 * ## Deep sleep is a reset
 *
 * On the ESP32-S3, waking from deep sleep re-enters the reset vector. Task state,
 * heap, and ordinary `.bss`/`.data` are gone. Only RTC memory survives, which is
 * what `power_retained_*()` manages. Light sleep, by contrast, resumes in place.
 *
 * ## Thread and ISR safety
 *
 * All functions are safe from any task. None is ISR-safe except
 * `power_get_wakeup_causes()`, which is a pure read.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the power subsystem.
 *
 * - Logs the wake causes of this boot at INFO (FW-PWR-003).
 * - If `CONFIG_PM_ENABLE` and `CONFIG_POWER_ENABLE_DFS` are both set, configures
 *   dynamic frequency scaling between `CONFIG_POWER_MIN_FREQ_MHZ` and
 *   `CONFIG_POWER_MAX_FREQ_MHZ` (FW-PWR-001). Otherwise leaves the default frequency.
 * - Validates the RTC-retained block (magic + CRC) and marks it valid or not
 *   (FW-PWR-010). A power-on or brownout reset always invalidates it.
 *
 * Idempotent: a second call returns ESP_OK and changes nothing (FW-SYS-021).
 *
 * @return
 *   - ESP_OK
 *   - (propagated)  `esp_pm_configure()` failure. Logged at WARN; the subsystem is
 *                   still initialised and usable — a device that cannot scale
 *                   frequency is degraded, not broken (FW-SYS-025).
 *
 * @note Thread-safety: not safe concurrently with itself or `power_deinit()`.
 * @note ISR-safety: no.
 * @note Blocking: never.
 */
esp_err_t power_init(void);

/**
 * @brief Restore the default frequency configuration, clear every wake source
 *        configured through this module, and return to the uninitialised state.
 *
 * Does not touch the RTC-retained block; that is data, not configuration.
 *
 * @return
 *   - ESP_OK                   also if never initialised
 *
 * @note Thread-safety: not safe concurrently with any other `power_*` call.
 * @note ISR-safety: no.
 * @note Blocking: never.
 */
esp_err_t power_deinit(void);

/**
 * @brief Enter deep sleep. **Does not return.**
 *
 * If `sleep_us` is non-zero, a timer wake is armed for that many microseconds.
 * If zero, no timer is armed and only previously configured wake sources
 * (see `power_enable_gpio_wakeup()`) can end the sleep — pass zero deliberately,
 * because a device with no wake source configured sleeps until power-cycled.
 *
 * Callers that hold a `pm_policy` lock must go through
 * `pm_policy_try_deep_sleep()` instead; this function performs no arbitration.
 *
 * @param[in] sleep_us  Timer wake in microseconds, or 0 for none.
 *
 * @note Thread-safety: n/a — the caller's core stops executing.
 * @note ISR-safety: no.
 * @note Blocking: forever, from the caller's point of view.
 */
void power_enter_deep_sleep(uint64_t sleep_us) __attribute__((noreturn));

/**
 * @brief Enter light sleep and return when woken.
 *
 * RAM, peripherals' register state, and task state are preserved. Unlike deep
 * sleep this returns, so it is safe to call from a task that owns resources.
 *
 * @param[in]  sleep_us      Timer wake in microseconds, or 0 for none (then a GPIO
 *                           or other configured source must end the sleep).
 * @param[out] wakeup_causes Optional. Bitmask of `esp_sleep_source_t` causes that
 *                           ended the sleep. Written only on ESP_OK.
 *
 * @warning On the ESP32-S3 the built-in USB-Serial/JTAG link **does not survive light
 *          sleep** — the host keeps the COM port listed but the chip never answers
 *          again until physically re-plugged. When a USJ host is connected this
 *          function therefore returns ESP_ERR_NOT_ALLOWED unless the build sets
 *          `CONFIG_POWER_LIGHT_SLEEP_ALLOW_WITH_USJ`. Production boards without a
 *          USJ console can enable that symbol safely.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `power_init()`
 *   - ESP_ERR_NOT_ALLOWED    a USB-Serial/JTAG host is connected and the override
 *                            is not configured (see warning)
 *   - (propagated)           `esp_light_sleep_start()` failure, e.g. a Wi-Fi or
 *                            Bluetooth stack that refuses sleep right now
 *
 * @note Thread-safety: safe. ISR-safety: no.
 * @note Blocking: yes — for the duration of the sleep.
 */
esp_err_t power_enter_light_sleep(uint64_t sleep_us, uint32_t *wakeup_causes);

/**
 * @brief Bitmask of every wake source that contributed to this boot.
 *
 * Zero means a cold boot (power-on, external reset, brownout, software reset).
 * ESP-IDF's raw bitmap marks that case with `BIT(ESP_SLEEP_WAKEUP_UNDEFINED)`;
 * this function strips that bit so zero is unambiguous. More than one bit may be
 * set. Test bits as `1u << ESP_SLEEP_WAKEUP_x`; do not compare for equality.
 *
 * @note Thread-safety: safe. ISR-safety: safe. Blocking: never.
 */
uint32_t power_get_wakeup_causes(void);

/**
 * @brief Add an RTC-capable GPIO to the EXT1 deep-sleep wake mask.
 *
 * Additive: enabling a second pin does not disturb the first (FW-PWR-008).
 *
 * @param[in] pin           RTC-capable GPIO (on the ESP32-S3: GPIO0..GPIO21).
 * @param[in] wake_on_high  true → wake when the pin is high; false → when low.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `power_init()`
 *   - ESP_ERR_INVALID_ARG    `pin` is not RTC-capable
 *   - (propagated)           other `esp_sleep_enable_ext1_wakeup_io()` failures
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: never.
 */
esp_err_t power_enable_gpio_wakeup(gpio_num_t pin, bool wake_on_high);

/**
 * @brief Remove one GPIO from the EXT1 wake mask, leaving others in place.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `power_init()`
 *   - (propagated)           `esp_sleep_disable_ext1_wakeup_io()` failure
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: never.
 */
esp_err_t power_disable_gpio_wakeup(gpio_num_t pin);

/* -------------------------------------------------------------------------- */
/* Deep-sleep retained state (FW-PWR-010, DES-PWR-007)                        */
/* -------------------------------------------------------------------------- */

/** Size in bytes of the caller-usable retained payload. */
#define POWER_RETAINED_PAYLOAD_BYTES CONFIG_POWER_RETAINED_PAYLOAD_BYTES

/**
 * @brief Whether the retained block survived from a previous run intact.
 *
 * False on cold boot, after a brownout, if the CRC does not match, or before
 * anything was ever written. Decided once, in `power_init()`.
 *
 * @note Thread-safety: safe. ISR-safety: safe. Blocking: never.
 */
bool power_retained_is_valid(void);

/**
 * @brief Number of boots (deep-sleep wakes included) since the block was last
 *        written from scratch. Zero if the block is not valid.
 *
 * @note Thread-safety: safe. ISR-safety: safe. Blocking: never.
 */
uint32_t power_retained_boot_count(void);

/**
 * @brief Copy the retained payload out.
 *
 * @param[out] dst  Destination, at least `len` bytes.
 * @param[in]  len  Bytes to copy, <= POWER_RETAINED_PAYLOAD_BYTES.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `power_init()`
 *   - ESP_ERR_INVALID_ARG    NULL `dst` or `len` too large
 *   - ESP_ERR_INVALID_CRC    the block is not valid; nothing copied
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: never.
 */
esp_err_t power_retained_read(void *dst, size_t len);

/**
 * @brief Write the retained payload and seal it with magic + CRC.
 *
 * Bytes beyond `len` up to POWER_RETAINED_PAYLOAD_BYTES are zeroed. After this
 * call the block is valid and `power_retained_boot_count()` continues counting
 * across subsequent deep-sleep wakes. Call this immediately before
 * `power_enter_deep_sleep()` / `pm_policy_try_deep_sleep()`.
 *
 * @param[in] src  Source bytes. May be NULL if `len` is 0 (writes an all-zero payload).
 * @param[in] len  Bytes to copy, <= POWER_RETAINED_PAYLOAD_BYTES.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `power_init()`
 *   - ESP_ERR_INVALID_ARG    NULL `src` with `len` > 0, or `len` too large
 *
 * @note Thread-safety: not safe concurrently with itself or `power_retained_read()`.
 * @note ISR-safety: no. Blocking: never.
 */
esp_err_t power_retained_write(const void *src, size_t len);

/**
 * @brief Invalidate the retained block so the next boot sees a clean slate.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before `power_init()`
 *
 * @note Thread-safety: not safe concurrently with `power_retained_*()`.
 * @note ISR-safety: no. Blocking: never.
 */
esp_err_t power_retained_clear(void);

#ifdef __cplusplus
}
#endif
