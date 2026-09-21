#include "pm_policy.h"
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "power.h"

/* Private definition behind pm_policy_lock_handle_t (DES-PMP-002). */
struct pm_policy_lock {
    const char *name;    /* Borrowed; static lifetime by contract (FW-PMP-020). */
    uint32_t    count;   /* Saturates at 0 on release underflow (DES-PMP-004).  */
    bool        in_use;  /* Set once, never cleared (DES-PMP-005).              */
};

static const char *TAG = "pm_policy";

/* Static table in BSS; no heap (ADR-005). Zero-initialised means "all free". */
static struct pm_policy_lock s_locks[CONFIG_PM_POLICY_MAX_LOCKS];
static SemaphoreHandle_t     s_mutex;
static StaticSemaphore_t     s_mutex_storage;
static bool                  s_initialised;

/* Every entry point after init goes through these. */
static inline void table_lock(void)   { xSemaphoreTake(s_mutex, portMAX_DELAY); }
static inline void table_unlock(void) { xSemaphoreGive(s_mutex); }

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t pm_policy_init(void)
{
    if (s_initialised) {
        return ESP_OK; /* FW-PMP-010: idempotent, table preserved. */
    }

    /* Static mutex: no heap, cannot fail for lack of memory. The ESP_ERR_NO_MEM
     * return documented in the header is retained for API stability should the
     * allocation strategy ever change. */
    s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_storage);
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_locks, 0, sizeof(s_locks));
    s_initialised = true;
    ESP_LOGI(TAG, "init: %d lock slots", CONFIG_PM_POLICY_MAX_LOCKS);
    return ESP_OK;
}

esp_err_t pm_policy_deinit(void)
{
    if (!s_initialised) {
        return ESP_OK;
    }
    s_initialised = false;
    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;
    memset(s_locks, 0, sizeof(s_locks));
    ESP_LOGI(TAG, "deinit: all handles invalidated");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Locks                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t pm_policy_lock_create(const char *name, pm_policy_lock_handle_t *out_handle)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (name == NULL || name[0] == '\0' || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_ERR_NO_MEM;
    struct pm_policy_lock *free_slot = NULL;

    table_lock();
    for (int i = 0; i < CONFIG_PM_POLICY_MAX_LOCKS; i++) {
        struct pm_policy_lock *l = &s_locks[i];
        if (l->in_use) {
            if (strcmp(l->name, name) == 0) {
                /* FW-PMP-012: same name returns the same lock. */
                *out_handle = l;
                err = ESP_OK;
                break;
            }
        } else if (free_slot == NULL) {
            free_slot = l;
        }
    }
    if (err != ESP_OK && free_slot != NULL) {
        free_slot->in_use = true;
        free_slot->name = name;
        free_slot->count = 0;
        *out_handle = free_slot;
        err = ESP_OK;
        ESP_LOGD(TAG, "created lock '%s'", name);
    }
    table_unlock();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no free lock slot for '%s' (max %d); raise CONFIG_PM_POLICY_MAX_LOCKS",
                 name, CONFIG_PM_POLICY_MAX_LOCKS);
    }
    return err;
}

esp_err_t pm_policy_lock_acquire(pm_policy_lock_handle_t handle)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    table_lock();
    handle->count++;
    ESP_LOGD(TAG, "acquire '%s' -> %" PRIu32, handle->name, handle->count);
    table_unlock();
    return ESP_OK;
}

esp_err_t pm_policy_lock_release(pm_policy_lock_handle_t handle)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    table_lock();
    if (handle->count == 0) {
        /* Caller bug. Saturate rather than wrap (DES-PMP-004). */
        ESP_LOGW(TAG, "release '%s' with count already 0 (unbalanced release)", handle->name);
    } else {
        handle->count--;
    }
    ESP_LOGD(TAG, "release '%s' -> %" PRIu32, handle->name, handle->count);
    table_unlock();
    return ESP_OK;
}

uint32_t pm_policy_lock_count(pm_policy_lock_handle_t handle)
{
    if (!s_initialised || handle == NULL) {
        return 0;
    }
    table_lock();
    uint32_t c = handle->count;
    table_unlock();
    return c;
}

/* -------------------------------------------------------------------------- */
/* Arbitration                                                                 */
/* -------------------------------------------------------------------------- */

bool pm_policy_can_deep_sleep(void)
{
    if (!s_initialised) {
        return false;
    }
    bool can_sleep = true;
    table_lock();
    for (int i = 0; i < CONFIG_PM_POLICY_MAX_LOCKS; i++) {
        if (s_locks[i].in_use && s_locks[i].count > 0) {
            can_sleep = false; /* Yes/no only, so the first holder settles it. */
            break;
        }
    }
    table_unlock();
    return can_sleep;
}

esp_err_t pm_policy_try_deep_sleep(uint64_t sleep_us)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }

    bool blocked = false;
    table_lock();
    /* Deliberately no early exit: log every holder, not just the first (DES-PMP-007). */
    for (int i = 0; i < CONFIG_PM_POLICY_MAX_LOCKS; i++) {
        if (s_locks[i].in_use && s_locks[i].count > 0) {
            ESP_LOGI(TAG, "deep sleep blocked by lock '%s' (count=%" PRIu32 ")",
                     s_locks[i].name, s_locks[i].count);
            blocked = true;
        }
    }
    if (blocked) {
        table_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    /* Hold the mutex across sleep entry so no acquire can slip in between the scan
     * and the point where the CPU stops. Deep sleep is a reset; the mutex is never
     * released and does not need to be (DES-PMP-008). */
    power_enter_deep_sleep(sleep_us);
}

/* -------------------------------------------------------------------------- */
/* Diagnostics                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t pm_policy_holders(pm_policy_holder_t *out, size_t max, size_t *count)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (count == NULL || (out == NULL && max > 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t n = 0;
    table_lock();
    for (int i = 0; i < CONFIG_PM_POLICY_MAX_LOCKS; i++) {
        if (!s_locks[i].in_use) {
            continue;
        }
        if (n < max) {
            out[n].name = s_locks[i].name;
            out[n].count = s_locks[i].count;
        }
        n++;
    }
    table_unlock();
    *count = n;
    return ESP_OK;
}
