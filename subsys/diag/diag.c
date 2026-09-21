#include "diag.h"
#include <inttypes.h>
#include <string.h>
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"
#include "cfg.h"

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#include "esp_core_dump.h"
#endif

static const char *TAG = "diag";

/* NVS keys in the "diag" namespace. */
#define KEY_BOOT_COUNT  "boot_count"
#define KEY_FAULT_COUNT "fault_count"
#define KEY_LAST_FAULT  "last_fault"

static bool              s_initialised;
static diag_state_t      s_state = DIAG_STATE_BOOT;
static diag_config_t     s_cfg;
static cfg_handle_t      s_ns;              /* NULL when cfg is unavailable. */
static uint32_t          s_boot_count;
static uint32_t          s_fault_count;
static uint32_t          s_last_fault_code;
static uint32_t          s_current_fault_code;
static SemaphoreHandle_t s_mutex;
static StaticSemaphore_t s_mutex_storage;

/* Tasks subscribed through this module, so deinit can unsubscribe exactly those. */
static TaskHandle_t s_wdt_tasks[CONFIG_DIAG_MAX_WDT_TASKS];
static uint8_t      s_wdt_task_count;

static uint32_t wakeup_causes_normalised(void)
{
    return esp_sleep_get_wakeup_causes() & ~(1u << ESP_SLEEP_WAKEUP_UNDEFINED);
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t diag_init(const diag_config_t *cfg)
{
    if (s_initialised) {
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_storage);
    memset(&s_cfg, 0, sizeof(s_cfg));
    if (cfg != NULL) {
        s_cfg = *cfg;
    }
    memset(s_wdt_tasks, 0, sizeof(s_wdt_tasks));
    s_wdt_task_count = 0;
    s_current_fault_code = 0;

    /* Persistence is best-effort: diag must work even if cfg was never initialised. */
    s_ns = NULL;
    if (cfg_open("diag", &s_ns) == ESP_OK) {
        cfg_get_u32(s_ns, KEY_BOOT_COUNT, &s_boot_count, 0);
        cfg_get_u32(s_ns, KEY_FAULT_COUNT, &s_fault_count, 0);
        cfg_get_u32(s_ns, KEY_LAST_FAULT, &s_last_fault_code, 0);
        s_boot_count++;
        esp_err_t err = cfg_set_u32(s_ns, KEY_BOOT_COUNT, s_boot_count);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "could not persist boot count: %s", esp_err_to_name(err));
        }
    } else {
        s_boot_count = 0;
        s_fault_count = 0;
        s_last_fault_code = 0;
        ESP_LOGW(TAG, "cfg unavailable; boot/fault counters will not persist");
    }

    s_state = DIAG_STATE_BOOT;
    s_initialised = true;

    ESP_LOGI(TAG, "init: reset=%d wake=0x%08" PRIx32 " boot#%" PRIu32 " faults=%" PRIu32
                  " last_fault=0x%08" PRIx32,
             (int)esp_reset_reason(), wakeup_causes_normalised(),
             s_boot_count, s_fault_count, s_last_fault_code);

    /* A previous crash left a coredump behind: say so at boot, once. */
    bool cd = false;
    if (diag_coredump_present(&cd) == ESP_OK && cd) {
        ESP_LOGW(TAG, "coredump present from a previous crash; retrieve then diag_coredump_erase()");
    }
    return ESP_OK;
}

esp_err_t diag_deinit(void)
{
    if (!s_initialised) {
        return ESP_OK;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
#if CONFIG_ESP_TASK_WDT_EN
    for (uint8_t i = 0; i < s_wdt_task_count; i++) {
        esp_task_wdt_delete(s_wdt_tasks[i]);
    }
#endif
    s_wdt_task_count = 0;
    xSemaphoreGive(s_mutex);

    if (s_ns != NULL) {
        cfg_close(s_ns);
        s_ns = NULL;
    }
    s_initialised = false;
    s_state = DIAG_STATE_BOOT;
    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

esp_err_t diag_mark_running(void)
{
    if (!s_initialised || s_state == DIAG_STATE_FAULT) {
        return ESP_ERR_INVALID_STATE;
    }
    s_state = DIAG_STATE_RUNNING;
    ESP_LOGI(TAG, "state: RUNNING");
    return ESP_OK;
}

diag_state_t diag_state(void)
{
    return s_state;
}

/* -------------------------------------------------------------------------- */
/* Task watchdog                                                               */
/* -------------------------------------------------------------------------- */

#if CONFIG_ESP_TASK_WDT_EN
static esp_err_t ensure_wdt_initialised(void)
{
    /* esp_task_wdt_add() returns INVALID_STATE if the WDT was never initialised
     * (CONFIG_ESP_TASK_WDT_INIT off). Initialise it ourselves in that case. */
#if CONFIG_ESP_TASK_WDT_INIT
    const uint32_t timeout_s = CONFIG_ESP_TASK_WDT_TIMEOUT_S;
#else
    const uint32_t timeout_s = CONFIG_DIAG_WDT_TIMEOUT_S;
#endif
    /* esp_task_wdt_status() answers ESP_ERR_INVALID_STATE only when the TWDT has
     * never been initialised; probing it first avoids the ERROR line that
     * esp_task_wdt_init() itself logs when called a second time. */
    if (esp_task_wdt_status(NULL) != ESP_ERR_INVALID_STATE) {
        return ESP_OK; /* Already initialised (by the system or by us). */
    }
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = timeout_s * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    return esp_task_wdt_init(&wdt_cfg);
}
#endif

esp_err_t diag_task_register(TaskHandle_t task)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
#if !CONFIG_ESP_TASK_WDT_EN
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (task == NULL) {
        task = xTaskGetCurrentTaskHandle();
    }
    esp_err_t err = ensure_wdt_initialised();
    if (err != ESP_OK) {
        return err;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_wdt_task_count >= CONFIG_DIAG_MAX_WDT_TASKS) {
        xSemaphoreGive(s_mutex);
        ESP_LOGE(TAG, "wdt task table full (max %d); raise CONFIG_DIAG_MAX_WDT_TASKS",
                 CONFIG_DIAG_MAX_WDT_TASKS);
        return ESP_ERR_NO_MEM;
    }
    err = esp_task_wdt_add(task);
    if (err == ESP_OK) {
        s_wdt_tasks[s_wdt_task_count++] = task;
        ESP_LOGD(TAG, "wdt: subscribed '%s'", pcTaskGetName(task));
    }
    xSemaphoreGive(s_mutex);
    return err;
#endif
}

esp_err_t diag_task_unregister(TaskHandle_t task)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
#if !CONFIG_ESP_TASK_WDT_EN
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (task == NULL) {
        task = xTaskGetCurrentTaskHandle();
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = esp_task_wdt_delete(task);
    if (err == ESP_OK) {
        for (uint8_t i = 0; i < s_wdt_task_count; i++) {
            if (s_wdt_tasks[i] == task) {
                s_wdt_tasks[i] = s_wdt_tasks[--s_wdt_task_count];
                break;
            }
        }
    }
    xSemaphoreGive(s_mutex);
    return err;
#endif
}

esp_err_t diag_task_feed(void)
{
#if !CONFIG_ESP_TASK_WDT_EN
    return ESP_ERR_NOT_SUPPORTED;
#else
    return esp_task_wdt_reset();
#endif
}

/* -------------------------------------------------------------------------- */
/* Health and faults                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t diag_health(diag_health_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->state = s_state;
    out->uptime_us = (uint64_t)esp_timer_get_time();
    out->reset_reason = esp_reset_reason();
    out->wakeup_causes = wakeup_causes_normalised();
    out->boot_count = s_boot_count;
    out->fault_count = s_fault_count;
    out->last_fault_code = s_last_fault_code;
    out->current_fault_code = s_current_fault_code;
    out->free_heap = esp_get_free_heap_size();
    out->min_free_heap_ever = esp_get_minimum_free_heap_size();
    out->free_internal_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    out->largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    out->wdt_tasks = s_wdt_task_count;
    bool cd = false;
    if (diag_coredump_present(&cd) == ESP_OK) {
        out->coredump_present = cd;
    }
    return ESP_OK;
}

void diag_fault(uint32_t code, const char *detail)
{
    if (code == 0) {
        code = 0xFFFFFFFFu; /* Zero is "no fault"; never let it through as one. */
    }
    if (detail == NULL) {
        detail = "";
    }

    bool first = false;
    if (s_initialised) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
    if (s_state != DIAG_STATE_FAULT) {
        s_state = DIAG_STATE_FAULT;
        s_current_fault_code = code;
        first = true;
    }
    s_fault_count++;
    s_last_fault_code = code;
    if (s_initialised) {
        xSemaphoreGive(s_mutex);
    }

    ESP_LOGE(TAG, "%sFAULT 0x%08" PRIx32 ": %s (fault #%" PRIu32 ")",
             first ? "" : "additional ", code, detail, s_fault_count);

    if (s_ns != NULL) {
        cfg_set_u32(s_ns, KEY_FAULT_COUNT, s_fault_count);
        cfg_set_u32(s_ns, KEY_LAST_FAULT, code);
    }

    if (first && s_cfg.fault_cb != NULL) {
        s_cfg.fault_cb(code, detail, s_cfg.fault_ctx);
    }
}

esp_err_t diag_set_log_level(const char *tag, esp_log_level_t level)
{
    if (tag == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_log_level_set(tag, level);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Coredump                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t diag_coredump_present(bool *present)
{
    if (present == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    *present = (esp_core_dump_image_check() == ESP_OK);
    return ESP_OK;
#else
    *present = false;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t diag_coredump_erase(void)
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    esp_err_t err = esp_core_dump_image_erase();
    return (err == ESP_ERR_NOT_FOUND) ? ESP_OK : err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
