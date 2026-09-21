#include "cfg.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "cfg";

/* Private definition behind cfg_handle_t. Static table, no heap (ADR-005). */
struct cfg_ns {
    nvs_handle_t nvs;
    bool         in_use;
    char         name[CFG_NAME_MAX + 1];
};

/* Blob header written ahead of every payload (DES-CFG-003). */
#define CFG_BLOB_MAGIC 0x42474643u /* "CFGB" */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t schema;
    uint16_t len;
} cfg_blob_hdr_t;

static struct cfg_ns      s_handles[CONFIG_CFG_MAX_HANDLES];
static SemaphoreHandle_t  s_mutex;          /* Guards the handle table only. */
static StaticSemaphore_t  s_mutex_storage;
static bool               s_initialised;

/* Scratch for blob assembly; guarded by s_blob_mutex so two tasks can't interleave. */
static uint8_t            s_blob_scratch[sizeof(cfg_blob_hdr_t) + CONFIG_CFG_MAX_BLOB_BYTES];
static SemaphoreHandle_t  s_blob_mutex;
static StaticSemaphore_t  s_blob_mutex_storage;

static bool name_ok(const char *n)
{
    if (n == NULL || n[0] == '\0') {
        return false;
    }
    return strnlen(n, CFG_NAME_MAX + 1) <= CFG_NAME_MAX;
}

static bool handle_ok(cfg_handle_t h)
{
    return h != NULL && h >= &s_handles[0] && h < &s_handles[CONFIG_CFG_MAX_HANDLES] && h->in_use;
}

/* Common preamble for every per-key operation. */
static esp_err_t check_hk(cfg_handle_t h, const char *key)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!handle_ok(h) || !name_ok(key)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t cfg_init(void)
{
    if (s_initialised) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Corrupt, full of stale pages, or from a newer NVS: configuration is lost,
         * but the device must still boot (FW-CFG-001). Say so loudly. */
        ESP_LOGW(TAG, "nvs_flash_init: %s; erasing partition and retrying", esp_err_to_name(err));
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_storage);
    s_blob_mutex = xSemaphoreCreateMutexStatic(&s_blob_mutex_storage);
    memset(s_handles, 0, sizeof(s_handles));
    s_initialised = true;

    nvs_stats_t st;
    if (nvs_get_stats(NULL, &st) == ESP_OK) {
        ESP_LOGI(TAG, "init: %u/%u entries used, %u namespaces",
                 (unsigned)st.used_entries, (unsigned)st.total_entries, (unsigned)st.namespace_count);
    } else {
        ESP_LOGI(TAG, "init");
    }
    return ESP_OK;
}

esp_err_t cfg_deinit(void)
{
    if (!s_initialised) {
        return ESP_OK;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < CONFIG_CFG_MAX_HANDLES; i++) {
        if (s_handles[i].in_use) {
            nvs_close(s_handles[i].nvs);
            s_handles[i].in_use = false;
        }
    }
    xSemaphoreGive(s_mutex);

    s_initialised = false;
    vSemaphoreDelete(s_mutex);
    vSemaphoreDelete(s_blob_mutex);
    s_mutex = NULL;
    s_blob_mutex = NULL;
    nvs_flash_deinit();
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

esp_err_t cfg_open(const char *ns, cfg_handle_t *out)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!name_ok(ns) || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct cfg_ns *slot = NULL;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < CONFIG_CFG_MAX_HANDLES; i++) {
        if (!s_handles[i].in_use) {
            slot = &s_handles[i];
            slot->in_use = true; /* Reserve under the lock; NVS open happens outside it. */
            break;
        }
    }
    xSemaphoreGive(s_mutex);

    if (slot == NULL) {
        ESP_LOGE(TAG, "no free handle for namespace '%s' (max %d); raise CONFIG_CFG_MAX_HANDLES",
                 ns, CONFIG_CFG_MAX_HANDLES);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = nvs_open(ns, NVS_READWRITE, &slot->nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open('%s') failed: %s", ns, esp_err_to_name(err));
        slot->in_use = false;
        return err;
    }
    strlcpy(slot->name, ns, sizeof(slot->name));
    *out = slot;
    ESP_LOGD(TAG, "opened '%s'", ns);
    return ESP_OK;
}

esp_err_t cfg_close(cfg_handle_t h)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (h == NULL) {
        return ESP_OK;
    }
    if (!handle_ok(h)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_close(h->nvs);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    h->in_use = false;
    h->name[0] = '\0';
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Scalars                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t cfg_get_u32(cfg_handle_t h, const char *key, uint32_t *out, uint32_t dflt)
{
    esp_err_t err = check_hk(h, key);
    if (err != ESP_OK) {
        return err;
    }
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_get_u32(h->nvs, key, out);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *out = dflt;
        esp_err_t werr = nvs_set_u32(h->nvs, key, dflt);
        if (werr == ESP_OK) {
            werr = nvs_commit(h->nvs);
        }
        if (werr != ESP_OK) {
            ESP_LOGW(TAG, "%s/%s: could not persist default: %s", h->name, key, esp_err_to_name(werr));
        }
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (err != ESP_OK) {
        *out = dflt;
    }
    return err;
}

esp_err_t cfg_set_u32(cfg_handle_t h, const char *key, uint32_t value)
{
    esp_err_t err = check_hk(h, key);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u32(h->nvs, key, value);
    return err == ESP_OK ? nvs_commit(h->nvs) : err;
}

esp_err_t cfg_get_i32(cfg_handle_t h, const char *key, int32_t *out, int32_t dflt)
{
    esp_err_t err = check_hk(h, key);
    if (err != ESP_OK) {
        return err;
    }
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_get_i32(h->nvs, key, out);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *out = dflt;
        esp_err_t werr = nvs_set_i32(h->nvs, key, dflt);
        if (werr == ESP_OK) {
            werr = nvs_commit(h->nvs);
        }
        if (werr != ESP_OK) {
            ESP_LOGW(TAG, "%s/%s: could not persist default: %s", h->name, key, esp_err_to_name(werr));
        }
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (err != ESP_OK) {
        *out = dflt;
    }
    return err;
}

esp_err_t cfg_set_i32(cfg_handle_t h, const char *key, int32_t value)
{
    esp_err_t err = check_hk(h, key);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_i32(h->nvs, key, value);
    return err == ESP_OK ? nvs_commit(h->nvs) : err;
}

/* -------------------------------------------------------------------------- */
/* Strings                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t cfg_get_str(cfg_handle_t h, const char *key, char *buf, size_t buf_len, const char *dflt)
{
    esp_err_t err = check_hk(h, key);
    if (err != ESP_OK) {
        return err;
    }
    if (buf == NULL || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dflt == NULL) {
        dflt = "";
    }

    size_t need = 0;
    err = nvs_get_str(h->nvs, key, NULL, &need);
    if (err == ESP_OK) {
        if (need > buf_len) {
            strlcpy(buf, dflt, buf_len);
            return ESP_ERR_INVALID_SIZE;
        }
        size_t len = buf_len;
        return nvs_get_str(h->nvs, key, buf, &len);
    }

    strlcpy(buf, dflt, buf_len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        esp_err_t werr = nvs_set_str(h->nvs, key, dflt);
        if (werr == ESP_OK) {
            werr = nvs_commit(h->nvs);
        }
        if (werr != ESP_OK) {
            ESP_LOGW(TAG, "%s/%s: could not persist default: %s", h->name, key, esp_err_to_name(werr));
        }
    }
    return err;
}

esp_err_t cfg_set_str(cfg_handle_t h, const char *key, const char *value)
{
    esp_err_t err = check_hk(h, key);
    if (err != ESP_OK) {
        return err;
    }
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_set_str(h->nvs, key, value);
    return err == ESP_OK ? nvs_commit(h->nvs) : err;
}

/* -------------------------------------------------------------------------- */
/* Versioned blobs                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t cfg_get_blob(cfg_handle_t h, const char *key, uint16_t schema,
                       void *buf, size_t buf_len, size_t *out_len, uint16_t *stored_schema)
{
    if (out_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_len = 0;
    esp_err_t err = check_hk(h, key);
    if (err != ESP_OK) {
        return err;
    }
    if (buf == NULL && buf_len > 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t stored = 0;
    err = nvs_get_blob(h->nvs, key, NULL, &stored);
    if (err != ESP_OK) {
        return err; /* ESP_ERR_NVS_NOT_FOUND passes straight through; no default. */
    }
    if (stored < sizeof(cfg_blob_hdr_t) || stored > sizeof(s_blob_scratch)) {
        return ESP_ERR_INVALID_CRC;
    }

    xSemaphoreTake(s_blob_mutex, portMAX_DELAY);
    size_t len = stored;
    err = nvs_get_blob(h->nvs, key, s_blob_scratch, &len);
    if (err == ESP_OK) {
        cfg_blob_hdr_t hdr;
        memcpy(&hdr, s_blob_scratch, sizeof(hdr));
        if (hdr.magic != CFG_BLOB_MAGIC || (size_t)hdr.len + sizeof(hdr) != len) {
            err = ESP_ERR_INVALID_CRC;
        } else if (hdr.schema != schema) {
            if (stored_schema != NULL) {
                *stored_schema = hdr.schema;
            }
            err = ESP_ERR_INVALID_VERSION;
        } else if (hdr.len > buf_len) {
            err = ESP_ERR_INVALID_SIZE;
        } else {
            memcpy(buf, s_blob_scratch + sizeof(hdr), hdr.len);
            *out_len = hdr.len;
        }
    }
    xSemaphoreGive(s_blob_mutex);
    return err;
}

esp_err_t cfg_set_blob(cfg_handle_t h, const char *key, uint16_t schema, const void *buf, size_t len)
{
    esp_err_t err = check_hk(h, key);
    if (err != ESP_OK) {
        return err;
    }
    if (buf == NULL && len > 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > CONFIG_CFG_MAX_BLOB_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }

    cfg_blob_hdr_t hdr = { .magic = CFG_BLOB_MAGIC, .schema = schema, .len = (uint16_t)len };

    xSemaphoreTake(s_blob_mutex, portMAX_DELAY);
    memcpy(s_blob_scratch, &hdr, sizeof(hdr));
    if (len > 0) {
        memcpy(s_blob_scratch + sizeof(hdr), buf, len);
    }
    err = nvs_set_blob(h->nvs, key, s_blob_scratch, sizeof(hdr) + len);
    if (err == ESP_OK) {
        err = nvs_commit(h->nvs);
    }
    xSemaphoreGive(s_blob_mutex);
    return err;
}

/* -------------------------------------------------------------------------- */
/* Maintenance                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t cfg_erase_key(cfg_handle_t h, const char *key)
{
    esp_err_t err = check_hk(h, key);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(h->nvs, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    return err == ESP_OK ? nvs_commit(h->nvs) : err;
}

esp_err_t cfg_erase_all(cfg_handle_t h)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!handle_ok(h)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = nvs_erase_all(h->nvs);
    return err == ESP_OK ? nvs_commit(h->nvs) : err;
}

esp_err_t cfg_commit(cfg_handle_t h)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!handle_ok(h)) {
        return ESP_ERR_INVALID_ARG;
    }
    return nvs_commit(h->nvs);
}

bool cfg_exists(cfg_handle_t h, const char *key)
{
    if (check_hk(h, key) != ESP_OK) {
        return false;
    }
    nvs_type_t type;
    return nvs_find_key(h->nvs, key, &type) == ESP_OK;
}
