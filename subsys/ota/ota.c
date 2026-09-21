#include "ota.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "pm_policy.h"

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

static const char *TAG = "ota";

#define OTA_URL_MAX 256

typedef enum { ST_UNINIT = 0, ST_IDLE, ST_RUNNING } state_t;

static state_t                 s_state;
static ota_config_t            s_cfg;
static pm_policy_lock_handle_t s_lock;
static const esp_partition_t  *s_running;
static const esp_partition_t  *s_next;
static bool                    s_pending_verify;
static char                    s_url[OTA_URL_MAX];
static TaskHandle_t            s_task;
static volatile bool           s_abort_requested;
static volatile size_t         s_bytes_read;
static volatile int            s_image_size;

static void emit(ota_event_t evt, ota_event_info_t *info)
{
    if (s_cfg.cb != NULL) {
        s_cfg.cb(evt, info, s_cfg.ctx);
    }
}

/* -------------------------------------------------------------------------- */
/* Worker task (DES-OTA-002, DES-OTA-004, DES-OTA-008)                         */
/* -------------------------------------------------------------------------- */

static void ota_task(void *arg)
{
    ota_event_info_t info = { .image_size = -1 };
    esp_https_ota_handle_t h = NULL;
    esp_err_t err;

    esp_http_client_config_t http = {
        .url = s_url,
        .timeout_ms = CONFIG_OTA_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
        .buffer_size = CONFIG_OTA_RECV_BUFFER_SIZE,
        .buffer_size_tx = 1024,
    };
    if (s_cfg.cert_pem != NULL) {
        http.cert_pem = s_cfg.cert_pem;
    }
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
    else if (s_cfg.use_cert_bundle) {
        http.crt_bundle_attach = esp_crt_bundle_attach;
    }
#endif

    esp_https_ota_config_t ota_cfg = {
        .http_config = &http,
        .buffer_caps = MALLOC_CAP_DEFAULT,
    };

    ESP_LOGI(TAG, "session: %s -> %s", s_url, s_next->label);

    err = esp_https_ota_begin(&ota_cfg, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_begin: %s", esp_err_to_name(err));
        goto failed;
    }

    /* Validate the image header before writing anything beyond it (FW-OTA-004). */
    esp_app_desc_t new_desc;
    err = esp_https_ota_get_img_desc(h, &new_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "image header unreadable: %s", esp_err_to_name(err));
        goto failed_abort;
    }
    {
        const esp_app_desc_t *cur = esp_app_get_description();
        strlcpy(info.new_version, new_desc.version, sizeof(info.new_version));
        if (strncmp(new_desc.project_name, cur->project_name, sizeof(cur->project_name)) != 0) {
            ESP_LOGE(TAG, "image is project '%s', running '%s': refusing", new_desc.project_name, cur->project_name);
            err = ESP_ERR_INVALID_VERSION;
            goto failed_abort;
        }
        if (s_cfg.reject_same_version &&
            strncmp(new_desc.version, cur->version, sizeof(cur->version)) == 0) {
            ESP_LOGW(TAG, "image version '%s' equals running version: refusing", new_desc.version);
            err = ESP_ERR_INVALID_VERSION;
            goto failed_abort;
        }
        ESP_LOGI(TAG, "image: project '%s' version '%s' (running '%s')",
                 new_desc.project_name, new_desc.version, cur->version);
    }

    s_image_size = esp_https_ota_get_image_size(h);
    info.image_size = s_image_size;
    emit(OTA_EVT_STARTED, &info);

    size_t last_reported = 0;
    for (;;) {
        if (s_abort_requested) {
            ESP_LOGW(TAG, "abort requested at %u bytes", (unsigned)s_bytes_read);
            esp_https_ota_abort(h);
            h = NULL;
            info.bytes_read = s_bytes_read;
            emit(OTA_EVT_ABORTED, &info);
            goto done;
        }
        err = esp_https_ota_perform(h);
        s_bytes_read = (size_t)esp_https_ota_get_image_len_read(h);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
        if (s_bytes_read - last_reported >= CONFIG_OTA_PROGRESS_INTERVAL_BYTES) {
            last_reported = s_bytes_read;
            info.bytes_read = s_bytes_read;
            emit(OTA_EVT_PROGRESS, &info);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "download failed at %u bytes: %s", (unsigned)s_bytes_read, esp_err_to_name(err));
        goto failed_abort;
    }
    if (!esp_https_ota_is_complete_data_received(h)) {
        ESP_LOGE(TAG, "connection closed before the image was complete");
        err = ESP_ERR_INVALID_SIZE;
        goto failed_abort;
    }

    info.bytes_read = s_bytes_read;
    emit(OTA_EVT_VERIFIED, &info);

    /* Verifies the image and switches otadata atomically (DES-OTA-006). */
    err = esp_https_ota_finish(h);
    h = NULL;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_finish: %s", esp_err_to_name(err));
        goto failed;
    }
    ESP_LOGI(TAG, "committed %u bytes to %s; reboot to activate", (unsigned)s_bytes_read, s_next->label);
    emit(OTA_EVT_COMMITTED, &info);
    goto done;

failed_abort:
    if (h != NULL) {
        esp_https_ota_abort(h); /* Leaves the target slot without a valid header. */
        h = NULL;
    }
failed:
    info.err = err;
    info.bytes_read = s_bytes_read;
    emit(OTA_EVT_FAILED, &info);

done:
    pm_policy_lock_release(s_lock);
    s_state = ST_IDLE;
    s_task = NULL;
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t ota_init(const ota_config_t *cfg)
{
    if (s_state != ST_UNINIT) {
        return ESP_OK;
    }
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->cert_pem == NULL && !cfg->use_cert_bundle) {
        ESP_LOGE(TAG, "no trust anchor: set cert_pem or use_cert_bundle (SYS-SEC-003)");
        return ESP_ERR_INVALID_ARG;
    }
#if !CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
    if (cfg->cert_pem == NULL && cfg->use_cert_bundle) {
        ESP_LOGE(TAG, "use_cert_bundle requires CONFIG_MBEDTLS_CERTIFICATE_BUNDLE");
        return ESP_ERR_INVALID_ARG;
    }
#endif

    /* Dual-slot layout is a hard precondition (DES-OTA-001, ADR-008). */
    s_running = esp_ota_get_running_partition();
    s_next = esp_ota_get_next_update_partition(NULL);
    if (s_running == NULL || s_next == NULL || s_next == s_running) {
        ESP_LOGE(TAG, "partition table has no second application slot; OTA impossible (see ADR-008)");
        return ESP_ERR_INVALID_STATE;
    }

    esp_ota_img_states_t st;
    s_pending_verify = (esp_ota_get_state_partition(s_running, &st) == ESP_OK) &&
                       (st == ESP_OTA_IMG_PENDING_VERIFY);

    esp_err_t err = pm_policy_lock_create("ota", &s_lock);
    if (err != ESP_OK) {
        return err;
    }

    s_cfg = *cfg;
    s_state = ST_IDLE;
    s_abort_requested = false;
    s_bytes_read = 0;
    s_image_size = -1;

    const esp_app_desc_t *cur = esp_app_get_description();
    ESP_LOGI(TAG, "init: running %s ('%s' v%s)%s, next slot %s",
             s_running->label, cur->project_name, cur->version,
             s_pending_verify ? " PENDING VERIFY - call ota_mark_valid()" : "", s_next->label);
    return ESP_OK;
}

esp_err_t ota_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    if (s_state == ST_RUNNING) {
        ota_abort();
        /* Wait for the worker to wind down; it releases the lock and resets state. */
        for (int i = 0; i < 200 && s_state == ST_RUNNING; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_state = ST_UNINIT;
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

esp_err_t ota_start(const char *url)
{
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state == ST_RUNNING) {
        return ESP_ERR_INVALID_STATE; /* FW-OTA-013 */
    }
    if (url == NULL || url[0] == '\0' || strlen(url) >= OTA_URL_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
#if !CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP
    if (strncmp(url, "https://", 8) != 0) {
        ESP_LOGE(TAG, "plaintext HTTP refused (ADR-016); use https://");
        return ESP_ERR_INVALID_ARG;
    }
#endif

    /* lwIP asserts ("Invalid mbox") if a socket is opened before esp_netif_init()
     * has started the TCP/IP thread. Refuse here instead of crashing there. Whether
     * the interface is actually connected is left to the normal FAILED path. */
    if (esp_netif_get_nr_of_ifs() == 0) {
        ESP_LOGE(TAG, "no network interface exists; initialise wifi_link (or another netif) first");
        return ESP_ERR_INVALID_STATE;
    }

    strlcpy(s_url, url, sizeof(s_url));
    s_abort_requested = false;
    s_bytes_read = 0;
    s_image_size = -1;
    s_state = ST_RUNNING;
    pm_policy_lock_acquire(s_lock);

    BaseType_t ok = xTaskCreatePinnedToCore(ota_task, "ota", CONFIG_OTA_TASK_STACK, NULL,
                                            CONFIG_OTA_TASK_PRIORITY, &s_task, CONFIG_OTA_TASK_CORE);
    if (ok != pdPASS) {
        pm_policy_lock_release(s_lock);
        s_state = ST_IDLE;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ota_abort(void)
{
    if (s_state != ST_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }
    s_abort_requested = true;
    return ESP_OK;
}

bool ota_in_progress(void)
{
    return s_state == ST_RUNNING;
}

/* -------------------------------------------------------------------------- */
/* Rollback control (DES-OTA-007)                                              */
/* -------------------------------------------------------------------------- */

esp_err_t ota_mark_valid(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_pending_verify) {
        return ESP_OK;
    }
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        s_pending_verify = false;
        ESP_LOGI(TAG, "running image %s marked valid", s_running->label);
    } else {
        ESP_LOGE(TAG, "mark valid: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ota_mark_invalid(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    /* ESP-IDF's only "mark invalid" API also reboots, which this SDK never does on
     * its own (DES-OTA-006). Selecting the other slot for the next boot has the same
     * effect for the application: the previous image runs after the next reset. */
    if (!esp_ota_check_rollback_is_possible()) {
        ESP_LOGE(TAG, "no valid previous image to roll back to");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = esp_ota_set_boot_partition(s_next);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "next boot will run %s instead of %s; reboot when ready", s_next->label, s_running->label);
    } else {
        ESP_LOGE(TAG, "set boot partition: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ota_status(ota_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(out, 0, sizeof(*out));
    strlcpy(out->running_label, s_running->label, sizeof(out->running_label));
    strlcpy(out->next_label, s_next->label, sizeof(out->next_label));
    strlcpy(out->running_version, esp_app_get_description()->version, sizeof(out->running_version));
    out->pending_verify = s_pending_verify;
    out->in_progress = (s_state == ST_RUNNING);
    out->bytes_read = s_bytes_read;
    out->image_size = s_image_size;
    return ESP_OK;
}
