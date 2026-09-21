#include "usb_hid.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "tusb.h"
#include "class/hid/hid_device.h"
#include "usb_device.h"

static const char *TAG = "usb_hid";

/* One TinyUSB HID instance: this module is a singleton bound to instance 0. */
#define HID_ITF 0

#if CFG_TUD_HID < 1
#error "usb_hid requires CONFIG_TINYUSB_HID_COUNT >= 1"
#endif

/* Fixed-size queue slot: no per-report allocation (DES-HID-002). */
typedef struct {
    uint8_t id;
    uint8_t len;
    uint8_t data[USB_HID_EP_SIZE_MAX];
} hid_slot_t;

static bool             s_initialised;
static usb_hid_config_t s_cfg;
static uint8_t          s_ep_size;
static uint8_t          s_poll_ms;

static QueueHandle_t    s_queue;
static StaticQueue_t    s_queue_ctrl;
static hid_slot_t       s_queue_storage[CONFIG_USB_HID_REPORT_QUEUE_LEN];
static SemaphoreHandle_t s_kick_mutex;   /* Serialises "arm next report" between tasks. */
static StaticSemaphore_t s_kick_mutex_storage;

static bool             s_mounted;
static usb_hid_stats_t  s_stats;

/* -------------------------------------------------------------------------- */
/* Endpoint servicing                                                          */
/* -------------------------------------------------------------------------- */

/*
 * Arm the endpoint with the oldest queued report if it is idle. Called from the
 * sending task and from the TinyUSB completion callback; the mutex makes the two
 * paths mutually exclusive so a report is never armed twice or skipped.
 */
static void kick(void)
{
    if (!s_mounted || s_kick_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_kick_mutex, portMAX_DELAY);
    hid_slot_t slot;
    if (tud_hid_n_ready(HID_ITF) && xQueuePeek(s_queue, &slot, 0) == pdTRUE) {
        if (tud_hid_n_report(HID_ITF, slot.id, slot.data, slot.len)) {
            xQueueReceive(s_queue, &slot, 0); /* Consume only once the stack took it. */
        }
    }
    xSemaphoreGive(s_kick_mutex);
}

/* -------------------------------------------------------------------------- */
/* usb_function_t contract (registered with usb_device)                        */
/* -------------------------------------------------------------------------- */

static usb_ep_budget_t hid_endpoint_request(void *ctx)
{
    return (usb_ep_budget_t) {
        .in_endpoints = 1,
        .out_endpoints = 0,
        .tx_fifo_bytes = s_ep_size,
    };
}

static size_t hid_descriptor_len(void *ctx)
{
    return TUD_HID_DESC_LEN;
}

static size_t hid_descriptor_write(void *ctx, uint8_t *buf, uint8_t itf_base,
                                   const uint8_t *ep_in, const uint8_t *ep_out)
{
    const uint8_t desc[] = {
        TUD_HID_DESCRIPTOR(itf_base, /*stridx*/ 0, (uint8_t)s_cfg.boot_protocol,
                           (uint16_t)s_cfg.report_descriptor_len,
                           ep_in[0], s_ep_size, s_poll_ms)
    };
    memcpy(buf, desc, sizeof(desc));
    return sizeof(desc);
}

static void hid_on_mount(void *ctx)
{
    s_mounted = true;
    kick(); /* Anything queued while unmounted goes out now. */
}

static void hid_on_unmount(void *ctx)
{
    /* Discard in-flight state: reports queued for a host that is gone are stale
     * by definition (FW-USB-006). */
    s_mounted = false;
    xQueueReset(s_queue);
    s_stats.queue_depth = 0;
}

static const usb_function_t s_function = {
    .name             = "hid",
    .interface_count  = 1,
    .endpoint_request = hid_endpoint_request,
    .descriptor_len   = hid_descriptor_len,
    .descriptor_write = hid_descriptor_write,
    .on_mount         = hid_on_mount,
    .on_unmount       = hid_on_unmount,
    .on_suspend       = NULL,
    .on_resume        = NULL,
    .ctx              = NULL,
};

/* -------------------------------------------------------------------------- */
/* TinyUSB class callbacks — USB device task context                           */
/* -------------------------------------------------------------------------- */

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    return (instance == HID_ITF && s_initialised) ? s_cfg.report_descriptor : NULL;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    if (instance != HID_ITF || !s_initialised || s_cfg.on_get_report == NULL) {
        return 0; /* Stall: nothing to say (FW-HID-007). */
    }
    return s_cfg.on_get_report(report_id, (usb_hid_report_type_t)report_type, buffer, reqlen, s_cfg.ctx);
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    if (instance != HID_ITF || !s_initialised) {
        return;
    }
    s_stats.output_reports++;
    if (s_cfg.on_output_report != NULL) {
        /* Already task context: TinyUSB invokes this from tud_task() (FW-HID-008). */
        s_cfg.on_output_report(report_id, (usb_hid_report_type_t)report_type, buffer, bufsize, s_cfg.ctx);
    }
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len)
{
    if (instance != HID_ITF) {
        return;
    }
    s_stats.reports_sent++;
    kick(); /* Arm the next one, if any (DES-HID-004: nothing queued → NAK). */
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t usb_hid_init(const usb_hid_config_t *cfg)
{
    if (s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || cfg->report_descriptor == NULL || cfg->report_descriptor_len == 0 ||
        cfg->ep_size > USB_HID_EP_SIZE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *cfg;
    s_ep_size = cfg->ep_size ? cfg->ep_size : CONFIG_USB_HID_EP_SIZE;
    s_poll_ms = cfg->poll_interval_ms ? cfg->poll_interval_ms : CONFIG_USB_HID_POLL_INTERVAL_MS;
    if (s_poll_ms == 0) {
        return ESP_ERR_INVALID_ARG; /* bInterval 0 is invalid on a full-speed interrupt EP. */
    }

    s_queue = xQueueCreateStatic(CONFIG_USB_HID_REPORT_QUEUE_LEN, sizeof(hid_slot_t),
                                 (uint8_t *)s_queue_storage, &s_queue_ctrl);
    s_kick_mutex = xSemaphoreCreateMutexStatic(&s_kick_mutex_storage);
    memset(&s_stats, 0, sizeof(s_stats));
    s_mounted = false;

    esp_err_t err = usb_device_register(&s_function);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_kick_mutex);
        s_kick_mutex = NULL;
        vQueueDelete(s_queue);
        s_queue = NULL;
        return err;
    }

    s_initialised = true;
    ESP_LOGI(TAG, "init: report descriptor %u B, ep %u B, bInterval %u ms, queue %d",
             (unsigned)cfg->report_descriptor_len, s_ep_size, s_poll_ms, CONFIG_USB_HID_REPORT_QUEUE_LEN);
    return ESP_OK;
}

esp_err_t usb_hid_deinit(void)
{
    if (!s_initialised) {
        return ESP_OK;
    }
    s_initialised = false;
    s_mounted = false;
    vSemaphoreDelete(s_kick_mutex);
    s_kick_mutex = NULL;
    vQueueDelete(s_queue);
    s_queue = NULL;
    memset(&s_cfg, 0, sizeof(s_cfg));
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

esp_err_t usb_hid_report_send(uint8_t report_id, const void *data, size_t len)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    /* A non-zero report ID occupies the first byte on the wire. */
    size_t wire = len + (report_id ? 1u : 0u);
    if (wire > s_ep_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (!usb_hid_ready()) {
        s_stats.reports_rejected++;
        return ESP_ERR_INVALID_STATE;
    }

    hid_slot_t slot = { .id = report_id, .len = (uint8_t)len };
    memcpy(slot.data, data, len);

    if (xQueueSend(s_queue, &slot, 0) != pdTRUE) {
        s_stats.reports_dropped++;
        return ESP_ERR_NO_MEM;
    }
    UBaseType_t depth = uxQueueMessagesWaiting(s_queue);
    s_stats.queue_depth = (uint8_t)depth;
    if (depth > s_stats.queue_high_water) {
        s_stats.queue_high_water = (uint8_t)depth;
    }

    kick();
    return ESP_OK;
}

bool usb_hid_ready(void)
{
    /* "Ready" means the host has configured us, not that the endpoint is idle:
     * a report may be queued while another is in flight. Endpoint idleness is
     * kick()'s concern via tud_hid_n_ready(). */
    return s_initialised && s_mounted && tud_mounted();
}

esp_err_t usb_hid_stats(usb_hid_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    if (s_queue != NULL) {
        out->queue_depth = (uint8_t)uxQueueMessagesWaiting(s_queue);
    }
    return ESP_OK;
}

esp_err_t usb_hid_stats_reset(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    return ESP_OK;
}
