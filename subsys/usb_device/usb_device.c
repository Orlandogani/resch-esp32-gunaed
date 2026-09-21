#include "usb_device.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "pm_policy.h"

static const char *TAG = "usb_device";

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

typedef enum { ST_UNINIT = 0, ST_INIT, ST_STARTED } state_t;

static state_t                 s_state;
static usb_device_config_t     s_cfg;
static const usb_function_t   *s_functions[CONFIG_USB_DEVICE_MAX_FUNCTIONS];
static uint8_t                 s_function_count;
static usb_ep_budget_t         s_used;                 /* Claimed by registered functions. */
static uint8_t                 s_interfaces_used;
static usb_device_event_cb_t   s_event_cb;
static void                   *s_event_ctx;
static pm_policy_lock_handle_t s_lock;
static bool                    s_lock_held;
static bool                    s_mounted;
static SemaphoreHandle_t       s_mutex;
static StaticSemaphore_t       s_mutex_storage;

/* Descriptor storage. Frozen by usb_device_start(); TinyUSB reads it by pointer. */
static tusb_desc_device_t s_dev_desc;
static uint8_t            s_cfg_desc[CONFIG_USB_DEVICE_CONFIG_DESC_MAX_BYTES];
static size_t             s_cfg_desc_len;
static char               s_serial[13];                /* 12 hex digits + NUL */
static const char        *s_strings[4];                /* langid, mfr, product, serial */
static const char         s_langid[2] = { 0x09, 0x04 }; /* English (US), as esp_tinyusb expects */

/* -------------------------------------------------------------------------- */
/* Sleep lock helpers (DES-USB-006)                                            */
/* -------------------------------------------------------------------------- */

static void lock_acquire_once(void)
{
    if (!s_lock_held && s_lock != NULL) {
        pm_policy_lock_acquire(s_lock);
        s_lock_held = true;
    }
}

static void lock_release_once(void)
{
    if (s_lock_held && s_lock != NULL) {
        pm_policy_lock_release(s_lock);
        s_lock_held = false;
    }
}

/* -------------------------------------------------------------------------- */
/* Event fan-out (DES-USB-005). Runs in the TinyUSB task.                      */
/* -------------------------------------------------------------------------- */

static void tinyusb_event(tinyusb_event_t *event, void *arg)
{
    usb_device_event_t evt;

    switch (event->id) {
    case TINYUSB_EVENT_ATTACHED:
        s_mounted = true;
        lock_acquire_once();
        for (uint8_t i = 0; i < s_function_count; i++) {
            if (s_functions[i]->on_mount) {
                s_functions[i]->on_mount(s_functions[i]->ctx);
            }
        }
        ESP_LOGI(TAG, "mounted");
        evt = USB_DEVICE_EVT_MOUNTED;
        break;

    case TINYUSB_EVENT_DETACHED:
        s_mounted = false;
        for (uint8_t i = 0; i < s_function_count; i++) {
            if (s_functions[i]->on_unmount) {
                s_functions[i]->on_unmount(s_functions[i]->ctx);
            }
        }
        lock_release_once();
        ESP_LOGI(TAG, "unmounted");
        evt = USB_DEVICE_EVT_UNMOUNTED;
        break;

#ifdef CONFIG_TINYUSB_SUSPEND_CALLBACK
    case TINYUSB_EVENT_SUSPENDED:
        for (uint8_t i = 0; i < s_function_count; i++) {
            if (s_functions[i]->on_suspend) {
                s_functions[i]->on_suspend(s_functions[i]->ctx, event->suspended.remote_wakeup);
            }
        }
        lock_release_once(); /* A suspended bus must not keep the device awake. */
        ESP_LOGD(TAG, "suspended (remote wakeup %s)", event->suspended.remote_wakeup ? "on" : "off");
        evt = USB_DEVICE_EVT_SUSPENDED;
        break;
#endif

#ifdef CONFIG_TINYUSB_RESUME_CALLBACK
    case TINYUSB_EVENT_RESUMED:
        if (s_mounted) {
            lock_acquire_once();
        }
        for (uint8_t i = 0; i < s_function_count; i++) {
            if (s_functions[i]->on_resume) {
                s_functions[i]->on_resume(s_functions[i]->ctx);
            }
        }
        ESP_LOGD(TAG, "resumed");
        evt = USB_DEVICE_EVT_RESUMED;
        break;
#endif

    default:
        return;
    }

    if (s_event_cb) {
        s_event_cb(evt, s_event_ctx);
    }
}

/* -------------------------------------------------------------------------- */
/* Descriptor assembly (DES-USB-002, DES-USB-003, DES-USB-004)                 */
/* -------------------------------------------------------------------------- */

static void build_serial_from_mac(void)
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY) != ESP_OK) {
        esp_efuse_mac_get_default(mac);
    }
    snprintf(s_serial, sizeof(s_serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void build_device_descriptor(void)
{
    s_dev_desc = (tusb_desc_device_t) {
        .bLength            = sizeof(tusb_desc_device_t),
        .bDescriptorType    = TUSB_DESC_DEVICE,
        .bcdUSB             = 0x0200,
        /* Composite with Interface Association Descriptors (audio needs one):
         * the Misc/Common/IAD triple tells the host to bind drivers per IAD. */
        .bDeviceClass       = TUSB_CLASS_MISC,
        .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
        .bDeviceProtocol    = MISC_PROTOCOL_IAD,
        .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor           = s_cfg.vid,
        .idProduct          = s_cfg.pid,
        .bcdDevice          = s_cfg.bcd_device,
        .iManufacturer      = 0x01,
        .iProduct           = 0x02,
        .iSerialNumber      = 0x03,
        .bNumConfigurations = 0x01,
    };
}

/* Two passes: size and validate, then write. Returns total length or 0 on failure. */
static esp_err_t build_config_descriptor(size_t *out_total)
{
    size_t total = TUD_CONFIG_DESC_LEN;
    for (uint8_t i = 0; i < s_function_count; i++) {
        total += s_functions[i]->descriptor_len(s_functions[i]->ctx);
    }
    if (total > sizeof(s_cfg_desc) || total > UINT16_MAX) {
        ESP_LOGE(TAG, "configuration descriptor needs %u bytes; CONFIG_USB_DEVICE_CONFIG_DESC_MAX_BYTES is %u",
                 (unsigned)total, (unsigned)sizeof(s_cfg_desc));
        return ESP_ERR_NO_MEM;
    }

    uint8_t attr = 0;
    if (s_cfg.self_powered) {
        attr |= TUSB_DESC_CONFIG_ATT_SELF_POWERED;
    }
    if (s_cfg.remote_wakeup) {
        attr |= TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP;
    }

    const uint8_t header[] = {
        TUD_CONFIG_DESCRIPTOR(1, s_interfaces_used, 0, (uint16_t)total, attr, s_cfg.max_power_ma)
    };
    memcpy(s_cfg_desc, header, sizeof(header));

    uint8_t *p = s_cfg_desc + sizeof(header);
    uint8_t itf_next = 0;
    uint8_t ep_in_next = 1;
    uint8_t ep_out_next = 1;

    for (uint8_t i = 0; i < s_function_count; i++) {
        const usb_function_t *fn = s_functions[i];
        usb_ep_budget_t need = fn->endpoint_request(fn->ctx);

        uint8_t ep_in[USB_DEVICE_MAX_IN_EP] = {0};
        uint8_t ep_out[USB_DEVICE_MAX_OUT_EP] = {0};
        for (uint8_t k = 0; k < need.in_endpoints; k++) {
            ep_in[k] = 0x80 | ep_in_next++;
        }
        for (uint8_t k = 0; k < need.out_endpoints; k++) {
            ep_out[k] = ep_out_next++;
        }

        size_t declared = fn->descriptor_len(fn->ctx);
        size_t written = fn->descriptor_write(fn->ctx, p, itf_next, ep_in, ep_out);
        if (written != declared) {
            ESP_LOGE(TAG, "function '%s' declared %u descriptor bytes but wrote %u",
                     fn->name, (unsigned)declared, (unsigned)written);
            return ESP_ERR_INVALID_SIZE;
        }
        ESP_LOGD(TAG, "function '%s': itf %u..%u, %u IN, %u OUT, %u bytes",
                 fn->name, itf_next, itf_next + fn->interface_count - 1,
                 need.in_endpoints, need.out_endpoints, (unsigned)written);

        p += written;
        itf_next += fn->interface_count;
    }

    s_cfg_desc_len = total;
    *out_total = total;
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t usb_device_init(const usb_device_config_t *cfg)
{
    if (s_state == ST_STARTED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state == ST_INIT) {
        return ESP_OK;
    }

    memset(&s_cfg, 0, sizeof(s_cfg));
    if (cfg != NULL) {
        s_cfg = *cfg;
    }
    if (s_cfg.vid == 0)          { s_cfg.vid = CONFIG_USB_DEVICE_VID; }
    if (s_cfg.pid == 0)          { s_cfg.pid = CONFIG_USB_DEVICE_PID; }
    if (s_cfg.bcd_device == 0)   { s_cfg.bcd_device = 0x0100; }
    if (s_cfg.manufacturer == NULL) { s_cfg.manufacturer = CONFIG_USB_DEVICE_MANUFACTURER; }
    if (s_cfg.product == NULL)   { s_cfg.product = CONFIG_USB_DEVICE_PRODUCT; }
    if (s_cfg.max_power_ma == 0) { s_cfg.max_power_ma = 100; }
    if (cfg == NULL)             { s_cfg.vbus_monitor_gpio = -1; }
    if (s_cfg.serial == NULL) {
        build_serial_from_mac();
        s_cfg.serial = s_serial;
    }

    esp_err_t err = pm_policy_lock_create("usb", &s_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pm_policy lock: %s", esp_err_to_name(err));
        return err;
    }

    s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_storage);
    memset(s_functions, 0, sizeof(s_functions));
    s_function_count = 0;
    memset(&s_used, 0, sizeof(s_used));
    s_interfaces_used = 0;
    s_mounted = false;
    s_lock_held = false;
    s_state = ST_INIT;

    ESP_LOGI(TAG, "init: VID 0x%04X PID 0x%04X serial %s", s_cfg.vid, s_cfg.pid, s_cfg.serial);
    return ESP_OK;
}

esp_err_t usb_device_register(const usb_function_t *fn)
{
    if (s_state == ST_UNINIT || s_state == ST_STARTED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (fn == NULL || fn->name == NULL || fn->interface_count == 0 ||
        fn->endpoint_request == NULL || fn->descriptor_len == NULL || fn->descriptor_write == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    usb_ep_budget_t need = fn->endpoint_request(fn->ctx);
    esp_err_t err = ESP_OK;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_function_count >= CONFIG_USB_DEVICE_MAX_FUNCTIONS) {
        ESP_LOGE(TAG, "'%s': function table full (max %d)", fn->name, CONFIG_USB_DEVICE_MAX_FUNCTIONS);
        err = ESP_ERR_NO_MEM;
    } else if (s_used.in_endpoints + need.in_endpoints > USB_DEVICE_MAX_IN_EP) {
        ESP_LOGE(TAG, "'%s': needs %u IN endpoints, %u of %u already claimed (CON-03)",
                 fn->name, need.in_endpoints, s_used.in_endpoints, USB_DEVICE_MAX_IN_EP);
        err = ESP_ERR_NO_MEM;
    } else if (s_used.out_endpoints + need.out_endpoints > USB_DEVICE_MAX_OUT_EP) {
        ESP_LOGE(TAG, "'%s': needs %u OUT endpoints, %u of %u already claimed",
                 fn->name, need.out_endpoints, s_used.out_endpoints, USB_DEVICE_MAX_OUT_EP);
        err = ESP_ERR_NO_MEM;
    } else if (USB_DEVICE_FIFO_RESERVED + s_used.tx_fifo_bytes + need.tx_fifo_bytes > USB_DEVICE_FIFO_BYTES) {
        ESP_LOGE(TAG, "'%s': needs %u TX FIFO bytes, %u used + %u reserved of %u",
                 fn->name, need.tx_fifo_bytes, s_used.tx_fifo_bytes,
                 USB_DEVICE_FIFO_RESERVED, USB_DEVICE_FIFO_BYTES);
        err = ESP_ERR_NO_MEM;
    } else if (s_interfaces_used + fn->interface_count > USB_DEVICE_MAX_INTERFACES) {
        ESP_LOGE(TAG, "'%s': needs %u interfaces, %u of %u already claimed",
                 fn->name, fn->interface_count, s_interfaces_used, USB_DEVICE_MAX_INTERFACES);
        err = ESP_ERR_NO_MEM;
    } else {
        s_functions[s_function_count++] = fn;
        s_used.in_endpoints += need.in_endpoints;
        s_used.out_endpoints += need.out_endpoints;
        s_used.tx_fifo_bytes += need.tx_fifo_bytes;
        s_interfaces_used += fn->interface_count;
        ESP_LOGI(TAG, "registered '%s' (budget now: %u/%u IN, %u/%u OUT, %u/%u FIFO B, %u itf)",
                 fn->name, s_used.in_endpoints, USB_DEVICE_MAX_IN_EP,
                 s_used.out_endpoints, USB_DEVICE_MAX_OUT_EP,
                 USB_DEVICE_FIFO_RESERVED + s_used.tx_fifo_bytes, USB_DEVICE_FIFO_BYTES,
                 s_interfaces_used);
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t usb_device_start(void)
{
    if (s_state != ST_INIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_function_count == 0) {
        ESP_LOGW(TAG, "starting with no class functions registered");
    }

    build_device_descriptor();
    size_t total = 0;
    esp_err_t err = build_config_descriptor(&total);
    if (err != ESP_OK) {
        return err;
    }

    s_strings[0] = s_langid;
    s_strings[1] = s_cfg.manufacturer;
    s_strings[2] = s_cfg.product;
    s_strings[3] = s_cfg.serial;

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(tinyusb_event, NULL);
    tusb_cfg.phy.self_powered      = s_cfg.self_powered;
    tusb_cfg.phy.vbus_monitor_io   = s_cfg.vbus_monitor_gpio;
    tusb_cfg.task.size             = CONFIG_USB_DEVICE_TASK_STACK;
    tusb_cfg.task.priority         = CONFIG_USB_DEVICE_TASK_PRIORITY;
    tusb_cfg.task.xCoreID          = CONFIG_USB_DEVICE_TASK_CORE;
    tusb_cfg.descriptor.device     = &s_dev_desc;
    tusb_cfg.descriptor.string     = s_strings;
    tusb_cfg.descriptor.string_count = 4;
    tusb_cfg.descriptor.full_speed_config = s_cfg_desc;
#if CONFIG_TINYUSB_PM
    /* esp_pm light-sleep lock while the bus is active; distinct from our
     * pm_policy deep-sleep lock and complementary to it. */
    tusb_cfg.pm_lock_enable = true;
#endif

    ESP_LOGI(TAG, "starting: %u function(s), %u interface(s), %u-byte configuration, task prio %d core %d",
             s_function_count, s_interfaces_used, (unsigned)total,
             CONFIG_USB_DEVICE_TASK_PRIORITY, CONFIG_USB_DEVICE_TASK_CORE);

    err = tinyusb_driver_install(&tusb_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tinyusb_driver_install: %s", esp_err_to_name(err));
        return err;
    }
    s_state = ST_STARTED;
    return ESP_OK;
}

esp_err_t usb_device_stop(void)
{
    if (s_state != ST_STARTED) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = tinyusb_driver_uninstall();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "tinyusb_driver_uninstall: %s", esp_err_to_name(err));
    }
    s_mounted = false;
    lock_release_once();
    s_state = ST_INIT;
    ESP_LOGI(TAG, "stopped");
    return err;
}

esp_err_t usb_device_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    if (s_state == ST_STARTED) {
        usb_device_stop();
    }
    lock_release_once();
    memset(s_functions, 0, sizeof(s_functions));
    s_function_count = 0;
    memset(&s_used, 0, sizeof(s_used));
    s_interfaces_used = 0;
    s_event_cb = NULL;
    s_event_ctx = NULL;
    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;
    s_state = ST_UNINIT;
    /* s_lock is intentionally kept: pm_policy locks are never destroyed. */
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Queries and controls                                                        */
/* -------------------------------------------------------------------------- */

bool usb_device_is_mounted(void)
{
    return s_state == ST_STARTED && s_mounted;
}

esp_err_t usb_device_set_event_cb(usb_device_event_cb_t cb, void *ctx)
{
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    s_event_ctx = ctx;
    s_event_cb = cb;
    return ESP_OK;
}

esp_err_t usb_device_remote_wakeup(void)
{
    if (s_state != ST_STARTED) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = tinyusb_remote_wakeup();
    if (err == ESP_ERR_NOT_ALLOWED) {
        return ESP_ERR_INVALID_STATE; /* not suspended */
    }
    return err;
}

esp_err_t usb_device_config_descriptor(const uint8_t **desc, size_t *len)
{
    if (desc == NULL || len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state == ST_INIT) {
        size_t total = 0;
        esp_err_t err = build_config_descriptor(&total);
        if (err != ESP_OK) {
            return err;
        }
    }
    *desc = s_cfg_desc;
    *len = s_cfg_desc_len;
    return ESP_OK;
}

esp_err_t usb_device_budget(usb_ep_budget_t *used, usb_ep_budget_t *max)
{
    if (used != NULL) {
        *used = s_used;
    }
    if (max != NULL) {
        max->in_endpoints = USB_DEVICE_MAX_IN_EP;
        max->out_endpoints = USB_DEVICE_MAX_OUT_EP;
        max->tx_fifo_bytes = USB_DEVICE_FIFO_BYTES - USB_DEVICE_FIFO_RESERVED;
    }
    return ESP_OK;
}
