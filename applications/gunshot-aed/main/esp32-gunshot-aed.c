/**
 * Reference application: wires the SDK together in the order SDD §10.1 specifies
 * and holds the policy decisions the SDK deliberately leaves to the application
 * (ADR-013). No business logic lives here.
 *
 * Init order matters:
 *   cfg → diag → power → pm_policy → audio_capture → usb_device → class functions
 *   → wifi_link/ble/ota → usb_device_start → audio_capture_start → diag_mark_running
 *
 * Mandatory vs optional is an application decision: here cfg, diag, power and
 * pm_policy are mandatory (their failure is a FAULT); everything else degrades.
 */
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "cfg.h"
#include "diag.h"
#include "power.h"
#include "pm_policy.h"
#include "audio_capture.h"
#include "usb_device.h"
#include "usb_hid.h"
#include "usb_audio.h"
#include "wifi_link.h"
#include "ble.h"
#include "ota.h"

static const char *TAG = "app";

/* Application fault codes: 0x0100.. is reserved for the application layer. */
#define APP_FAULT_MANDATORY_INIT 0x0101

/* -------------------------------------------------------------------------- */
/* Policy: HID report descriptor (vendor-defined, 8 bytes in / 8 bytes out)    */
/* -------------------------------------------------------------------------- */

#if CONFIG_APP_ENABLE_USB
static const uint8_t s_hid_report_desc[] = {
    0x06, 0x00, 0xFF,  /* Usage Page (Vendor 0xFF00) */
    0x09, 0x01,        /* Usage (0x01)               */
    0xA1, 0x01,        /* Collection (Application)   */
    0x15, 0x00,        /*   Logical Minimum (0)      */
    0x26, 0xFF, 0x00,  /*   Logical Maximum (255)    */
    0x75, 0x08,        /*   Report Size (8)          */
    0x95, 0x08,        /*   Report Count (8)         */
    0x09, 0x01,        /*   Usage (0x01)             */
    0x81, 0x02,        /*   Input (Data,Var,Abs)     */
    0x09, 0x01,        /*   Usage (0x01)             */
    0x91, 0x02,        /*   Output (Data,Var,Abs)    */
    0xC0,              /* End Collection             */
};
#endif

/* -------------------------------------------------------------------------- */
/* Callbacks: where product behaviour would go                                 */
/* -------------------------------------------------------------------------- */

static void on_fault(uint32_t code, const char *detail, void *ctx)
{
    /* Policy: stay up and observable. A product might blink an LED or reboot. */
    ESP_LOGE(TAG, "system entered FAULT (0x%08" PRIx32 "): %s", code, detail);
}

#if CONFIG_APP_ENABLE_USB
static void on_hid_output(uint8_t report_id, usb_hid_report_type_t type,
                          const uint8_t *data, size_t len, void *ctx)
{
    ESP_LOGI(TAG, "HID output report id %u type %d len %u", report_id, (int)type, (unsigned)len);
}

static void on_usb_event(usb_device_event_t evt, void *ctx)
{
    static const char *names[] = { "mounted", "unmounted", "suspended", "resumed" };
    ESP_LOGI(TAG, "USB %s", names[evt]);
}
#endif

static void on_ota_event(ota_event_t evt, const ota_event_info_t *info, void *ctx)
{
    switch (evt) {
    case OTA_EVT_PROGRESS:
        ESP_LOGI(TAG, "OTA %u/%d bytes", (unsigned)info->bytes_read, info->image_size);
        break;
    case OTA_EVT_COMMITTED:
        ESP_LOGW(TAG, "OTA committed (v%s); reboot to activate", info->new_version);
        break;
    case OTA_EVT_FAILED:
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(info->err));
        break;
    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Boot                                                                        */
/* -------------------------------------------------------------------------- */

static void mandatory(esp_err_t err, const char *what)
{
    if (err != ESP_OK) {
        diag_fault(APP_FAULT_MANDATORY_INIT, what);
        /* diag_fault() returns; the system runs degraded and observable. */
    }
}

static void optional(esp_err_t err, const char *what)
{
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "%s unavailable: %s", what, esp_err_to_name(err));
    }
}

void app_main(void)
{
    /* --- Platform: mandatory --------------------------------------------- */
    mandatory(cfg_init(), "cfg_init");
    diag_config_t diag_cfg = { .fault_cb = on_fault };
    mandatory(diag_init(&diag_cfg), "diag_init");
    mandatory(power_init(), "power_init");
    mandatory(pm_policy_init(), "pm_policy_init");

    diag_health_t h;
    if (diag_health(&h) == ESP_OK) {
        ESP_LOGI(TAG, "boot #%" PRIu32 ", reset %d, wake 0x%08" PRIx32 ", retained %s, heap %u",
                 h.boot_count, (int)h.reset_reason, h.wakeup_causes,
                 power_retained_is_valid() ? "valid" : "none", (unsigned)h.free_heap);
    }

    /* --- Audio capture (needs a microphone: pins from Kconfig) ----------- */
    audio_capture_config_t cap = {
        .interface = CONFIG_APP_MIC_PDM ? AUDIO_CAPTURE_IF_PDM : AUDIO_CAPTURE_IF_I2S_STD,
        .sample_rate_hz = CONFIG_APP_AUDIO_SAMPLE_RATE_HZ,
        .channels = 1,
        .bits_per_sample = 16,
        .slot = AUDIO_CAPTURE_SLOT_LEFT,
        .pins = { .clk = CONFIG_APP_MIC_PIN_CLK, .ws = CONFIG_APP_MIC_PIN_WS,
                  .din = CONFIG_APP_MIC_PIN_DIN, .mclk = -1 },
    };
    esp_err_t audio_ok = audio_capture_init(&cap);
    optional(audio_ok, "audio_capture");

    /* --- USB composite: UAC2 microphone + vendor HID ---------------------- */
#if CONFIG_APP_ENABLE_USB
    esp_err_t usb_ok = usb_device_init(NULL);
    optional(usb_ok, "usb_device");
    if (usb_ok == ESP_OK) {
        usb_device_set_event_cb(on_usb_event, NULL);
        if (audio_ok == ESP_OK) {
            /* Microphone only: this product has no speaker (SYS-AUD-009 is the
             * headset's requirement, not this one). */
            usb_audio_config_t uac = {
                .direction = USB_AUDIO_DIR_MIC,
                .mic = {
                    .ring = audio_capture_get_ring(),
                    .sample_rate_hz = CONFIG_APP_AUDIO_SAMPLE_RATE_HZ,
                    .channels = 1,
                    .bits_per_sample = 16,
                },
            };
            optional(usb_audio_init(&uac), "usb_audio");
        }
        usb_hid_config_t hid = {
            .report_descriptor = s_hid_report_desc,
            .report_descriptor_len = sizeof(s_hid_report_desc),
            .on_output_report = on_hid_output,
        };
        optional(usb_hid_init(&hid), "usb_hid");
    }
#else
    esp_err_t usb_ok = ESP_ERR_NOT_SUPPORTED;
    ESP_LOGW(TAG, "USB device disabled (CONFIG_APP_ENABLE_USB=n): USB-Serial/JTAG console preserved");
#endif

    /* --- Connectivity ---------------------------------------------------- */
    optional(wifi_link_init(), "wifi_link");
    ble_config_t ble_cfg = { 0 }; /* zero-init = LE SC bonding on, auto re-advertise on */
    optional(ble_init(&ble_cfg), "ble");
    ota_config_t ota_cfg = { .use_cert_bundle = true, .cb = on_ota_event };
    optional(ota_init(&ota_cfg), "ota");

    /* --- Go live --------------------------------------------------------- */
    if (usb_ok == ESP_OK) {
        optional(usb_device_start(), "usb_device_start");
    }
    if (audio_ok == ESP_OK) {
        optional(audio_capture_start(), "audio_capture_start");
    }

    /* A freshly OTA'd image confirms itself once everything above came up. */
    ota_mark_valid();

    if (diag_state() != DIAG_STATE_FAULT) {
        diag_mark_running();
    }
    ESP_LOGI(TAG, "up: state=%d", (int)diag_state());
}
