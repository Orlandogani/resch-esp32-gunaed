/**
 * Headset: mode state machine and resource hand-off. No profile logic lives here
 * (SAD §2.2, ADR-013) — main/ decides *which* profile runs, never *how* it behaves.
 *
 * A mode switch is exactly this: revoke the four resources from profile A, grant
 * them to profile B (applications/headset/docs/design.md).
 *
 *     [*] --> IDLE
 *     IDLE     --> USOUND   : USB transport available (boot, with HEADSET_ENABLE_USB)
 *     IDLE     --> WIRELESS : boot without USB, action press, or the retry timer
 *     USOUND   --> IDLE     : USB detached
 *     WIRELESS --> IDLE     : dongle gone longer than the profile's grace period
 *     USOUND  <--> WIRELESS : action long press (both profiles compiled)
 *
 * The whole state machine runs on the main task. Button events arrive on the buttons
 * task, so a mode request is posted to the main task as a notification bit rather
 * than acted on where it is noticed.
 *
 * The rings are owned here, not by a profile, so their storage survives a mode
 * change: re-allocating a ring on every switch would fragment the heap and race
 * the playback driver's read cursor.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "buttons.h"
#include "cfg.h"
#include "diag.h"
#include "headset_profile.h"
#include "pm_policy.h"
#include "power.h"
#include "ringbuf.h"
#include "usound.h"
#if CONFIG_HEADSET_ENABLE_WIRELESS
#include "wlink.h"
#endif

static const char *TAG = "headset";

/* Application fault codes: 0x0100.. is reserved for the application layer. */
#define APP_FAULT_MANDATORY_INIT 0x0101
#define APP_FAULT_NO_RING        0x0102

typedef enum {
    MODE_IDLE = 0,
    MODE_USOUND,
    MODE_WIRELESS,
} headset_mode_t;

static const char *const s_mode_names[] = { "IDLE", "USOUND", "WIRELESS" };

/* Requests posted to the main task (notification bits). */
#define REQ_OVERRIDE   (1u << 0)   /* action long press: switch wired <-> wireless */
#define REQ_WAKE_LINK  (1u << 1)   /* action press while IDLE: advertise now       */

/* -------------------------------------------------------------------------- */
/* Owned resources                                                             */
/* -------------------------------------------------------------------------- */

static struct {
    ringbuf_t                playback_ring;
    void                    *playback_storage;
    ringbuf_t                mic_ring;       /* Written by audio_capture, which the
                                              * active profile initialises; main/ only
                                              * holds the control block's storage. */
    pm_policy_lock_handle_t  stream_lock;

    headset_mode_t           mode;
    const headset_profile_t *active;
    TaskHandle_t             main_task;
    int64_t                  idle_since_us;
} s_hs;

/* -------------------------------------------------------------------------- */
/* Policy: buttons                                                             */
/* -------------------------------------------------------------------------- */

/* Provisional pins: the enclosure is not designed and the board is a devkit.
 * GPIO 0 is the BOOT button, which is genuinely present on every S3 devkit and so
 * is the one control that can be exercised today. */
static const buttons_pin_config_t s_button_pins[] = {
    { .gpio = CONFIG_HEADSET_BTN_PIN_VOL_DOWN, .active_low = true, .pull_enable = true },
    { .gpio = CONFIG_HEADSET_BTN_PIN_VOL_UP,   .active_low = true, .pull_enable = true },
    { .gpio = CONFIG_HEADSET_BTN_PIN_ACTION,   .active_low = true, .pull_enable = true,
      .wake_from_deep_sleep = true },
};

#define HEADSET_BTN_ACTION 2

static void on_fault(uint32_t code, const char *detail, void *ctx)
{
    (void)ctx;
    ESP_LOGE(TAG, "system entered FAULT (0x%08" PRIx32 "): %s", code, detail);
}

static const headset_profile_t *profile_for(headset_mode_t mode)
{
    switch (mode) {
    case MODE_USOUND:
        return usound_profile();
#if CONFIG_HEADSET_ENABLE_WIRELESS
    case MODE_WIRELESS:
        return wlink_profile();
#endif
    default:
        return NULL;
    }
}

/* -------------------------------------------------------------------------- */
/* Mode transitions — main task only                                           */
/* -------------------------------------------------------------------------- */

static void mode_enter(headset_mode_t mode)
{
    if (s_hs.mode == mode) {
        return;
    }

    /* Revoke from the outgoing profile first. Two profiles must never hold the
     * playback ring's writer at once — it is single-writer by contract. */
    if (s_hs.active != NULL) {
        ESP_LOGI(TAG, "%s -> stopping", s_hs.active->name);
        (void)s_hs.active->stop(s_hs.active->ctx);
        s_hs.active = NULL;
    }

    const headset_profile_t *profile = profile_for(mode);
    if (profile != NULL) {
        const headset_profile_resources_t res = {
            .playback_ring = &s_hs.playback_ring,
            .mic_ring      = &s_hs.mic_ring,
            .stream_lock   = s_hs.stream_lock,
        };
        esp_err_t err = profile->start(&res, profile->ctx);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s failed to start (%s); staying IDLE",
                     profile->name, esp_err_to_name(err));
            mode = MODE_IDLE;
        } else {
            s_hs.active = profile;
        }
    }

    s_hs.mode = mode;
    if (mode == MODE_IDLE) {
        s_hs.idle_since_us = esp_timer_get_time();
    }
    ESP_LOGI(TAG, "mode = %s", s_mode_names[mode]);
}

static void handle_requests(uint32_t req)
{
    if (req & REQ_OVERRIDE) {
#if CONFIG_HEADSET_ENABLE_USB && CONFIG_HEADSET_ENABLE_WIRELESS
        mode_enter(s_hs.mode == MODE_USOUND ? MODE_WIRELESS : MODE_USOUND);
#else
        /* With one profile compiled in there is nothing to switch to, and saying so
         * beats silence. */
        ESP_LOGI(TAG, "mode override requested; no alternative profile in this image");
#endif
    }
#if CONFIG_HEADSET_ENABLE_WIRELESS
    if ((req & REQ_WAKE_LINK) && s_hs.mode == MODE_IDLE) {
        mode_enter(MODE_WIRELESS);
    }
#endif
}

/* Button events reach main/ first so it can consume the mode controls before the
 * active profile ever sees them. Buttons task context: post, never act. */
static void on_button(buttons_event_t evt, const buttons_event_info_t *info, void *ctx)
{
    (void)ctx;

    if (info->index == HEADSET_BTN_ACTION) {
        if (evt == BUTTONS_EVENT_LONG_PRESS) {
            xTaskNotify(s_hs.main_task, REQ_OVERRIDE, eSetBits);
            return;
        }
        if (evt == BUTTONS_EVENT_PRESSED && s_hs.mode == MODE_IDLE) {
            xTaskNotify(s_hs.main_task, REQ_WAKE_LINK, eSetBits);
            return;
        }
    }

    const headset_profile_t *active = s_hs.active;
    if (active != NULL && active->on_button != NULL) {
        active->on_button(evt, info, active->ctx);
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

static size_t ring_bytes(uint32_t rate_hz, uint32_t channels, uint32_t ms)
{
    return (size_t)rate_hz * channels * 2u / 1000u * ms;
}

static esp_err_t rings_init(void)
{
    /* One playback ring serves every profile, sized for the most demanding. Each
     * size is milliseconds of that profile's speaker format:
     *  - usound: the elastic buffer the USB feedback servo regulates (ADR-021). It
     *    must exceed CONFIG_USB_AUDIO_SPEAKER_TARGET_MS plus a packet or
     *    usb_audio_init() refuses it (FW-AUD-059).
     *  - wlink: the servo setpoint of the dongle's path plus radio bursts (ADR-022).
     *
     * Internal RAM, not PSRAM: this is on the audio hot path, and ADR-014 keeps
     * PSRAM off hot paths. */
    size_t capacity = ring_bytes(CONFIG_USOUND_SPEAKER_SAMPLE_RATE_HZ, CONFIG_USOUND_SPEAKER_CHANNELS,
                                 CONFIG_USOUND_PLAYBACK_RING_MS);
#if CONFIG_HEADSET_ENABLE_WIRELESS
    size_t wl = ring_bytes(CONFIG_WLINK_SPEAKER_SAMPLE_RATE_HZ, CONFIG_WLINK_SPEAKER_CHANNELS,
                           CONFIG_WLINK_PLAYBACK_RING_MS);
    if (wl > capacity) {
        capacity = wl;
    }
#endif

    s_hs.playback_storage = heap_caps_malloc(capacity, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_hs.playback_storage == NULL) {
        ESP_LOGE(TAG, "playback ring: %u bytes internal unavailable", (unsigned)capacity);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = ringbuf_init(&s_hs.playback_ring, s_hs.playback_storage, capacity);
    if (err != ESP_OK) {
        heap_caps_free(s_hs.playback_storage);
        s_hs.playback_storage = NULL;
        return err;
    }

    ESP_LOGI(TAG, "playback ring %u B", (unsigned)capacity);
    return ESP_OK;
}

void app_main(void)
{
    s_hs.main_task = xTaskGetCurrentTaskHandle();

    /* --- Platform: mandatory --------------------------------------------- */
    mandatory(cfg_init(), "cfg_init");
    diag_config_t diag_cfg = { .fault_cb = on_fault };
    mandatory(diag_init(&diag_cfg), "diag_init");
    mandatory(power_init(), "power_init");     /* Before buttons: EXT1 wake needs it. */
    mandatory(pm_policy_init(), "pm_policy_init");
    mandatory(pm_policy_lock_create("headset_stream", &s_hs.stream_lock), "stream lock");

    diag_health_t h;
    if (diag_health(&h) == ESP_OK) {
        ESP_LOGI(TAG, "boot #%" PRIu32 ", reset %d, wake 0x%08" PRIx32 ", heap %u",
                 h.boot_count, (int)h.reset_reason, h.wakeup_causes, (unsigned)h.free_heap);
    }

    /* --- Resources the profiles borrow ----------------------------------- */
    if (rings_init() != ESP_OK) {
        diag_fault(APP_FAULT_NO_RING, "playback ring allocation");
        return;  /* Without the ring there is no audio path at all. */
    }

    /* --- Controls --------------------------------------------------------- */
    const buttons_config_t btn = {
        .pins        = s_button_pins,
        .count       = sizeof(s_button_pins) / sizeof(s_button_pins[0]),
        .auto_repeat = true,   /* Volume ramps while held. */
        .cb          = on_button,
    };
    esp_err_t btn_ok = buttons_init(&btn);
    optional(btn_ok, "buttons");
    if (btn_ok == ESP_OK) {
        optional(buttons_start(), "buttons_start");
    }

    if (diag_state() != DIAG_STATE_FAULT) {
        diag_mark_running();
    }

    /* --- Mode state machine ---------------------------------------------- */
    s_hs.idle_since_us = esp_timer_get_time();
#if CONFIG_HEADSET_ENABLE_USB
    /* The board is bus-powered, so "USB available" is true at boot. VBUS detection
     * replaces this when the battery board exists; "wired wins" lives here. */
    mode_enter(MODE_USOUND);
#endif
#if CONFIG_HEADSET_ENABLE_WIRELESS
    if (s_hs.mode == MODE_IDLE) {
        mode_enter(MODE_WIRELESS);
    }
#endif
#if !CONFIG_HEADSET_ENABLE_USB && !CONFIG_HEADSET_ENABLE_WIRELESS
    ESP_LOGW(TAG, "no profile enabled (HEADSET_ENABLE_USB=n, HEADSET_ENABLE_WIRELESS=n): staying IDLE");
#endif

    /* The mode task. It wakes on a posted request or once a second, which is far
     * below human notice for a transport change: the only other triggers — the
     * host or the dongle going away — have no event of their own yet. */
    while (true) {
        uint32_t req = 0;
        (void)xTaskNotifyWait(0, UINT32_MAX, &req, pdMS_TO_TICKS(1000));
        handle_requests(req);

        if (s_hs.active != NULL && s_hs.active->is_live != NULL
            && !s_hs.active->is_live(s_hs.active->ctx)) {
            ESP_LOGI(TAG, "%s transport gone", s_hs.active->name);
            mode_enter(MODE_IDLE);
        }
#if CONFIG_HEADSET_ENABLE_WIRELESS
        if (s_hs.mode == MODE_IDLE &&
            esp_timer_get_time() - s_hs.idle_since_us >= (int64_t)CONFIG_HEADSET_WIRELESS_RETRY_S * 1000000) {
            mode_enter(MODE_WIRELESS);
            if (s_hs.mode == MODE_IDLE) {
                s_hs.idle_since_us = esp_timer_get_time();   /* failed: wait a full period */
            }
        }
#endif
    }
}
