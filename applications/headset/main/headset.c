/**
 * Headset: mode state machine and resource hand-off. No profile logic lives here
 * (SAD §2.2, ADR-013) — main/ decides *which* profile runs, never *how* it behaves.
 *
 * A mode switch is exactly this: revoke the four resources from profile A, grant
 * them to profile B. Phase 1 ships one profile, but the arbitration is built for
 * the hand-off from day one so that Phase 2 adds a profile rather than a redesign
 * (applications/headset/docs/design.md).
 *
 *     [*] --> IDLE
 *     IDLE     --> USOUND   : USB transport available
 *     USOUND   --> IDLE     : USB detached
 *     IDLE     --> WIRELESS : Phase 2, dongle link up
 *     WIRELESS --> USOUND   : wired wins
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
#include "sdkconfig.h"

#include "buttons.h"
#include "cfg.h"
#include "diag.h"
#include "headset_profile.h"
#include "pm_policy.h"
#include "power.h"
#include "ringbuf.h"
#include "usound.h"

static const char *TAG = "headset";

/* Application fault codes: 0x0100.. is reserved for the application layer. */
#define APP_FAULT_MANDATORY_INIT 0x0101
#define APP_FAULT_NO_RING        0x0102

typedef enum {
    MODE_IDLE = 0,
    MODE_USOUND,
} headset_mode_t;

static const char *const s_mode_names[] = { "IDLE", "USOUND" };

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

/* -------------------------------------------------------------------------- */
/* Mode transitions                                                            */
/* -------------------------------------------------------------------------- */

static void mode_enter(headset_mode_t mode, const headset_profile_t *profile)
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
            s_hs.mode = MODE_IDLE;
            ESP_LOGI(TAG, "mode = %s", s_mode_names[s_hs.mode]);
            return;
        }
        s_hs.active = profile;
    }

    s_hs.mode = mode;
    ESP_LOGI(TAG, "mode = %s", s_mode_names[mode]);
}

/* Button events reach main/ first so it can consume the mode override before the
 * active profile ever sees it. */
static void on_button(buttons_event_t evt, const buttons_event_info_t *info, void *ctx)
{
    (void)ctx;

    if (evt == BUTTONS_EVENT_LONG_PRESS && info->index == HEADSET_BTN_ACTION) {
        /* Phase 2: this is the wired/wireless override. With one profile compiled
         * in there is nothing to switch to, and saying so beats silence. */
        ESP_LOGI(TAG, "mode override requested; no alternative profile in this image");
        return;
    }

    if (s_hs.active != NULL && s_hs.active->on_button != NULL) {
        s_hs.active->on_button(evt, info, s_hs.active->ctx);
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

static esp_err_t rings_init(void)
{
    /* Sized in milliseconds of the speaker format, which is the ring that matters:
     * it is the elastic buffer the feedback servo regulates (ADR-021). It must
     * exceed CONFIG_USB_AUDIO_SPEAKER_TARGET_MS plus a packet or usb_audio_init()
     * refuses it (FW-AUD-059).
     *
     * Internal RAM, not PSRAM: this is on the 1 ms USB path, and ADR-014 keeps
     * PSRAM off hot paths. */
    const size_t bytes_per_ms = (size_t)CONFIG_USOUND_SPEAKER_SAMPLE_RATE_HZ
                              * CONFIG_USOUND_SPEAKER_CHANNELS * 2u / 1000u;
    const size_t capacity = bytes_per_ms * CONFIG_USOUND_PLAYBACK_RING_MS;

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

    ESP_LOGI(TAG, "playback ring %u B (%d ms at %d Hz x%d)", (unsigned)capacity,
             CONFIG_USOUND_PLAYBACK_RING_MS, CONFIG_USOUND_SPEAKER_SAMPLE_RATE_HZ,
             CONFIG_USOUND_SPEAKER_CHANNELS);
    return ESP_OK;
}

void app_main(void)
{
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
#if CONFIG_HEADSET_ENABLE_USB
    /* Phase 1 has one transport and the board is bus-powered, so "USB available"
     * is true at boot. Phase 2 replaces this with a VBUS check and a dongle-link
     * check, and the arbitration below is where "wired wins" will live. */
    mode_enter(MODE_USOUND, usound_profile());
#else
    ESP_LOGW(TAG, "USB disabled (CONFIG_HEADSET_ENABLE_USB=n): "
                  "USB-Serial/JTAG console preserved, staying IDLE");
#endif

    /* The mode task. It polls rather than waits on an event group because the
     * only Phase 1 trigger — the host going away — has no event of its own until
     * usb_device grows one. One second is far below human notice for a transport
     * change and costs nothing. */
    while (true) {
        if (s_hs.active != NULL && s_hs.active->is_live != NULL
            && !s_hs.active->is_live(s_hs.active->ctx)) {
            ESP_LOGI(TAG, "%s transport gone", s_hs.active->name);
            mode_enter(MODE_IDLE, NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
