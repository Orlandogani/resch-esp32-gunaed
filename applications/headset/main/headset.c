/**
 * Headset: board bring-up, the mode state machine, and the device-level policies (power,
 * battery, wear, status LED). No profile logic lives here (SAD §2.2, ADR-013) — main/
 * decides *which* profile runs, never *how* it behaves — and no pin: everything about the
 * hardware comes from the selected board (ADR-025) through modules/hs_*.
 *
 * A mode switch is exactly this: revoke the four resources from profile A, grant them to
 * profile B (applications/headset/docs/design.md).
 *
 *     [*] --> IDLE
 *     IDLE     --> USOUND   : USB available (boot, or the charger reports power), with
 *                             HEADSET_ENABLE_USB
 *     IDLE     --> WIRELESS : boot without USB, action press, or the retry timer
 *     WIRELESS --> USOUND   : external power appears ("wired wins")
 *     USOUND   --> IDLE     : USB detached
 *     WIRELESS --> IDLE     : dongle gone longer than the profile's grace period
 *     USOUND  <--> WIRELESS : action long press (both profiles compiled)
 *     any      --> off      : power held, battery critical, or not worn too long on battery
 *
 * Everything runs on the main task. Inputs, battery and wear arrive on other tasks and
 * are posted here as events; only head pose goes straight to the profile, because it is a
 * stream, not an event.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "board.h"
#include "cfg.h"
#include "diag.h"
#include "headset_profile.h"
#include "hs_audio.h"
#include "hs_power.h"
#include "hs_sense.h"
#include "hs_ui.h"
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
#define APP_FAULT_NO_CODEC       0x0103

/* A profile is given this long to find its far end before is_live() is believed: a USB
 * host needs a moment to enumerate, a charger never will. */
#define LIVE_GRACE_US (3 * 1000 * 1000)

typedef enum {
    MODE_IDLE = 0,
    MODE_USOUND,
    MODE_WIRELESS,
} headset_mode_t;

static const char *const s_mode_names[] = { "IDLE", "USOUND", "WIRELESS" };

typedef enum {
    EV_INPUT = 0,   /* arg: hs_input_t                  */
    EV_BATTERY,     /* state in hs_power_get()           */
    EV_WEAR,        /* arg: worn                         */
} event_type_t;

typedef struct {
    event_type_t type;
    int32_t      arg;
} event_t;

/* -------------------------------------------------------------------------- */
/* Owned resources                                                             */
/* -------------------------------------------------------------------------- */

static struct {
    ringbuf_t                playback_ring;
    void                    *playback_storage;
    ringbuf_t                mic_ring;       /* Unused storage-wise: the microphone ring
                                              * belongs to audio_capture (via hs_audio),
                                              * which outlives a mode only as long as the
                                              * profile that opened it. Kept for the
                                              * vtable's shape. */
    pm_policy_lock_handle_t  stream_lock;

    headset_mode_t           mode;
    const headset_profile_t *volatile active;
    QueueHandle_t            events;
    int64_t                  idle_since_us;
    int64_t                  mode_since_us;
    int64_t                  off_head_since_us;
    hs_battery_t             battery;
    bool                     worn;
    bool                     pose_on;
    uint32_t                 events_dropped;
} s_hs = { .worn = true };

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
/* Event plumbing: other tasks post, the main task acts                        */
/* -------------------------------------------------------------------------- */

static void post(event_type_t type, int32_t arg)
{
    const event_t ev = { .type = type, .arg = arg };
    if (s_hs.events == NULL || xQueueSend(s_hs.events, &ev, 0) != pdTRUE) {
        s_hs.events_dropped++;
    }
}

static void on_input(hs_input_t in, void *ctx)
{
    (void)ctx;
    post(EV_INPUT, (int32_t)in);
}

static void on_battery(const hs_battery_t *b, void *ctx)
{
    (void)ctx;
    (void)b;
    post(EV_BATTERY, 0);
}

static void on_wear(bool worn, void *ctx)
{
    (void)ctx;
    post(EV_WEAR, worn ? 1 : 0);
}

/* IMU task context: a stream, so straight to the profile (DES-HSS-002). */
static void on_pose(float w, float x, float y, float z, void *ctx)
{
    (void)ctx;
    const headset_profile_t *p = s_hs.active;
    if (p != NULL && p->on_status != NULL && (p->caps & HEADSET_PROFILE_CAP_HEAD_POSE)) {
        const headset_status_t st = { .kind = HEADSET_STATUS_HEAD_POSE, .pose = { w, x, y, z } };
        p->on_status(&st, p->ctx);
    }
}

/* -------------------------------------------------------------------------- */
/* Policy: status LED and head pose follow the state                          */
/* -------------------------------------------------------------------------- */

static bool connected(void)
{
    const headset_profile_t *p = s_hs.active;
    if (p == NULL) {
        return false;
    }
    if (p->is_connected != NULL) {
        return p->is_connected(p->ctx);
    }
    return p->is_live == NULL || p->is_live(p->ctx);
}

static void update_led(void)
{
    const hs_battery_t *b = &s_hs.battery;
    hs_led_t led;
    if (b->valid && b->low) {
        led = HS_LED_LOW_BATTERY;
    } else if (s_hs.mode == MODE_USOUND) {
        led = HS_LED_USB;
    } else if (s_hs.mode == MODE_WIRELESS) {
        led = connected() ? HS_LED_LINKED : HS_LED_ADVERTISING;
    } else if (b->valid && b->ext_power) {
        led = b->charging ? HS_LED_CHARGING : HS_LED_CHARGED;
    } else {
        led = HS_LED_IDLE;
    }
    /* Off the head and on battery, nobody is looking: save the current (DES-HSU-004). */
    if (!s_hs.worn && !(b->valid && b->ext_power) && led != HS_LED_LOW_BATTERY) {
        led = HS_LED_OFF;
    }
    hs_ui_led(led);
}

static void update_pose(void)
{
    const headset_profile_t *p = s_hs.active;
    bool want = p != NULL && (p->caps & HEADSET_PROFILE_CAP_HEAD_POSE) && s_hs.worn && connected();
    if (want != s_hs.pose_on && hs_sense_pose(want) == ESP_OK) {
        s_hs.pose_on = want;
    }
}

static void send_status(headset_status_kind_t kind)
{
    const headset_profile_t *p = s_hs.active;
    if (p == NULL || p->on_status == NULL) {
        return;
    }
    headset_status_t st = { .kind = kind };
    if (kind == HEADSET_STATUS_BATTERY) {
        if (!s_hs.battery.valid) {
            return;
        }
        st.battery.percent = s_hs.battery.percent;
        st.battery.ext_power = s_hs.battery.ext_power;
        st.battery.charging = s_hs.battery.charging;
    } else {
        st.worn = s_hs.worn;
    }
    p->on_status(&st, p->ctx);
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
     * playback ring's writer at once — it is single-writer by contract. Head pose
     * stops first, so the IMU task never calls into a profile that is stopping. */
    if (s_hs.pose_on && hs_sense_pose(false) == ESP_OK) {
        s_hs.pose_on = false;
    }
    if (s_hs.active != NULL) {
        const headset_profile_t *old = s_hs.active;
        s_hs.active = NULL;
        ESP_LOGI(TAG, "%s -> stopping", old->name);
        (void)old->stop(old->ctx);
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
    s_hs.mode_since_us = esp_timer_get_time();
    if (mode == MODE_IDLE) {
        s_hs.idle_since_us = s_hs.mode_since_us;
    }
    ESP_LOGI(TAG, "mode = %s", s_mode_names[mode]);
    send_status(HEADSET_STATUS_BATTERY);   /* a new far end starts with the current state */
    send_status(HEADSET_STATUS_WEAR);
    update_led();
}

/* -------------------------------------------------------------------------- */
/* Power off                                                                   */
/* -------------------------------------------------------------------------- */

static void power_off(const char *why)
{
    if (!hs_power_has_latch()) {
        ESP_LOGW(TAG, "power off (%s) requested; this board cannot switch itself off", why);
        return;
    }
    ESP_LOGW(TAG, "power off: %s", why);
    mode_enter(MODE_IDLE);                      /* profile stopped, codec muted and down */
    hs_ui_led(HS_LED_POWER_OFF);
    vTaskDelay(pdMS_TO_TICKS(400));
    hs_ui_led(HS_LED_OFF);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_err_t err = hs_power_off(hs_ui_power_pressed);
    /* Only reached if power persisted: carry on, visibly. */
    ESP_LOGE(TAG, "still powered after release (%s)", esp_err_to_name(err));
    update_led();
}

/* -------------------------------------------------------------------------- */
/* Event handling — main task only                                             */
/* -------------------------------------------------------------------------- */

static void to_profile(headset_ctrl_t ctrl)
{
    const headset_profile_t *p = s_hs.active;
    if (p != NULL && p->on_control != NULL) {
        p->on_control(ctrl, p->ctx);
    }
}

static void handle_input(hs_input_t in)
{
    switch (in) {
    case HS_IN_ACTION_LONG:
#if CONFIG_HEADSET_ENABLE_USB && CONFIG_HEADSET_ENABLE_WIRELESS
        mode_enter(s_hs.mode == MODE_USOUND ? MODE_WIRELESS : MODE_USOUND);
#else
        /* With one profile compiled in there is nothing to switch to, and saying so
         * beats silence. */
        ESP_LOGI(TAG, "mode override requested; no alternative profile in this image");
#endif
        break;
    case HS_IN_ACTION_PRESS:
#if CONFIG_HEADSET_ENABLE_WIRELESS
        if (s_hs.mode == MODE_IDLE) {
            mode_enter(MODE_WIRELESS);
            break;
        }
#endif
        to_profile(HEADSET_CTRL_PLAY_PAUSE);
        break;
    case HS_IN_VOL_UP:
        to_profile(HEADSET_CTRL_VOL_UP);
        break;
    case HS_IN_VOL_DOWN:
        to_profile(HEADSET_CTRL_VOL_DOWN);
        break;
    case HS_IN_POWER_SHORT: {
        uint8_t pct = s_hs.battery.valid ? s_hs.battery.percent : 100;
        hs_ui_led_flash(pct >= 50 ? HS_LED_LEVEL_HIGH : pct >= 20 ? HS_LED_LEVEL_MID : HS_LED_LEVEL_LOW, 2000);
        ESP_LOGI(TAG, "battery %u%%", pct);
        break;
    }
    case HS_IN_POWER_HOLD:
        power_off("power button");
        break;
    case HS_IN_JACK_IN:
    case HS_IN_JACK_OUT:
        ESP_LOGI(TAG, "boom microphone %s", in == HS_IN_JACK_IN ? "plugged" : "unplugged");
        hs_audio_set_jack(in == HS_IN_JACK_IN);
        break;
    default:
        break;
    }
}

static void handle_battery(void)
{
    hs_battery_t prev = s_hs.battery;
    hs_power_get(&s_hs.battery);
    const hs_battery_t *b = &s_hs.battery;

    if (b->critical) {
        power_off("battery critical");
        return;
    }
    if (b->low && !prev.low) {
        ESP_LOGW(TAG, "battery low: %u%% (%u mV)", b->percent, b->mv);
    }
#if CONFIG_HEADSET_ENABLE_USB
    /* Wired wins: power on the USB-C means a cable, and probably a host (DES-HSM-002). */
    if (b->ext_power && !(prev.valid && prev.ext_power) && s_hs.mode != MODE_USOUND) {
        ESP_LOGI(TAG, "external power: trying USB");
        mode_enter(MODE_USOUND);
    }
#endif
    if (b->percent != prev.percent || b->ext_power != prev.ext_power || b->charging != prev.charging) {
        send_status(HEADSET_STATUS_BATTERY);
    }
    update_led();
}

static void handle_wear(bool worn)
{
    if (worn == s_hs.worn) {
        return;
    }
    s_hs.worn = worn;
    s_hs.off_head_since_us = worn ? 0 : esp_timer_get_time();
    ESP_LOGI(TAG, "%s", worn ? "worn (moving)" : "off head (still)");
    send_status(HEADSET_STATUS_WEAR);
    update_pose();
    update_led();
}

/* Once a second, and after every event. */
static void periodic(void)
{
    int64_t now = esp_timer_get_time();
    const headset_profile_t *p = s_hs.active;

    if (p != NULL && p->is_live != NULL && now - s_hs.mode_since_us >= LIVE_GRACE_US &&
        !p->is_live(p->ctx)) {
        ESP_LOGI(TAG, "%s transport gone", p->name);
        mode_enter(MODE_IDLE);
    }
#if CONFIG_HEADSET_ENABLE_WIRELESS
    if (s_hs.mode == MODE_IDLE &&
        now - s_hs.idle_since_us >= (int64_t)CONFIG_HEADSET_WIRELESS_RETRY_S * 1000000) {
        mode_enter(MODE_WIRELESS);
        if (s_hs.mode == MODE_IDLE) {
            s_hs.idle_since_us = now;   /* failed: wait a full period */
        }
    }
#endif
#if CONFIG_HS_POWER_AUTO_OFF_MIN > 0
    if (!s_hs.worn && s_hs.off_head_since_us != 0 && !(s_hs.battery.valid && s_hs.battery.ext_power) &&
        hs_power_has_latch() &&
        now - s_hs.off_head_since_us >= (int64_t)CONFIG_HS_POWER_AUTO_OFF_MIN * 60 * 1000000) {
        power_off("not worn");
    }
#endif
    update_pose();
    update_led();
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
    /* --- First: keep the power on (soft latch), before anything slow --- */
    esp_err_t latch = hs_power_hold();
    (void)board_init();

    /* --- Platform: mandatory --------------------------------------------- */
    mandatory(cfg_init(), "cfg_init");
    diag_config_t diag_cfg = { .fault_cb = on_fault };
    mandatory(diag_init(&diag_cfg), "diag_init");
    mandatory(power_init(), "power_init");     /* Before buttons: EXT1 wake needs it. */
    mandatory(pm_policy_init(), "pm_policy_init");
    mandatory(pm_policy_lock_create("headset_stream", &s_hs.stream_lock), "stream lock");
    optional(latch, "power latch");
    ESP_LOGI(TAG, "board %s", board_desc()->name);

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
    s_hs.events = xQueueCreate(16, sizeof(event_t));
    if (s_hs.events == NULL) {
        diag_fault(APP_FAULT_MANDATORY_INIT, "event queue");
        return;
    }

    /* --- Board peripherals, all optional except the codec on a codec board -- */
    esp_err_t codec = hs_audio_init();
    if (codec != ESP_OK && board_desc()->codec.present) {
        diag_fault(APP_FAULT_NO_CODEC, "codec");   /* no sound; still observable */
    }
    optional(hs_ui_init(on_input, NULL), "inputs");
    optional(hs_power_init(on_battery, NULL), "battery monitor");
    hs_power_get(&s_hs.battery);
    if (s_hs.battery.critical) {
        hs_ui_led(HS_LED_LOW_BATTERY);
        vTaskDelay(pdMS_TO_TICKS(1500));
        power_off("battery critical at boot");
    }
    optional(hs_sense_init(on_wear, on_pose, NULL), "IMU");

    if (diag_state() != DIAG_STATE_FAULT) {
        diag_mark_running();
    }

    /* --- Mode state machine ---------------------------------------------- */
    s_hs.idle_since_us = esp_timer_get_time();
#if CONFIG_HEADSET_ENABLE_USB
    /* USB is "available" when the board is bus-powered (no battery to say otherwise) or
     * the charger reports power on the USB-C. "Wired wins" lives here. */
    if (!s_hs.battery.valid || s_hs.battery.ext_power) {
        mode_enter(MODE_USOUND);
    }
#endif
#if CONFIG_HEADSET_ENABLE_WIRELESS
    if (s_hs.mode == MODE_IDLE) {
        mode_enter(MODE_WIRELESS);
    }
#endif
#if !CONFIG_HEADSET_ENABLE_USB && !CONFIG_HEADSET_ENABLE_WIRELESS
    ESP_LOGW(TAG, "no profile enabled (HEADSET_ENABLE_USB=n, HEADSET_ENABLE_WIRELESS=n): staying IDLE");
#endif
    update_led();

    /* The mode task. It wakes on an event or once a second, which is far below human
     * notice for a transport change: the host or the dongle going away has no event of
     * its own yet. */
    while (true) {
        event_t ev;
        if (xQueueReceive(s_hs.events, &ev, pdMS_TO_TICKS(1000)) == pdTRUE) {
            switch (ev.type) {
            case EV_INPUT:   handle_input((hs_input_t)ev.arg); break;
            case EV_BATTERY: handle_battery();                 break;
            case EV_WEAR:    handle_wear(ev.arg != 0);         break;
            default:         break;
            }
        }
        periodic();
    }
}
