/**
 * Phase 1 profile: wired USB headset.
 *
 * Owns its transport the way a wireless profile will own its radio stack: it brings
 * TinyUSB up in start() and tears it down in stop(), because a UAC2 descriptor is
 * fixed at enumeration and therefore cannot be assembled after usb_device_start().
 * The four resources in `headset_profile_resources_t` are main/'s and are only
 * borrowed for the lifetime of a start()/stop() pair.
 *
 * Bring-up order, and why:
 *   hs_audio_open        - playback (and codec) needs the ring before anything can fill
 *                          it; the microphone is optional (no mic -> speaker only)
 *   usb_device_init      - must precede any class function registration
 *   usb_audio_init       - registers the duplex UAC2 function (ADR-021)
 *   usb_hid_init         - registers the consumer-control function
 *   usb_device_start     - descriptors are frozen from here on
 *   hs_audio_start       - after USB, so the first host packets meet a running sink
 *
 * The board's audio path — which DAC or codec, which microphone, the boom jack — is
 * modules/hs_audio's business (ADR-025); this profile only moves bytes to and from USB.
 */
#include <string.h>

#include "esp_log.h"
#include "sdkconfig.h"

#include "hs_audio.h"
#include "usb_audio.h"
#include "usb_device.h"
#include "usb_hid.h"
#include "usound.h"

static const char *TAG = "usound";

/* -------------------------------------------------------------------------- */
/* Policy: HID consumer-control descriptor                                     */
/* -------------------------------------------------------------------------- */
/* One byte, eight bits, no report ID. Consumer control is what a headset should
 * expose: it asks the *host* to change its own volume, which keeps one volume
 * setting on screen instead of a device-side attenuation the user cannot see.
 * Whether volume instead rides the UAC2 feature unit is still open
 * (applications/headset/docs/design.md, "Open questions"); both can coexist,
 * which is why on_control() below is wired even though it only logs. */
static const uint8_t s_hid_report_desc[] = {
    0x05, 0x0C,        /* Usage Page (Consumer)          */
    0x09, 0x01,        /* Usage (Consumer Control)       */
    0xA1, 0x01,        /* Collection (Application)       */
    0x15, 0x00,        /*   Logical Minimum (0)          */
    0x25, 0x01,        /*   Logical Maximum (1)          */
    0x75, 0x01,        /*   Report Size (1)              */
    0x95, 0x08,        /*   Report Count (8)             */
    0x09, 0xE9,        /*   Usage (Volume Increment)     */
    0x09, 0xEA,        /*   Usage (Volume Decrement)     */
    0x09, 0xE2,        /*   Usage (Mute)                 */
    0x09, 0xCD,        /*   Usage (Play/Pause)           */
    0x09, 0xB5,        /*   Usage (Scan Next Track)      */
    0x09, 0xB6,        /*   Usage (Scan Previous Track)  */
    0x09, 0xB7,        /*   Usage (Stop)                 */
    0x09, 0xB3,        /*   Usage (Fast Forward)         */
    0x81, 0x02,        /*   Input (Data,Var,Abs)         */
    0xC0,              /* End Collection                 */
};

/* Bit positions in that one-byte report, in descriptor order. */
#define USOUND_HID_VOL_UP    (1u << 0)
#define USOUND_HID_VOL_DOWN  (1u << 1)
#define USOUND_HID_MUTE      (1u << 2)
#define USOUND_HID_PLAY      (1u << 3)

/* -------------------------------------------------------------------------- */
/* State                                                                        */
/* -------------------------------------------------------------------------- */

static struct {
    headset_profile_resources_t res;
    bool started;          /* start() completed; stop() has work to do        */
    bool audio_up;         /* hs_audio_open() succeeded                       */
    bool capture_up;       /* a microphone came up with it                    */
    bool usb_up;           /* usb_device_init() succeeded                     */
    bool usb_running;      /* usb_device_start() succeeded                    */
} s_usound;

/* -------------------------------------------------------------------------- */
/* Callbacks                                                                    */
/* -------------------------------------------------------------------------- */

/* The feedback servo's only honest input (ADR-021, DES-AUD-023). Runs on the
 * usb_audio feeder task once per millisecond, so it must not block: stats() is a
 * plain struct copy. */
static uint32_t playback_backlog(void *ctx)
{
    (void)ctx;
    return hs_audio_playback_backlog();
}

/* Host moved its own volume or mute on one direction's feature unit. The SDK applies
 * neither (ADR-013); this profile hands the speaker's to hs_audio, which sets the codec's
 * DAC where the board has one (with the hearing-safety cap) and declines otherwise, in
 * which case the host's own attenuation is all there is — as before. */
static void on_usb_control(usb_audio_stream_t which, bool mute, int16_t volume_db256, void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "host %s control: mute=%d volume=%d.%02u dB",
             which == USB_AUDIO_STREAM_SPEAKER ? "speaker" : "mic",
             (int)mute, volume_db256 / 256, (unsigned)((volume_db256 & 0xFF) * 100 / 256));
    if (which == USB_AUDIO_STREAM_SPEAKER) {
        (void)hs_audio_set_host_volume(mute, volume_db256);
    }
}

static void on_usb_event(usb_device_event_t evt, void *ctx)
{
    (void)ctx;
    static const char *const names[] = { "mounted", "unmounted", "suspended", "resumed" };
    ESP_LOGI(TAG, "USB %s", names[evt]);
}

static void hid_tap(uint8_t bits)
{
    if (!usb_hid_ready()) {
        return;  /* Not enumerated, or the interface is idle. Dropping is correct. */
    }
    /* A consumer control is edge-triggered by a press followed by an all-zero
     * release; sending only the press leaves the host repeating the key. */
    uint8_t report = bits;
    (void)usb_hid_report_send(0, &report, sizeof(report));
    report = 0;
    (void)usb_hid_report_send(0, &report, sizeof(report));
}

/* One control step: a volume key press or repeat, a wheel detent, a play/pause press.
 * main/ has already resolved the board's inputs (ADR-025), so there is nothing to
 * debounce or ramp here. */
static void usound_on_control(headset_ctrl_t ctrl, void *ctx)
{
    (void)ctx;
    switch (ctrl) {
    case HEADSET_CTRL_VOL_DOWN:   hid_tap(USOUND_HID_VOL_DOWN); break;
    case HEADSET_CTRL_VOL_UP:     hid_tap(USOUND_HID_VOL_UP);   break;
    case HEADSET_CTRL_PLAY_PAUSE: hid_tap(USOUND_HID_PLAY);     break;
    case HEADSET_CTRL_MIC_MUTE:   hid_tap(USOUND_HID_MUTE);     break;
    default:                      break;
    }
}

static bool usound_is_live(void *ctx)
{
    (void)ctx;
    /* Mounted, not streaming: a host that has enumerated but selected the
     * zero-bandwidth alternate setting is still a live wired connection, and
     * dropping to IDLE there would fight the host's own idle handling. */
    return s_usound.usb_running && usb_device_is_mounted();
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                    */
/* -------------------------------------------------------------------------- */

static esp_err_t usound_stop(void *ctx);

static esp_err_t usound_start(const headset_profile_resources_t *res, void *ctx)
{
    (void)ctx;
    if (res == NULL || res->playback_ring == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_usound.started) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_usound, 0, sizeof(s_usound));
    s_usound.res = *res;

    esp_err_t err;

    /* --- Audio: speaker (and codec), optional microphone --------------- */
    const hs_audio_config_t ac = {
        .playback_ring = res->playback_ring,
        .spk_rate_hz   = CONFIG_USOUND_SPEAKER_SAMPLE_RATE_HZ,
        .spk_channels  = CONFIG_USOUND_SPEAKER_CHANNELS,
        .mic_rate_hz   = CONFIG_USOUND_MIC_SAMPLE_RATE_HZ,
    };
    err = hs_audio_open(&ac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hs_audio_open: %s", esp_err_to_name(err));
        goto fail;  /* Without a sink there is no headset. */
    }
    s_usound.audio_up = true;
    s_usound.capture_up = (hs_audio_mic_ring() != NULL);

    /* --- USB: descriptors are frozen once usb_device_start() runs --------- */
    err = usb_device_init(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_device_init: %s", esp_err_to_name(err));
        goto fail;
    }
    s_usound.usb_up = true;
    (void)usb_device_set_event_cb(on_usb_event, NULL);

    usb_audio_config_t uac = {
        .direction = s_usound.capture_up ? USB_AUDIO_DIR_HEADSET : USB_AUDIO_DIR_SPEAKER,
        .speaker = {
            .ring            = res->playback_ring,
            .sample_rate_hz  = CONFIG_USOUND_SPEAKER_SAMPLE_RATE_HZ,
            .channels        = CONFIG_USOUND_SPEAKER_CHANNELS,
            .bits_per_sample = 16,
        },
        .speaker_backlog_cb = playback_backlog,
        .speaker_target_ms  = 0,  /* CONFIG_USB_AUDIO_SPEAKER_TARGET_MS */
        .on_control         = on_usb_control,
    };
    if (s_usound.capture_up) {
        uac.mic = (usb_audio_stream_config_t){
            .ring            = hs_audio_mic_ring(),
            .sample_rate_hz  = CONFIG_USOUND_MIC_SAMPLE_RATE_HZ,
            .channels        = 1,
            .bits_per_sample = 16,
        };
    }
    err = usb_audio_init(&uac);
    if (err != ESP_OK) {
        /* ESP_ERR_INVALID_ARG here is usually the ring: FW-AUD-059 refuses a ring
         * that cannot hold the setpoint plus a packet. Raise
         * CONFIG_USOUND_PLAYBACK_RING_MS or lower the speaker target. */
        ESP_LOGE(TAG, "usb_audio_init: %s", esp_err_to_name(err));
        goto fail;
    }

    const usb_hid_config_t hid = {
        .report_descriptor     = s_hid_report_desc,
        .report_descriptor_len = sizeof(s_hid_report_desc),
    };
    err = usb_hid_init(&hid);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no HID controls (%s); audio still works", esp_err_to_name(err));
    }

    err = usb_device_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_device_start: %s", esp_err_to_name(err));
        goto fail;
    }
    s_usound.usb_running = true;

    /* --- Go live --------------------------------------------------------- */
    err = hs_audio_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hs_audio_start: %s", esp_err_to_name(err));
        goto fail;
    }

    /* Last, so a failure above never leaves sleep blocked. */
    err = pm_policy_lock_acquire(res->stream_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "stream lock: %s", esp_err_to_name(err));
        goto fail;
    }

    s_usound.started = true;
    ESP_LOGI(TAG, "up: %s, speaker %d Hz x%d, mic %s",
             s_usound.capture_up ? "duplex" : "speaker-only",
             CONFIG_USOUND_SPEAKER_SAMPLE_RATE_HZ, CONFIG_USOUND_SPEAKER_CHANNELS,
             s_usound.capture_up ? "16 kHz mono" : "absent");
    return ESP_OK;

fail:
    /* main/ does not call stop() after a failed start(), so unwind here. */
    (void)usound_stop(NULL);
    return err;
}

static esp_err_t usound_stop(void *ctx)
{
    (void)ctx;

    /* Idempotent, and safe when start() never ran: every step is guarded by the
     * flag its own bring-up set. Torn down in reverse order, USB first so the
     * host stops delivering into a ring whose reader is about to go away. */
    if (s_usound.started && s_usound.res.stream_lock != NULL) {
        (void)pm_policy_lock_release(s_usound.res.stream_lock);
    }
    if (s_usound.usb_running) {
        (void)usb_device_stop();
        s_usound.usb_running = false;
    }
    if (s_usound.usb_up) {
        (void)usb_device_deinit();  /* Unregisters the class functions with it. */
        s_usound.usb_up = false;
    }
    if (s_usound.audio_up) {
        /* Also drops whatever the host queued but never rendered, so the next profile
         * does not open with this one's tail (DES-APB-007). */
        (void)hs_audio_close();
        s_usound.audio_up = false;
        s_usound.capture_up = false;
    }

    s_usound.started = false;
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

const headset_profile_t *usound_profile(void)
{
    static const headset_profile_t profile = {
        .name      = "usound",
        .start     = usound_start,
        .stop      = usound_stop,
        .on_control = usound_on_control,
        .on_status  = NULL,      /* no host surface for battery or pose yet (TBD-014) */
        .caps       = 0,
        .is_live   = usound_is_live,
        .ctx       = NULL,
    };
    return &profile;
}
