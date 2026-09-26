/**
 * Phase 2 profile: wireless headset, peripheral end of subsys/audio_link.
 *
 * Bring-up order, and why:
 *   audio_playback_init  - the sink must exist before the link can fill its ring
 *   audio_capture_init   - independent; failure is tolerated (no mic -> speaker only)
 *   audio_link_init      - decoder writes main/'s playback ring (single writer);
 *                          encoder reads the capture ring through its own cursor
 *   audio_playback_start - running before the first frame is decoded
 *   audio_capture_start
 *   audio_link_start     - BLE up and advertising; the dongle connects when it can
 *
 * The profile stays live while the link is up, and for CONFIG_WLINK_RECONNECT_GRACE_S
 * after it drops: a dongle that comes back quickly finds the headset still
 * advertising and no mode change happens.
 */
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "audio_capture.h"
#include "audio_link.h"
#include "audio_playback.h"
#include "headset_link_proto.h"
#include "wlink.h"

static const char *TAG = "wlink";

/* Button indices, matching the pins array main/ passes to buttons_init(). */
#define WLINK_BTN_VOL_DOWN  0
#define WLINK_BTN_VOL_UP    1
#define WLINK_BTN_ACTION    2

static struct {
    headset_profile_resources_t res;
    bool    started;
    bool    playback_up;
    bool    capture_up;
    bool    link_init;
    bool    link_started;
    int64_t live_until_us;     /* is_live() stays true until then while the link is down. */
    esp_timer_handle_t stats_timer;
} s_w;

#if CONFIG_WLINK_STATS_PERIOD_S > 0
/* The headset half of a two-board soak (SDD §16.3): the dongle logs its side, this
 * logs ours, and scripts/link_soak.py lines the two up. esp_timer task context. */
static void log_stats(void *arg)
{
    (void)arg;
    audio_link_stats_t l;
    audio_playback_stats_t p;
    if (audio_link_stats(&l) != ESP_OK || audio_playback_stats(&p) != ESP_OK) {
        return;
    }
    ESP_LOGI(TAG, "LINK %s rx %lu lost %lu late %lu conceal %lu resync %lu | tx %lu busy %lu | "
                  "dec %lu/%lu us enc %lu/%lu us stack %lu | PLAY backlog %lu B underruns %lu "
                  "starv %lu prefills %lu",
             l.up ? "UP" : "down", (unsigned long)l.rx_frames, (unsigned long)l.rx_lost,
             (unsigned long)l.rx_late, (unsigned long)l.rx_concealed, (unsigned long)l.rx_resyncs,
             (unsigned long)l.tx_frames, (unsigned long)l.tx_busy,
             (unsigned long)l.decode_us_avg, (unsigned long)l.decode_us_max,
             (unsigned long)l.encode_us_avg, (unsigned long)l.encode_us_max,
             (unsigned long)l.task_stack_free_min, (unsigned long)p.source_backlog_bytes,
             (unsigned long)p.underruns, (unsigned long)p.starvations, (unsigned long)p.prefills);
}
#endif

/* -------------------------------------------------------------------------- */
/* Callbacks                                                                   */
/* -------------------------------------------------------------------------- */

/* What the dongle's rate servo regulates (ADR-022): audio queued for playback.
 * Called once per frame on the link task; stats() is a struct copy. */
static uint32_t playback_backlog(void *ctx)
{
    (void)ctx;
    audio_playback_stats_t st;
    return (audio_playback_stats(&st) == ESP_OK) ? st.source_backlog_bytes : 0;
}

static void on_link_event(audio_link_event_t evt, void *ctx)
{
    (void)ctx;
    switch (evt) {
    case AUDIO_LINK_EVT_UP:
        ESP_LOGI(TAG, "dongle linked");
        s_w.live_until_us = INT64_MAX;
        break;
    case AUDIO_LINK_EVT_DOWN:
        ESP_LOGI(TAG, "dongle lost; advertising for %d s", CONFIG_WLINK_RECONNECT_GRACE_S);
        s_w.live_until_us = esp_timer_get_time() + (int64_t)CONFIG_WLINK_RECONNECT_GRACE_S * 1000000;
        /* Drop the stale tail rather than play it after the gap (DES-APB-007). */
        (void)audio_playback_flush();
        break;
    case AUDIO_LINK_EVT_FORMAT_MISMATCH:
        ESP_LOGE(TAG, "dongle sends a different audio format; check WLINK_* against DONGLE_*");
        break;
    default:
        break;
    }
}

static void on_control(const uint8_t *msg, size_t len, void *ctx)
{
    (void)ctx;
    if (len >= HLP_HOST_VOLUME_LEN && msg[0] == HLP_MSG_HOST_VOLUME) {
        int16_t vol = (int16_t)(msg[3] | (msg[4] << 8));
        ESP_LOGI(TAG, "host %s: mute=%u volume=%d.%02u dB", msg[1] ? "mic" : "speaker", msg[2],
                 vol / 256, (unsigned)((vol & 0xFF) * 100 / 256));
    }
}

static void send_button(uint8_t bits)
{
    const uint8_t msg[HLP_BUTTON_LEN] = { HLP_MSG_BUTTON, bits };
    esp_err_t err = audio_link_send_control(msg, sizeof(msg));
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "button not sent: %s", esp_err_to_name(err));
    }
}

static void wlink_on_button(buttons_event_t evt, const buttons_event_info_t *info, void *ctx)
{
    (void)ctx;
    /* Same semantics as usound: PRESSED and REPEAT act (volume ramps while held),
     * RELEASED does not — the dongle sends the HID release itself. */
    if (evt != BUTTONS_EVENT_PRESSED && evt != BUTTONS_EVENT_REPEAT) {
        return;
    }
    switch (info->index) {
    case WLINK_BTN_VOL_DOWN: send_button(HLP_HID_VOL_DOWN); break;
    case WLINK_BTN_VOL_UP:   send_button(HLP_HID_VOL_UP);   break;
    case WLINK_BTN_ACTION:
        if (evt == BUTTONS_EVENT_PRESSED) {
            send_button(HLP_HID_PLAY);
        }
        break;
    default:
        break;
    }
}

static bool wlink_is_live(void *ctx)
{
    (void)ctx;
    return s_w.link_started && (audio_link_is_up() || esp_timer_get_time() < s_w.live_until_us);
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

static esp_err_t wlink_stop(void *ctx);

static esp_err_t wlink_start(const headset_profile_resources_t *res, void *ctx)
{
    (void)ctx;
    if (res == NULL || res->playback_ring == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_w.started) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(&s_w, 0, sizeof(s_w));
    s_w.res = *res;

    /* --- Speaker ---------------------------------------------------------- */
    const audio_playback_config_t pb = {
        .source          = res->playback_ring,
        .sample_rate_hz  = CONFIG_WLINK_SPEAKER_SAMPLE_RATE_HZ,
        .channels        = CONFIG_WLINK_SPEAKER_CHANNELS,
        .bits_per_sample = 16,
        .slot            = AUDIO_PLAYBACK_SLOT_BOTH,
        .port            = -1,
        .pins = { .bclk = CONFIG_HEADSET_SPK_PIN_BCLK,
                  .ws   = CONFIG_HEADSET_SPK_PIN_WS,
                  .dout = CONFIG_HEADSET_SPK_PIN_DOUT,
                  .mclk = CONFIG_HEADSET_SPK_PIN_MCLK },
    };
    esp_err_t err = audio_playback_init(&pb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "audio_playback_init: %s", esp_err_to_name(err));
        goto fail;
    }
    s_w.playback_up = true;

    /* --- Microphone: optional ---------------------------------------------- */
    const audio_capture_config_t cap = {
        .interface       = CONFIG_HEADSET_MIC_PDM ? AUDIO_CAPTURE_IF_PDM : AUDIO_CAPTURE_IF_I2S_STD,
        .sample_rate_hz  = CONFIG_WLINK_MIC_SAMPLE_RATE_HZ,
        .channels        = 1,
        .bits_per_sample = 16,
        .slot            = AUDIO_CAPTURE_SLOT_LEFT,
        .pins = { .clk  = CONFIG_HEADSET_MIC_PIN_CLK,
                  .ws   = CONFIG_HEADSET_MIC_PIN_WS,
                  .din  = CONFIG_HEADSET_MIC_PIN_DIN,
                  .mclk = -1 },
    };
    err = audio_capture_init(&cap);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no microphone (%s); continuing speaker-only", esp_err_to_name(err));
    } else {
        s_w.capture_up = true;
    }

    /* --- The link ----------------------------------------------------------- */
    const audio_link_config_t lc = {
        .role = AUDIO_LINK_ROLE_PERIPHERAL,
        .transport = AUDIO_LINK_TRANSPORT_BLE,
        .tx = { .ring = s_w.capture_up ? audio_capture_get_ring() : NULL,
                .sample_rate_hz = CONFIG_WLINK_MIC_SAMPLE_RATE_HZ, .channels = 1 },
        .rx = { .ring = res->playback_ring,
                .sample_rate_hz = CONFIG_WLINK_SPEAKER_SAMPLE_RATE_HZ,
                .channels = CONFIG_WLINK_SPEAKER_CHANNELS },
        .rx_backlog_cb = playback_backlog,
        .on_event = on_link_event,
        .on_control = on_control,
        .device_name = CONFIG_WLINK_DEVICE_NAME,
    };
    err = audio_link_init(&lc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "audio_link_init: %s", esp_err_to_name(err));
        goto fail;
    }
    s_w.link_init = true;

    /* --- Go live ------------------------------------------------------------ */
    err = audio_playback_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "audio_playback_start: %s", esp_err_to_name(err));
        goto fail;
    }
    if (s_w.capture_up && (err = audio_capture_start()) != ESP_OK) {
        ESP_LOGW(TAG, "audio_capture_start: %s; speaker-only", esp_err_to_name(err));
    }
    err = audio_link_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "audio_link_start: %s", esp_err_to_name(err));
        goto fail;
    }
    s_w.link_started = true;
    s_w.live_until_us = esp_timer_get_time() + (int64_t)CONFIG_WLINK_RECONNECT_GRACE_S * 1000000;

#if CONFIG_WLINK_STATS_PERIOD_S > 0
    const esp_timer_create_args_t targs = { .callback = log_stats, .name = "wlink_stats" };
    if (esp_timer_create(&targs, &s_w.stats_timer) == ESP_OK) {
        (void)esp_timer_start_periodic(s_w.stats_timer, (uint64_t)CONFIG_WLINK_STATS_PERIOD_S * 1000000u);
    }
#endif

    /* Last, so a failure above never leaves sleep blocked. */
    err = pm_policy_lock_acquire(res->stream_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "stream lock: %s", esp_err_to_name(err));
        goto fail;
    }

    s_w.started = true;
    ESP_LOGI(TAG, "up: advertising as '%s', speaker %d Hz x%d, mic %s",
             CONFIG_WLINK_DEVICE_NAME, CONFIG_WLINK_SPEAKER_SAMPLE_RATE_HZ, CONFIG_WLINK_SPEAKER_CHANNELS,
             s_w.capture_up ? "16 kHz mono" : "absent");
    return ESP_OK;

fail:
    /* main/ does not call stop() after a failed start(), so unwind here. */
    (void)wlink_stop(NULL);
    return err;
}

static esp_err_t wlink_stop(void *ctx)
{
    (void)ctx;
    /* Idempotent and safe after a partial start: each step is guarded by the flag its
     * own bring-up set. Radio first, so nothing more is decoded into a ring whose
     * reader is about to stop. */
    if (s_w.started && s_w.res.stream_lock != NULL) {
        (void)pm_policy_lock_release(s_w.res.stream_lock);
    }
    if (s_w.stats_timer != NULL) {
        (void)esp_timer_stop(s_w.stats_timer);
        (void)esp_timer_delete(s_w.stats_timer);
        s_w.stats_timer = NULL;
    }
    if (s_w.link_init) {
        (void)audio_link_deinit();          /* stops the link and the radio first */
        s_w.link_init = false;
        s_w.link_started = false;
    }
    if (s_w.capture_up) {
        (void)audio_capture_stop();
        (void)audio_capture_deinit();
        s_w.capture_up = false;
    }
    if (s_w.playback_up) {
        (void)audio_playback_stop();
        (void)audio_playback_flush();
        (void)audio_playback_deinit();
        s_w.playback_up = false;
    }
    s_w.started = false;
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

const headset_profile_t *wlink_profile(void)
{
    static const headset_profile_t profile = {
        .name      = "wlink",
        .start     = wlink_start,
        .stop      = wlink_stop,
        .on_button = wlink_on_button,
        .is_live   = wlink_is_live,
        .ctx       = NULL,
    };
    return &profile;
}
