/**
 * Phase 2 profile: wireless headset, peripheral end of subsys/audio_link.
 *
 * Bring-up order, and why:
 *   hs_audio_open        - the sink (and codec) must exist before the link can fill its
 *                          ring; the microphone is optional (no mic -> speaker only)
 *   audio_link_init      - decoder writes main/'s playback ring (single writer);
 *                          encoder reads the mic ring through its own cursor
 *   hs_audio_start       - running before the first frame is decoded
 *   audio_link_start     - BLE up and advertising; the dongle connects when it can
 *
 * Besides audio the profile carries the headset's status to the dongle: battery, wear
 * and head pose (headset_link_proto, ADR-025 integration).
 *
 * The profile stays live while the link is up, and for CONFIG_WLINK_RECONNECT_GRACE_S
 * after it drops: a dongle that comes back quickly finds the headset still
 * advertising and no mode change happens.
 */
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "audio_link.h"
#include "headset_link_proto.h"
#include "hs_audio.h"
#include "wlink.h"

static const char *TAG = "wlink";

static struct {
    headset_profile_resources_t res;
    bool    started;
    bool    audio_up;
    bool    capture_up;
    uint8_t battery_msg[HLP_BATTERY_LEN];   /* last battery report, re-sent on link-up */
    bool    battery_known;
    int64_t pose_next_us;                   /* head-pose rate limit                    */
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
    if (audio_link_stats(&l) != ESP_OK) {
        return;
    }
    ESP_LOGI(TAG, "LINK %s rx %lu lost %lu late %lu conceal %lu resync %lu | tx %lu busy %lu | "
                  "dec %lu/%lu us enc %lu/%lu us stack %lu | PLAY backlog %lu B",
             l.up ? "UP" : "down", (unsigned long)l.rx_frames, (unsigned long)l.rx_lost,
             (unsigned long)l.rx_late, (unsigned long)l.rx_concealed, (unsigned long)l.rx_resyncs,
             (unsigned long)l.tx_frames, (unsigned long)l.tx_busy,
             (unsigned long)l.decode_us_avg, (unsigned long)l.decode_us_max,
             (unsigned long)l.encode_us_avg, (unsigned long)l.encode_us_max,
             (unsigned long)l.task_stack_free_min, (unsigned long)hs_audio_playback_backlog());
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
    return hs_audio_playback_backlog();
}

static void send_msg(const uint8_t *msg, size_t len)
{
    esp_err_t err = audio_link_send_control(msg, len);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "control 0x%02x not sent: %s", msg[0], esp_err_to_name(err));
    }
}

static void on_link_event(audio_link_event_t evt, void *ctx)
{
    (void)ctx;
    switch (evt) {
    case AUDIO_LINK_EVT_UP:
        ESP_LOGI(TAG, "dongle linked");
        s_w.live_until_us = INT64_MAX;
        if (s_w.battery_known) {
            send_msg(s_w.battery_msg, sizeof(s_w.battery_msg));
        }
        break;
    case AUDIO_LINK_EVT_DOWN:
        ESP_LOGI(TAG, "dongle lost; advertising for %d s", CONFIG_WLINK_RECONNECT_GRACE_S);
        s_w.live_until_us = esp_timer_get_time() + (int64_t)CONFIG_WLINK_RECONNECT_GRACE_S * 1000000;
        /* Drop the stale tail rather than play it after the gap (DES-APB-007). */
        hs_audio_flush();
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
        if (msg[1] == 0) {
            (void)hs_audio_set_host_volume(msg[2] != 0, vol);   /* codec DAC, capped */
        }
    }
}

static void send_button(uint8_t bits)
{
    const uint8_t msg[HLP_BUTTON_LEN] = { HLP_MSG_BUTTON, bits };
    send_msg(msg, sizeof(msg));
}

/* One control step; the dongle sends the HID release itself. */
static void wlink_on_control(headset_ctrl_t ctrl, void *ctx)
{
    (void)ctx;
    switch (ctrl) {
    case HEADSET_CTRL_VOL_DOWN:   send_button(HLP_HID_VOL_DOWN); break;
    case HEADSET_CTRL_VOL_UP:     send_button(HLP_HID_VOL_UP);   break;
    case HEADSET_CTRL_PLAY_PAUSE: send_button(HLP_HID_PLAY);     break;
    case HEADSET_CTRL_MIC_MUTE:   send_button(HLP_HID_MUTE);     break;
    default:                      break;
    }
}

static int16_t q14(float v)
{
    float s = v * 16384.0f;
    if (s > 32767.0f) {
        s = 32767.0f;
    } else if (s < -32768.0f) {
        s = -32768.0f;
    }
    return (int16_t)s;
}

/* Battery and wear on the main task; head pose on the IMU task at the sensor rate,
 * thinned to CONFIG_WLINK_POSE_MAX_HZ so the link's CONTROL budget is not the IMU's. */
static void wlink_on_status(const headset_status_t *st, void *ctx)
{
    (void)ctx;
    switch (st->kind) {
    case HEADSET_STATUS_BATTERY:
        s_w.battery_msg[0] = HLP_MSG_BATTERY;
        s_w.battery_msg[1] = st->battery.percent;
        s_w.battery_msg[2] = (uint8_t)((st->battery.ext_power ? HLP_BATTERY_EXT_POWER : 0u) |
                                       (st->battery.charging ? HLP_BATTERY_CHARGING : 0u));
        s_w.battery_known = true;
        send_msg(s_w.battery_msg, sizeof(s_w.battery_msg));
        break;
    case HEADSET_STATUS_WEAR: {
        const uint8_t msg[HLP_WEAR_LEN] = { HLP_MSG_WEAR, st->worn ? 1u : 0u };
        send_msg(msg, sizeof(msg));
        break;
    }
    case HEADSET_STATUS_HEAD_POSE: {
        int64_t now = esp_timer_get_time();
        if (now < s_w.pose_next_us || !audio_link_is_up()) {
            break;
        }
        s_w.pose_next_us = now + 1000000 / CONFIG_WLINK_POSE_MAX_HZ;
        int16_t c[4] = { q14(st->pose.w), q14(st->pose.x), q14(st->pose.y), q14(st->pose.z) };
        uint8_t msg[HLP_HEAD_POSE_LEN] = { HLP_MSG_HEAD_POSE };
        for (int i = 0; i < 4; i++) {
            msg[1 + 2 * i] = (uint8_t)((uint16_t)c[i] & 0xFFu);
            msg[2 + 2 * i] = (uint8_t)((uint16_t)c[i] >> 8);
        }
        send_msg(msg, sizeof(msg));
        break;
    }
    default:
        break;
    }
}

static bool wlink_is_live(void *ctx)
{
    (void)ctx;
    return s_w.link_started && (audio_link_is_up() || esp_timer_get_time() < s_w.live_until_us);
}

static bool wlink_is_connected(void *ctx)
{
    (void)ctx;
    return s_w.link_started && audio_link_is_up();
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

    /* --- Audio: speaker (and codec), optional microphone --------------- */
    const hs_audio_config_t ac = {
        .playback_ring = res->playback_ring,
        .spk_rate_hz   = CONFIG_WLINK_SPEAKER_SAMPLE_RATE_HZ,
        .spk_channels  = CONFIG_WLINK_SPEAKER_CHANNELS,
        .mic_rate_hz   = CONFIG_WLINK_MIC_SAMPLE_RATE_HZ,
    };
    esp_err_t err = hs_audio_open(&ac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hs_audio_open: %s", esp_err_to_name(err));
        goto fail;
    }
    s_w.audio_up = true;
    s_w.capture_up = (hs_audio_mic_ring() != NULL);

    /* --- The link ----------------------------------------------------------- */
    const audio_link_config_t lc = {
        .role = AUDIO_LINK_ROLE_PERIPHERAL,
        .transport = AUDIO_LINK_TRANSPORT_BLE,
        .tx = { .ring = hs_audio_mic_ring(),
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
    err = hs_audio_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hs_audio_start: %s", esp_err_to_name(err));
        goto fail;
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
    if (s_w.audio_up) {
        (void)hs_audio_close();
        s_w.audio_up = false;
        s_w.capture_up = false;
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
        .on_control = wlink_on_control,
        .on_status  = wlink_on_status,
        .caps       = HEADSET_PROFILE_CAP_HEAD_POSE,
        .is_live   = wlink_is_live,
        .is_connected = wlink_is_connected,
        .ctx       = NULL,
    };
    return &profile;
}
