#include <string.h>

#include "esp_log.h"
#include "sdkconfig.h"

#include "audio_link.h"
#include "headset_link_proto.h"
#include "wlink_central.h"

static const char *TAG = "wlink";

static struct {
    dongle_backend_resources_t res;
    bool started;
} s_c;

static void on_link_event(audio_link_event_t evt, void *ctx)
{
    (void)ctx;
    switch (evt) {
    case AUDIO_LINK_EVT_UP:
        ESP_LOGI(TAG, "headset linked");
        break;
    case AUDIO_LINK_EVT_DOWN:
        ESP_LOGI(TAG, "headset lost; scanning");
        break;
    case AUDIO_LINK_EVT_FORMAT_MISMATCH:
        ESP_LOGE(TAG, "headset sends a different audio format; check DONGLE_* against WLINK_*");
        break;
    default:
        break;
    }
}

/* Link task context: the tap goes to the host side's HID queue, which is safe from
 * any task and never blocks. */
static void on_control(const uint8_t *msg, size_t len, void *ctx)
{
    (void)ctx;
    if (len >= HLP_BUTTON_LEN && msg[0] == HLP_MSG_BUTTON) {
        s_c.res.hid_tap(msg[1]);
    }
}

static esp_err_t central_start(const dongle_backend_resources_t *res, void *ctx)
{
    (void)ctx;
    if (res == NULL || res->speaker_ring == NULL || res->mic_ring == NULL || res->hid_tap == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_c.started) {
        return ESP_ERR_INVALID_STATE;
    }
    s_c.res = *res;

    const audio_link_config_t lc = {
        .role = AUDIO_LINK_ROLE_CENTRAL,
        .transport = AUDIO_LINK_TRANSPORT_BLE,
        .tx = { .ring = res->speaker_ring,
                .sample_rate_hz = res->format.speaker_rate_hz,
                .channels = res->format.speaker_channels },
        .rx = { .ring = res->mic_ring,
                .sample_rate_hz = res->format.mic_rate_hz,
                .channels = 1 },
        /* No rx_backlog_cb: the host side drains the mic ring with its own IN flow
         * control, so there is nothing for the headset to regulate. */
        .on_event = on_link_event,
        .on_control = on_control,
        .device_name = CONFIG_DONGLE_DEVICE_NAME,
    };
    esp_err_t err = audio_link_init(&lc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "audio_link_init: %s", esp_err_to_name(err));
        return err;
    }
    err = audio_link_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "audio_link_start: %s", esp_err_to_name(err));
        (void)audio_link_deinit();
        return err;
    }
    s_c.started = true;
    return ESP_OK;
}

static esp_err_t central_stop(void *ctx)
{
    (void)ctx;
    if (s_c.started) {
        (void)audio_link_deinit();
        s_c.started = false;
    }
    return ESP_OK;
}

static uint32_t central_backlog(void *ctx)
{
    (void)ctx;
    return audio_link_path_backlog_bytes();
}

static bool central_linked(void *ctx)
{
    (void)ctx;
    return audio_link_is_up();
}

static void central_host_volume(bool mic, bool mute, int16_t volume_db256, void *ctx)
{
    (void)ctx;
    const uint8_t msg[HLP_HOST_VOLUME_LEN] = {
        HLP_MSG_HOST_VOLUME, mic ? 1u : 0u, mute ? 1u : 0u,
        (uint8_t)((uint16_t)volume_db256 & 0xFFu), (uint8_t)((uint16_t)volume_db256 >> 8),
    };
    /* Best effort: informational, and the host re-sends on the next change. */
    (void)audio_link_send_control(msg, sizeof(msg));
}

static void central_log(void *ctx)
{
    (void)ctx;
    audio_link_stats_t st;
    if (audio_link_stats(&st) != ESP_OK) {
        return;
    }
    ESP_LOGI(TAG, "%s | tx %lu busy %lu skip %lu | rx %lu lost %lu late %lu conceal %lu | "
                  "peer backlog %u fr (%lu ms old) | enc %lu/%lu us dec %lu/%lu us | stack free %lu",
             st.up ? "UP" : "down",
             (unsigned long)st.tx_frames, (unsigned long)st.tx_busy, (unsigned long)st.tx_skipped,
             (unsigned long)st.rx_frames, (unsigned long)st.rx_lost, (unsigned long)st.rx_late,
             (unsigned long)st.rx_concealed, st.peer_backlog_frames,
             (unsigned long)(st.peer_report_age_ms == UINT32_MAX ? 0 : st.peer_report_age_ms),
             (unsigned long)st.encode_us_avg, (unsigned long)st.encode_us_max,
             (unsigned long)st.decode_us_avg, (unsigned long)st.decode_us_max,
             (unsigned long)st.task_stack_free_min);
}

const dongle_backend_t *wlink_central_backend(void)
{
    static const dongle_backend_t backend = {
        .name            = "wlink",
        .start           = central_start,
        .stop            = central_stop,
        .speaker_backlog = central_backlog,
        .is_linked       = central_linked,
        .on_host_volume  = central_host_volume,
        .log_stats       = central_log,
        .ctx             = NULL,
    };
    return &backend;
}
