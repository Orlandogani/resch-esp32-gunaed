#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "tone_backend.h"

static const char *TAG = "tone";

#define TICK_US        10000u
#define MAX_TICK_BYTES (48000u / 100u * 2u * 2u)   /* 10 ms of 48 kHz stereo 16-bit */
#define MIC_TONE_HZ    440.0f
#define MIC_AMPL       6000.0f
#define TWO_PI         6.28318531f

static struct {
    dongle_backend_resources_t res;
    bool               started;
    ringbuf_reader_t   spk_reader;
    esp_timer_handle_t timer;
    uint32_t           mic_phase;
    size_t             spk_tick_bytes;
    uint32_t           mic_tick_samples;
    int16_t            spk_buf[MAX_TICK_BYTES / 2];
    int16_t            mic_buf[48000u / 100u];
    /* Counters: written on the esp_timer task, read by log_stats. */
    uint64_t           spk_bytes;
    uint32_t           spk_short_ticks;
    uint32_t           spk_rms;
    uint64_t           mic_samples;
} s_t;

/* Runs every 10 ms on the esp_timer task: one render period and one capture period
 * at the dongle's own clock. */
static void tick(void *arg)
{
    (void)arg;

    /* Speaker: take up to one tick's worth, like a DAC would. */
    size_t got = 0;
    esp_err_t err = ringbuf_read(&s_t.spk_reader, s_t.spk_buf, s_t.spk_tick_bytes, &got);
    if (err == ESP_OK) {
        if (got < s_t.spk_tick_bytes) {
            s_t.spk_short_ticks++;
        }
        size_t n = got / sizeof(int16_t);
        if (n > 0) {
            uint64_t acc = 0;
            for (size_t i = 0; i < n; i++) {
                int32_t v = s_t.spk_buf[i];
                acc += (uint64_t)((int64_t)v * v);
            }
            s_t.spk_rms = (uint32_t)sqrtf((float)(acc / n));
        }
        s_t.spk_bytes += got;
    }

    /* Microphone: one tick of a continuous tone. The phase wraps once a second,
     * where 440 whole cycles end, so there is no discontinuity. */
    const uint32_t rate = s_t.res.format.mic_rate_hz;
    for (uint32_t i = 0; i < s_t.mic_tick_samples; i++) {
        float x = TWO_PI * MIC_TONE_HZ * (float)s_t.mic_phase / (float)rate;
        s_t.mic_buf[i] = (int16_t)(MIC_AMPL * sinf(x));
        if (++s_t.mic_phase >= rate) {
            s_t.mic_phase = 0;
        }
    }
    (void)ringbuf_write(s_t.res.mic_ring, s_t.mic_buf, s_t.mic_tick_samples * sizeof(int16_t));
    s_t.mic_samples += s_t.mic_tick_samples;
}

static esp_err_t tone_stop(void *ctx);

static esp_err_t tone_start(const dongle_backend_resources_t *res, void *ctx)
{
    (void)ctx;
    if (res == NULL || res->speaker_ring == NULL || res->mic_ring == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_t.started) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(&s_t, 0, sizeof(s_t));
    s_t.res = *res;
    s_t.spk_tick_bytes = (size_t)(res->format.speaker_rate_hz / 100u) * res->format.speaker_channels * 2u;
    s_t.mic_tick_samples = res->format.mic_rate_hz / 100u;
    if (s_t.spk_tick_bytes > sizeof(s_t.spk_buf) || s_t.mic_tick_samples > sizeof(s_t.mic_buf) / 2) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ringbuf_reader_open(res->speaker_ring, &s_t.spk_reader);
    if (err != ESP_OK) {
        return err;
    }
    const esp_timer_create_args_t args = { .callback = tick, .name = "tone" };
    err = esp_timer_create(&args, &s_t.timer);
    if (err == ESP_OK) {
        err = esp_timer_start_periodic(s_t.timer, TICK_US);
    }
    if (err != ESP_OK) {
        (void)tone_stop(NULL);
        return err;
    }
    s_t.started = true;
    ESP_LOGI(TAG, "up: speaker consumed at %lu Hz x%u, mic = %.0f Hz tone at %lu Hz",
             (unsigned long)res->format.speaker_rate_hz, res->format.speaker_channels,
             (double)MIC_TONE_HZ, (unsigned long)res->format.mic_rate_hz);
    return ESP_OK;
}

static esp_err_t tone_stop(void *ctx)
{
    (void)ctx;
    if (s_t.timer != NULL) {
        (void)esp_timer_stop(s_t.timer);
        (void)esp_timer_delete(s_t.timer);
        s_t.timer = NULL;
    }
    s_t.started = false;
    return ESP_OK;
}

static uint32_t tone_backlog(void *ctx)
{
    (void)ctx;
    return s_t.started ? (uint32_t)ringbuf_available(&s_t.spk_reader) : 0;
}

static bool tone_linked(void *ctx)
{
    (void)ctx;
    return s_t.started;
}

static void tone_log(void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "speaker %llu B consumed, RMS %lu, short ticks %lu | mic %llu samples",
             (unsigned long long)s_t.spk_bytes, (unsigned long)s_t.spk_rms,
             (unsigned long)s_t.spk_short_ticks, (unsigned long long)s_t.mic_samples);
}

const dongle_backend_t *tone_backend(void)
{
    static const dongle_backend_t backend = {
        .name            = "tone",
        .start           = tone_start,
        .stop            = tone_stop,
        .speaker_backlog = tone_backlog,
        .is_linked       = tone_linked,
        .log_stats       = tone_log,
        .ctx             = NULL,
    };
    return &backend;
}
