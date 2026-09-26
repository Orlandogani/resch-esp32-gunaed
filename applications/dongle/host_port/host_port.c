#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "host_port.h"

static const char *TAG = "host";

static host_port_config_t s_cfg;
static bool               s_started;

bool host_port_is_usb(void)
{
#if CONFIG_DONGLE_ENABLE_USB
    return true;
#else
    return false;
#endif
}

#if CONFIG_DONGLE_ENABLE_USB

/* -------------------------------------------------------------------------- */
/* Real host: UAC2 duplex + HID consumer control                               */
/* -------------------------------------------------------------------------- */

#include "usb_audio.h"
#include "usb_device.h"
#include "usb_hid.h"

/* One byte, eight bits, no report ID — the same consumer-control report the
 * headset's usound profile exposes, so both products look alike to the host. The
 * bit order is headset_link_proto.h's HLP_HID_*. */
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

static bool s_usb_up;
static bool s_usb_running;

static uint32_t backlog(void *ctx)
{
    (void)ctx;
    return s_cfg.speaker_backlog_cb(s_cfg.ctx);
}

static void on_usb_control(usb_audio_stream_t which, bool mute, int16_t volume_db256, void *ctx)
{
    (void)ctx;
    if (s_cfg.on_volume != NULL) {
        s_cfg.on_volume(which == USB_AUDIO_STREAM_MIC, mute, volume_db256, s_cfg.ctx);
    }
}

static void on_usb_event(usb_device_event_t evt, void *ctx)
{
    (void)ctx;
    static const char *const names[] = { "mounted", "unmounted", "suspended", "resumed" };
    ESP_LOGI(TAG, "USB %s", names[evt]);
}

esp_err_t host_port_start(const host_port_config_t *cfg)
{
    if (cfg == NULL || cfg->speaker_ring == NULL || cfg->mic_ring == NULL || cfg->speaker_backlog_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    s_cfg = *cfg;

    const usb_device_config_t dev = { .product = CONFIG_DONGLE_USB_PRODUCT };
    esp_err_t err = usb_device_init(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_device_init: %s", esp_err_to_name(err));
        return err;
    }
    s_usb_up = true;
    (void)usb_device_set_event_cb(on_usb_event, NULL);

    const usb_audio_config_t uac = {
        .direction = USB_AUDIO_DIR_HEADSET,
        .speaker = { .ring = cfg->speaker_ring, .sample_rate_hz = cfg->speaker_rate_hz,
                     .channels = cfg->speaker_channels, .bits_per_sample = 16 },
        .mic     = { .ring = cfg->mic_ring, .sample_rate_hz = cfg->mic_rate_hz,
                     .channels = 1, .bits_per_sample = 16 },
        .speaker_backlog_cb = backlog,
        .speaker_target_ms  = cfg->speaker_target_ms,
        .on_control         = on_usb_control,
    };
    err = usb_audio_init(&uac);
    if (err != ESP_OK) {
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
    s_usb_running = true;
    s_started = true;
    ESP_LOGI(TAG, "USB duplex audio: speaker %lu Hz x%u, mic %lu Hz mono, path target %u ms",
             (unsigned long)cfg->speaker_rate_hz, cfg->speaker_channels,
             (unsigned long)cfg->mic_rate_hz, cfg->speaker_target_ms);
    return ESP_OK;

fail:
    (void)host_port_stop();
    return err;
}

esp_err_t host_port_stop(void)
{
    if (s_usb_running) {
        (void)usb_device_stop();
        s_usb_running = false;
    }
    if (s_usb_up) {
        (void)usb_device_deinit();
        s_usb_up = false;
    }
    s_started = false;
    return ESP_OK;
}

void host_port_hid_tap(uint8_t bits)
{
    if (!usb_hid_ready()) {
        return;  /* Not enumerated, or the interface is idle: dropping is correct. */
    }
    uint8_t report = bits;
    (void)usb_hid_report_send(0, &report, sizeof(report));
    report = 0;
    (void)usb_hid_report_send(0, &report, sizeof(report));
}

void host_port_log_stats(void)
{
    usb_audio_stats_t st;
    if (usb_audio_stats(&st) != ESP_OK) {
        return;
    }
    ESP_LOGI(TAG, "USB spk %s pkts %lu (nom %lu short %lu long %lu) backlog %lu/%lu B fb 0x%08lx clamp %lu | "
                  "mic %s pkts %lu (-1 %lu +1 %lu zlp %lu)",
             st.speaker.streaming ? "on" : "off", (unsigned long)st.speaker.packets,
             (unsigned long)st.speaker.packets_nominal, (unsigned long)st.speaker.packets_short,
             (unsigned long)st.speaker.packets_long, (unsigned long)st.speaker.backlog_bytes,
             (unsigned long)st.speaker.target_bytes, (unsigned long)st.speaker.feedback_value,
             (unsigned long)st.speaker.feedback_clamped,
             st.mic.streaming ? "on" : "off", (unsigned long)st.mic.packets,
             (unsigned long)st.mic.packets_minus_one, (unsigned long)st.mic.packets_plus_one,
             (unsigned long)st.mic.zero_length_packets);
}

#else /* !CONFIG_DONGLE_ENABLE_USB */

/* -------------------------------------------------------------------------- */
/* Emulated host: console-safe                                                  */
/* -------------------------------------------------------------------------- */

#define TICK_US          1000u
#define SERVO_PERIOD_MS  16u         /* matches CONFIG_USB_AUDIO_SPEAKER_FB_PERIOD_MS's default */
#define TONE_HZ          1000.0f
#define TONE_AMPL        8000.0f
#define TWO_PI           6.28318531f
#define MAX_SPK_SAMPLES  ((48000u / 1000u + 1u) * 2u)
#define MAX_MIC_SAMPLES  (48000u / 1000u)

static struct {
    esp_timer_handle_t timer;
    ringbuf_reader_t   mic_reader;
    uint32_t           spk_nominal;      /* sample frames per ms            */
    uint32_t           mic_nominal;
    uint32_t           target_bytes;
    uint32_t           hysteresis_bytes; /* one nominal packet              */
    int32_t            adj;              /* -1, 0, +1 sample frames per ms  */
    uint64_t           backlog_acc;
    uint32_t           backlog_n;
    uint32_t           phase;
    int16_t            spk[MAX_SPK_SAMPLES];
    int16_t            mic[MAX_MIC_SAMPLES];
    /* Counters */
    uint64_t           spk_frames;
    uint32_t           adj_up;
    uint32_t           adj_down;
    uint32_t           last_backlog;
    uint64_t           mic_samples;
    uint32_t           mic_short;
    uint32_t           mic_rms;
    uint64_t           mic_acc;
    uint32_t           mic_acc_n;
} s_e;

/* 1 ms, esp_timer task. The emulated host follows the servo exactly as a real one
 * follows the feedback endpoint: nominal ± 1 sample frame per millisecond. */
static void emu_tick(void *arg)
{
    (void)arg;

    /* Servo: average the backlog over the period, then decide the trim. */
    uint32_t b = s_cfg.speaker_backlog_cb(s_cfg.ctx);
    s_e.backlog_acc += b;
    if (++s_e.backlog_n >= SERVO_PERIOD_MS) {
        uint32_t avg = (uint32_t)(s_e.backlog_acc / s_e.backlog_n);
        s_e.last_backlog = avg;
        s_e.backlog_acc = 0;
        s_e.backlog_n = 0;
        if (avg + s_e.hysteresis_bytes < s_e.target_bytes) {
            s_e.adj = 1;
            s_e.adj_up++;
        } else if (avg > s_e.target_bytes + s_e.hysteresis_bytes) {
            s_e.adj = -1;
            s_e.adj_down++;
        } else {
            s_e.adj = 0;
        }
    }

    /* Speaker: one packet of a continuous tone. */
    uint32_t frames = (uint32_t)((int32_t)s_e.spk_nominal + s_e.adj);
    const uint32_t rate = s_cfg.speaker_rate_hz;
    for (uint32_t i = 0; i < frames; i++) {
        int16_t v = (int16_t)(TONE_AMPL * sinf(TWO_PI * TONE_HZ * (float)s_e.phase / (float)rate));
        for (uint8_t c = 0; c < s_cfg.speaker_channels; c++) {
            s_e.spk[i * s_cfg.speaker_channels + c] = v;
        }
        if (++s_e.phase >= rate) {
            s_e.phase = 0;
        }
    }
    (void)ringbuf_write(s_cfg.speaker_ring, s_e.spk, frames * s_cfg.speaker_channels * sizeof(int16_t));
    s_e.spk_frames += frames;

    /* Microphone: one nominal packet, measured. */
    size_t got = 0;
    if (ringbuf_read(&s_e.mic_reader, s_e.mic, s_e.mic_nominal * sizeof(int16_t), &got) == ESP_OK) {
        size_t n = got / sizeof(int16_t);
        if (n < s_e.mic_nominal) {
            s_e.mic_short++;
        }
        for (size_t i = 0; i < n; i++) {
            int32_t v = s_e.mic[i];
            s_e.mic_acc += (uint64_t)((int64_t)v * v);
        }
        s_e.mic_acc_n += (uint32_t)n;
        s_e.mic_samples += n;
        if (s_e.mic_acc_n >= s_cfg.mic_rate_hz / 10u) {      /* 100 ms window */
            s_e.mic_rms = (uint32_t)sqrtf((float)(s_e.mic_acc / s_e.mic_acc_n));
            s_e.mic_acc = 0;
            s_e.mic_acc_n = 0;
        }
    }
}

esp_err_t host_port_start(const host_port_config_t *cfg)
{
    if (cfg == NULL || cfg->speaker_ring == NULL || cfg->mic_ring == NULL || cfg->speaker_backlog_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    s_cfg = *cfg;
    memset(&s_e, 0, sizeof(s_e));
    s_e.spk_nominal = cfg->speaker_rate_hz / 1000u;
    s_e.mic_nominal = cfg->mic_rate_hz / 1000u;
    if ((s_e.spk_nominal + 1u) * cfg->speaker_channels > MAX_SPK_SAMPLES || s_e.mic_nominal > MAX_MIC_SAMPLES) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t bytes_per_ms = s_e.spk_nominal * cfg->speaker_channels * (uint32_t)sizeof(int16_t);
    s_e.target_bytes = bytes_per_ms * cfg->speaker_target_ms;
    s_e.hysteresis_bytes = bytes_per_ms;

    esp_err_t err = ringbuf_reader_open(cfg->mic_ring, &s_e.mic_reader);
    if (err != ESP_OK) {
        return err;
    }
    const esp_timer_create_args_t args = { .callback = emu_tick, .name = "host_emu" };
    err = esp_timer_create(&args, &s_e.timer);
    if (err == ESP_OK) {
        err = esp_timer_start_periodic(s_e.timer, TICK_US);
    }
    if (err != ESP_OK) {
        (void)host_port_stop();
        return err;
    }
    s_started = true;
    ESP_LOGW(TAG, "EMULATED host (DONGLE_ENABLE_USB=n): %.0f Hz tone into the speaker path at "
                  "%lu Hz x%u, servo target %u ms; USB-Serial/JTAG console kept",
             (double)TONE_HZ, (unsigned long)cfg->speaker_rate_hz, cfg->speaker_channels,
             cfg->speaker_target_ms);
    return ESP_OK;
}

esp_err_t host_port_stop(void)
{
    if (s_e.timer != NULL) {
        (void)esp_timer_stop(s_e.timer);
        (void)esp_timer_delete(s_e.timer);
        s_e.timer = NULL;
    }
    s_started = false;
    return ESP_OK;
}

void host_port_hid_tap(uint8_t bits)
{
    ESP_LOGI(TAG, "HID tap 0x%02x (emulated host: not sent)", bits);
}

void host_port_log_stats(void)
{
    ESP_LOGI(TAG, "emu spk %llu frames, trim %+ld (up %lu down %lu), path backlog %lu/%lu B | "
                  "mic %llu samples, short %lu, RMS %lu",
             (unsigned long long)s_e.spk_frames, (long)s_e.adj, (unsigned long)s_e.adj_up,
             (unsigned long)s_e.adj_down, (unsigned long)s_e.last_backlog, (unsigned long)s_e.target_bytes,
             (unsigned long long)s_e.mic_samples, (unsigned long)s_e.mic_short, (unsigned long)s_e.mic_rms);
}

#endif /* CONFIG_DONGLE_ENABLE_USB */
