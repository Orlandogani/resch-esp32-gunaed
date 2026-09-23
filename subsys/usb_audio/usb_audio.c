#include "usb_audio.h"
#include <stdatomic.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "tusb.h"
#include "class/audio/audio_device.h"
#include "usb_device.h"

static const char *TAG = "usb_audio";

#if CFG_TUD_AUDIO < 1
#error "usb_audio requires CONFIG_TINYUSB_AUDIO_ENABLED"
#endif

#define AUDIO_FUNC 0   /* TinyUSB audio function index; this module is a singleton. */

/* Entity IDs. Fixed here rather than by a TinyUSB template, because no template
 * covers a duplex function. The two directions are separate chains with separate
 * clock sources, so a 16 kHz microphone can sit beside a 48 kHz speaker. */
#define ENT_MIC_CLOCK   0x01
#define ENT_MIC_IT      0x02   /* Physical microphone                */
#define ENT_MIC_FU      0x03
#define ENT_MIC_OT      0x04   /* USB streaming, device to host      */
#define ENT_SPK_CLOCK   0x05
#define ENT_SPK_IT      0x06   /* USB streaming, host to device      */
#define ENT_SPK_FU      0x07
#define ENT_SPK_OT      0x08   /* Physical speaker                   */

/* UAC2 feedback packets are always 4 bytes on the wire (audio_device.c formats
 * 10.14 into 3 bytes for UAC1 only). */
#define FB_EP_SIZE 4

/* Bytes a feeder tick moves at most, per direction: two nominal packets keeps the
 * FIFO level right without letting one tick monopolise the task. */
#define CHUNK_MAX_IN  (2u * CONFIG_TINYUSB_AUDIO_EP_IN_SZ_MAX)
#if CFG_TUD_AUDIO_ENABLE_EP_OUT
#define CHUNK_MAX_OUT (2u * CONFIG_TINYUSB_AUDIO_EP_OUT_SZ_MAX)
#else
#define CHUNK_MAX_OUT 1u
#endif

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
    bool     present;
    usb_audio_stream_config_t cfg;
    uint32_t sample_rate;
    uint16_t bytes_per_frame;   /* One sample across all channels. */
    uint16_t nominal_packet;    /* Bytes the host should see per 1 ms frame. */
    uint16_t ep_size;           /* wMaxPacketSize: nominal + one sample of slack. */
    uint8_t  itf;               /* AS interface number assigned at descriptor time. */
    bool     mute;
    int16_t  volume;
    volatile bool streaming;
} stream_t;

static bool               s_initialised;
static usb_audio_direction_t s_direction;
static usb_audio_control_cb_t s_on_control;
static void              *s_ctx;

static stream_t           s_mic;
static stream_t           s_spk;

static TaskHandle_t       s_feeder;
static esp_timer_handle_t s_tick;

static ringbuf_reader_t   s_mic_reader;

/* IN FIFO fill is our own writes minus the ISR's completions, which is exact
 * because tu_fifo is strictly single-producer/single-consumer here. */
static _Atomic uint32_t   s_fifo_in;
static _Atomic uint32_t   s_fifo_out;

static usb_audio_stats_t  s_stats;

#if CFG_TUD_AUDIO_ENABLE_EP_OUT
/* Feedback servo (ADR-021, DES-AUD-020). Speaker-only, so guarded: an unused
 * static here is a real sign that the direction was compiled out by mistake. */
static usb_audio_backlog_cb_t s_backlog_cb;
static uint32_t           s_fb_nominal;      /* 16.16 samples per frame at nominal. */
static uint32_t           s_fb_target_bytes; /* Backlog setpoint. */
static uint64_t           s_fb_backlog_sum;  /* Mean over one servo period. */
static uint32_t           s_fb_backlog_n;
#endif

static uint8_t            s_chunk[CHUNK_MAX_IN > CHUNK_MAX_OUT ? CHUNK_MAX_IN : CHUNK_MAX_OUT];

/* Control state, one range per direction. */
static audio20_control_range_4_n_t(1) s_mic_rate_range;
static audio20_control_range_4_n_t(1) s_spk_rate_range;
static audio20_control_range_2_n_t(1) s_volume_range;

/* -------------------------------------------------------------------------- */
/* Microphone: ring -> TinyUSB IN FIFO (DES-AUD-002..004)                      */
/* -------------------------------------------------------------------------- */

static inline uint32_t fifo_in_fill(void)
{
    return atomic_load_explicit(&s_fifo_in, memory_order_relaxed) -
           atomic_load_explicit(&s_fifo_out, memory_order_relaxed);
}

static void mic_feed_once(void)
{
    const uint32_t depth = CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ;
    uint32_t fill = fifo_in_fill();
    if (fill >= depth) {
        return;
    }
    uint32_t space = depth - fill;
    uint32_t want = space < CHUNK_MAX_IN ? space : CHUNK_MAX_IN;

    /* Keep whole audio frames: never split one across two FIFO writes. */
    want -= want % s_mic.bytes_per_frame;
    if (want == 0) {
        return;
    }

    size_t got = 0;
    esp_err_t err = ringbuf_read(&s_mic_reader, s_chunk, want, &got);
    if (err == ESP_ERR_INVALID_STATE) {
        s_stats.mic.ring_overruns++;
        return; /* Cursor resynchronised; next tick resumes. */
    }
    if (err != ESP_OK || got == 0) {
        return;
    }
    got -= got % s_mic.bytes_per_frame;

    uint16_t written = tud_audio_n_write(AUDIO_FUNC, s_chunk, (uint16_t)got);
    atomic_fetch_add_explicit(&s_fifo_in, written, memory_order_relaxed);
    s_stats.mic.bytes_streamed += written;
    if (written < got) {
        s_stats.mic.fifo_overflows++; /* Our accounting and TinyUSB's disagree: investigate. */
    }
}

/* -------------------------------------------------------------------------- */
/* Speaker: TinyUSB OUT FIFO -> ring, and the feedback servo                   */
/* -------------------------------------------------------------------------- */

#if CFG_TUD_AUDIO_ENABLE_EP_OUT

static void spk_drain_once(void)
{
    uint16_t avail = tud_audio_n_available(AUDIO_FUNC);
    while (avail > 0) {
        uint16_t want = avail < CHUNK_MAX_OUT ? avail : (uint16_t)CHUNK_MAX_OUT;
        want -= want % s_spk.bytes_per_frame;
        if (want == 0) {
            break;
        }
        uint16_t got = tud_audio_n_read(AUDIO_FUNC, s_chunk, want);
        if (got == 0) {
            break;
        }
        if (ringbuf_write(s_spk.cfg.ring, s_chunk, got) != ESP_OK) {
            s_stats.speaker.ring_write_failures++;
        } else {
            s_stats.speaker.bytes_received += got;
        }
        avail = (avail > got) ? (uint16_t)(avail - got) : 0;
    }
    s_stats.speaker.fifo_fill_bytes = tud_audio_n_available(AUDIO_FUNC);
}

/**
 * Proportional servo on the playback ring's backlog (ADR-021, FW-AUD-056).
 *
 * The measurement is the *ring's* backlog, not the OUT FIFO's fill: this module
 * empties that FIFO every millisecond, so its level shows host jitter, while the
 * ring is the elastic buffer whose level shows genuine clock drift.
 *
 * Authority is one sample per frame in each direction, which is also the clamp
 * TinyUSB applies, so asking for more would only be silently trimmed. Full
 * authority is reached at a backlog error of one setpoint.
 */
static void spk_feedback_update(void)
{
    s_fb_backlog_sum += s_backlog_cb(s_ctx);
    s_fb_backlog_n++;
    if (s_fb_backlog_n < CONFIG_USB_AUDIO_SPEAKER_FB_PERIOD_MS) {
        return;
    }

    uint32_t backlog = (uint32_t)(s_fb_backlog_sum / s_fb_backlog_n);
    s_fb_backlog_sum = 0;
    s_fb_backlog_n = 0;
    s_stats.speaker.backlog_bytes = backlog;

    /* Positive error means too much audio is queued: ask the host to slow down. */
    int64_t error = (int64_t)backlog - (int64_t)s_fb_target_bytes;
    int64_t adj = -(error * (int64_t)(1u << 16)) / (int64_t)s_fb_target_bytes;

    const int64_t authority = (int64_t)(1u << 16);   /* One sample per frame. */
    if (adj > authority) {
        adj = authority;
        s_stats.speaker.feedback_clamped++;
    } else if (adj < -authority) {
        adj = -authority;
        s_stats.speaker.feedback_clamped++;
    }

    uint32_t fb = (uint32_t)((int64_t)s_fb_nominal + adj);
    tud_audio_n_fb_set(AUDIO_FUNC, fb);
    s_stats.speaker.feedback_value = fb;
    s_stats.speaker.feedback_updates++;
}

/* TinyUSB asks once per alternate-setting change how to compute feedback. We
 * compute it ourselves from the ring, so the driver's own methods are disabled —
 * but sample_freq is still read, to set the driver's min/max clamp. */
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf,
                                  audio_feedback_params_t *feedback_param)
{
    feedback_param->method = AUDIO_FEEDBACK_METHOD_DISABLED;
    feedback_param->sample_freq = s_spk.sample_rate;
}

/* ISR context. Counting only — no logging, no ring access. */
bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id,
                           uint8_t ep_out, uint8_t cur_alt_setting)
{
    s_stats.speaker.packets++;
    if (n_bytes_received == 0) {
        s_stats.speaker.zero_length_packets++;
    } else if (n_bytes_received == s_spk.nominal_packet) {
        s_stats.speaker.packets_nominal++;
    } else if (n_bytes_received > s_spk.nominal_packet) {
        s_stats.speaker.packets_long++;
    } else {
        s_stats.speaker.packets_short++;
    }
    return true;
}

#endif /* CFG_TUD_AUDIO_ENABLE_EP_OUT */

/* -------------------------------------------------------------------------- */
/* Feeder task: both directions, one 1 ms tick                                 */
/* -------------------------------------------------------------------------- */

static void tick_cb(void *arg)
{
    /* esp_timer callback: task context (the esp_timer task), but keep it minimal. */
    if (s_feeder != NULL) {
        xTaskNotifyGive(s_feeder);
    }
}

static void feeder_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_mic.present && s_mic.streaming) {
            mic_feed_once();
        }
#if CFG_TUD_AUDIO_ENABLE_EP_OUT
        if (s_spk.present && s_spk.streaming) {
            spk_drain_once();
            spk_feedback_update();
        }
#endif
    }
}

static bool any_streaming(void)
{
    return (s_mic.present && s_mic.streaming) || (s_spk.present && s_spk.streaming);
}

static void tick_sync(void)
{
    /* One timer serves both directions; run it while either is streaming. */
    if (any_streaming()) {
        esp_timer_start_periodic(s_tick, CONFIG_USB_AUDIO_FEED_PERIOD_US);
    } else {
        esp_timer_stop(s_tick);
    }
}

static void mic_streaming_set(bool on)
{
    if (!s_mic.present || s_mic.streaming == on) {
        return;
    }
    if (on) {
        /* Open at the newest sample: never replay what accumulated while idle. */
        ringbuf_reader_open(s_mic.cfg.ring, &s_mic_reader);
        tud_audio_n_clear_ep_in_ff(AUDIO_FUNC);
        atomic_store_explicit(&s_fifo_in, 0, memory_order_relaxed);
        atomic_store_explicit(&s_fifo_out, 0, memory_order_relaxed);
    } else {
        tud_audio_n_clear_ep_in_ff(AUDIO_FUNC);
    }
    s_mic.streaming = on;
    tick_sync();
    ESP_LOGI(TAG, "mic streaming %s", on ? "started" : "stopped");
}

static void spk_streaming_set(bool on)
{
#if CFG_TUD_AUDIO_ENABLE_EP_OUT
    if (!s_spk.present || s_spk.streaming == on) {
        return;
    }
    if (on) {
        tud_audio_n_clear_ep_out_ff(AUDIO_FUNC);
        s_fb_backlog_sum = 0;
        s_fb_backlog_n = 0;
        tud_audio_n_fb_set(AUDIO_FUNC, s_fb_nominal);
        s_stats.speaker.feedback_value = s_fb_nominal;
    } else {
        tud_audio_n_clear_ep_out_ff(AUDIO_FUNC);
    }
    s_spk.streaming = on;
    tick_sync();
    ESP_LOGI(TAG, "speaker streaming %s", on ? "started" : "stopped");
#else
    (void)on;
#endif
}

/* -------------------------------------------------------------------------- */
/* Descriptor (DES-AUD-021): hand-built, because no TinyUSB template is duplex  */
/* -------------------------------------------------------------------------- */

static size_t ac_units_len(const stream_t *s)
{
    return TUD_AUDIO20_DESC_CLK_SRC_LEN
         + TUD_AUDIO20_DESC_INPUT_TERM_LEN
         + TUD_AUDIO20_DESC_FEATURE_UNIT_LEN(s->cfg.channels)
         + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN;
}

static size_t as_len(bool with_feedback_ep)
{
    return 2u * TUD_AUDIO20_DESC_STD_AS_LEN      /* alt 0 and alt 1 */
         + TUD_AUDIO20_DESC_CS_AS_INT_LEN
         + TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN
         + TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN
         + TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN
         + (with_feedback_ep ? TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP_LEN : 0u);
}

static size_t total_ac_units_len(void)
{
    size_t n = 0;
    if (s_mic.present) { n += ac_units_len(&s_mic); }
    if (s_spk.present) { n += ac_units_len(&s_spk); }
    return n;
}

static size_t audio_descriptor_len(void *ctx)
{
    size_t n = TUD_AUDIO20_DESC_IAD_LEN + TUD_AUDIO20_DESC_STD_AC_LEN + TUD_AUDIO20_DESC_CS_AC_LEN;
    n += total_ac_units_len();
    if (s_mic.present) { n += as_len(false); }
    if (s_spk.present) { n += as_len(true); }
    return n;
}

static uint8_t itf_count(void)
{
    return (uint8_t)(1u + (s_mic.present ? 1u : 0u) + (s_spk.present ? 1u : 0u));
}

static uint32_t channel_config(uint8_t channels)
{
    return (channels == 2)
        ? ((uint32_t)AUDIO20_CHANNEL_CONFIG_FRONT_LEFT | (uint32_t)AUDIO20_CHANNEL_CONFIG_FRONT_RIGHT)
        : (uint32_t)AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED;
}

/* Mute and volume, readable and writable, on the master channel and each channel. */
#define FU_CTRL (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS | \
                 AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS)

static size_t emit(uint8_t *dst, const uint8_t *src, size_t n)
{
    memcpy(dst, src, n);
    return n;
}

/** Feature unit: variadic on channel count, so mono and stereo are separate literals. */
static size_t emit_feature_unit(uint8_t *p, uint8_t unit_id, uint8_t src_id, uint8_t channels)
{
    if (channels == 2) {
        const uint8_t d[] = { TUD_AUDIO20_DESC_FEATURE_UNIT(unit_id, src_id, 0x00,
                                                            FU_CTRL, FU_CTRL, FU_CTRL) };
        return emit(p, d, sizeof(d));
    }
    const uint8_t d[] = { TUD_AUDIO20_DESC_FEATURE_UNIT(unit_id, src_id, 0x00,
                                                        FU_CTRL, FU_CTRL) };
    return emit(p, d, sizeof(d));
}

/** The four AudioControl unit descriptors for one direction's chain. */
static size_t emit_ac_units(uint8_t *p, const stream_t *s, bool is_mic)
{
    size_t n = 0;
    const uint8_t clk_id = is_mic ? ENT_MIC_CLOCK : ENT_SPK_CLOCK;
    const uint8_t it_id  = is_mic ? ENT_MIC_IT    : ENT_SPK_IT;
    const uint8_t fu_id  = is_mic ? ENT_MIC_FU    : ENT_SPK_FU;
    const uint8_t ot_id  = is_mic ? ENT_MIC_OT    : ENT_SPK_OT;

    {
        const uint8_t d[] = { TUD_AUDIO20_DESC_CLK_SRC(
            clk_id, AUDIO20_CLOCK_SOURCE_ATT_INT_FIX_CLK,
            (AUDIO20_CTRL_R << AUDIO20_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), it_id, 0x00) };
        n += emit(p + n, d, sizeof(d));
    }
    if (is_mic) {
        /* Physical microphone in, USB streaming out. */
        const uint8_t d[] = { TUD_AUDIO20_DESC_INPUT_TERM(
            it_id, AUDIO_TERM_TYPE_IN_GENERIC_MIC, ot_id, clk_id,
            s->cfg.channels, channel_config(s->cfg.channels), 0x00,
            AUDIO20_CTRL_R << AUDIO20_IN_TERM_CTRL_CONNECTOR_POS, 0x00) };
        n += emit(p + n, d, sizeof(d));
        n += emit_feature_unit(p + n, fu_id, it_id, s->cfg.channels);
        const uint8_t o[] = { TUD_AUDIO20_DESC_OUTPUT_TERM(
            ot_id, AUDIO_TERM_TYPE_USB_STREAMING, it_id, fu_id, clk_id, 0x0000, 0x00) };
        n += emit(p + n, o, sizeof(o));
    } else {
        /* USB streaming in, headphones out. */
        const uint8_t d[] = { TUD_AUDIO20_DESC_INPUT_TERM(
            it_id, AUDIO_TERM_TYPE_USB_STREAMING, 0x00, clk_id,
            s->cfg.channels, channel_config(s->cfg.channels), 0x00, 0x0000, 0x00) };
        n += emit(p + n, d, sizeof(d));
        n += emit_feature_unit(p + n, fu_id, it_id, s->cfg.channels);
        const uint8_t o[] = { TUD_AUDIO20_DESC_OUTPUT_TERM(
            ot_id, AUDIO_TERM_TYPE_OUT_HEADPHONES, it_id, fu_id, clk_id, 0x0000, 0x00) };
        n += emit(p + n, o, sizeof(o));
    }
    return n;
}

/** One AudioStreaming interface: alt 0 (idle), alt 1 (streaming), format, endpoints. */
static size_t emit_as(uint8_t *p, const stream_t *s, uint8_t itf, uint8_t term_id,
                      uint8_t ep, uint8_t ep_attr, uint8_t ep_fb)
{
    size_t n = 0;
    const uint8_t n_eps = (uint8_t)(ep_fb ? 2 : 1);
    const uint8_t bytes_per_sample = (uint8_t)(s->cfg.bits_per_sample / 8);

    {
        const uint8_t d[] = { TUD_AUDIO20_DESC_STD_AS_INT(itf, 0x00, 0x00, 0x00) };
        n += emit(p + n, d, sizeof(d));
    }
    {
        const uint8_t d[] = { TUD_AUDIO20_DESC_STD_AS_INT(itf, 0x01, n_eps, 0x00) };
        n += emit(p + n, d, sizeof(d));
    }
    {
        const uint8_t d[] = { TUD_AUDIO20_DESC_CS_AS_INT(
            term_id, AUDIO20_CTRL_NONE, AUDIO20_FORMAT_TYPE_I, AUDIO20_DATA_FORMAT_TYPE_I_PCM,
            s->cfg.channels, channel_config(s->cfg.channels), 0x00) };
        n += emit(p + n, d, sizeof(d));
    }
    {
        const uint8_t d[] = { TUD_AUDIO20_DESC_TYPE_I_FORMAT(bytes_per_sample,
                                                             s->cfg.bits_per_sample) };
        n += emit(p + n, d, sizeof(d));
    }
    {
        const uint8_t d[] = { TUD_AUDIO20_DESC_STD_AS_ISO_EP(ep, ep_attr, s->ep_size, 0x01) };
        n += emit(p + n, d, sizeof(d));
    }
    {
        const uint8_t d[] = { TUD_AUDIO20_DESC_CS_AS_ISO_EP(
            AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, AUDIO20_CTRL_NONE,
            AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 0x0000) };
        n += emit(p + n, d, sizeof(d));
    }
    if (ep_fb) {
        const uint8_t d[] = { TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP(ep_fb, FB_EP_SIZE, 0x01) };
        n += emit(p + n, d, sizeof(d));
    }
    return n;
}

static size_t audio_descriptor_write(void *ctx, uint8_t *buf, uint8_t itf_base,
                                     const uint8_t *ep_in, const uint8_t *ep_out)
{
    size_t n = 0;
    uint8_t next_itf = (uint8_t)(itf_base + 1);
    /* IN order is [mic data, feedback]; a speaker-only device has no mic data EP. */
    uint8_t ep_mic = s_mic.present ? ep_in[0] : 0;
    uint8_t ep_fb  = s_spk.present ? ep_in[s_mic.present ? 1 : 0] : 0;
    uint8_t ep_spk = s_spk.present ? ep_out[0] : 0;

    {
        const uint8_t d[] = { TUD_AUDIO20_DESC_IAD(itf_base, itf_count(), 0x00) };
        n += emit(buf + n, d, sizeof(d));
    }
    {
        const uint8_t d[] = { TUD_AUDIO20_DESC_STD_AC(itf_base, 0x00, 0x00) };
        n += emit(buf + n, d, sizeof(d));
    }
    {
        const uint8_t category = (s_direction == USB_AUDIO_DIR_HEADSET) ? AUDIO20_FUNC_HEADSET
                               : (s_direction == USB_AUDIO_DIR_MIC)     ? AUDIO20_FUNC_MICROPHONE
                                                                        : AUDIO20_FUNC_DESKTOP_SPEAKER;
        const uint8_t d[] = { TUD_AUDIO20_DESC_CS_AC(0x0200, category, total_ac_units_len(),
                                                     AUDIO20_CS_AS_INTERFACE_CTRL_LATENCY_POS) };
        n += emit(buf + n, d, sizeof(d));
    }

    if (s_mic.present) { n += emit_ac_units(buf + n, &s_mic, true); }
    if (s_spk.present) { n += emit_ac_units(buf + n, &s_spk, false); }

    if (s_mic.present) {
        s_mic.itf = next_itf++;
        /* Asynchronous source: the device decides the packet size (FW-AUD-024). */
        const uint8_t attr = (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS |
                                       (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS |
                                       (uint8_t)TUSB_ISO_EP_ATT_DATA);
        n += emit_as(buf + n, &s_mic, s_mic.itf, ENT_MIC_OT, ep_mic, attr, 0);
    }
    if (s_spk.present) {
        s_spk.itf = next_itf++;
        /* Asynchronous sink: the host decides, guided by our feedback EP (ADR-021). */
        const uint8_t attr = (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS |
                                       (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS |
                                       (uint8_t)TUSB_ISO_EP_ATT_DATA);
        n += emit_as(buf + n, &s_spk, s_spk.itf, ENT_SPK_IT, ep_spk, attr, ep_fb);
    }

    return n;
}

static usb_ep_budget_t audio_endpoint_request(void *ctx)
{
    usb_ep_budget_t b = { 0 };
    if (s_mic.present) {
        b.in_endpoints++;
        b.tx_fifo_bytes = (uint16_t)(b.tx_fifo_bytes + s_mic.ep_size);
    }
    if (s_spk.present) {
        b.out_endpoints++;                 /* Shares the RX FIFO: no TX FIFO cost. */
        b.in_endpoints++;                  /* Feedback. */
        b.tx_fifo_bytes = (uint16_t)(b.tx_fifo_bytes + FB_EP_SIZE);
    }
    return b;
}

static void audio_on_unmount(void *ctx)
{
    mic_streaming_set(false);
    spk_streaming_set(false);
}

static const usb_function_t s_function = {
    .name             = "uac2",
    .interface_count  = 0,   /* Filled in at init: depends on the direction. */
    .endpoint_request = audio_endpoint_request,
    .descriptor_len   = audio_descriptor_len,
    .descriptor_write = audio_descriptor_write,
    .on_mount         = NULL,
    .on_unmount       = audio_on_unmount,
    .on_suspend       = NULL,
    .on_resume        = NULL,
    .ctx              = NULL,
};
static usb_function_t s_function_inst;

/* -------------------------------------------------------------------------- */
/* TinyUSB audio class callbacks                                               */
/* -------------------------------------------------------------------------- */

/* ISR context (audiod_xfer_isr). Counting only — no logging, no ring access. */
bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id,
                           uint8_t ep_in, uint8_t cur_alt_setting)
{
    atomic_fetch_add_explicit(&s_fifo_out, n_bytes_sent, memory_order_relaxed);
    s_stats.mic.packets++;
    if (n_bytes_sent == 0) {
        s_stats.mic.zero_length_packets++;
    } else if (n_bytes_sent == s_mic.nominal_packet) {
        s_stats.mic.packets_nominal++;
    } else if (n_bytes_sent == s_mic.nominal_packet + s_mic.bytes_per_frame) {
        s_stats.mic.packets_plus_one++;
    } else if (n_bytes_sent + s_mic.bytes_per_frame == s_mic.nominal_packet) {
        s_stats.mic.packets_minus_one++;
    }
    return true;
}

/* USB device task context from here on. */

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uint8_t alt = TU_U16_LOW(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);

    if (s_mic.present && itf == s_mic.itf) {
        mic_streaming_set(alt != 0);
    } else if (s_spk.present && itf == s_spk.itf) {
        spk_streaming_set(alt != 0);
    }
    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    if (s_mic.present && itf == s_mic.itf) {
        mic_streaming_set(false);
    } else if (s_spk.present && itf == s_spk.itf) {
        spk_streaming_set(false);
    }
    return true;
}

/** Map an entity ID to its direction, or NULL if we do not advertise it. */
static stream_t *entity_stream(uint8_t entity)
{
    switch (entity) {
    case ENT_MIC_CLOCK: case ENT_MIC_IT: case ENT_MIC_FU: case ENT_MIC_OT:
        return s_mic.present ? &s_mic : NULL;
    case ENT_SPK_CLOCK: case ENT_SPK_IT: case ENT_SPK_FU: case ENT_SPK_OT:
        return s_spk.present ? &s_spk : NULL;
    default:
        return NULL;
    }
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uint8_t ctrl_sel = TU_U16_HIGH(p_request->wValue);
    uint8_t entity = TU_U16_HIGH(p_request->wIndex);
    stream_t *s = entity_stream(entity);
    if (s == NULL) {
        return false;
    }
    const bool is_mic = (s == &s_mic);

    switch (entity) {
    case ENT_MIC_CLOCK:
    case ENT_SPK_CLOCK:
        if (ctrl_sel == AUDIO20_CS_CTRL_SAM_FREQ) {
            if (p_request->bRequest == AUDIO20_CS_REQ_CUR) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request,
                                                                 &s->sample_rate, sizeof(s->sample_rate));
            }
            if (p_request->bRequest == AUDIO20_CS_REQ_RANGE) {
                void *r = is_mic ? (void *)&s_mic_rate_range : (void *)&s_spk_rate_range;
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request,
                                                                 r, sizeof(s_mic_rate_range));
            }
        } else if (ctrl_sel == AUDIO20_CS_CTRL_CLK_VALID && p_request->bRequest == AUDIO20_CS_REQ_CUR) {
            static uint8_t clk_valid = 1;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &clk_valid, 1);
        }
        break;

    case ENT_MIC_FU:
    case ENT_SPK_FU:
        if (ctrl_sel == AUDIO20_FU_CTRL_MUTE && p_request->bRequest == AUDIO20_CS_REQ_CUR) {
            static uint8_t mute_cur;
            mute_cur = s->mute ? 1 : 0;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute_cur, 1);
        }
        if (ctrl_sel == AUDIO20_FU_CTRL_VOLUME) {
            if (p_request->bRequest == AUDIO20_CS_REQ_CUR) {
                static int16_t vol_cur;
                vol_cur = s->volume;
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request,
                                                                 &vol_cur, sizeof(vol_cur));
            }
            if (p_request->bRequest == AUDIO20_CS_REQ_RANGE) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request,
                                                                 &s_volume_range, sizeof(s_volume_range));
            }
        }
        break;

    case ENT_MIC_IT:
    case ENT_SPK_IT:
        if (ctrl_sel == AUDIO20_TE_CTRL_CONNECTOR && p_request->bRequest == AUDIO20_CS_REQ_CUR) {
            static audio20_desc_channel_cluster_t cluster;
            cluster.bNrChannels = s->cfg.channels;
            cluster.bmChannelConfig = (audio20_channel_config_t)channel_config(s->cfg.channels);
            cluster.iChannelNames = 0;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cluster, sizeof(cluster));
        }
        break;

    default:
        break;
    }
    /* Anything we do not advertise is stalled, per FW-AUD-027. */
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    uint8_t ctrl_sel = TU_U16_HIGH(p_request->wValue);
    uint8_t entity = TU_U16_HIGH(p_request->wIndex);
    stream_t *s = entity_stream(entity);

    if (s == NULL || p_request->bRequest != AUDIO20_CS_REQ_CUR) {
        return false;
    }

    if (entity == ENT_MIC_FU || entity == ENT_SPK_FU) {
        bool changed = false;
        if (ctrl_sel == AUDIO20_FU_CTRL_MUTE) {
            s->mute = ((audio20_control_cur_1_t *)pBuff)->bCur != 0;
            changed = true;
        } else if (ctrl_sel == AUDIO20_FU_CTRL_VOLUME) {
            s->volume = ((audio20_control_cur_2_t *)pBuff)->bCur;
            changed = true;
        }
        if (changed) {
            usb_audio_stream_t which = (s == &s_mic) ? USB_AUDIO_STREAM_MIC : USB_AUDIO_STREAM_SPEAKER;
            ESP_LOGD(TAG, "host set %s mute=%d volume=%d/256 dB",
                     which == USB_AUDIO_STREAM_MIC ? "mic" : "speaker", s->mute, s->volume);
            if (s_on_control) {
                s_on_control(which, s->mute, s->volume, s_ctx);
            }
            return true;
        }
        return false;
    }

    if ((entity == ENT_MIC_CLOCK || entity == ENT_SPK_CLOCK) && ctrl_sel == AUDIO20_CS_CTRL_SAM_FREQ) {
        /* Single-rate per direction: accept only our own rate. Refusing here stalls
         * the request so the host keeps the advertised rate. */
        uint32_t req = ((audio20_control_cur_4_t *)pBuff)->bCur;
        if (req == s->sample_rate) {
            return true;
        }
        ESP_LOGW(TAG, "host requested %u Hz; only %u Hz is supported on that direction",
                 (unsigned)req, (unsigned)s->sample_rate);
        return false;
    }
    return false;
}

/* Endpoint- and interface-recipient requests: nothing advertised, stall. */
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)  { return false; }
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) { return false; }
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)  { return false; }
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) { return false; }

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

static esp_err_t validate_stream(const usb_audio_stream_config_t *c, uint16_t ep_max,
                                 stream_t *out)
{
    if (c->ring == NULL || c->sample_rate_hz < 8000 || c->sample_rate_hz > 48000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->channels != 1 && c->channels != 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->bits_per_sample != 16 && c->bits_per_sample != 24 && c->bits_per_sample != 32) {
        return ESP_ERR_INVALID_ARG;
    }

    out->cfg = *c;
    out->sample_rate = c->sample_rate_hz;
    out->bytes_per_frame = (uint16_t)((c->bits_per_sample / 8) * c->channels);
    out->nominal_packet = (uint16_t)((c->sample_rate_hz / 1000) * out->bytes_per_frame);
    /* Full speed: one frame per ms, plus one sample of slack for the servo on
     * either side of nominal. */
    uint32_t samples_per_frame = (c->sample_rate_hz + 999) / 1000;
    uint32_t ep_size = (samples_per_frame + 1) * out->bytes_per_frame;
    if (ep_size > ep_max) {
        ESP_LOGE(TAG, "needs a %u-byte endpoint; the configured maximum is %u",
                 (unsigned)ep_size, (unsigned)ep_max);
        return ESP_ERR_INVALID_SIZE;
    }
    out->ep_size = (uint16_t)ep_size;
    out->present = true;
    out->mute = false;
    out->volume = 0;
    out->streaming = false;
    return ESP_OK;
}

esp_err_t usb_audio_init(const usb_audio_config_t *cfg)
{
    if (s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || cfg->direction > USB_AUDIO_DIR_HEADSET) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool want_mic = (cfg->direction != USB_AUDIO_DIR_SPEAKER);
    const bool want_spk = (cfg->direction != USB_AUDIO_DIR_MIC);

#if !CFG_TUD_AUDIO_ENABLE_EP_OUT
    if (want_spk) {
        ESP_LOGE(TAG, "a speaker was requested; enable CONFIG_TINYUSB_AUDIO_SPEAKER_ENABLED");
        return ESP_ERR_NOT_SUPPORTED;
    }
#endif

    memset(&s_mic, 0, sizeof(s_mic));
    memset(&s_spk, 0, sizeof(s_spk));

    esp_err_t err;
    if (want_mic) {
        err = validate_stream(&cfg->mic, CONFIG_TINYUSB_AUDIO_EP_IN_SZ_MAX, &s_mic);
        if (err != ESP_OK) {
            goto reset;
        }
    }
#if CFG_TUD_AUDIO_ENABLE_EP_OUT
    if (want_spk) {
        if (cfg->speaker_backlog_cb == NULL) {
            ESP_LOGE(TAG, "a speaker needs speaker_backlog_cb: the feedback servo has "
                          "nothing to regulate without it (ADR-021)");
            err = ESP_ERR_INVALID_ARG;
            goto reset;
        }
        err = validate_stream(&cfg->speaker, CONFIG_TINYUSB_AUDIO_EP_OUT_SZ_MAX, &s_spk);
        if (err != ESP_OK) {
            goto reset;
        }
        uint16_t target_ms = cfg->speaker_target_ms ? cfg->speaker_target_ms
                                                    : CONFIG_USB_AUDIO_SPEAKER_TARGET_MS;
        s_fb_target_bytes = (uint32_t)target_ms * (s_spk.sample_rate / 1000) * s_spk.bytes_per_frame;
        if (s_fb_target_bytes == 0 ||
            s_fb_target_bytes + s_spk.ep_size > ringbuf_capacity(cfg->speaker.ring)) {
            ESP_LOGE(TAG, "speaker ring is %u B; a %u ms target backlog plus one packet needs more",
                     (unsigned)ringbuf_capacity(cfg->speaker.ring), target_ms);
            err = ESP_ERR_INVALID_ARG;
            goto reset;
        }
        /* 16.16 samples per (1 ms) frame. */
        s_fb_nominal = (uint32_t)(((uint64_t)s_spk.sample_rate << 16) / 1000u);
        s_stats.speaker.target_bytes = s_fb_target_bytes;
        s_stats.speaker.nominal_packet_bytes = s_spk.nominal_packet;
    }
#endif

    s_direction  = cfg->direction;
    s_on_control = cfg->on_control;
    s_ctx        = cfg->ctx;
#if CFG_TUD_AUDIO_ENABLE_EP_OUT
    s_backlog_cb = cfg->speaker_backlog_cb;
#endif

    s_mic_rate_range.wNumSubRanges = 1;
    s_mic_rate_range.subrange[0].bMin = s_mic.sample_rate;
    s_mic_rate_range.subrange[0].bMax = s_mic.sample_rate;
    s_mic_rate_range.subrange[0].bRes = 0;
    s_spk_rate_range.wNumSubRanges = 1;
    s_spk_rate_range.subrange[0].bMin = s_spk.sample_rate;
    s_spk_rate_range.subrange[0].bMax = s_spk.sample_rate;
    s_spk_rate_range.subrange[0].bRes = 0;
    s_volume_range.wNumSubRanges = 1;
    s_volume_range.subrange[0].bMin = (int16_t)(-90 * 256);
    s_volume_range.subrange[0].bMax = 0;
    s_volume_range.subrange[0].bRes = 256;

    s_stats.mic.nominal_packet_bytes = s_mic.nominal_packet;

    const esp_timer_create_args_t targs = {
        .callback = tick_cb,
        .name = "usb_audio_tick",
        .dispatch_method = ESP_TIMER_TASK,
    };
    err = esp_timer_create(&targs, &s_tick);
    if (err != ESP_OK) {
        goto reset;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(feeder_task, "usb_audio_feed",
                                            CONFIG_USB_AUDIO_FEED_TASK_STACK, NULL,
                                            CONFIG_USB_AUDIO_FEED_TASK_PRIORITY, &s_feeder,
                                            CONFIG_USB_AUDIO_FEED_TASK_CORE);
    if (ok != pdPASS) {
        esp_timer_delete(s_tick);
        s_tick = NULL;
        err = ESP_ERR_NO_MEM;
        goto reset;
    }

    s_function_inst = s_function;
    s_function_inst.interface_count = itf_count();
    err = usb_device_register(&s_function_inst);
    if (err != ESP_OK) {
        vTaskDelete(s_feeder);
        s_feeder = NULL;
        esp_timer_delete(s_tick);
        s_tick = NULL;
        goto reset;
    }

    s_initialised = true;
    if (s_mic.present) {
        ESP_LOGI(TAG, "mic: %u Hz %uch %u-bit, ep %u B, nominal %u B/frame, FIFO %u B",
                 (unsigned)s_mic.sample_rate, s_mic.cfg.channels, s_mic.cfg.bits_per_sample,
                 s_mic.ep_size, s_mic.nominal_packet,
                 (unsigned)CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ);
    }
#if CFG_TUD_AUDIO_ENABLE_EP_OUT
    if (s_spk.present) {
        ESP_LOGI(TAG, "speaker: %u Hz %uch %u-bit, ep %u B, nominal %u B/frame, FIFO %u B, "
                      "feedback target %u B every %d ms",
                 (unsigned)s_spk.sample_rate, s_spk.cfg.channels, s_spk.cfg.bits_per_sample,
                 s_spk.ep_size, s_spk.nominal_packet,
                 (unsigned)CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ,
                 (unsigned)s_fb_target_bytes, CONFIG_USB_AUDIO_SPEAKER_FB_PERIOD_MS);
    }
#endif
    return ESP_OK;

reset:
    memset(&s_mic, 0, sizeof(s_mic));
    memset(&s_spk, 0, sizeof(s_spk));
    return err;
}

esp_err_t usb_audio_deinit(void)
{
    if (!s_initialised) {
        return ESP_OK;
    }
    mic_streaming_set(false);
    spk_streaming_set(false);
    s_initialised = false;
    esp_timer_delete(s_tick);
    s_tick = NULL;
    vTaskDelete(s_feeder);
    s_feeder = NULL;
    memset(&s_mic, 0, sizeof(s_mic));
    memset(&s_spk, 0, sizeof(s_spk));
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

bool usb_audio_is_streaming(usb_audio_stream_t which)
{
    if (!s_initialised) {
        return false;
    }
    return (which == USB_AUDIO_STREAM_MIC) ? (s_mic.present && s_mic.streaming)
                                           : (s_spk.present && s_spk.streaming);
}

esp_err_t usb_audio_stats(usb_audio_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    out->mic.streaming      = s_mic.present && s_mic.streaming;
    out->mic.fifo_fill_bytes = (uint16_t)fifo_in_fill();
    out->mic.mute           = s_mic.mute;
    out->mic.volume_db256   = s_mic.volume;
    out->speaker.streaming    = s_spk.present && s_spk.streaming;
    out->speaker.mute         = s_spk.mute;
    out->speaker.volume_db256 = s_spk.volume;
    return ESP_OK;
}

esp_err_t usb_audio_stats_reset(void)
{
    uint16_t mic_nominal = s_stats.mic.nominal_packet_bytes;
    uint16_t spk_nominal = s_stats.speaker.nominal_packet_bytes;
    uint32_t target = s_stats.speaker.target_bytes;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.mic.nominal_packet_bytes = mic_nominal;
    s_stats.speaker.nominal_packet_bytes = spk_nominal;
    s_stats.speaker.target_bytes = target;
    return ESP_OK;
}
