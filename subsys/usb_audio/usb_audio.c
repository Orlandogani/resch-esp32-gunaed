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

/* Entity IDs fixed by TUD_AUDIO20_MIC_ONE_CH_DESCRIPTOR. */
#define ENT_INPUT_TERMINAL  0x01
#define ENT_FEATURE_UNIT    0x02
#define ENT_OUTPUT_TERMINAL 0x03
#define ENT_CLOCK_SOURCE    0x04

#define AUDIO_FUNC 0   /* TinyUSB audio function index; this module is a singleton. */

/* Bytes the feeder moves per tick at most: two nominal packets keeps the FIFO
 * topped up without letting one tick starve the ring's other readers of CPU. */
#define FEED_CHUNK_MAX (2u * CONFIG_TINYUSB_AUDIO_EP_IN_SZ_MAX)

static bool                s_initialised;
static usb_audio_config_t  s_cfg;
static uint16_t            s_bytes_per_sample;
static uint16_t            s_nominal_packet;
static uint16_t            s_ep_size;

static ringbuf_reader_t    s_reader;
static volatile bool       s_streaming;
static TaskHandle_t        s_feeder;
static esp_timer_handle_t  s_tick;

/* FIFO fill is tracked from our own writes minus the ISR's completions, which is
 * exact because tu_fifo is strictly single-producer/single-consumer here. */
static _Atomic uint32_t    s_fifo_in;
static _Atomic uint32_t    s_fifo_out;

static usb_audio_stats_t   s_stats;

/* UAC2 control state. */
static uint32_t            s_sample_rate;
static bool                s_mute;
static int16_t             s_volume;
static audio20_control_range_4_n_t(1) s_rate_range;
static audio20_control_range_2_n_t(1) s_volume_range;

static uint8_t             s_feed_buf[FEED_CHUNK_MAX];

/* -------------------------------------------------------------------------- */
/* Feeder (DES-AUD-002..004)                                                   */
/* -------------------------------------------------------------------------- */

static inline uint32_t fifo_fill(void)
{
    return atomic_load_explicit(&s_fifo_in, memory_order_relaxed) -
           atomic_load_explicit(&s_fifo_out, memory_order_relaxed);
}

static void tick_cb(void *arg)
{
    /* esp_timer callback: task context (the esp_timer task), but keep it minimal. */
    if (s_feeder != NULL) {
        xTaskNotifyGive(s_feeder);
    }
}

static void feed_once(void)
{
    const uint32_t depth = CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ;
    uint32_t fill = fifo_fill();
    if (fill >= depth) {
        return;
    }
    uint32_t space = depth - fill;
    uint32_t want = space < FEED_CHUNK_MAX ? space : FEED_CHUNK_MAX;

    /* Keep whole samples: never split a sample across two FIFO writes. */
    want -= want % s_bytes_per_sample;
    if (want == 0) {
        return;
    }

    size_t got = 0;
    esp_err_t err = ringbuf_read(&s_reader, s_feed_buf, want, &got);
    if (err == ESP_ERR_INVALID_STATE) {
        s_stats.ring_overruns++;
        return; /* Cursor resynchronised; next tick resumes. */
    }
    if (err != ESP_OK || got == 0) {
        return;
    }
    got -= got % s_bytes_per_sample;

    uint16_t written = tud_audio_n_write(AUDIO_FUNC, s_feed_buf, (uint16_t)got);
    atomic_fetch_add_explicit(&s_fifo_in, written, memory_order_relaxed);
    s_stats.bytes_streamed += written;
    if (written < got) {
        s_stats.fifo_overflows++; /* Our accounting and TinyUSB's disagree: investigate. */
    }
}

static void feeder_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_streaming) {
            feed_once();
        }
    }
}

static void streaming_start(void)
{
    if (s_streaming) {
        return;
    }
    /* Open at the newest sample: never replay what accumulated while idle. */
    ringbuf_reader_open(s_cfg.ring, &s_reader);
    tud_audio_n_clear_ep_in_ff(AUDIO_FUNC);
    atomic_store_explicit(&s_fifo_in, 0, memory_order_relaxed);
    atomic_store_explicit(&s_fifo_out, 0, memory_order_relaxed);
    s_streaming = true;
    esp_timer_start_periodic(s_tick, CONFIG_USB_AUDIO_FEED_PERIOD_US);
    ESP_LOGI(TAG, "streaming: %u Hz, %u B/sample, nominal %u B/frame",
             (unsigned)s_sample_rate, s_bytes_per_sample, s_nominal_packet);
}

static void streaming_stop(void)
{
    if (!s_streaming) {
        return;
    }
    s_streaming = false;
    esp_timer_stop(s_tick);
    tud_audio_n_clear_ep_in_ff(AUDIO_FUNC);
    ESP_LOGI(TAG, "streaming stopped");
}

/* -------------------------------------------------------------------------- */
/* usb_function_t contract                                                     */
/* -------------------------------------------------------------------------- */

static usb_ep_budget_t audio_endpoint_request(void *ctx)
{
    return (usb_ep_budget_t) {
        .in_endpoints = 1,
        .out_endpoints = 0,
        .tx_fifo_bytes = s_ep_size,
    };
}

static size_t audio_descriptor_len(void *ctx)
{
    return TUD_AUDIO20_MIC_ONE_CH_DESC_LEN;
}

static size_t audio_descriptor_write(void *ctx, uint8_t *buf, uint8_t itf_base,
                                     const uint8_t *ep_in, const uint8_t *ep_out)
{
    const uint8_t desc[] = {
        TUD_AUDIO20_MIC_ONE_CH_DESCRIPTOR(itf_base, /*stridx*/ 0,
                                          (uint8_t)s_bytes_per_sample,
                                          (uint8_t)s_cfg.bits_per_sample,
                                          ep_in[0], s_ep_size)
    };
    memcpy(buf, desc, sizeof(desc));
    return sizeof(desc);
}

static void audio_on_unmount(void *ctx)
{
    streaming_stop();
}

static const usb_function_t s_function = {
    .name             = "uac2-mic",
    .interface_count  = 2,   /* AudioControl + AudioStreaming */
    .endpoint_request = audio_endpoint_request,
    .descriptor_len   = audio_descriptor_len,
    .descriptor_write = audio_descriptor_write,
    .on_mount         = NULL,
    .on_unmount       = audio_on_unmount,
    .on_suspend       = NULL,
    .on_resume        = NULL,
    .ctx              = NULL,
};

/* -------------------------------------------------------------------------- */
/* TinyUSB audio class callbacks                                               */
/* -------------------------------------------------------------------------- */

/* ISR context (audiod_xfer_isr). Counting only — no logging, no ring access. */
bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id,
                           uint8_t ep_in, uint8_t cur_alt_setting)
{
    atomic_fetch_add_explicit(&s_fifo_out, n_bytes_sent, memory_order_relaxed);
    s_stats.packets++;
    if (n_bytes_sent == 0) {
        s_stats.zero_length_packets++;
    } else if (n_bytes_sent == s_nominal_packet) {
        s_stats.packets_nominal++;
    } else if (n_bytes_sent == s_nominal_packet + s_bytes_per_sample) {
        s_stats.packets_plus_one++;
    } else if (n_bytes_sent + s_bytes_per_sample == s_nominal_packet) {
        s_stats.packets_minus_one++;
    }
    return true;
}

/* USB device task context from here on. */

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uint8_t alt = TU_U16_LOW(p_request->wValue);
    if (alt == 0) {
        streaming_stop();
    } else {
        streaming_start();
    }
    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    streaming_stop();
    return true;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uint8_t ctrl_sel = TU_U16_HIGH(p_request->wValue);
    uint8_t entity = TU_U16_HIGH(p_request->wIndex);

    switch (entity) {
    case ENT_CLOCK_SOURCE:
        if (ctrl_sel == AUDIO20_CS_CTRL_SAM_FREQ) {
            if (p_request->bRequest == AUDIO20_CS_REQ_CUR) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request,
                                                                 &s_sample_rate, sizeof(s_sample_rate));
            }
            if (p_request->bRequest == AUDIO20_CS_REQ_RANGE) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request,
                                                                 &s_rate_range, sizeof(s_rate_range));
            }
        } else if (ctrl_sel == AUDIO20_CS_CTRL_CLK_VALID && p_request->bRequest == AUDIO20_CS_REQ_CUR) {
            static uint8_t clk_valid = 1;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &clk_valid, 1);
        }
        break;

    case ENT_FEATURE_UNIT:
        if (ctrl_sel == AUDIO20_FU_CTRL_MUTE && p_request->bRequest == AUDIO20_CS_REQ_CUR) {
            static uint8_t mute_cur;
            mute_cur = s_mute ? 1 : 0;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute_cur, 1);
        }
        if (ctrl_sel == AUDIO20_FU_CTRL_VOLUME) {
            if (p_request->bRequest == AUDIO20_CS_REQ_CUR) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request,
                                                                 &s_volume, sizeof(s_volume));
            }
            if (p_request->bRequest == AUDIO20_CS_REQ_RANGE) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request,
                                                                 &s_volume_range, sizeof(s_volume_range));
            }
        }
        break;

    case ENT_INPUT_TERMINAL:
        if (ctrl_sel == AUDIO20_TE_CTRL_CONNECTOR && p_request->bRequest == AUDIO20_CS_REQ_CUR) {
            static audio20_desc_channel_cluster_t cluster;
            cluster.bNrChannels = 1;
            cluster.bmChannelConfig = (audio20_channel_config_t)0;
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

    if (p_request->bRequest != AUDIO20_CS_REQ_CUR) {
        return false;
    }

    if (entity == ENT_FEATURE_UNIT) {
        bool changed = false;
        if (ctrl_sel == AUDIO20_FU_CTRL_MUTE) {
            s_mute = ((audio20_control_cur_1_t *)pBuff)->bCur != 0;
            changed = true;
        } else if (ctrl_sel == AUDIO20_FU_CTRL_VOLUME) {
            s_volume = ((audio20_control_cur_2_t *)pBuff)->bCur;
            changed = true;
        }
        if (changed) {
            s_stats.mute = s_mute;
            s_stats.volume_db256 = s_volume;
            ESP_LOGD(TAG, "host set mute=%d volume=%d/256 dB", s_mute, s_volume);
            if (s_cfg.on_control) {
                s_cfg.on_control(s_mute, s_volume, s_cfg.ctx);
            }
            return true;
        }
        return false;
    }

    if (entity == ENT_CLOCK_SOURCE && ctrl_sel == AUDIO20_CS_CTRL_SAM_FREQ) {
        /* Single-rate device: accept only our own rate. TinyUSB has already
         * copied the value into its sample_rate_tx; refusing here stalls the
         * request so the host keeps the advertised rate. */
        uint32_t req = ((audio20_control_cur_4_t *)pBuff)->bCur;
        if (req == s_sample_rate) {
            return true;
        }
        ESP_LOGW(TAG, "host requested %u Hz; only %u Hz is supported", (unsigned)req, (unsigned)s_sample_rate);
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

esp_err_t usb_audio_init(const usb_audio_config_t *cfg)
{
    if (s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || cfg->ring == NULL || cfg->sample_rate_hz < 8000 || cfg->sample_rate_hz > 48000 ||
        (cfg->bits_per_sample != 16 && cfg->bits_per_sample != 24 && cfg->bits_per_sample != 32)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->channels != 1) {
        ESP_LOGE(TAG, "%u channels requested; this version supports mono only", cfg->channels);
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_cfg = *cfg;
    s_bytes_per_sample = cfg->bits_per_sample / 8;
    s_sample_rate = cfg->sample_rate_hz;
    /* Full speed: one frame per ms. +1 sample of slack for the flow-control servo. */
    uint32_t samples_per_frame = (cfg->sample_rate_hz + 999) / 1000;
    s_nominal_packet = (uint16_t)((cfg->sample_rate_hz / 1000) * s_bytes_per_sample);
    uint32_t ep_size = (samples_per_frame + 1) * s_bytes_per_sample;
    if (ep_size > CONFIG_TINYUSB_AUDIO_EP_IN_SZ_MAX) {
        ESP_LOGE(TAG, "needs a %u-byte endpoint; CONFIG_TINYUSB_AUDIO_EP_IN_SZ_MAX is %d",
                 (unsigned)ep_size, CONFIG_TINYUSB_AUDIO_EP_IN_SZ_MAX);
        return ESP_ERR_INVALID_SIZE;
    }
    s_ep_size = (uint16_t)ep_size;

    s_rate_range.wNumSubRanges = 1;
    s_rate_range.subrange[0].bMin = s_sample_rate;
    s_rate_range.subrange[0].bMax = s_sample_rate;
    s_rate_range.subrange[0].bRes = 0;
    s_volume_range.wNumSubRanges = 1;
    s_volume_range.subrange[0].bMin = (int16_t)(-90 * 256);
    s_volume_range.subrange[0].bMax = 0;
    s_volume_range.subrange[0].bRes = 256;
    s_mute = false;
    s_volume = 0;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.nominal_packet_bytes = s_nominal_packet;
    s_streaming = false;

    const esp_timer_create_args_t targs = {
        .callback = tick_cb,
        .name = "usb_audio_tick",
        .dispatch_method = ESP_TIMER_TASK,
    };
    esp_err_t err = esp_timer_create(&targs, &s_tick);
    if (err != ESP_OK) {
        return err;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(feeder_task, "usb_audio_feed",
                                            CONFIG_USB_AUDIO_FEED_TASK_STACK, NULL,
                                            CONFIG_USB_AUDIO_FEED_TASK_PRIORITY, &s_feeder,
                                            CONFIG_USB_AUDIO_FEED_TASK_CORE);
    if (ok != pdPASS) {
        esp_timer_delete(s_tick);
        s_tick = NULL;
        return ESP_ERR_NO_MEM;
    }

    err = usb_device_register(&s_function);
    if (err != ESP_OK) {
        vTaskDelete(s_feeder);
        s_feeder = NULL;
        esp_timer_delete(s_tick);
        s_tick = NULL;
        return err;
    }

    s_initialised = true;
    ESP_LOGI(TAG, "init: %u Hz mono %u-bit, ep %u B, nominal %u B/frame, FIFO %u B",
             (unsigned)s_sample_rate, cfg->bits_per_sample, s_ep_size, s_nominal_packet,
             (unsigned)CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ);
    return ESP_OK;
}

esp_err_t usb_audio_deinit(void)
{
    if (!s_initialised) {
        return ESP_OK;
    }
    streaming_stop();
    s_initialised = false;
    esp_timer_delete(s_tick);
    s_tick = NULL;
    vTaskDelete(s_feeder);
    s_feeder = NULL;
    memset(&s_cfg, 0, sizeof(s_cfg));
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

bool usb_audio_is_streaming(void)
{
    return s_initialised && s_streaming;
}

esp_err_t usb_audio_stats(usb_audio_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    out->streaming = s_streaming;
    out->fifo_fill_bytes = (uint16_t)fifo_fill();
    out->mute = s_mute;
    out->volume_db256 = s_volume;
    return ESP_OK;
}

esp_err_t usb_audio_stats_reset(void)
{
    uint16_t nominal = s_stats.nominal_packet_bytes;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.nominal_packet_bytes = nominal;
    return ESP_OK;
}
