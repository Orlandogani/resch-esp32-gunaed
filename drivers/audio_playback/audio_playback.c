#include "audio_playback.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "diag.h"
#include "pm_policy.h"

static const char *TAG = "audio_playback";

/* One GDMA descriptor cannot carry more than this (SDD §15.5). */
#define APB_MAX_DESC_BYTES 4092

typedef enum { ST_UNINIT = 0, ST_READY, ST_RUNNING } state_t;

static state_t                  s_state;
static audio_playback_config_t  s_cfg;
static audio_playback_format_t  s_fmt;
static i2s_chan_handle_t        s_chan;
static size_t                   s_frame_bytes;     /* One DMA descriptor's payload. */
static uint8_t                 *s_frame_buf;       /* Task-side frame under construction. */
static ringbuf_reader_t         s_reader;          /* Our cursor into the caller's ring. */
static TaskHandle_t             s_task;
static pm_policy_lock_handle_t  s_lock;
static uint32_t                 s_write_timeout_ms;
static bool                     s_primed;          /* Task-owned; published via stats. */
static volatile bool            s_flush_req;       /* Set by any task, cleared by ours. */
static volatile uint32_t        s_isr_q_ovf;       /* Written in ISR, read in task. */
static audio_playback_stats_t   s_stats;

/* -------------------------------------------------------------------------- */
/* ISR: count only (FW-AUD-036, DR5)                                           */
/* -------------------------------------------------------------------------- */

/* Pacing is the blocking i2s_channel_write(), which the I2S driver's own ISR
 * releases; there is nothing for us to signal here. Both callbacks exist purely
 * so the send queue's behaviour is observable in audio_playback_stats(). */

static bool IRAM_ATTR on_sent_isr(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    return false;
}

static bool IRAM_ATTR on_send_q_ovf_isr(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    s_isr_q_ovf++;
    return false;
}

/* -------------------------------------------------------------------------- */
/* Playback task (DES-APB-004, DES-APB-005)                                    */
/* -------------------------------------------------------------------------- */

/**
 * Fill `s_frame_buf` with one DMA frame's worth of audio, padding with silence
 * whatever the source could not supply. Returns the number of real audio bytes.
 */
static size_t build_frame(void)
{
    if (!s_primed) {
        /* Hysteresis: nothing is played until a prefill's worth has accumulated,
         * so a source running at exactly nominal rate cannot oscillate between
         * "one frame ready" and "nothing ready" (FW-AUD-037). */
        if (ringbuf_available(&s_reader) < s_fmt.prefill_bytes) {
            memset(s_frame_buf, 0, s_frame_bytes);
            return 0;
        }
        s_primed = true;
    }

    size_t got = 0;
    esp_err_t err = ringbuf_read(&s_reader, s_frame_buf, s_frame_bytes, &got);
    if (err == ESP_ERR_INVALID_STATE) {
        /* The writer outran us: the cursor has already been resynchronised to the
         * oldest surviving byte, so the next frame is valid again. */
        s_stats.ring_overruns++;
        DIAG_LOG_RL(ESP_LOGW, TAG, 1000, "source ring overrun (total %u)",
                    (unsigned)s_stats.ring_overruns);
        got = 0;
    } else if (err != ESP_OK) {
        got = 0;
    }

    if (got < s_frame_bytes) {
        memset(s_frame_buf + got, 0, s_frame_bytes - got);
        s_stats.underruns++;
        if (got == 0) {
            /* Complete starvation. Re-enter prefill rather than clicking once per
             * frame while the source catches up (FW-AUD-038). */
            s_stats.starvations++;
            s_stats.prefills++;
            s_primed = false;
        }
        DIAG_LOG_RL(ESP_LOGW, TAG, 1000, "underrun: %u of %u B (total %u)",
                    (unsigned)got, (unsigned)s_frame_bytes, (unsigned)s_stats.underruns);
    }
    return got;
}

static void playback_task(void *arg)
{
    uint32_t last_isr_q_ovf = 0;

    for (;;) {
        if (s_flush_req) {
            s_flush_req = false;
            ringbuf_reader_skip_to_newest(&s_reader);
            if (s_primed) {
                s_stats.prefills++;
            }
            s_primed = false;
        }

        if (s_state != ST_RUNNING) {
            /* start(), flush() and deinit() all notify; nothing spins while stopped. */
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        uint32_t q_ovf = s_isr_q_ovf;
        if (q_ovf != last_isr_q_ovf) {
            last_isr_q_ovf = q_ovf;
            DIAG_LOG_RL(ESP_LOGW, TAG, 1000, "I2S send queue overflow (total %u)", (unsigned)q_ovf);
        }

        size_t real = build_frame();

        size_t written = 0;
        esp_err_t err = i2s_channel_write(s_chan, s_frame_buf, s_frame_bytes, &written,
                                          s_write_timeout_ms);
        if (err == ESP_ERR_TIMEOUT) {
            /* The DMA did not free a descriptor in time. The frame is gone; with
             * auto_clear the hardware sends silence rather than repeating it. */
            s_stats.write_timeouts++;
            DIAG_LOG_RL(ESP_LOGW, TAG, 1000, "i2s write timeout (total %u)",
                        (unsigned)s_stats.write_timeouts);
        } else if (err == ESP_ERR_INVALID_STATE) {
            /* stop() disabled the channel between our state check and the write. */
            continue;
        } else if (err != ESP_OK) {
            DIAG_LOG_RL(ESP_LOGW, TAG, 1000, "i2s_channel_write: %s", esp_err_to_name(err));
            continue;
        }

        s_stats.bytes_played += written;
        s_stats.bytes_silence += (written > real) ? (written - real) : 0;
        s_stats.dma_frames++;

        UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
        if (s_stats.task_stack_free_min == 0 || hw < s_stats.task_stack_free_min) {
            s_stats.task_stack_free_min = (uint32_t)hw;
        }
        diag_task_feed();
    }
}

/* -------------------------------------------------------------------------- */
/* Configuration helpers                                                       */
/* -------------------------------------------------------------------------- */

static esp_err_t validate(const audio_playback_config_t *cfg)
{
    if (cfg == NULL || cfg->source == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->sample_rate_hz < 8000 || cfg->sample_rate_hz > 48000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->channels != 1 && cfg->channels != 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->bits_per_sample != 16 && cfg->bits_per_sample != 32) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->slot > AUDIO_PLAYBACK_SLOT_BOTH) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->port < -1) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->pins.bclk < 0 || cfg->pins.ws < 0 || cfg->pins.dout < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t frames = cfg->dma_frame_count ? cfg->dma_frame_count : CONFIG_AUDIO_PLAYBACK_DMA_FRAME_COUNT;
    if (frames < 2) {
        return ESP_ERR_INVALID_ARG; /* FW-AUD-032: double-buffering minimum. */
    }
    return ESP_OK;
}

static esp_err_t init_channel(uint32_t dma_frame_num, uint32_t dma_desc_num)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(s_cfg.port, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = dma_desc_num;
    chan_cfg.dma_frame_num = dma_frame_num;
    /* Send silence, never a repeat of the last descriptor, if we ever fail to keep
     * the DMA fed (FW-AUD-038). */
    chan_cfg.auto_clear = true;

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_chan, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        return err;
    }

    i2s_slot_mode_t mode = (s_cfg.channels == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;
    i2s_data_bit_width_t width = (s_cfg.bits_per_sample == 16) ? I2S_DATA_BIT_WIDTH_16BIT
                                                               : I2S_DATA_BIT_WIDTH_32BIT;
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(s_cfg.sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(width, mode),
        .gpio_cfg = {
            .mclk = (s_cfg.pins.mclk >= 0) ? s_cfg.pins.mclk : I2S_GPIO_UNUSED,
            .bclk = s_cfg.pins.bclk,
            .ws   = s_cfg.pins.ws,
            .dout = s_cfg.pins.dout,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    if (mode == I2S_SLOT_MODE_MONO) {
        switch (s_cfg.slot) {
        case AUDIO_PLAYBACK_SLOT_RIGHT: std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT; break;
        case AUDIO_PLAYBACK_SLOT_BOTH:  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;  break;
        default:                        std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;  break;
        }
    }
    err = i2s_channel_init_std_mode(s_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s channel mode init: %s", esp_err_to_name(err));
        i2s_del_channel(s_chan);
        s_chan = NULL;
        return err;
    }

    const i2s_event_callbacks_t cbs = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
        .on_sent = on_sent_isr,
        .on_send_q_ovf = on_send_q_ovf_isr,
    };
    err = i2s_channel_register_event_callback(s_chan, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register callbacks: %s", esp_err_to_name(err));
        i2s_del_channel(s_chan);
        s_chan = NULL;
    }
    return err;
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t audio_playback_init(const audio_playback_config_t *cfg)
{
    if (s_state != ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = validate(cfg);
    if (err != ESP_OK) {
        return err;
    }

    s_cfg = *cfg;
    uint8_t  frame_ms   = cfg->dma_frame_ms ? cfg->dma_frame_ms : CONFIG_AUDIO_PLAYBACK_DMA_FRAME_MS;
    uint8_t  frame_cnt  = cfg->dma_frame_count ? cfg->dma_frame_count : CONFIG_AUDIO_PLAYBACK_DMA_FRAME_COUNT;
    uint16_t prefill_ms = cfg->prefill_ms ? cfg->prefill_ms : CONFIG_AUDIO_PLAYBACK_PREFILL_MS;

    s_fmt.sample_rate_hz   = cfg->sample_rate_hz;
    s_fmt.channels         = cfg->channels;
    s_fmt.bits_per_sample  = cfg->bits_per_sample;
    s_fmt.bytes_per_sample = (cfg->bits_per_sample == 16) ? 2 : 4;
    s_fmt.bytes_per_ms     = (uint16_t)((cfg->sample_rate_hz * cfg->channels * s_fmt.bytes_per_sample) / 1000);

    uint32_t dma_frame_num = (cfg->sample_rate_hz * frame_ms) / 1000;   /* frames per descriptor */
    s_frame_bytes = dma_frame_num * cfg->channels * s_fmt.bytes_per_sample;
    if (s_frame_bytes == 0 || s_frame_bytes > APB_MAX_DESC_BYTES) {
        ESP_LOGE(TAG, "%u ms per descriptor is %u B; the GDMA limit is %d",
                 frame_ms, (unsigned)s_frame_bytes, APB_MAX_DESC_BYTES);
        return ESP_ERR_INVALID_ARG;   /* FW-AUD-033 */
    }
    s_fmt.frame_bytes   = (uint16_t)s_frame_bytes;
    s_fmt.prefill_bytes = (uint32_t)s_fmt.bytes_per_ms * prefill_ms;

    /* A reader is overrun once it is a full capacity behind, so the ring must hold
     * the prefill backlog and the frame we are about to read out of it. */
    size_t ring_bytes = ringbuf_capacity(cfg->source);
    if (ring_bytes < s_fmt.prefill_bytes + s_frame_bytes) {
        ESP_LOGE(TAG, "source ring is %u B; %u B prefill + %u B frame needs more",
                 (unsigned)ring_bytes, (unsigned)s_fmt.prefill_bytes, (unsigned)s_frame_bytes);
        return ESP_ERR_INVALID_ARG;   /* FW-AUD-037 */
    }

    /* Hot-path storage stays in internal SRAM (ADR-014). */
    s_frame_buf = heap_caps_malloc(s_frame_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_frame_buf == NULL) {
        ESP_LOGE(TAG, "no internal SRAM for a %u B frame", (unsigned)s_frame_bytes);
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    err = ringbuf_reader_open(cfg->source, &s_reader);
    if (err != ESP_OK) {
        goto fail;
    }

    err = pm_policy_lock_create("audio_out", &s_lock);
    if (err != ESP_OK) {
        goto fail;
    }

    err = init_channel(dma_frame_num, frame_cnt);
    if (err != ESP_OK) {
        goto fail;
    }

    /* Long enough that a healthy DMA never trips it, short enough that stop() is
     * not held up: the whole DMA chain drains in frame_ms x frame_cnt. */
    s_write_timeout_ms = (uint32_t)frame_ms * frame_cnt * 4;
    if (s_write_timeout_ms < 20) {
        s_write_timeout_ms = 20;
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_isr_q_ovf = 0;
    s_flush_req = false;
    s_primed = false;
    s_state = ST_READY; /* Task checks this; set before creating it. */

    BaseType_t ok = xTaskCreatePinnedToCore(playback_task, "audio_play",
                                            CONFIG_AUDIO_PLAYBACK_TASK_STACK, NULL,
                                            CONFIG_AUDIO_PLAYBACK_TASK_PRIORITY, &s_task,
                                            CONFIG_AUDIO_PLAYBACK_TASK_CORE);
    if (ok != pdPASS) {
        err = ESP_ERR_NO_MEM;
        s_state = ST_UNINIT;
        goto fail;
    }

    ESP_LOGI(TAG, "init: I2S %u Hz %uch %u-bit, DMA %u x %u B (%u ms), prefill %u B (%u ms), "
                  "source ring %u B, task prio %d core %d",
             (unsigned)cfg->sample_rate_hz, cfg->channels, cfg->bits_per_sample,
             frame_cnt, (unsigned)s_frame_bytes, frame_ms,
             (unsigned)s_fmt.prefill_bytes, prefill_ms, (unsigned)ring_bytes,
             CONFIG_AUDIO_PLAYBACK_TASK_PRIORITY, CONFIG_AUDIO_PLAYBACK_TASK_CORE);
    return ESP_OK;

fail:
    if (s_chan != NULL) {
        i2s_del_channel(s_chan);
        s_chan = NULL;
    }
    heap_caps_free(s_frame_buf);
    s_frame_buf = NULL;
    memset(&s_reader, 0, sizeof(s_reader));
    return err;
}

esp_err_t audio_playback_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    audio_playback_stop();

    if (s_task != NULL) {
        TaskHandle_t t = s_task;
        s_task = NULL;
        vTaskDelete(t);
    }
    if (s_chan != NULL) {
        i2s_del_channel(s_chan);
        s_chan = NULL;
    }
    heap_caps_free(s_frame_buf);
    s_frame_buf = NULL;
    memset(&s_reader, 0, sizeof(s_reader));
    s_primed = false;
    s_state = ST_UNINIT;
    /* s_lock is kept: pm_policy locks are never destroyed. */
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

esp_err_t audio_playback_start(void)
{
    if (s_state != ST_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Never open on stale audio — the same rationale as ringbuf_reader_open(). */
    ringbuf_reader_skip_to_newest(&s_reader);
    s_primed = false;
    s_flush_req = false;
    s_stats.prefills++;

    esp_err_t err = i2s_channel_enable(s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable: %s", esp_err_to_name(err));
        return err;
    }
    pm_policy_lock_acquire(s_lock);
    diag_task_register(s_task);
    s_stats.running = true;
    s_state = ST_RUNNING;
    xTaskNotifyGive(s_task);
    ESP_LOGI(TAG, "started");
    return ESP_OK;
}

esp_err_t audio_playback_stop(void)
{
    if (s_state != ST_RUNNING) {
        return s_state == ST_UNINIT ? ESP_ERR_INVALID_STATE : ESP_OK;
    }
    s_state = ST_READY;
    /* Blocks on the channel mutex until any in-flight write returns, which is
     * bounded by s_write_timeout_ms. */
    esp_err_t err = i2s_channel_disable(s_chan);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_disable: %s", esp_err_to_name(err));
    }
    diag_task_unregister(s_task);
    pm_policy_lock_release(s_lock);
    s_stats.running = false;
    ESP_LOGI(TAG, "stopped");
    return err;
}

esp_err_t audio_playback_flush(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    /* The ring contract allows one caller per reader, so the cursor is only ever
     * touched by the playback task; this just asks it to (DES-APB-007). */
    s_flush_req = true;
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Queries                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t audio_playback_get_format(audio_playback_format_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    *out = s_fmt;
    return ESP_OK;
}

esp_err_t audio_playback_stats(audio_playback_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    out->running = (s_state == ST_RUNNING);
    out->primed = s_primed;
    out->source_backlog_bytes = (s_state != ST_UNINIT) ? (uint32_t)ringbuf_available(&s_reader) : 0;
    return ESP_OK;
}

esp_err_t audio_playback_stats_reset(void)
{
    bool running = s_stats.running;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.running = running;
    return ESP_OK;
}
