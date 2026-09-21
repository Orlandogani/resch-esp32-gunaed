#include "audio_capture.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"
#include "diag.h"
#include "pm_policy.h"

static const char *TAG = "audio_capture";

typedef enum { ST_UNINIT = 0, ST_READY, ST_RUNNING } state_t;

static state_t                  s_state;
static audio_capture_config_t   s_cfg;
static audio_capture_format_t   s_fmt;
static i2s_chan_handle_t        s_chan;
static size_t                   s_frame_bytes;     /* One DMA descriptor's payload. */
static uint8_t                 *s_frame_buf;       /* Task-side copy of one descriptor. */
static uint8_t                 *s_ring_storage;
static ringbuf_t                s_ring;
static TaskHandle_t             s_task;
static pm_policy_lock_handle_t  s_lock;
static volatile uint32_t        s_isr_overruns;    /* Written in ISR, read in task. */
static audio_capture_stats_t    s_stats;

/* -------------------------------------------------------------------------- */
/* ISR: notify only (FW-AUD-007, DR5)                                          */
/* -------------------------------------------------------------------------- */

static bool IRAM_ATTR on_recv_isr(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    if (s_task != NULL) {
        vTaskNotifyGiveFromISR(s_task, &woken);
    }
    return woken == pdTRUE;
}

static bool IRAM_ATTR on_recv_q_ovf_isr(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    s_isr_overruns++;
    return false;
}

/* -------------------------------------------------------------------------- */
/* Capture task (DES-ACAP-004, DES-ACAP-005)                                   */
/* -------------------------------------------------------------------------- */

static void capture_task(void *arg)
{
    uint32_t last_isr_overruns = 0;

    for (;;) {
        /* Woken once per completed DMA descriptor. A count > 1 means we fell behind. */
        uint32_t pending = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        if (s_state != ST_RUNNING) {
            continue;
        }

        uint32_t isr_ovf = s_isr_overruns;
        if (isr_ovf != last_isr_overruns) {
            s_stats.dma_overruns += isr_ovf - last_isr_overruns;
            last_isr_overruns = isr_ovf;
            DIAG_LOG_RL(ESP_LOGW, TAG, 1000, "DMA overrun (total %u)", (unsigned)s_stats.dma_overruns);
        }

        /* Drain every completed descriptor the notification count tells us about,
         * so a late wake-up catches up instead of leaving frames queued. */
        while (pending-- > 0) {
            size_t got = 0;
            esp_err_t err = i2s_channel_read(s_chan, s_frame_buf, s_frame_bytes, &got, 0);
            if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
                DIAG_LOG_RL(ESP_LOGW, TAG, 1000, "i2s_channel_read: %s", esp_err_to_name(err));
                break;
            }
            if (got == 0) {
                break;
            }
            if (got < s_frame_bytes) {
                s_stats.short_reads++;
            }
            if (ringbuf_write(&s_ring, s_frame_buf, got) != ESP_OK) {
                s_stats.ring_write_failures++;
            } else {
                s_stats.bytes_captured += got;
            }
            s_stats.dma_frames++;
        }

        /* Cheap enough per frame, and the number the SDD §13.2 budget asks for. */
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

static esp_err_t validate(const audio_capture_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->sample_rate_hz < 8000 || cfg->sample_rate_hz > 48000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->channels != 1 && cfg->channels != 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->interface == AUDIO_CAPTURE_IF_PDM) {
        if (cfg->bits_per_sample != 16) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (cfg->bits_per_sample != 16 && cfg->bits_per_sample != 32) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->pins.clk < 0 || cfg->pins.din < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->interface == AUDIO_CAPTURE_IF_I2S_STD && cfg->pins.ws < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t frames = cfg->dma_frame_count ? cfg->dma_frame_count : CONFIG_AUDIO_CAPTURE_DMA_FRAME_COUNT;
    if (frames < 2) {
        return ESP_ERR_INVALID_ARG; /* FW-AUD-002: double-buffering minimum. */
    }
    return ESP_OK;
}

static esp_err_t init_channel(uint32_t dma_frame_num, uint32_t dma_desc_num)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = dma_desc_num;
    chan_cfg.dma_frame_num = dma_frame_num;
    chan_cfg.auto_clear = false;

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        return err;
    }

    i2s_slot_mode_t mode = (s_cfg.channels == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;

    if (s_cfg.interface == AUDIO_CAPTURE_IF_PDM) {
        i2s_pdm_rx_config_t pdm_cfg = {
            .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(s_cfg.sample_rate_hz),
            .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, mode),
            .gpio_cfg = {
                .clk = s_cfg.pins.clk,
                .din = s_cfg.pins.din,
                .invert_flags = { .clk_inv = false },
            },
        };
        err = i2s_channel_init_pdm_rx_mode(s_chan, &pdm_cfg);
    } else {
        i2s_data_bit_width_t width = (s_cfg.bits_per_sample == 16) ? I2S_DATA_BIT_WIDTH_16BIT
                                                                   : I2S_DATA_BIT_WIDTH_32BIT;
        i2s_std_config_t std_cfg = {
            .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(s_cfg.sample_rate_hz),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(width, mode),
            .gpio_cfg = {
                .mclk = (s_cfg.pins.mclk >= 0) ? s_cfg.pins.mclk : I2S_GPIO_UNUSED,
                .bclk = s_cfg.pins.clk,
                .ws   = s_cfg.pins.ws,
                .dout = I2S_GPIO_UNUSED,
                .din  = s_cfg.pins.din,
                .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
            },
        };
        if (mode == I2S_SLOT_MODE_MONO) {
            std_cfg.slot_cfg.slot_mask = (s_cfg.slot == AUDIO_CAPTURE_SLOT_RIGHT) ? I2S_STD_SLOT_RIGHT
                                                                                  : I2S_STD_SLOT_LEFT;
        }
        err = i2s_channel_init_std_mode(s_chan, &std_cfg);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s channel mode init: %s", esp_err_to_name(err));
        i2s_del_channel(s_chan);
        s_chan = NULL;
        return err;
    }

    const i2s_event_callbacks_t cbs = {
        .on_recv = on_recv_isr,
        .on_recv_q_ovf = on_recv_q_ovf_isr,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
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

esp_err_t audio_capture_init(const audio_capture_config_t *cfg)
{
    if (s_state != ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = validate(cfg);
    if (err != ESP_OK) {
        return err;
    }

    s_cfg = *cfg;
    uint8_t  frame_ms   = cfg->dma_frame_ms ? cfg->dma_frame_ms : CONFIG_AUDIO_CAPTURE_DMA_FRAME_MS;
    uint8_t  frame_cnt  = cfg->dma_frame_count ? cfg->dma_frame_count : CONFIG_AUDIO_CAPTURE_DMA_FRAME_COUNT;
    uint16_t ring_ms    = cfg->ring_ms ? cfg->ring_ms : CONFIG_AUDIO_CAPTURE_RING_MS;

    /* ESP-IDF stores 24/32-bit slots in 4 bytes; 16-bit in 2. */
    s_fmt.sample_rate_hz  = cfg->sample_rate_hz;
    s_fmt.channels        = cfg->channels;
    s_fmt.bits_per_sample = cfg->bits_per_sample;
    s_fmt.bytes_per_sample = (cfg->bits_per_sample == 16) ? 2 : 4;
    s_fmt.bytes_per_ms    = (uint16_t)((cfg->sample_rate_hz * cfg->channels * s_fmt.bytes_per_sample) / 1000);

    uint32_t dma_frame_num = (cfg->sample_rate_hz * frame_ms) / 1000;   /* frames per descriptor */
    s_frame_bytes = dma_frame_num * cfg->channels * s_fmt.bytes_per_sample;
    size_t ring_bytes = (size_t)s_fmt.bytes_per_ms * ring_ms;
    if (ring_bytes < s_frame_bytes * 2) {
        ESP_LOGW(TAG, "ring (%u B) smaller than two DMA frames (%u B); raising to that",
                 (unsigned)ring_bytes, (unsigned)(s_frame_bytes * 2));
        ring_bytes = s_frame_bytes * 2;
    }

    /* Hot-path storage stays in internal SRAM (ADR-014). */
    s_frame_buf = heap_caps_malloc(s_frame_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_ring_storage = heap_caps_malloc(ring_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_frame_buf == NULL || s_ring_storage == NULL) {
        ESP_LOGE(TAG, "no internal SRAM for %u B frame + %u B ring", (unsigned)s_frame_bytes, (unsigned)ring_bytes);
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    err = ringbuf_init(&s_ring, s_ring_storage, ring_bytes);
    if (err != ESP_OK) {
        goto fail;
    }

    err = pm_policy_lock_create("audio", &s_lock);
    if (err != ESP_OK) {
        goto fail;
    }

    err = init_channel(dma_frame_num, frame_cnt);
    if (err != ESP_OK) {
        goto fail;
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_isr_overruns = 0;
    s_state = ST_READY; /* Task checks this; set before creating it. */

    BaseType_t ok = xTaskCreatePinnedToCore(capture_task, "audio_cap",
                                            CONFIG_AUDIO_CAPTURE_TASK_STACK, NULL,
                                            CONFIG_AUDIO_CAPTURE_TASK_PRIORITY, &s_task,
                                            CONFIG_AUDIO_CAPTURE_TASK_CORE);
    if (ok != pdPASS) {
        err = ESP_ERR_NO_MEM;
        s_state = ST_UNINIT;
        goto fail;
    }

    ESP_LOGI(TAG, "init: %s %u Hz %uch %u-bit, DMA %u x %u B (%u ms), ring %u B (%u ms), task prio %d core %d",
             cfg->interface == AUDIO_CAPTURE_IF_PDM ? "PDM" : "I2S",
             (unsigned)cfg->sample_rate_hz, cfg->channels, cfg->bits_per_sample,
             frame_cnt, (unsigned)s_frame_bytes, frame_ms, (unsigned)ring_bytes, ring_ms,
             CONFIG_AUDIO_CAPTURE_TASK_PRIORITY, CONFIG_AUDIO_CAPTURE_TASK_CORE);
    return ESP_OK;

fail:
    if (s_chan != NULL) {
        i2s_del_channel(s_chan);
        s_chan = NULL;
    }
    heap_caps_free(s_frame_buf);
    heap_caps_free(s_ring_storage);
    s_frame_buf = NULL;
    s_ring_storage = NULL;
    memset(&s_ring, 0, sizeof(s_ring));
    return err;
}

esp_err_t audio_capture_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    audio_capture_stop();

    if (s_task != NULL) {
        TaskHandle_t t = s_task;
        s_task = NULL;          /* ISR checks this before notifying. */
        vTaskDelete(t);
    }
    if (s_chan != NULL) {
        i2s_del_channel(s_chan);
        s_chan = NULL;
    }
    heap_caps_free(s_frame_buf);
    heap_caps_free(s_ring_storage);
    s_frame_buf = NULL;
    s_ring_storage = NULL;
    memset(&s_ring, 0, sizeof(s_ring));
    s_state = ST_UNINIT;
    /* s_lock is kept: pm_policy locks are never destroyed. */
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

esp_err_t audio_capture_start(void)
{
    if (s_state != ST_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = i2s_channel_enable(s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable: %s", esp_err_to_name(err));
        return err;
    }
    pm_policy_lock_acquire(s_lock);
    diag_task_register(s_task);
    s_stats.running = true;
    s_state = ST_RUNNING;
    ESP_LOGI(TAG, "started");
    return ESP_OK;
}

esp_err_t audio_capture_stop(void)
{
    if (s_state != ST_RUNNING) {
        return s_state == ST_UNINIT ? ESP_ERR_INVALID_STATE : ESP_OK;
    }
    s_state = ST_READY;
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

/* -------------------------------------------------------------------------- */
/* Queries                                                                     */
/* -------------------------------------------------------------------------- */

ringbuf_t *audio_capture_get_ring(void)
{
    return s_state != ST_UNINIT ? &s_ring : NULL;
}

esp_err_t audio_capture_get_format(audio_capture_format_t *out)
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

esp_err_t audio_capture_stats(audio_capture_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    out->running = (s_state == ST_RUNNING);
    return ESP_OK;
}

esp_err_t audio_capture_stats_reset(void)
{
    bool running = s_stats.running;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.running = running;
    return ESP_OK;
}
