#include "audio_link.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
#include "link_priv.h"

static const char *TAG = "audio_link";

#define FRAME_US        ((uint32_t)CONFIG_AUDIO_LINK_FRAME_US)
#define BYTES_PER_SAMPLE 2u
#define TX_FRAMES_PER_TICK 2   /* Catch-up allowance after a late tick; more is a backlog. */

typedef enum { ST_UNINIT = 0, ST_READY, ST_RUNNING } state_t;

static struct {
    state_t             state;
    audio_link_config_t cfg;

    /* tx: tx.ring → encoder → transport */
    bool                tx_on;
    link_codec_t        enc;
    ringbuf_reader_t    tx_reader;
    int16_t            *tx_pcm;
    size_t              tx_frame_bytes;
    uint8_t             tx_format;
    uint16_t            tx_seq;
    uint8_t             tx_buf[AUDIO_LINK_FRAME_MAX_LEN];
    uint8_t             opus_out[AUDIO_LINK_FRAME_MAX_PAYLOAD];

    /* rx: transport → decoder → rx.ring */
    bool                rx_on;
    link_codec_t        dec;
    int16_t            *rx_pcm;
    size_t              rx_frame_bytes;
    uint8_t             rx_format;
    uint16_t            rx_expected;
    bool                rx_synced;
    uint32_t            rx_conceal_run;
    int64_t             rx_last_write_us;
    bool                mismatch_reported;

    /* Far end's last report (written on the link task, read from any task). */
    uint16_t            peer_backlog;
    int64_t             peer_report_us;

    QueueHandle_t       q;
    StaticQueue_t       q_ctrl;
    TaskHandle_t        task;
    SemaphoreHandle_t   ack;
    StaticSemaphore_t   ack_ctrl;
    volatile bool       up;
    bool                running;       /* Link task's view: framing active. */

    audio_link_stats_t  stats;
    portMUX_TYPE        mux;
#if CONFIG_PM_ENABLE && CONFIG_AUDIO_LINK_CPU_FREQ_LOCK
    esp_pm_lock_handle_t cpu_lock;
#endif
#if CONFIG_AUDIO_LINK_TEST_HOOKS
    uint32_t            drop_next;
    bool                swap_next;
    bool                holding;
    link_item_t         held;
#endif
} s = { .mux = portMUX_INITIALIZER_UNLOCKED };

static uint8_t     s_qbuf[CONFIG_AUDIO_LINK_RX_QUEUE_DEPTH * sizeof(link_item_t)];
static link_item_t s_item;       /* Link task only: keeps a 260-byte item off its stack. */
static link_item_t s_post;       /* Link task only: loopback re-post buffer.            */

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static void emit(audio_link_event_t evt)
{
    if (s.cfg.on_event != NULL) {
        s.cfg.on_event(evt, s.cfg.ctx);
    }
}

static uint32_t tx_bitrate(const audio_link_stream_t *t)
{
    if (t->bitrate_bps != 0) {
        return t->bitrate_bps;
    }
    return t->channels == 2 ? CONFIG_AUDIO_LINK_BITRATE_STEREO : CONFIG_AUDIO_LINK_BITRATE_MONO;
}

/* CBR: every packet is exactly this long (FW-LNK-016). */
static size_t cbr_payload(uint32_t bitrate)
{
    return (size_t)(((uint64_t)bitrate * FRAME_US) / 8000000u);
}

size_t link_core_max_frame_len(void)
{
    size_t max = AUDIO_LINK_FRAME_HDR_LEN;
    if (s.tx_on) {
        max = AUDIO_LINK_FRAME_HDR_LEN + cbr_payload(tx_bitrate(&s.cfg.tx));
    }
    /* The far end's frames are sized by its own bitrate, which this end cannot
     * know; the stereo default bounds the common case and the payload limit bounds
     * every case. */
    size_t rx = AUDIO_LINK_FRAME_HDR_LEN + cbr_payload(s.cfg.rx.channels == 2 ? CONFIG_AUDIO_LINK_BITRATE_STEREO
                                                                              : CONFIG_AUDIO_LINK_BITRATE_MONO);
    if (s.rx_on && rx > max) {
        max = rx;
    }
    return max > AUDIO_LINK_FRAME_MAX_LEN ? AUDIO_LINK_FRAME_MAX_LEN : max;
}

static uint16_t local_rx_backlog_frames(void)
{
    if (!s.rx_on || s.cfg.rx_backlog_cb == NULL) {
        return AUDIO_LINK_BACKLOG_UNKNOWN;
    }
    uint32_t frames = s.cfg.rx_backlog_cb(s.cfg.ctx) / (s.cfg.rx.channels * BYTES_PER_SAMPLE);
    return frames >= AUDIO_LINK_BACKLOG_UNKNOWN ? (uint16_t)(AUDIO_LINK_BACKLOG_UNKNOWN - 1) : (uint16_t)frames;
}

static void time_codec(uint32_t *max, uint32_t *avg, int64_t t0)
{
    uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    if (us > *max) {
        *max = us;
    }
    *avg = *avg == 0 ? us : *avg - (*avg >> 4) + (us >> 4);
}

/* -------------------------------------------------------------------------- */
/* Transport dispatch                                                          */
/* -------------------------------------------------------------------------- */

static esp_err_t loopback_send(const uint8_t *frame, size_t len)
{
#if CONFIG_AUDIO_LINK_TEST_HOOKS
    bool audio = len > 0 && (frame[0] & 0x0Fu) == AUDIO_LINK_FRAME_AUDIO;
    if (audio && s.drop_next > 0) {
        s.drop_next--;
        return ESP_OK;              /* "Sent" and lost on the air. */
    }
    if (audio && s.swap_next) {
        s.swap_next = false;
        s.held.kind = ITEM_FRAME;
        s.held.len = (uint16_t)len;
        memcpy(s.held.data, frame, len);
        s.holding = true;
        return ESP_OK;
    }
#endif
    link_core_post_frame(frame, len);
#if CONFIG_AUDIO_LINK_TEST_HOOKS
    if (s.holding) {
        s.holding = false;
        link_core_post_frame(s.held.data, s.held.len);
    }
#endif
    return ESP_OK;
}

static esp_err_t transport_send(const uint8_t *frame, size_t len)
{
    if (s.cfg.transport == AUDIO_LINK_TRANSPORT_LOOPBACK) {
        return loopback_send(frame, len);
    }
    return link_ble_send(frame, len);
}

void link_core_post_frame(const uint8_t *data, size_t len)
{
    if (s.q == NULL || len > AUDIO_LINK_FRAME_MAX_LEN) {
        return;
    }
    /* Called from the BTC task (BLE) or from the link task itself (loopback); both
     * may use a stack or static item, but the link task must not reuse s_item
     * while it is being processed. */
    link_item_t *it = (xTaskGetCurrentTaskHandle() == s.task) ? &s_post : NULL;
    link_item_t local;
    if (it == NULL) {
        it = &local;
    }
    it->kind = ITEM_FRAME;
    it->len = (uint16_t)len;
    memcpy(it->data, data, len);
    if (xQueueSend(s.q, it, 0) != pdTRUE) {
        portENTER_CRITICAL(&s.mux);
        s.stats.rx_queue_overflow++;
        portEXIT_CRITICAL(&s.mux);
    }
}

void link_core_post_event(uint8_t evt, uint8_t status, const uint8_t *peer, uint8_t addr_type, uint16_t handle)
{
    if (s.q == NULL) {
        return;
    }
    /* Events carry no payload, but the queue item is fixed-size. */
    static link_item_t ev;   /* BTC task only. */
    ev.kind = ITEM_EVENT;
    ev.evt = evt;
    ev.status = status;
    ev.addr_type = addr_type;
    ev.handle = handle;
    ev.len = 0;
    if (peer != NULL) {
        memcpy(ev.peer, peer, 6);
    } else {
        memset(ev.peer, 0, 6);
    }
    if (xQueueSend(s.q, &ev, pdMS_TO_TICKS(20)) != pdTRUE) {
        ESP_LOGW(TAG, "event %u dropped: queue full", evt);
    }
}

void link_core_set_up(bool up)
{
    if (up == s.up) {
        return;
    }
    s.up = up;
    portENTER_CRITICAL(&s.mux);
    s.stats.up = up;
    if (up) {
        s.stats.link_ups++;
    } else {
        s.stats.link_downs++;
    }
    s.peer_backlog = AUDIO_LINK_BACKLOG_UNKNOWN;
    s.peer_report_us = 0;
    portEXIT_CRITICAL(&s.mux);

    if (up) {
        s.rx_synced = false;
        s.rx_conceal_run = 0;
        s.mismatch_reported = false;
        if (s.tx_on) {
            ringbuf_reader_skip_to_newest(&s.tx_reader);   /* never send stale audio */
        }
#if CONFIG_PM_ENABLE && CONFIG_AUDIO_LINK_CPU_FREQ_LOCK
        esp_pm_lock_acquire(s.cpu_lock);
#endif
        ESP_LOGI(TAG, "link up");
        emit(AUDIO_LINK_EVT_UP);
    } else {
#if CONFIG_PM_ENABLE && CONFIG_AUDIO_LINK_CPU_FREQ_LOCK
        esp_pm_lock_release(s.cpu_lock);
#endif
        ESP_LOGI(TAG, "link down");
        emit(AUDIO_LINK_EVT_DOWN);
    }
}

/* -------------------------------------------------------------------------- */
/* Receive path — link task                                                    */
/* -------------------------------------------------------------------------- */

static void rx_write(int samples)
{
    if (samples <= 0) {
        return;
    }
    size_t bytes = (size_t)samples * s.cfg.rx.channels * BYTES_PER_SAMPLE;
    ringbuf_write(s.cfg.rx.ring, s.rx_pcm, bytes);
    s.rx_last_write_us = esp_timer_get_time();
}

static void conceal_one(void)
{
    int64_t t0 = esp_timer_get_time();
    int n = link_decode(&s.dec, NULL, 0, s.rx_pcm);
    time_codec(&s.stats.decode_us_max, &s.stats.decode_us_avg, t0);
    if (n < 0) {
        s.stats.rx_decode_errors++;
        return;
    }
    rx_write(n);
    s.stats.rx_concealed++;
    s.rx_expected++;
    s.rx_conceal_run++;
}

static void handle_audio(const audio_link_frame_t *f, size_t wire_len)
{
    if (!s.rx_on) {
        return;
    }
    if (f->format != s.rx_format) {
        s.stats.rx_format_mismatch++;
        if (!s.mismatch_reported) {
            s.mismatch_reported = true;
            ESP_LOGE(TAG, "peer format 0x%02x, expected 0x%02x: dropping its audio", f->format, s.rx_format);
            emit(AUDIO_LINK_EVT_FORMAT_MISMATCH);
        }
        return;
    }
    if (!s.rx_synced) {
        s.rx_expected = f->seq;
        s.rx_synced = true;
    }
    int16_t d = audio_link_seq_delta(f->seq, s.rx_expected);
    if (d < 0) {
        s.stats.rx_late++;     /* Already concealed, or a duplicate: never rewind. */
        return;
    }
    if (d > 0) {
        s.stats.rx_lost += (uint32_t)d;
        if (d <= CONFIG_AUDIO_LINK_MAX_CONCEAL_FRAMES) {
            for (int i = 0; i < d; i++) {
                conceal_one();
            }
        } else {
            s.stats.rx_resyncs++;
        }
        s.rx_expected = f->seq;
    }
    int64_t t0 = esp_timer_get_time();
    int n = link_decode(&s.dec, f->payload, f->payload_len, s.rx_pcm);
    time_codec(&s.stats.decode_us_max, &s.stats.decode_us_avg, t0);
    s.rx_expected = (uint16_t)(f->seq + 1u);
    if (n < 0) {
        s.stats.rx_decode_errors++;
        return;
    }
    rx_write(n);
    s.rx_conceal_run = 0;
    s.stats.rx_frames++;
    s.stats.rx_bytes += (uint32_t)wire_len;
}

static void handle_frame(const link_item_t *it)
{
    audio_link_frame_t f;
    if (audio_link_frame_parse(it->data, it->len, &f) != ESP_OK) {
        s.stats.rx_bad_frames++;
        return;
    }
    /* CONTROL frames are sent from application tasks that do not know the backlog;
     * only AUDIO and REPORT frames carry a report. */
    if (f.type != AUDIO_LINK_FRAME_CONTROL) {
        portENTER_CRITICAL(&s.mux);
        s.peer_backlog = f.backlog_frames;
        s.peer_report_us = esp_timer_get_time();
        portEXIT_CRITICAL(&s.mux);
    }

    switch (f.type) {
    case AUDIO_LINK_FRAME_AUDIO:
        handle_audio(&f, it->len);
        break;
    case AUDIO_LINK_FRAME_CONTROL:
        s.stats.rx_control++;
        if (s.cfg.on_control != NULL) {
            s.cfg.on_control(f.payload, f.payload_len, s.cfg.ctx);
        }
        break;
    case AUDIO_LINK_FRAME_REPORT:
    default:
        break;
    }
}

/* Frames stopped arriving: keep the consumer fed from the codec's concealment
 * rather than let it run dry because the radio is retransmitting (FW-LNK-021). */
static void conceal_if_starving(int64_t now_us)
{
    if (!s.rx_on || !s.rx_synced || s.rx_conceal_run >= CONFIG_AUDIO_LINK_MAX_CONCEAL_FRAMES) {
        return;
    }
    int64_t since = now_us - s.rx_last_write_us;
    if (since < (int64_t)(FRAME_US + FRAME_US / 2)) {
        return;
    }
    if (s.cfg.rx_backlog_cb != NULL) {
        uint32_t low = (uint32_t)((uint64_t)s.cfg.rx.sample_rate_hz * CONFIG_AUDIO_LINK_CONCEAL_LOW_WATER_MS / 1000u)
                     * s.cfg.rx.channels * BYTES_PER_SAMPLE;
        if (s.cfg.rx_backlog_cb(s.cfg.ctx) >= low) {
            return;
        }
    } else if (since < (int64_t)(2 * FRAME_US + FRAME_US / 2)) {
        /* Without a consumer's word for it, only a frame that is well overdue counts:
         * a source one tick out of phase delivers 0 then 2 frames, and two periods
         * would conceal the second — which then arrives and is dropped as late. */
        return;
    }
    conceal_one();
}

/* -------------------------------------------------------------------------- */
/* Transmit path — link task                                                   */
/* -------------------------------------------------------------------------- */

static bool send_frame(const audio_link_frame_t *f)
{
    size_t len = audio_link_frame_pack(f, s.tx_buf, sizeof(s.tx_buf));
    if (len == 0) {
        return false;
    }
    esp_err_t err = transport_send(s.tx_buf, len);
    if (err != ESP_OK) {
        s.stats.tx_busy++;
        return false;
    }
    s.stats.tx_bytes += (uint32_t)len;
    return true;
}

static bool tx_one(void)
{
    size_t got = 0;
    esp_err_t err = ringbuf_read(&s.tx_reader, s.tx_pcm, s.tx_frame_bytes, &got);
    if (err == ESP_ERR_INVALID_STATE) {
        s.stats.tx_skipped++;          /* Overrun: cursor resynchronised by ringbuf. */
        return false;
    }
    if (err != ESP_OK || got != s.tx_frame_bytes) {
        return false;
    }
    int64_t t0 = esp_timer_get_time();
    int n = link_encode(&s.enc, s.tx_pcm, s.opus_out, sizeof(s.opus_out));
    time_codec(&s.stats.encode_us_max, &s.stats.encode_us_avg, t0);
    uint16_t seq = s.tx_seq++;        /* Consumed even on failure: the far end sees a gap. */
    if (n < 0) {
        s.stats.tx_encode_errors++;
        return false;
    }
    const audio_link_frame_t f = {
        .type = AUDIO_LINK_FRAME_AUDIO,
        .format = s.tx_format,
        .seq = seq,
        .backlog_frames = local_rx_backlog_frames(),
        .payload = s.opus_out,
        .payload_len = (uint16_t)n,
    };
    if (send_frame(&f)) {
        s.stats.tx_frames++;
    }
    return true;
}

static void tick(void)
{
    int64_t now = esp_timer_get_time();
    bool sent_audio = false;

    if (s.tx_on) {
        if (!s.up) {
            ringbuf_reader_skip_to_newest(&s.tx_reader);
        } else {
            size_t max_backlog = (size_t)((uint64_t)s.cfg.tx.sample_rate_hz * CONFIG_AUDIO_LINK_TX_MAX_BACKLOG_MS / 1000u)
                               * s.cfg.tx.channels * BYTES_PER_SAMPLE;
            size_t avail = ringbuf_available(&s.tx_reader);
            if (avail > max_backlog) {
                s.stats.tx_skipped += (uint32_t)(avail / s.tx_frame_bytes);
                ringbuf_reader_skip_to_newest(&s.tx_reader);
            }
            for (int i = 0; i < TX_FRAMES_PER_TICK && ringbuf_available(&s.tx_reader) >= s.tx_frame_bytes; i++) {
                sent_audio |= tx_one();
            }
        }
    }

    /* The far end's servo needs this end's backlog every frame, audio or not. */
    if (s.up && !sent_audio && s.cfg.rx_backlog_cb != NULL && s.rx_on) {
        const audio_link_frame_t r = {
            .type = AUDIO_LINK_FRAME_REPORT,
            .backlog_frames = local_rx_backlog_frames(),
        };
        if (send_frame(&r)) {
            s.stats.tx_reports++;
        }
    }

    if (s.up) {
        conceal_if_starving(now);
    }
    if (s.cfg.transport == AUDIO_LINK_TRANSPORT_BLE) {
        link_ble_poll();
    }
}

/* -------------------------------------------------------------------------- */
/* Link task                                                                   */
/* -------------------------------------------------------------------------- */

static void link_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(FRAME_US / 1000u);
    TickType_t next = 0;

    for (;;) {
        TickType_t wait = portMAX_DELAY;
        if (s.running) {
            int32_t d = (int32_t)(next - xTaskGetTickCount());
            if (d <= 0) {
                tick();
                next += period;
                /* Fell more than a period behind: resume the cadence, don't burst. */
                if ((int32_t)(xTaskGetTickCount() - next) > (int32_t)period) {
                    next = xTaskGetTickCount() + period;
                }
                continue;
            }
            wait = (TickType_t)d;
        }
        if (xQueueReceive(s.q, &s_item, wait) != pdTRUE) {
            continue;
        }
        switch (s_item.kind) {
        case ITEM_START:
            s.running = true;
            s.tx_seq = 0;
            next = xTaskGetTickCount() + period;
            if (s.cfg.transport == AUDIO_LINK_TRANSPORT_LOOPBACK) {
                link_core_set_up(true);
            }
            xSemaphoreGive(s.ack);
            break;
        case ITEM_STOP:
            s.running = false;
            link_core_set_up(false);
            xSemaphoreGive(s.ack);
            break;
        case ITEM_EXIT:
            xSemaphoreGive(s.ack);
            vTaskDelete(NULL);
            return;
        case ITEM_FRAME:
            if (s.running) {
                handle_frame(&s_item);
            }
            break;
        case ITEM_EVENT:
            if (s.running && s.cfg.transport == AUDIO_LINK_TRANSPORT_BLE) {
                link_ble_handle_event(&s_item);
            }
            break;
        default:
            break;
        }
        uint32_t free_bytes = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
        if (s.stats.task_stack_free_min == 0 || free_bytes < s.stats.task_stack_free_min) {
            s.stats.task_stack_free_min = free_bytes;
        }
    }
}

static esp_err_t post_ctrl(link_item_kind_t kind)
{
    link_item_t it = { .kind = (uint8_t)kind };
    if (xQueueSendToFront(s.q, &it, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return xSemaphoreTake(s.ack, pdMS_TO_TICKS(2000)) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

static bool stream_valid(const audio_link_stream_t *st, bool tx)
{
    if (st->ring == NULL) {
        return true;
    }
    if (audio_link_format_code(st->sample_rate_hz, st->channels, FRAME_US) == 0xFF) {
        return false;
    }
    if (tx) {
        uint32_t br = tx_bitrate(st);
        if (br < 6000 || cbr_payload(br) > AUDIO_LINK_FRAME_MAX_PAYLOAD) {
            return false;
        }
    }
    return true;
}

static void release_all(void)
{
    link_codec_destroy(&s.enc);
    link_codec_destroy(&s.dec);
    heap_caps_free(s.tx_pcm);
    heap_caps_free(s.rx_pcm);
    s.tx_pcm = NULL;
    s.rx_pcm = NULL;
    if (s.q != NULL) {
        vQueueDelete(s.q);
        s.q = NULL;
    }
    if (s.ack != NULL) {
        vSemaphoreDelete(s.ack);
        s.ack = NULL;
    }
}

esp_err_t audio_link_init(const audio_link_config_t *cfg)
{
    if (s.state != ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || (cfg->tx.ring == NULL && cfg->rx.ring == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->role != AUDIO_LINK_ROLE_CENTRAL && cfg->role != AUDIO_LINK_ROLE_PERIPHERAL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->transport != AUDIO_LINK_TRANSPORT_BLE && cfg->transport != AUDIO_LINK_TRANSPORT_LOOPBACK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!stream_valid(&cfg->tx, true) || !stream_valid(&cfg->rx, false)) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Loopback decodes what it encodes, so the two directions must be one format. */
    if (cfg->transport == AUDIO_LINK_TRANSPORT_LOOPBACK && cfg->tx.ring && cfg->rx.ring &&
        (cfg->tx.sample_rate_hz != cfg->rx.sample_rate_hz || cfg->tx.channels != cfg->rx.channels)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s.stats, 0, sizeof(s.stats));
#if CONFIG_AUDIO_LINK_TEST_HOOKS
    /* A frame held by swap_next() in a previous session must not surface in this one. */
    s.drop_next = 0;
    s.swap_next = false;
    s.holding = false;
#endif
    s.cfg = *cfg;
    s.tx_on = cfg->tx.ring != NULL;
    s.rx_on = cfg->rx.ring != NULL;
    s.up = false;
    s.running = false;
    s.peer_backlog = AUDIO_LINK_BACKLOG_UNKNOWN;
    s.peer_report_us = 0;
    esp_err_t err = ESP_OK;

    if (s.tx_on) {
        err = link_enc_create(&s.enc, cfg->tx.sample_rate_hz, cfg->tx.channels, tx_bitrate(&cfg->tx), FRAME_US);
        if (err != ESP_OK) {
            goto fail;
        }
        s.tx_frame_bytes = (size_t)s.enc.frame_samples * cfg->tx.channels * BYTES_PER_SAMPLE;
        s.tx_pcm = heap_caps_malloc(s.tx_frame_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (s.tx_pcm == NULL) {
            err = ESP_ERR_NO_MEM;
            goto fail;
        }
        s.tx_format = audio_link_format_code(cfg->tx.sample_rate_hz, cfg->tx.channels, FRAME_US);
        err = ringbuf_reader_open(cfg->tx.ring, &s.tx_reader);
        if (err != ESP_OK) {
            goto fail;
        }
    }
    if (s.rx_on) {
        err = link_dec_create(&s.dec, cfg->rx.sample_rate_hz, cfg->rx.channels, FRAME_US);
        if (err != ESP_OK) {
            goto fail;
        }
        s.rx_frame_bytes = (size_t)s.dec.frame_samples * cfg->rx.channels * BYTES_PER_SAMPLE;
        s.rx_pcm = heap_caps_malloc(s.rx_frame_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (s.rx_pcm == NULL) {
            err = ESP_ERR_NO_MEM;
            goto fail;
        }
        s.rx_format = audio_link_format_code(cfg->rx.sample_rate_hz, cfg->rx.channels, FRAME_US);
    }
    s.stats.codec_state_bytes = (uint32_t)(s.enc.state_bytes + s.dec.state_bytes);
    s.stats.peer_backlog_frames = AUDIO_LINK_BACKLOG_UNKNOWN;
    s.stats.peer_report_age_ms = UINT32_MAX;

    s.q = xQueueCreateStatic(CONFIG_AUDIO_LINK_RX_QUEUE_DEPTH, sizeof(link_item_t), s_qbuf, &s.q_ctrl);
    s.ack = xSemaphoreCreateBinaryStatic(&s.ack_ctrl);
#if CONFIG_PM_ENABLE && CONFIG_AUDIO_LINK_CPU_FREQ_LOCK
    if (s.cpu_lock == NULL) {
        err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "audio_link", &s.cpu_lock);
        if (err != ESP_OK) {
            goto fail;
        }
    }
#endif
    if (xTaskCreatePinnedToCore(link_task, "audio_link", CONFIG_AUDIO_LINK_TASK_STACK, NULL,
                                CONFIG_AUDIO_LINK_TASK_PRIORITY, &s.task, CONFIG_AUDIO_LINK_TASK_CORE) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    s.state = ST_READY;
    ESP_LOGI(TAG, "init: %s over %s, tx %s, rx %s, %lu us frames, codec state %lu B",
             cfg->role == AUDIO_LINK_ROLE_CENTRAL ? "central" : "peripheral",
             cfg->transport == AUDIO_LINK_TRANSPORT_BLE ? "BLE" : "loopback",
             s.tx_on ? "on" : "off", s.rx_on ? "on" : "off", (unsigned long)FRAME_US,
             (unsigned long)s.stats.codec_state_bytes);
    return ESP_OK;

fail:
    release_all();
    return err;
}

esp_err_t audio_link_start(void)
{
    if (s.state != ST_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    /* The task must be consuming events before the transport can produce any: a
     * scan report dropped here is not repeated, because the controller filters
     * duplicates for the rest of the scan. */
    esp_err_t err = post_ctrl(ITEM_START);
    if (err != ESP_OK) {
        return err;
    }
    if (s.cfg.transport == AUDIO_LINK_TRANSPORT_BLE) {
        err = link_ble_start(&s.cfg);
        if (err != ESP_OK) {
            post_ctrl(ITEM_STOP);
            return err;
        }
    }
    s.state = ST_RUNNING;
    return ESP_OK;
}

esp_err_t audio_link_stop(void)
{
    if (s.state != ST_RUNNING) {
        return s.state == ST_UNINIT ? ESP_ERR_INVALID_STATE : ESP_OK;
    }
    esp_err_t err = post_ctrl(ITEM_STOP);
    if (s.cfg.transport == AUDIO_LINK_TRANSPORT_BLE) {
        link_ble_stop();
    }
    xQueueReset(s.q);       /* Drop frames that arrived for a link that is gone. */
    s.state = ST_READY;
    return err;
}

esp_err_t audio_link_deinit(void)
{
    if (s.state == ST_UNINIT) {
        return ESP_OK;
    }
    audio_link_stop();
    post_ctrl(ITEM_EXIT);
    s.task = NULL;
    release_all();
    s.state = ST_UNINIT;
    return ESP_OK;
}

bool audio_link_is_up(void)
{
    return s.state == ST_RUNNING && s.up;
}

esp_err_t audio_link_send_control(const void *msg, size_t len)
{
    if (msg == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > AUDIO_LINK_FRAME_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (!audio_link_is_up()) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[AUDIO_LINK_FRAME_MAX_LEN];
    const audio_link_frame_t f = {
        .type = AUDIO_LINK_FRAME_CONTROL,
        .backlog_frames = AUDIO_LINK_BACKLOG_UNKNOWN,
        .payload = msg,
        .payload_len = (uint16_t)len,
    };
    size_t n = audio_link_frame_pack(&f, buf, sizeof(buf));
    if (s.cfg.transport == AUDIO_LINK_TRANSPORT_LOOPBACK) {
        link_core_post_frame(buf, n);
        return ESP_OK;
    }
    return link_ble_send(buf, n);
}

uint32_t audio_link_path_backlog_bytes(void)
{
    if (s.state == ST_UNINIT || !s.tx_on) {
        return 0;
    }
    uint32_t local = (uint32_t)ringbuf_available(&s.tx_reader);
    uint16_t peer;
    portENTER_CRITICAL(&s.mux);
    peer = s.peer_backlog;
    portEXIT_CRITICAL(&s.mux);
    if (peer == AUDIO_LINK_BACKLOG_UNKNOWN) {
        return local;
    }
    return local + (uint32_t)peer * s.cfg.tx.channels * BYTES_PER_SAMPLE;
}

esp_err_t audio_link_peer_backlog(uint16_t *frames, uint32_t *age_ms)
{
    if (frames == NULL || age_ms == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t peer;
    int64_t at;
    portENTER_CRITICAL(&s.mux);
    peer = s.peer_backlog;
    at = s.peer_report_us;
    portEXIT_CRITICAL(&s.mux);
    if (at == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    *frames = peer;
    *age_ms = (uint32_t)((esp_timer_get_time() - at) / 1000);
    return ESP_OK;
}

esp_err_t audio_link_stats(audio_link_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s.state == ST_UNINIT) {
        memset(out, 0, sizeof(*out));
        return ESP_OK;
    }
    portENTER_CRITICAL(&s.mux);
    *out = s.stats;
    out->peer_backlog_frames = s.peer_backlog;
    int64_t at = s.peer_report_us;
    portEXIT_CRITICAL(&s.mux);
    out->peer_report_age_ms = at == 0 ? UINT32_MAX : (uint32_t)((esp_timer_get_time() - at) / 1000);
    return ESP_OK;
}

#if CONFIG_AUDIO_LINK_TEST_HOOKS
void audio_link_test_drop_next(uint32_t n)
{
    s.drop_next = n;
}

void audio_link_test_swap_next(void)
{
    s.swap_next = true;
}
#endif
