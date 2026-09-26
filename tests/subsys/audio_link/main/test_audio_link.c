/**
 * On-target Unity tests for subsys/audio_link.
 *
 * Everything except the radio runs on one board: the loopback transport hands every
 * frame this end sends straight back to its own receive path, so the real codec, the
 * framing, sequencing, loss and reorder handling, concealment, backlog reporting and
 * control frames are all exercised end to end. Loss and reorder are injected
 * deterministically (CONFIG_AUDIO_LINK_TEST_HOOKS). The BLE transport needs two
 * boards and is a bench item (SDD §16.3).
 *
 * The codec timing and memory figures the SDD quotes come from the "benchmark" case,
 * which prints them.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "ringbuf.h"
#include "audio_link.h"

#define FRAME_US        CONFIG_AUDIO_LINK_FRAME_US
#define FRAME_TICKS     pdMS_TO_TICKS(FRAME_US / 1000)
#define MAX_RATE        48000
#define MAX_CH          2
#define MAX_FRAME_SAMP  (MAX_RATE * FRAME_US / 1000000)
#define MAX_FRAME_BYTES (MAX_FRAME_SAMP * MAX_CH * 2)
#define TONE_HZ         1000
#define TONE_AMPL       8000.0f

static uint8_t          s_tx_store[MAX_FRAME_BYTES * 12];
static uint8_t          s_rx_store[MAX_FRAME_BYTES * 40];
static ringbuf_t        s_tx;
static ringbuf_t        s_rx;
static ringbuf_reader_t s_rx_reader;
static int16_t          s_frame[MAX_FRAME_SAMP * MAX_CH];
static uint32_t         s_phase;
static uint32_t         s_rate;
static uint8_t          s_ch;
static size_t           s_frame_bytes;

static int      s_ups;
static int      s_downs;
static int      s_mismatch;
static uint32_t s_backlog_bytes;
static uint8_t  s_ctrl[64];
static size_t   s_ctrl_len;
static int      s_ctrl_count;

static void on_event(audio_link_event_t evt, void *ctx)
{
    if (evt == AUDIO_LINK_EVT_UP) {
        s_ups++;
    } else if (evt == AUDIO_LINK_EVT_DOWN) {
        s_downs++;
    } else if (evt == AUDIO_LINK_EVT_FORMAT_MISMATCH) {
        s_mismatch++;
    }
}

static void on_control(const uint8_t *msg, size_t len, void *ctx)
{
    s_ctrl_len = len < sizeof(s_ctrl) ? len : sizeof(s_ctrl);
    memcpy(s_ctrl, msg, s_ctrl_len);
    s_ctrl_count++;
}

static uint32_t backlog_cb(void *ctx) { return s_backlog_bytes; }

static void gen_frame(void)
{
    uint32_t n = s_rate * FRAME_US / 1000000;
    for (uint32_t i = 0; i < n; i++, s_phase++) {
        float v = TONE_AMPL * sinf(2.0f * (float)M_PI * TONE_HZ * (float)s_phase / (float)s_rate);
        for (uint8_t c = 0; c < s_ch; c++) {
            s_frame[i * s_ch + c] = (int16_t)v;
        }
    }
}

/* Produce `frames` frames at the frame rate, as a real source would. */
static void feed(int frames)
{
    TickType_t last = xTaskGetTickCount();
    for (int f = 0; f < frames; f++) {
        gen_frame();
        TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&s_tx, s_frame, s_frame_bytes));
        vTaskDelayUntil(&last, FRAME_TICKS);
    }
}

static void rings(uint32_t rate, uint8_t ch)
{
    /* A failed assertion longjmps past the previous test's own teardown; start clean
     * so one failure does not cascade (deinit is idempotent). */
    audio_link_deinit();
    s_rate = rate;
    s_ch = ch;
    s_phase = 0;
    s_frame_bytes = (size_t)(rate * FRAME_US / 1000000) * ch * 2;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&s_tx, s_tx_store, sizeof(s_tx_store)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&s_rx, s_rx_store, sizeof(s_rx_store)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&s_rx, &s_rx_reader));
}

static audio_link_config_t loop_cfg(uint32_t rate, uint8_t ch, audio_link_backlog_cb_t cb)
{
    const audio_link_config_t c = {
        .role = AUDIO_LINK_ROLE_CENTRAL,
        .transport = AUDIO_LINK_TRANSPORT_LOOPBACK,
        .tx = { .ring = &s_tx, .sample_rate_hz = rate, .channels = ch },
        .rx = { .ring = &s_rx, .sample_rate_hz = rate, .channels = ch },
        .rx_backlog_cb = cb,
        .on_event = on_event,
        .on_control = on_control,
    };
    return c;
}

static void start_loopback(uint32_t rate, uint8_t ch, audio_link_backlog_cb_t cb)
{
    s_ups = s_downs = s_mismatch = 0;
    s_ctrl_count = 0;
    rings(rate, ch);
    audio_link_config_t c = loop_cfg(rate, ch, cb);
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_start());
    TEST_ASSERT_TRUE(audio_link_is_up());
    TEST_ASSERT_EQUAL(1, s_ups);
    /* Two frames of slack so a source that is a tick late never starves the link. */
    feed(2);
}

static void finish(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stop());
    TEST_ASSERT_FALSE(audio_link_is_up());
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_deinit());
}

/* Every sequence slot must be filled exactly once — decoded or concealed. */
static void assert_continuous(const audio_link_stats_t *st)
{
    ringbuf_stats_t rs;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_stats(&s_rx, &rs));
    /* 32-bit compare: this Unity is built without 64-bit assertions, and a test run
     * writes a few hundred KiB at most. */
    TEST_ASSERT_TRUE(rs.bytes_written < UINT32_MAX);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)((st->rx_frames + st->rx_concealed) * s_frame_bytes),
                             (uint32_t)rs.bytes_written);
}

/* ------------------------------------------------------------------------- */
/* Frame format                                                              */
/* ------------------------------------------------------------------------- */

TEST_CASE("frame pack/parse round-trips every type and rejects malformed input", "[audio_link]")
{
    uint8_t buf[AUDIO_LINK_FRAME_MAX_LEN];
    const uint8_t payload[5] = { 1, 2, 3, 4, 5 };
    audio_link_frame_t in = { .type = AUDIO_LINK_FRAME_AUDIO, .format = 0x2C, .seq = 0xBEEF,
                              .backlog_frames = 1234, .payload = payload, .payload_len = 5 };
    size_t n = audio_link_frame_pack(&in, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(AUDIO_LINK_FRAME_HDR_LEN + 5, n);
    TEST_ASSERT_EQUAL_HEX8((AUDIO_LINK_FRAME_VERSION << 4) | AUDIO_LINK_FRAME_AUDIO, buf[0]);
    audio_link_frame_t out;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_frame_parse(buf, n, &out));
    TEST_ASSERT_EQUAL(AUDIO_LINK_FRAME_AUDIO, out.type);
    TEST_ASSERT_EQUAL_HEX8(0x2C, out.format);
    TEST_ASSERT_EQUAL_HEX16(0xBEEF, out.seq);
    TEST_ASSERT_EQUAL(1234, out.backlog_frames);
    TEST_ASSERT_EQUAL(5, out.payload_len);
    TEST_ASSERT_EQUAL_MEMORY(payload, out.payload, 5);

    /* REPORT: header only; format and seq are not carried. */
    audio_link_frame_t rep = { .type = AUDIO_LINK_FRAME_REPORT, .format = 0x55, .seq = 9, .backlog_frames = 7 };
    n = audio_link_frame_pack(&rep, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(AUDIO_LINK_FRAME_HDR_LEN, n);
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_frame_parse(buf, n, &out));
    TEST_ASSERT_EQUAL(AUDIO_LINK_FRAME_REPORT, out.type);
    TEST_ASSERT_EQUAL(0, out.seq);
    TEST_ASSERT_EQUAL(7, out.backlog_frames);
    TEST_ASSERT_NULL(out.payload);

    /* Pack refuses what parse would refuse. */
    rep.payload = payload;
    rep.payload_len = 1;
    TEST_ASSERT_EQUAL(0, audio_link_frame_pack(&rep, buf, sizeof(buf)));
    in.payload_len = AUDIO_LINK_FRAME_MAX_PAYLOAD + 1;
    TEST_ASSERT_EQUAL(0, audio_link_frame_pack(&in, buf, sizeof(buf)));
    in.payload_len = 5;
    TEST_ASSERT_EQUAL(0, audio_link_frame_pack(&in, buf, 8));          /* cap too small */
    in.payload = NULL;
    TEST_ASSERT_EQUAL(0, audio_link_frame_pack(&in, buf, sizeof(buf)));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, audio_link_frame_parse(buf, 5, &out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, audio_link_frame_parse(buf, AUDIO_LINK_FRAME_MAX_LEN + 1, &out));
    uint8_t bad[8] = { 0x21, 0, 0, 0, 0, 0, 0, 0 };                     /* version 2 */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_VERSION, audio_link_frame_parse(bad, sizeof(bad), &out));
    bad[0] = (AUDIO_LINK_FRAME_VERSION << 4) | 0x0F;                   /* unknown type */
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, audio_link_frame_parse(bad, sizeof(bad), &out));
    bad[0] = (AUDIO_LINK_FRAME_VERSION << 4) | AUDIO_LINK_FRAME_REPORT; /* report with payload */
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, audio_link_frame_parse(bad, sizeof(bad), &out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_link_frame_parse(NULL, 8, &out));
}

TEST_CASE("format codes and sequence arithmetic", "[audio_link]")
{
    TEST_ASSERT_EQUAL_HEX8(0x2C, audio_link_format_code(48000, 2, 10000));
    TEST_ASSERT_EQUAL_HEX8(0x22, audio_link_format_code(16000, 1, 10000));
    TEST_ASSERT_EQUAL_HEX8(0x00, audio_link_format_code(8000, 1, 2500));
    TEST_ASSERT_EQUAL_HEX8(0x3B, audio_link_format_code(24000, 2, 20000));
    TEST_ASSERT_EQUAL_HEX8(0xFF, audio_link_format_code(44100, 2, 10000));
    TEST_ASSERT_EQUAL_HEX8(0xFF, audio_link_format_code(48000, 3, 10000));
    TEST_ASSERT_EQUAL_HEX8(0xFF, audio_link_format_code(48000, 2, 7500));

    TEST_ASSERT_EQUAL(0, audio_link_seq_delta(100, 100));
    TEST_ASSERT_EQUAL(3, audio_link_seq_delta(103, 100));
    TEST_ASSERT_EQUAL(-1, audio_link_seq_delta(99, 100));
    TEST_ASSERT_EQUAL(2, audio_link_seq_delta(1, 0xFFFF));             /* across the wrap */
    TEST_ASSERT_EQUAL(-2, audio_link_seq_delta(0xFFFF, 1));
}

/* ------------------------------------------------------------------------- */
/* Lifecycle and validation                                                  */
/* ------------------------------------------------------------------------- */

TEST_CASE("init validates the configuration and is a singleton", "[audio_link]")
{
    rings(48000, 2);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_link_init(NULL));

    audio_link_config_t c = loop_cfg(48000, 2, NULL);
    c.tx.ring = NULL;
    c.rx.ring = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_link_init(&c));           /* nothing to do */

    c = loop_cfg(44100, 2, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_link_init(&c));           /* not an Opus rate */
    c = loop_cfg(48000, 3, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_link_init(&c));
    c = loop_cfg(48000, 2, NULL);
    c.tx.bitrate_bps = 400000;                                             /* frame > one ATT payload */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_link_init(&c));
    c = loop_cfg(48000, 2, NULL);
    c.rx.sample_rate_hz = 16000;                                           /* loopback: one format */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_link_init(&c));
    c = loop_cfg(48000, 2, NULL);
    c.transport = (audio_link_transport_t)7;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_link_init(&c));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_link_start());          /* before init */
    c = loop_cfg(48000, 2, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_link_init(&c));
    TEST_ASSERT_FALSE(audio_link_is_up());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_link_send_control("x", 1));
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stop());                          /* idempotent when not started */
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_deinit());
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_deinit());
}

TEST_CASE("init/deinit returns the codec state and task memory", "[audio_link]")
{
    rings(48000, 2);
    audio_link_config_t c = loop_cfg(48000, 2, NULL);
    /* The first cycle may allocate once-only objects (the CPU frequency lock). */
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_deinit());
    vTaskDelay(pdMS_TO_TICKS(20));   /* let the idle task reclaim the deleted task */
    size_t before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, audio_link_init(&c));
        TEST_ASSERT_EQUAL(ESP_OK, audio_link_start());
        TEST_ASSERT_EQUAL(ESP_OK, audio_link_stop());
        TEST_ASSERT_EQUAL(ESP_OK, audio_link_deinit());
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    size_t after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    TEST_ASSERT_INT_WITHIN_MESSAGE(256, before, after, "internal heap shrinks per cycle");
}

/* ------------------------------------------------------------------------- */
/* End to end over loopback                                                  */
/* ------------------------------------------------------------------------- */

static void check_tone(int frames_to_skip)
{
    /* Read everything decoded; judge the tone after the codec has settled. */
    static int16_t pcm[MAX_FRAME_SAMP * MAX_CH];
    size_t got;
    int frame = 0;
    int crossings = 0;
    int16_t prev = 0;
    double energy = 0;
    uint32_t samples = 0;
    for (;;) {
        esp_err_t err = ringbuf_read(&s_rx_reader, pcm, s_frame_bytes, &got);
        if (err == ESP_ERR_INVALID_STATE) {
            /* A second of stereo overruns this ring before we read it: the cursor has
             * been resynchronised to the oldest audio still held, so keep reading. */
            continue;
        }
        if (err != ESP_OK || got != s_frame_bytes) {
            break;
        }
        if (frame++ < frames_to_skip) {
            continue;
        }
        uint32_t n = (uint32_t)(got / (2u * s_ch));
        for (uint32_t i = 0; i < n; i++) {
            int16_t v = pcm[i * s_ch];           /* first channel */
            if ((prev < 0 && v >= 0) || (prev >= 0 && v < 0)) {
                crossings++;
            }
            prev = v;
            energy += (double)v * v;
            samples++;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(samples > s_rate / 4, "less than 250 ms of decoded audio");
    double seconds = (double)samples / s_rate;
    double hz = crossings / 2.0 / seconds;
    double rms = sqrt(energy / samples);
    printf("decoded %.2f s: tone %.1f Hz, RMS %.0f (source %.0f)\n", seconds, hz, rms, TONE_AMPL / sqrt(2.0));
    TEST_ASSERT_TRUE_MESSAGE(hz > TONE_HZ * 0.95 && hz < TONE_HZ * 1.05, "decoded tone frequency off");
    TEST_ASSERT_TRUE_MESSAGE(rms > TONE_AMPL / sqrt(2.0) * 0.7 && rms < TONE_AMPL / sqrt(2.0) * 1.3,
                             "decoded tone level off");
}

TEST_CASE("48 kHz stereo round-trips through Opus with no loss and the tone intact", "[audio_link]")
{
    start_loopback(48000, 2, NULL);
    feed(100);                                  /* one second */
    vTaskDelay(3 * FRAME_TICKS);
    audio_link_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    printf("tx %lu rx %lu lost %lu late %lu concealed %lu busy %lu skipped %lu\n",
           (unsigned long)st.tx_frames, (unsigned long)st.rx_frames, (unsigned long)st.rx_lost,
           (unsigned long)st.rx_late, (unsigned long)st.rx_concealed, (unsigned long)st.tx_busy,
           (unsigned long)st.tx_skipped);
    TEST_ASSERT_TRUE(st.tx_frames >= 100);
    TEST_ASSERT_EQUAL(st.tx_frames, st.rx_frames);
    TEST_ASSERT_EQUAL(0, st.rx_lost);
    TEST_ASSERT_EQUAL(0, st.rx_late);
    TEST_ASSERT_EQUAL(0, st.rx_concealed);
    TEST_ASSERT_EQUAL(0, st.rx_decode_errors);
    TEST_ASSERT_EQUAL(0, st.tx_encode_errors);
    TEST_ASSERT_EQUAL(0, st.rx_bad_frames);
    assert_continuous(&st);
    check_tone(5);
    finish();
    TEST_ASSERT_EQUAL(1, s_downs);
}

TEST_CASE("16 kHz mono (the microphone format) round-trips too", "[audio_link]")
{
    start_loopback(16000, 1, NULL);
    feed(50);
    vTaskDelay(3 * FRAME_TICKS);
    audio_link_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    TEST_ASSERT_TRUE(st.rx_frames >= 50);
    TEST_ASSERT_EQUAL(0, st.rx_lost);
    assert_continuous(&st);
    check_tone(5);
    finish();
}

TEST_CASE("lost frames are concealed one for one and the stream stays continuous", "[audio_link]")
{
    start_loopback(48000, 2, NULL);
    feed(20);
    audio_link_test_drop_next(3);
    feed(30);
    vTaskDelay(3 * FRAME_TICKS);
    audio_link_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    printf("lost %lu concealed %lu late %lu\n", (unsigned long)st.rx_lost,
           (unsigned long)st.rx_concealed, (unsigned long)st.rx_late);
    /* The gap is filled either when the next frame reveals it or, earlier, by the
     * starvation path; either way three slots are concealed, none twice. */
    TEST_ASSERT_EQUAL(3, st.rx_concealed);
    TEST_ASSERT_EQUAL(st.tx_frames - 3, st.rx_frames);
    TEST_ASSERT_EQUAL(0, st.rx_resyncs);
    assert_continuous(&st);
    finish();
}

TEST_CASE("an out-of-order frame is concealed in place and then dropped as late", "[audio_link]")
{
    start_loopback(48000, 2, NULL);
    feed(10);
    audio_link_test_swap_next();
    feed(10);
    vTaskDelay(3 * FRAME_TICKS);
    audio_link_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    TEST_ASSERT_EQUAL(1, st.rx_lost);
    TEST_ASSERT_EQUAL(1, st.rx_late);
    TEST_ASSERT_EQUAL(1, st.rx_concealed);
    assert_continuous(&st);
    finish();
}

TEST_CASE("a starving consumer is fed by concealment, capped, then left to underrun", "[audio_link]")
{
    s_backlog_bytes = 0;                /* consumer reports itself empty */
    start_loopback(48000, 2, backlog_cb);
    feed(10);
    audio_link_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    uint32_t before = st.rx_concealed;
    vTaskDelay(20 * FRAME_TICKS);       /* source stops for 200 ms */
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    TEST_ASSERT_EQUAL(CONFIG_AUDIO_LINK_MAX_CONCEAL_FRAMES, st.rx_concealed - before);

    /* A consumer with plenty queued is left alone. */
    s_backlog_bytes = UINT32_MAX / 2;
    feed(5);
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    before = st.rx_concealed;
    vTaskDelay(20 * FRAME_TICKS);
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    TEST_ASSERT_EQUAL(before, st.rx_concealed);
    finish();
}

TEST_CASE("the far end's backlog report reaches the rate servo's measurement", "[audio_link]")
{
    s_backlog_bytes = 4800;             /* 25 ms of 48 kHz stereo = 1200 sample frames */
    start_loopback(48000, 2, backlog_cb);
    feed(10);
    uint16_t frames = 0;
    uint32_t age = 0;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_peer_backlog(&frames, &age));
    TEST_ASSERT_EQUAL(1200, frames);
    TEST_ASSERT_TRUE(age < 3 * FRAME_US / 1000);
    uint32_t path = audio_link_path_backlog_bytes();
    TEST_ASSERT_TRUE(path >= 4800 && path < 4800 + 3 * s_frame_bytes);

    /* With nothing to send, a REPORT still goes out every frame. */
    audio_link_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    uint32_t reports = st.tx_reports;
    vTaskDelay(10 * FRAME_TICKS);
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    TEST_ASSERT_TRUE(st.tx_reports - reports >= 8);
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_peer_backlog(&frames, &age));
    TEST_ASSERT_TRUE(age < 3 * FRAME_US / 1000);
    finish();
}

TEST_CASE("control frames are delivered intact and bounded", "[audio_link]")
{
    start_loopback(48000, 2, NULL);
    const uint8_t msg[] = { 0x01, 0xE9, 0x00, 0x7F };
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_send_control(msg, sizeof(msg)));
    vTaskDelay(3 * FRAME_TICKS);
    TEST_ASSERT_EQUAL(1, s_ctrl_count);
    TEST_ASSERT_EQUAL(sizeof(msg), s_ctrl_len);
    TEST_ASSERT_EQUAL_MEMORY(msg, s_ctrl, sizeof(msg));
    static uint8_t big[AUDIO_LINK_FRAME_MAX_PAYLOAD + 1];
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, audio_link_send_control(big, sizeof(big)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_link_send_control(NULL, 1));
    finish();
}

/* The headset's microphone uplink: 16 kHz mono. Loopback runs its encode and decode on
 * one core; a real headset encodes this and decodes the stereo downlink. */
TEST_CASE("benchmark: 16 kHz mono codec time per frame", "[audio_link]")
{
    start_loopback(16000, 1, NULL);
    feed(200);
    vTaskDelay(3 * FRAME_TICKS);
    audio_link_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    printf("BENCH mono16k encode avg %lu us max %lu us | decode avg %lu us max %lu us | codec state %lu B\n",
           (unsigned long)st.encode_us_avg, (unsigned long)st.encode_us_max,
           (unsigned long)st.decode_us_avg, (unsigned long)st.decode_us_max,
           (unsigned long)st.codec_state_bytes);
    TEST_ASSERT_TRUE_MESSAGE(st.encode_us_max + st.decode_us_max < FRAME_US, "codec slower than real time");
    finish();
}

TEST_CASE("benchmark: codec time per frame, state and stack", "[audio_link]")
{
    size_t heap_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    start_loopback(48000, 2, NULL);
    feed(300);                          /* three seconds */
    vTaskDelay(3 * FRAME_TICKS);
    audio_link_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_link_stats(&st));
    size_t heap_during = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    printf("BENCH frame %d us, complexity %d, %d bit/s stereo\n", FRAME_US,
           CONFIG_AUDIO_LINK_OPUS_COMPLEXITY, CONFIG_AUDIO_LINK_BITRATE_STEREO);
    printf("BENCH encode avg %lu us max %lu us | decode avg %lu us max %lu us\n",
           (unsigned long)st.encode_us_avg, (unsigned long)st.encode_us_max,
           (unsigned long)st.decode_us_avg, (unsigned long)st.decode_us_max);
    printf("BENCH codec state %lu B, link task stack free min %lu B of %d, internal heap used %u B\n",
           (unsigned long)st.codec_state_bytes, (unsigned long)st.task_stack_free_min,
           CONFIG_AUDIO_LINK_TASK_STACK, (unsigned)(heap_before - heap_during));
    /* Real time: one frame must encode and decode within one frame duration. */
    TEST_ASSERT_TRUE_MESSAGE(st.encode_us_max + st.decode_us_max < FRAME_US, "codec slower than real time");
    TEST_ASSERT_TRUE(st.codec_state_bytes > 0);
    TEST_ASSERT_TRUE_MESSAGE(st.task_stack_free_min > 1024, "link task stack nearly exhausted");
    finish();
}

void app_main(void)
{
    /* A USB-Serial/JTAG console drops output while no host has the port open, and
     * this suite starts printing at once: give the capture script time to attach. */
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
