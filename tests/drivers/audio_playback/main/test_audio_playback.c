/**
 * On-target Unity tests for drivers/audio_playback.
 *
 * No DAC is needed. The I2S TX peripheral clocks and its DMA drains whether or not
 * anything is listening, so the driver's real behaviour — lifecycle, consumption
 * rate at the sample clock, prefill hysteresis, underrun accounting, reader overrun,
 * flush and the pm_policy lock — is all observable from the counters. That the
 * samples leaving DOUT are the right samples needs a real DAC and is a bench item
 * (TBD-010).
 *
 * The last test closes that gap as far as the board alone can: it routes DOUT into
 * `audio_capture`'s DIN over the GPIO matrix (no wiring) and checks the played bytes
 * actually reach the pin. It cannot check sample alignment, because two independently
 * enabled master channels share a frequency but not a phase — see DES-APB-009.
 *
 * Pins default to GPIO 15/16/17 for playback and 4/5 for the loopback capture clocks,
 * all free on a stock ESP32-S3 devkit with octal PSRAM (which takes 26..37).
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "cfg.h"
#include "diag.h"
#include "pm_policy.h"
#include "power.h"
#include "ringbuf.h"
#include "audio_playback.h"
#include "audio_capture.h"

#define PIN_BCLK      15
#define PIN_WS        16
#define PIN_DOUT      17
#define PIN_CAP_CLK    4
#define PIN_CAP_WS     5

/* 16 kHz mono 16-bit = 32 B/ms, the same arithmetic the capture tests use. */
#define RATE_HZ       16000
#define BYTES_PER_MS  32

#define RING_MS       100
#define RING_BYTES    (BYTES_PER_MS * RING_MS)

static ringbuf_t        s_ring;
static uint8_t          s_ring_storage[RING_BYTES];
static TaskHandle_t     s_feeder;
static volatile bool    s_feeder_run;
static volatile uint32_t s_feeder_bytes;

static audio_playback_config_t base_cfg(void)
{
    audio_playback_config_t c = {
        .source = &s_ring,
        .sample_rate_hz = RATE_HZ,
        .channels = 1,
        .bits_per_sample = 16,
        .slot = AUDIO_PLAYBACK_SLOT_BOTH,
        .port = -1,
        .prefill_ms = 10,
        .pins = { .bclk = PIN_BCLK, .ws = PIN_WS, .dout = PIN_DOUT, .mclk = -1 },
    };
    return c;
}

static void feeder_stop(void);

static void fresh(void)
{
    /* A failed assertion longjmps out of the test body, so the test that failed
     * never ran its own cleanup. Undo it here rather than letting a leftover
     * feeder task write into the next test's ring. */
    feeder_stop();
    audio_playback_deinit();
    audio_capture_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    TEST_ASSERT_EQUAL(ESP_OK, diag_init(NULL));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&s_ring, s_ring_storage, sizeof(s_ring_storage)));
}

/* A ramp, so a silence check cannot pass on accident and a stuck byte is visible. */
static void fill_pattern(uint8_t *buf, size_t len, uint32_t seq)
{
    for (size_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)(seq + i);
    }
}

/* Writes `chunk_ms` of audio every `chunk_ms` — nominal rate, 10 ms granularity. */
static void feeder_task(void *arg)
{
    const size_t chunk_ms = 10;
    uint8_t chunk[BYTES_PER_MS * 10];
    uint32_t seq = 0;
    while (s_feeder_run) {
        fill_pattern(chunk, sizeof(chunk), seq);
        if (ringbuf_write(&s_ring, chunk, sizeof(chunk)) == ESP_OK) {
            s_feeder_bytes += sizeof(chunk);
        }
        seq += sizeof(chunk);
        vTaskDelay(pdMS_TO_TICKS(chunk_ms));
    }
    s_feeder = NULL;
    vTaskDelete(NULL);
}

static void feeder_start(void)
{
    s_feeder_run = true;
    s_feeder_bytes = 0;
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreatePinnedToCore(feeder_task, "feeder", 3072, NULL, 10,
                                                      &s_feeder, 0));
}

static void feeder_stop(void)
{
    s_feeder_run = false;
    while (s_feeder != NULL) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ------------------------------------------------------------------------- */
/* Lifecycle and validation                                                   */
/* ------------------------------------------------------------------------- */

TEST_CASE("before init: queries fail cleanly", "[audio_playback]")
{
    audio_playback_deinit();
    audio_playback_format_t f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_playback_get_format(&f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_playback_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_playback_stop());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_playback_flush());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_get_format(NULL));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit()); /* idempotent */
}

TEST_CASE("init rejects unsupported configurations without side effects", "[audio_playback]")
{
    fresh();
    audio_playback_config_t c;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(NULL));
    c = base_cfg(); c.source = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));
    c = base_cfg(); c.sample_rate_hz = 96000;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));
    c = base_cfg(); c.channels = 3;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));
    c = base_cfg(); c.bits_per_sample = 24;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));
    c = base_cfg(); c.port = -2;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));
    c = base_cfg(); c.pins.dout = -1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));
    c = base_cfg(); c.pins.ws = -1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));
    c = base_cfg(); c.pins.bclk = -1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));
    c = base_cfg(); c.dma_frame_count = 1;          /* below double-buffering */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));

    /* 48 kHz stereo 32-bit at 20 ms is 7680 B — over the 4092 B GDMA limit. */
    c = base_cfg();
    c.sample_rate_hz = 48000; c.channels = 2; c.bits_per_sample = 32; c.dma_frame_ms = 20;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));

    /* A ring that cannot hold the prefill plus one frame can never prime. */
    c = base_cfg(); c.prefill_ms = RING_MS;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_playback_init(&c));

    /* Nothing was left initialised. */
    audio_playback_format_t f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_playback_get_format(&f));
}

TEST_CASE("init/deinit cycle is clean and repeatable", "[audio_playback]")
{
    fresh();
    audio_playback_config_t c = base_cfg();
    size_t heap_before = esp_get_free_heap_size();
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, audio_playback_init(&c));
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_playback_init(&c)); /* singleton */
        TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit());
    }
    size_t heap_after = esp_get_free_heap_size();
    TEST_ASSERT_TRUE_MESSAGE(heap_before - heap_after < 512, "heap leak across init/deinit cycles");
}

TEST_CASE("format and prefill sizing follow the configuration", "[audio_playback]")
{
    fresh();
    audio_playback_config_t c = base_cfg();
    c.dma_frame_ms = 2;
    c.prefill_ms = 10;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_init(&c));

    audio_playback_format_t f;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_get_format(&f));
    TEST_ASSERT_EQUAL(RATE_HZ, f.sample_rate_hz);
    TEST_ASSERT_EQUAL(1, f.channels);
    TEST_ASSERT_EQUAL(2, f.bytes_per_sample);
    TEST_ASSERT_EQUAL(BYTES_PER_MS, f.bytes_per_ms);
    TEST_ASSERT_EQUAL(BYTES_PER_MS * 2, f.frame_bytes);
    TEST_ASSERT_EQUAL(BYTES_PER_MS * 10, f.prefill_bytes);
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit());

    /* 32-bit slots are stored in 4 bytes, and stereo doubles again. */
    c.bits_per_sample = 32;
    c.channels = 2;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_get_format(&f));
    TEST_ASSERT_EQUAL(4, f.bytes_per_sample);
    TEST_ASSERT_EQUAL(128, f.bytes_per_ms);
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit());
}

/* ------------------------------------------------------------------------- */
/* Running                                                                    */
/* ------------------------------------------------------------------------- */

TEST_CASE("start consumes at the sample clock's rate; stop halts it", "[audio_playback]")
{
    fresh();
    audio_playback_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_init(&c));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());

    feeder_start();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_playback_start());
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep()); /* "audio_out" lock held */

    vTaskDelay(pdMS_TO_TICKS(500));

    audio_playback_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_TRUE(st.running);
    TEST_ASSERT_TRUE(st.primed);
    /* 500 ms x 32 B/ms = 16000 B, paced by the I2S clock, not by the feeder. */
    TEST_ASSERT_TRUE_MESSAGE(st.bytes_played > 14000 && st.bytes_played < 18000,
                             "playback rate off nominal");
    TEST_ASSERT_TRUE(st.dma_frames > 200);          /* ~250 frames of 2 ms */
    TEST_ASSERT_EQUAL(0, st.write_timeouts);
    TEST_ASSERT_EQUAL(0, st.ring_overruns);
    TEST_ASSERT_EQUAL(0, st.starvations);
    /* Only the prefill window should have been silence. */
    TEST_ASSERT_TRUE_MESSAGE(st.bytes_silence < 2000, "too much silence with a healthy feeder");
    TEST_ASSERT_TRUE(st.task_stack_free_min > 256);  /* words of headroom */

    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stop());
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stop());  /* idempotent */
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());

    /* Nothing more is consumed after stop. */
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    uint32_t at_stop = (uint32_t)st.bytes_played;
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_EQUAL_UINT32(at_stop, (uint32_t)st.bytes_played);

    /* Restart without re-init (DR1). */
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_start());
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_TRUE(st.bytes_played > at_stop);

    feeder_stop();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit());
}

TEST_CASE("an empty source plays silence, counts it, and recovers when fed", "[audio_playback]")
{
    fresh();
    audio_playback_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_start());

    vTaskDelay(pdMS_TO_TICKS(200));

    audio_playback_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_FALSE(st.primed);
    TEST_ASSERT_TRUE(st.bytes_played > 4000);
    /* Every byte sent was silence, and the DMA never stalled. */
    TEST_ASSERT_EQUAL_UINT32((uint32_t)st.bytes_played, (uint32_t)st.bytes_silence);
    TEST_ASSERT_EQUAL(0, st.write_timeouts);

    /* Now feed it: it primes and stops emitting silence. */
    feeder_start();
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats_reset());
    vTaskDelay(pdMS_TO_TICKS(200));

    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_TRUE(st.primed);
    TEST_ASSERT_TRUE(st.bytes_played > 4000);
    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)st.bytes_silence);
    TEST_ASSERT_EQUAL(0, st.starvations);

    feeder_stop();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit());
}

TEST_CASE("prefill hysteresis holds playback until the configured depth", "[audio_playback]")
{
    fresh();
    audio_playback_config_t c = base_cfg();
    c.prefill_ms = 40;                       /* 1280 B */
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_init(&c));

    audio_playback_format_t f;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_get_format(&f));
    TEST_ASSERT_EQUAL(BYTES_PER_MS * 40, f.prefill_bytes);

    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_start());

    /* One byte short of the threshold: still silent. */
    uint8_t buf[256];
    fill_pattern(buf, sizeof(buf), 0);
    for (size_t sent = 0; sent + sizeof(buf) <= f.prefill_bytes - sizeof(buf); sent += sizeof(buf)) {
        TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&s_ring, buf, sizeof(buf)));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    audio_playback_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_FALSE_MESSAGE(st.primed, "primed below the prefill threshold");
    TEST_ASSERT_EQUAL_UINT32((uint32_t)st.bytes_played, (uint32_t)st.bytes_silence);

    /* Cross it, and playback starts. */
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&s_ring, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&s_ring, buf, sizeof(buf)));
    vTaskDelay(pdMS_TO_TICKS(20));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_TRUE_MESSAGE(st.primed, "did not prime once the threshold was crossed");

    /* Draining it dry re-enters prefill rather than clicking per frame. */
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_FALSE(st.primed);
    TEST_ASSERT_EQUAL(1, st.starvations);
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit());
}

TEST_CASE("a writer that outruns playback overruns the reader and is counted", "[audio_playback]")
{
    fresh();
    /* A ring just big enough to be legal, so the overrun arrives in milliseconds. */
    static uint8_t small_storage[BYTES_PER_MS * 16];
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&s_ring, small_storage, sizeof(small_storage)));

    audio_playback_config_t c = base_cfg();
    c.prefill_ms = 4;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_start());

    /* Write ~10x nominal for 200 ms. */
    uint8_t buf[64];
    fill_pattern(buf, sizeof(buf), 0);
    TickType_t t0 = xTaskGetTickCount();
    while (xTaskGetTickCount() - t0 < pdMS_TO_TICKS(200)) {
        for (int i = 0; i < 5; i++) {
            TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&s_ring, buf, sizeof(buf)));
        }
        vTaskDelay(1);
    }

    audio_playback_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_TRUE_MESSAGE(st.ring_overruns > 0, "an outrun reader was not reported");
    /* Overrun is not a stall: the pipeline kept running at nominal rate. */
    TEST_ASSERT_TRUE(st.bytes_played > 4000);
    TEST_ASSERT_EQUAL(0, st.write_timeouts);
    TEST_ASSERT_TRUE(st.running);

    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit());
}

TEST_CASE("flush drops buffered audio and re-enters prefill", "[audio_playback]")
{
    fresh();
    audio_playback_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_init(&c));
    feeder_start();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_start());
    vTaskDelay(pdMS_TO_TICKS(150));

    audio_playback_stats_t before;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&before));
    TEST_ASSERT_TRUE(before.primed);

    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_flush());
    vTaskDelay(pdMS_TO_TICKS(5));

    audio_playback_stats_t after;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&after));
    TEST_ASSERT_EQUAL(before.prefills + 1, after.prefills);
    TEST_ASSERT_FALSE(after.primed);

    /* And it primes again from the live feeder, without a restart. */
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&after));
    TEST_ASSERT_TRUE(after.primed);

    /* flush() while stopped is legal and takes effect. */
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stop());
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_flush());

    feeder_stop();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit());
}

/* ------------------------------------------------------------------------- */
/* The pin actually carries the audio                                         */
/* ------------------------------------------------------------------------- */

TEST_CASE("played bytes reach DOUT: GPIO-matrix loopback into audio_capture", "[audio_playback]")
{
    fresh();

    /* Capture listens on the playback data pin, over the GPIO matrix, no wiring.
     *
     * Order matters, and not obviously. ESP-IDF routes an I2S output pin with
     * esp_rom_gpio_connect_out_signal() and an input pin with gpio_input_enable()
     * plus connect_in_signal(); neither disturbs the other, so initialising
     * playback (output) first and capture (input) second leaves both live.
     * gpio_set_direction(INPUT_OUTPUT) must NOT be used to "help" here: it calls
     * gpio_output_enable(), whose first act is gpio_hal_matrix_out_default() —
     * it unroutes I2S from the pin and the DOUT line goes quiet. */
    audio_playback_config_t pc = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_init(&pc));

    audio_capture_config_t cc = {
        .interface = AUDIO_CAPTURE_IF_I2S_STD,
        .sample_rate_hz = RATE_HZ,
        .channels = 1,
        .bits_per_sample = 16,
        .slot = AUDIO_CAPTURE_SLOT_LEFT,
        .pins = { .clk = PIN_CAP_CLK, .ws = PIN_CAP_WS, .din = PIN_DOUT, .mclk = -1 },
    };
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&cc));

    /* Input-path only; safe to repeat and it cannot unroute the output. */
    TEST_ASSERT_EQUAL(ESP_OK, gpio_input_enable(PIN_DOUT));

    ringbuf_reader_t r;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(audio_capture_get_ring(), &r));

    feeder_start();
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_start());
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_start());

    /* Count non-zero bytes arriving at DIN over 200 ms. A floating pin reads a
     * constant; a driven one carries the feeder's ramp. */
    size_t total = 0, nonzero = 0;
    uint8_t buf[256];
    TickType_t t0 = xTaskGetTickCount();
    while (xTaskGetTickCount() - t0 < pdMS_TO_TICKS(200)) {
        size_t got = 0;
        if (ringbuf_read(&r, buf, sizeof(buf), &got) != ESP_OK) {
            continue;
        }
        if (got == 0) {
            vTaskDelay(1);
            continue;
        }
        total += got;
        for (size_t i = 0; i < got; i++) {
            if (buf[i] != 0) {
                nonzero++;
            }
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(total > 4000, "capture did not run");
    /* The ramp is 255/256 non-zero; allow for word alignment and the prefill gap. */
    TEST_ASSERT_TRUE_MESSAGE(nonzero > total / 2, "DOUT looked silent — data is not reaching the pin");

    audio_playback_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_stats(&st));
    TEST_ASSERT_TRUE(st.primed);
    TEST_ASSERT_EQUAL(0, st.write_timeouts);

    feeder_stop();
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());
    TEST_ASSERT_EQUAL(ESP_OK, audio_playback_deinit());
}

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
