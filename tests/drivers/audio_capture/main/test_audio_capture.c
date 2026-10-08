/**
 * On-target Unity tests for drivers/audio_capture.
 *
 * Without a microphone the I2S peripheral still clocks and the DMA still runs —
 * the data is whatever the floating DIN pin reads. That is enough to verify the
 * driver's lifecycle, DMA cadence, ring publication rate, overrun accounting and
 * the pm_policy lock, which is what these tests do. Content correctness needs a
 * real microphone and is a bench item (TBD-002).
 *
 * Pins default to GPIO 4/5/6, which are free on most ESP32-S3 dev boards. Change
 * them with CONFIG_TEST_AUDIO_CAPTURE_PIN_* in this project's sdkconfig if not.
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cfg.h"
#include "diag.h"
#include "pm_policy.h"
#include "power.h"
#include "audio_capture.h"

#ifndef TEST_PIN_CLK
#define TEST_PIN_CLK 4
#endif
#ifndef TEST_PIN_WS
#define TEST_PIN_WS 5
#endif
#ifndef TEST_PIN_DIN
#define TEST_PIN_DIN 6
#endif

static audio_capture_config_t base_cfg(void)
{
    audio_capture_config_t c = {
        .interface = AUDIO_CAPTURE_IF_I2S_STD,
        .sample_rate_hz = 16000,
        .channels = 1,
        .bits_per_sample = 16,
        .slot = AUDIO_CAPTURE_SLOT_LEFT,
        .pins = { .clk = TEST_PIN_CLK, .ws = TEST_PIN_WS, .din = TEST_PIN_DIN, .mclk = -1 },
    };
    return c;
}

static void fresh(void)
{
    audio_capture_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    TEST_ASSERT_EQUAL(ESP_OK, diag_init(NULL));
}

/* ------------------------------------------------------------------------- */
/* Lifecycle and validation                                                   */
/* ------------------------------------------------------------------------- */

TEST_CASE("before init: queries fail cleanly", "[audio_capture]")
{
    audio_capture_deinit();
    audio_capture_format_t f;
    TEST_ASSERT_NULL(audio_capture_get_ring());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_capture_get_format(&f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_capture_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_capture_stop());
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit()); /* idempotent */
}

TEST_CASE("init rejects unsupported configurations without side effects", "[audio_capture]")
{
    fresh();
    audio_capture_config_t c;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(NULL));
    c = base_cfg(); c.sample_rate_hz = 96000;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.channels = 3;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.bits_per_sample = 24;          /* I2S std: 16 or 32 only */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.interface = AUDIO_CAPTURE_IF_PDM; c.bits_per_sample = 32;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.pins.din = -1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.pins.ws = -1;                  /* required for I2S std */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.dma_frame_count = 1;           /* below double-buffering */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));

    /* Nothing was allocated or created. */
    TEST_ASSERT_NULL(audio_capture_get_ring());
}

TEST_CASE("init/deinit cycle is clean and repeatable", "[audio_capture]")
{
    fresh();
    audio_capture_config_t c = base_cfg();
    size_t heap_before = esp_get_free_heap_size();
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&c));
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_capture_init(&c)); /* singleton */
        TEST_ASSERT_NOT_NULL(audio_capture_get_ring());
        TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());
    }
    /* Allow for allocator fragmentation but not a leak per cycle. */
    size_t heap_after = esp_get_free_heap_size();
    TEST_ASSERT_TRUE_MESSAGE(heap_before - heap_after < 512, "heap leak across init/deinit cycles");
}

TEST_CASE("format and ring sizing follow the configuration", "[audio_capture]")
{
    fresh();
    audio_capture_config_t c = base_cfg();
    c.dma_frame_ms = 2;
    c.dma_frame_count = 4;
    c.ring_ms = 40;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&c));

    audio_capture_format_t f;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_get_format(&f));
    TEST_ASSERT_EQUAL(16000, f.sample_rate_hz);
    TEST_ASSERT_EQUAL(1, f.channels);
    TEST_ASSERT_EQUAL(2, f.bytes_per_sample);
    TEST_ASSERT_EQUAL(32, f.bytes_per_ms);
    TEST_ASSERT_EQUAL(32 * 40, ringbuf_capacity(audio_capture_get_ring()));
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());

    /* 32-bit slots are stored in 4 bytes. */
    c.bits_per_sample = 32;
    c.channels = 2;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_get_format(&f));
    TEST_ASSERT_EQUAL(4, f.bytes_per_sample);
    TEST_ASSERT_EQUAL(128, f.bytes_per_ms);
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());
}

/* ------------------------------------------------------------------------- */
/* Running                                                                    */
/* ------------------------------------------------------------------------- */

TEST_CASE("start publishes into the ring at the configured rate; stop halts it", "[audio_capture]")
{
    fresh();
    audio_capture_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&c));

    ringbuf_reader_t r;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(audio_capture_get_ring(), &r));
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());

    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_capture_start());
    TEST_ASSERT_FALSE(pm_policy_can_deep_sleep()); /* "audio" lock held */

    /* Drain for 500 ms and count bytes; expect 32 B/ms within tolerance. */
    uint8_t buf[512];
    size_t total = 0;
    uint32_t overruns = 0;
    TickType_t t0 = xTaskGetTickCount();
    while (xTaskGetTickCount() - t0 < pdMS_TO_TICKS(500)) {
        size_t got = 0;
        esp_err_t err = ringbuf_read(&r, buf, sizeof(buf), &got);
        if (err == ESP_ERR_INVALID_STATE) {
            overruns++;
            continue;
        }
        TEST_ASSERT_EQUAL(ESP_OK, err);
        total += got;
        if (got == 0) {
            vTaskDelay(1);
        }
    }
    /* 500 ms x 32 B/ms = 16000 B; allow the first DMA frames to spin up. */
    TEST_ASSERT_TRUE_MESSAGE(total > 14000 && total < 18000, "capture rate off nominal");
    TEST_ASSERT_EQUAL(0, overruns);

    audio_capture_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_stats(&st));
    TEST_ASSERT_TRUE(st.running);
    TEST_ASSERT_TRUE(st.dma_frames > 200);          /* ~250 frames of 2 ms */
    TEST_ASSERT_EQUAL(0, st.dma_overruns);
    TEST_ASSERT_EQUAL(0, st.ring_write_failures);
    TEST_ASSERT_TRUE(st.task_stack_free_min > 256);  /* words of headroom */

    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_stop());
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_stop());  /* idempotent */
    TEST_ASSERT_TRUE(pm_policy_can_deep_sleep());

    /* Nothing more arrives after stop. */
    ringbuf_reader_skip_to_newest(&r);
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL(0, ringbuf_available(&r));

    /* Restart without re-init (DR1). */
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_start());
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(ringbuf_available(&r) > 0);
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());
}

TEST_CASE("a reader that stops draining is overrun and told so; capture continues", "[audio_capture]")
{
    fresh();
    audio_capture_config_t c = base_cfg();
    c.ring_ms = 20; /* small ring so the overrun arrives quickly */
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&c));
    ringbuf_reader_t r;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(audio_capture_get_ring(), &r));
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_start());

    vTaskDelay(pdMS_TO_TICKS(100)); /* 5 ring depths go by unread */

    uint8_t buf[64];
    size_t got = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ringbuf_read(&r, buf, sizeof(buf), &got));
    ringbuf_reader_stats_t rs;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_stats(&r, &rs));
    TEST_ASSERT_EQUAL(1, rs.overruns);
    TEST_ASSERT_TRUE(rs.bytes_lost > 0);

    /* After resync, reads succeed and the driver itself never overran DMA. */
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&r, buf, sizeof(buf), &got));
    TEST_ASSERT_TRUE(got > 0);
    audio_capture_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_stats(&st));
    TEST_ASSERT_EQUAL(0, st.dma_overruns);
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());
}

TEST_CASE("PDM mode initialises and runs", "[audio_capture]")
{
    fresh();
    audio_capture_config_t c = base_cfg();
    c.interface = AUDIO_CAPTURE_IF_PDM;
    c.pins.ws = -1; /* not used by PDM */
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&c));
    ringbuf_reader_t r;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(audio_capture_get_ring(), &r));
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_start());
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_TRUE(ringbuf_available(&r) > 0);
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());
}

/* ------------------------------------------------------------------------- */
/* Bus rate, PDM clock band, input switching (FW-AUD-070..075)               */
/* ------------------------------------------------------------------------- */

TEST_CASE("new options are validated against the interface", "[audio_capture]")
{
    fresh();
    audio_capture_config_t c;

    c = base_cfg(); c.pdm_oversample = 128;            /* PDM-only option on std */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.interface = AUDIO_CAPTURE_IF_PDM; c.bus_slave = true;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.interface = AUDIO_CAPTURE_IF_PDM; c.pdm_oversample = 96;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.decimation = 7;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.decimation = 3; c.bits_per_sample = 32;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.sample_rate_hz = 48000; c.decimation = 3;   /* 144 kHz bus */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    c = base_cfg(); c.port = (audio_capture_port_t)3;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_init(&c));
    TEST_ASSERT_NULL(audio_capture_get_ring());
}

TEST_CASE("PDM at 16 kHz with 128x oversampling runs at the ring rate", "[audio_capture]")
{
    fresh();
    audio_capture_config_t c = base_cfg();
    c.interface = AUDIO_CAPTURE_IF_PDM;
    c.pins.ws = -1;
    c.port = AUDIO_CAPTURE_PORT_I2S0;     /* PDM RX lives on I2S0 on the S3 */
    c.pdm_oversample = 128;               /* 2.048 MHz: inside the IM73D122 bands */
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_start());
    vTaskDelay(pdMS_TO_TICKS(500));
    audio_capture_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_stats(&st));
    /* 32 B/ms for 500 ms, within a DMA frame or two of scheduling slack. */
    TEST_ASSERT_UINT64_WITHIN(1600, 16000, st.bytes_captured);
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());
}

TEST_CASE("decimation: 48 kHz bus delivers 16 kHz into the ring", "[audio_capture]")
{
    fresh();
    audio_capture_config_t c = base_cfg();
    c.decimation = 3;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&c));
    audio_capture_format_t f;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_get_format(&f));
    TEST_ASSERT_EQUAL(16000, f.sample_rate_hz);
    TEST_ASSERT_EQUAL(32, f.bytes_per_ms);
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_start());
    vTaskDelay(pdMS_TO_TICKS(500));
    audio_capture_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_stats(&st));
    TEST_ASSERT_UINT64_WITHIN(1600, 16000, st.bytes_captured);
    TEST_ASSERT_EQUAL(0, st.ring_write_failures);
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());
}

TEST_CASE("switch_input keeps the ring and its readers across interfaces", "[audio_capture]")
{
    fresh();
    audio_capture_config_t std_cfg = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_init(&std_cfg));
    ringbuf_t *ring = audio_capture_get_ring();
    ringbuf_reader_t r;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(ring, &r));
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_start());
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Format changes are refused and leave the input running. */
    audio_capture_config_t bad = base_cfg();
    bad.sample_rate_hz = 48000;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, audio_capture_switch_input(&bad));

    /* std -> PDM (another controller) -> std with a 3x bus, all while running. */
    audio_capture_config_t pdm = base_cfg();
    pdm.interface = AUDIO_CAPTURE_IF_PDM;
    pdm.pins.ws = -1;
    pdm.pdm_oversample = 128;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_switch_input(&pdm));
    TEST_ASSERT_EQUAL_PTR(ring, audio_capture_get_ring());
    vTaskDelay(pdMS_TO_TICKS(50));

    audio_capture_config_t fast = base_cfg();
    fast.decimation = 3;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_switch_input(&fast));
    TEST_ASSERT_EQUAL_PTR(ring, audio_capture_get_ring());

    size_t before = ringbuf_available(&r);
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_TRUE_MESSAGE(ringbuf_available(&r) > before, "capture stalled after switching");

    audio_capture_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_stats(&st));
    TEST_ASSERT_EQUAL(2, st.input_switches);
    TEST_ASSERT_TRUE(st.running);

    /* Switching while stopped leaves it stopped. */
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_stop());
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_switch_input(&std_cfg));
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_stats(&st));
    TEST_ASSERT_FALSE(st.running);
    TEST_ASSERT_EQUAL(ESP_OK, audio_capture_deinit());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, audio_capture_switch_input(&std_cfg));
}

void app_main(void)
{
    /* Let a USB-Serial/JTAG capture attach first (.claude/BACKLOG.md). */
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
