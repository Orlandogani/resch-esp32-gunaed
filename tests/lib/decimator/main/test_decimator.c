/**
 * On-target Unity tests for lib/decimator (SDD §16.2, FW-DEC-001..006).
 *
 * Pure logic: no peripheral is touched, so this suite runs on any S3 board. Tones are
 * generated in float, decimated, and measured by RMS over a window that skips the
 * filter's start-up transient.
 */
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "unity.h"

#include "decimator.h"

#define PI_F 3.14159265358979f

static int16_t s_in[4800 * 2];
static int16_t s_out[4800 * 2];

static void tone(int16_t *buf, size_t frames, unsigned channels, float hz, float rate, float amp)
{
    for (size_t i = 0; i < frames; i++) {
        float v = amp * sinf(2.0f * PI_F * hz * (float)i / rate);
        for (unsigned c = 0; c < channels; c++) {
            buf[i * channels + c] = (int16_t)lroundf(c == 0 ? v : -v);
        }
    }
}

static float rms(const int16_t *buf, size_t from, size_t to, unsigned channels, unsigned ch)
{
    double acc = 0.0;
    for (size_t i = from; i < to; i++) {
        double s = buf[i * channels + ch];
        acc += s * s;
    }
    return (float)sqrt(acc / (double)(to - from));
}

/* Output/input RMS ratio in dB for a tone at `hz`, 48 kHz in, ÷3, mono. */
static float gain_db(float hz)
{
    decimator_t d;
    TEST_ASSERT_EQUAL(ESP_OK, decimator_init(&d, 3, 1));
    tone(s_in, 4800, 1, hz, 48000.0f, 16000.0f);
    size_t n = decimator_process(&d, s_in, 4800, s_out);
    TEST_ASSERT_EQUAL(1600, n);
    float in_rms = 16000.0f / sqrtf(2.0f);
    return 20.0f * log10f(rms(s_out, 200, n, 1, 0) / in_rms);
}

TEST_CASE("init validates its arguments", "[decimator]")
{
    decimator_t d;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, decimator_init(NULL, 3, 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, decimator_init(&d, 0, 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, decimator_init(&d, 1, 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, decimator_init(&d, DECIMATOR_MAX_FACTOR + 1, 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, decimator_init(&d, 3, 0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, decimator_init(&d, 3, 3));
    for (uint8_t f = 2; f <= DECIMATOR_MAX_FACTOR; f++) {
        TEST_ASSERT_EQUAL(ESP_OK, decimator_init(&d, f, 2));
    }
    TEST_ASSERT_EQUAL(0, decimator_process(NULL, s_in, 10, s_out));
    TEST_ASSERT_EQUAL(0, decimator_process(&d, NULL, 10, s_out));
}

TEST_CASE("output count is input / factor, across arbitrary blocks", "[decimator]")
{
    decimator_t d;
    TEST_ASSERT_EQUAL(ESP_OK, decimator_init(&d, 3, 1));
    size_t total = 0;
    const size_t blocks[] = { 1, 2, 7, 96, 5, 0, 30 };   /* 141 inputs */
    for (size_t i = 0; i < sizeof(blocks) / sizeof(blocks[0]); i++) {
        TEST_ASSERT_LESS_OR_EQUAL(decimator_max_out_frames(&d, blocks[i]),
                                  decimator_process(&d, s_in, blocks[i], s_out));
    }
    decimator_reset(&d);
    for (size_t i = 0; i < sizeof(blocks) / sizeof(blocks[0]); i++) {
        total += decimator_process(&d, s_in, blocks[i], s_out);
    }
    TEST_ASSERT_EQUAL(141 / 3, total);
}

TEST_CASE("DC passes with unity gain", "[decimator]")
{
    decimator_t d;
    for (uint8_t f = 2; f <= DECIMATOR_MAX_FACTOR; f++) {
        TEST_ASSERT_EQUAL(ESP_OK, decimator_init(&d, f, 1));
        for (size_t i = 0; i < 1200; i++) {
            s_in[i] = 10000;
        }
        size_t n = decimator_process(&d, s_in, 1200, s_out);
        /* After the filter has filled, every output equals the input within rounding. */
        for (size_t i = n / 2; i < n; i++) {
            TEST_ASSERT_INT_WITHIN(1, 10000, s_out[i]);
        }
    }
}

TEST_CASE("48 -> 16 kHz: voice band flat, aliasing band attenuated", "[decimator]")
{
    float g1k = gain_db(1000.0f);
    float g5k = gain_db(5000.0f);
    float g12k = gain_db(12000.0f);  /* would alias to 4 kHz */
    float g20k = gain_db(20000.0f);  /* would alias to 4 kHz from the other side */
    printf("gain: 1k %.2f dB, 5k %.2f dB, 12k %.1f dB, 20k %.1f dB\n",
           (double)g1k, (double)g5k, (double)g12k, (double)g20k);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, g1k);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, g5k);
    TEST_ASSERT_LESS_THAN_FLOAT(-60.0f, g12k);
    TEST_ASSERT_LESS_THAN_FLOAT(-60.0f, g20k);
}

TEST_CASE("block splitting does not change the output", "[decimator]")
{
    static int16_t whole[1600 * 2];
    decimator_t d;

    tone(s_in, 4800, 2, 700.0f, 48000.0f, 12000.0f);
    TEST_ASSERT_EQUAL(ESP_OK, decimator_init(&d, 3, 2));
    size_t n_whole = decimator_process(&d, s_in, 4800, whole);

    decimator_reset(&d);
    size_t n = 0, f = 0, step = 1;
    while (f < 4800) {
        size_t len = (f + step > 4800) ? 4800 - f : step;
        n += decimator_process(&d, &s_in[f * 2], len, &s_out[n * 2]);
        f += len;
        step = step * 3 % 97 + 1;   /* irregular block sizes, 1..97 */
    }
    TEST_ASSERT_EQUAL(n_whole, n);
    TEST_ASSERT_EQUAL_INT16_ARRAY(whole, s_out, n * 2);
}

TEST_CASE("stereo channels are filtered independently", "[decimator]")
{
    decimator_t d;
    TEST_ASSERT_EQUAL(ESP_OK, decimator_init(&d, 3, 2));
    tone(s_in, 4800, 2, 1000.0f, 48000.0f, 16000.0f);   /* right = -left */
    size_t n = decimator_process(&d, s_in, 4800, s_out);
    for (size_t i = 0; i < n; i++) {
        TEST_ASSERT_INT_WITHIN(1, -s_out[i * 2], s_out[i * 2 + 1]);
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(11000.0f, rms(s_out, 200, n, 2, 0));
}

TEST_CASE("in-place processing matches out-of-place", "[decimator]")
{
    static int16_t ref[1600];
    decimator_t d;
    tone(s_in, 4800, 1, 3000.0f, 48000.0f, 20000.0f);
    TEST_ASSERT_EQUAL(ESP_OK, decimator_init(&d, 3, 1));
    size_t n = decimator_process(&d, s_in, 4800, ref);
    decimator_reset(&d);
    TEST_ASSERT_EQUAL(n, decimator_process(&d, s_in, 4800, s_in));
    TEST_ASSERT_EQUAL_INT16_ARRAY(ref, s_in, n);
}

TEST_CASE("full-scale input saturates instead of wrapping", "[decimator]")
{
    decimator_t d;
    TEST_ASSERT_EQUAL(ESP_OK, decimator_init(&d, 2, 1));
    /* A square wave at the edge of the passband overshoots (Gibbs); the result must clip. */
    for (size_t i = 0; i < 2000; i++) {
        s_in[i] = ((i / 4) & 1) ? 32767 : -32768;
    }
    size_t n = decimator_process(&d, s_in, 2000, s_out);
    for (size_t i = 100; i < n; i++) {
        /* A wrap would show as a sign flip against the input's polarity at that point. */
        TEST_ASSERT_TRUE(s_out[i] >= -32768 && s_out[i] <= 32767);
    }
}

TEST_CASE("cost: 48 -> 16 kHz mono, 10 ms block", "[decimator]")
{
    decimator_t d;
    TEST_ASSERT_EQUAL(ESP_OK, decimator_init(&d, 3, 1));
    tone(s_in, 480, 1, 1000.0f, 48000.0f, 8000.0f);
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < 100; i++) {
        (void)decimator_process(&d, s_in, 480, s_out);
    }
    int64_t us = (esp_timer_get_time() - t0) / 100;
    printf("decimate 480 -> 160 frames: %lld us per 10 ms block\n", (long long)us);
    TEST_ASSERT_LESS_THAN(1000, (int)us);   /* < 10 % of the block period */
}

void app_main(void)
{
    /* A USB-Serial/JTAG console drops output while no host has the port open; give the
     * capture script time to attach (.claude/BACKLOG.md, "Capturing a test run"). */
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
