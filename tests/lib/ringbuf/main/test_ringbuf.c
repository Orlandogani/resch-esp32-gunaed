/**
 * On-target Unity tests for lib/ringbuf.
 *
 * Covers SDD §16.2: round trip, wrap-around at a non-power-of-two capacity, two
 * readers at different rates, overrun reporting and loss accounting, and a genuine
 * cross-core SMP producer/consumer run that checks for torn reads.
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ringbuf.h"

/* ------------------------------------------------------------------------- */
/* Argument validation                                                        */
/* ------------------------------------------------------------------------- */

TEST_CASE("init rejects invalid arguments", "[ringbuf]")
{
    ringbuf_t rb;
    uint8_t storage[16];

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ringbuf_init(NULL, storage, sizeof(storage)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ringbuf_init(&rb, NULL, sizeof(storage)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ringbuf_init(&rb, storage, 0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ringbuf_init(&rb, storage, RINGBUF_MAX_CAPACITY + 1));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&rb, storage, sizeof(storage)));
    TEST_ASSERT_EQUAL(sizeof(storage), ringbuf_capacity(&rb));
}

TEST_CASE("operations on an uninitialised ring fail cleanly", "[ringbuf]")
{
    ringbuf_t rb;
    ringbuf_reader_t r;
    uint8_t buf[4];
    size_t got = 99;

    memset(&rb, 0, sizeof(rb));
    TEST_ASSERT_EQUAL(0, ringbuf_capacity(&rb));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ringbuf_write(&rb, buf, 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ringbuf_reader_open(&rb, &r));

    memset(&r, 0, sizeof(r));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ringbuf_read(&r, buf, sizeof(buf), &got));
    TEST_ASSERT_EQUAL(0, got);
    TEST_ASSERT_EQUAL(0, ringbuf_available(&r));
}

TEST_CASE("write larger than capacity is rejected without side effect", "[ringbuf]")
{
    ringbuf_t rb;
    ringbuf_reader_t r;
    uint8_t storage[8];
    uint8_t big[9] = {0};

    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&rb, storage, sizeof(storage)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&rb, &r));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, ringbuf_write(&rb, big, sizeof(big)));
    TEST_ASSERT_EQUAL(0, ringbuf_available(&r));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&rb, big, 8));
    TEST_ASSERT_EQUAL(8, ringbuf_available(&r));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&rb, big, 0)); /* zero-length no-op */
    TEST_ASSERT_EQUAL(8, ringbuf_available(&r));
}

/* ------------------------------------------------------------------------- */
/* Data integrity                                                             */
/* ------------------------------------------------------------------------- */

TEST_CASE("round trip and partial reads", "[ringbuf]")
{
    ringbuf_t rb;
    ringbuf_reader_t r;
    uint8_t storage[32];
    uint8_t out[32];
    size_t got;

    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&rb, storage, sizeof(storage)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&rb, &r));

    /* Nothing yet: ESP_OK with 0 bytes, not an error. */
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&r, out, sizeof(out), &got));
    TEST_ASSERT_EQUAL(0, got);

    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&rb, "hello", 5));
    TEST_ASSERT_EQUAL(5, ringbuf_available(&r));

    /* Ask for more than available: get what exists. */
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&r, out, sizeof(out), &got));
    TEST_ASSERT_EQUAL(5, got);
    TEST_ASSERT_EQUAL_MEMORY("hello", out, 5);

    /* Ask for less than available: cursor advances by what was taken. */
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&rb, "abcdef", 6));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&r, out, 2, &got));
    TEST_ASSERT_EQUAL(2, got);
    TEST_ASSERT_EQUAL_MEMORY("ab", out, 2);
    TEST_ASSERT_EQUAL(4, ringbuf_available(&r));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&r, out, 4, &got));
    TEST_ASSERT_EQUAL(4, got);
    TEST_ASSERT_EQUAL_MEMORY("cdef", out, 4);
}

TEST_CASE("wrap-around preserves byte order at a non-power-of-two capacity", "[ringbuf]")
{
    ringbuf_t rb;
    ringbuf_reader_t r;
    uint8_t storage[7]; /* Deliberately awkward. */
    uint8_t out[7];
    size_t got;
    uint8_t seq = 0;

    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&rb, storage, sizeof(storage)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&rb, &r));

    /* Write 3 bytes at a time, read 3 at a time, for many laps of the storage. */
    uint8_t expect = 0;
    for (int i = 0; i < 200; i++) {
        uint8_t in[3] = { seq, (uint8_t)(seq + 1), (uint8_t)(seq + 2) };
        seq = (uint8_t)(seq + 3);
        TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&rb, in, 3));
        TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&r, out, 3, &got));
        TEST_ASSERT_EQUAL(3, got);
        for (int k = 0; k < 3; k++) {
            TEST_ASSERT_EQUAL_UINT8(expect, out[k]);
            expect++;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Multiple readers and overrun                                               */
/* ------------------------------------------------------------------------- */

TEST_CASE("two readers at different rates see identical data", "[ringbuf]")
{
    ringbuf_t rb;
    ringbuf_reader_t fast, slow;
    uint8_t storage[64];
    uint8_t out_fast[64], out_slow[64];
    size_t got;

    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&rb, storage, sizeof(storage)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&rb, &fast));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&rb, &slow));

    uint8_t expected[40];
    for (int i = 0; i < 40; i++) {
        expected[i] = (uint8_t)(i * 7);
    }

    /* Fast reader drains after every 4-byte write; slow reader waits for all 40. */
    size_t fast_total = 0;
    for (int i = 0; i < 10; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&rb, &expected[i * 4], 4));
        TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&fast, out_fast + fast_total, 64, &got));
        TEST_ASSERT_EQUAL(4, got);
        fast_total += got;
    }
    TEST_ASSERT_EQUAL(40, fast_total);
    TEST_ASSERT_EQUAL(0, ringbuf_available(&fast));
    TEST_ASSERT_EQUAL(40, ringbuf_available(&slow));

    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&slow, out_slow, 64, &got));
    TEST_ASSERT_EQUAL(40, got);
    TEST_ASSERT_EQUAL_MEMORY(expected, out_fast, 40);
    TEST_ASSERT_EQUAL_MEMORY(expected, out_slow, 40);

    ringbuf_reader_stats_t sf, ss;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_stats(&fast, &sf));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_stats(&slow, &ss));
    TEST_ASSERT_EQUAL(0, sf.overruns);
    TEST_ASSERT_EQUAL(0, ss.overruns);
    TEST_ASSERT_EQUAL(40, sf.bytes_read);
    TEST_ASSERT_EQUAL(40, ss.bytes_read);
}

TEST_CASE("slow reader is overrun, resynchronised, and keeps going; fast reader unaffected", "[ringbuf]")
{
    ringbuf_t rb;
    ringbuf_reader_t fast, slow;
    uint8_t storage[8];
    uint8_t out[8];
    size_t got;

    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&rb, storage, sizeof(storage)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&rb, &fast));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&rb, &slow));

    /* Write 12 bytes into an 8-byte ring, fast reader keeping up. */
    for (uint8_t i = 0; i < 12; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&rb, &i, 1));
        TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&fast, out, 1, &got));
        TEST_ASSERT_EQUAL(1, got);
        TEST_ASSERT_EQUAL_UINT8(i, out[0]);
    }

    /* Slow reader is 12 behind in an 8-byte ring: 4 bytes are gone. */
    TEST_ASSERT_EQUAL(8, ringbuf_available(&slow)); /* saturates at capacity */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ringbuf_read(&slow, out, sizeof(out), &got));
    TEST_ASSERT_EQUAL(0, got);

    ringbuf_reader_stats_t ss;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_stats(&slow, &ss));
    TEST_ASSERT_EQUAL(1, ss.overruns);
    TEST_ASSERT_EQUAL(4, ss.bytes_lost);
    TEST_ASSERT_EQUAL(8, ss.backlog);

    /* Resynchronised to the oldest surviving byte: 4..11. */
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&slow, out, sizeof(out), &got));
    TEST_ASSERT_EQUAL(8, got);
    for (int k = 0; k < 8; k++) {
        TEST_ASSERT_EQUAL_UINT8(4 + k, out[k]);
    }

    /* Fast reader never noticed. */
    ringbuf_reader_stats_t sf;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_stats(&fast, &sf));
    TEST_ASSERT_EQUAL(0, sf.overruns);
    TEST_ASSERT_EQUAL(0, sf.bytes_lost);
    TEST_ASSERT_EQUAL(12, sf.bytes_read);
}

TEST_CASE("skip_to_newest discards backlog and accounts for it", "[ringbuf]")
{
    ringbuf_t rb;
    ringbuf_reader_t r;
    uint8_t storage[16];
    uint8_t buf[16] = {0};
    size_t got;

    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&rb, storage, sizeof(storage)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&rb, &r));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_write(&rb, buf, 10));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_skip_to_newest(&r));
    TEST_ASSERT_EQUAL(0, ringbuf_available(&r));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_read(&r, buf, sizeof(buf), &got));
    TEST_ASSERT_EQUAL(0, got);

    ringbuf_reader_stats_t s;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_stats(&r, &s));
    TEST_ASSERT_EQUAL(10, s.bytes_lost);
    TEST_ASSERT_EQUAL(0, s.overruns);
}

/* ------------------------------------------------------------------------- */
/* Cross-core SMP producer/consumer                                           */
/* ------------------------------------------------------------------------- */

#define SMP_CAP        1024
#define SMP_CHUNK      37      /* Prime, so chunks straddle the wrap constantly */
#define SMP_TOTAL      (SMP_CHUNK * 5536)   /* Exact multiple of the chunk */

typedef struct {
    ringbuf_t *rb;
    volatile bool done;
    volatile esp_err_t write_err;  /* Unity asserts must not run in a non-test task */
} smp_ctx_t;

static void smp_writer_task(void *arg)
{
    smp_ctx_t *ctx = (smp_ctx_t *)arg;
    uint8_t chunk[SMP_CHUNK];
    uint32_t seq = 0;
    size_t written = 0;

    while (written < SMP_TOTAL) {
        for (int i = 0; i < SMP_CHUNK; i++) {
            chunk[i] = (uint8_t)(seq++);
        }
        esp_err_t err = ringbuf_write(ctx->rb, chunk, SMP_CHUNK);
        if (err != ESP_OK) {
            ctx->write_err = err;
            break;
        }
        written += SMP_CHUNK;
        /* Pace the writer well below the reader so no overrun is expected. */
        if ((written / SMP_CHUNK) % 8 == 0) {
            vTaskDelay(1);
        }
    }
    ctx->done = true;
    vTaskDelete(NULL);
}

TEST_CASE("cross-core writer and reader: continuous sequence, no torn reads", "[ringbuf][smp]")
{
    static uint8_t storage[SMP_CAP];
    ringbuf_t rb;
    ringbuf_reader_t r;
    smp_ctx_t ctx = { .rb = &rb, .done = false, .write_err = ESP_OK };

    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&rb, storage, sizeof(storage)));
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_reader_open(&rb, &r));

    /* Writer on the other core from this test task. */
    BaseType_t my_core = xPortGetCoreID();
    BaseType_t other = (my_core == 0) ? 1 : 0;
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreatePinnedToCore(smp_writer_task, "rb_wr", 3072, &ctx,
                                                      tskIDLE_PRIORITY + 2, NULL, other));

    uint8_t out[256];
    uint32_t expect = 0;
    uint64_t total = 0;
    uint32_t overruns = 0;

    while (!ctx.done || ringbuf_available(&r) > 0) {
        size_t got = 0;
        esp_err_t err = ringbuf_read(&r, out, sizeof(out), &got);
        if (err == ESP_ERR_INVALID_STATE) {
            /* Not expected at this pacing, but if it happens it must be *reported*,
             * not silently produce a discontinuity. Resync the expected sequence. */
            overruns++;
            ringbuf_reader_stats_t s;
            ringbuf_reader_stats(&r, &s);
            expect = (uint32_t)(s.bytes_read + s.bytes_lost);
            continue;
        }
        TEST_ASSERT_EQUAL(ESP_OK, err);
        for (size_t i = 0; i < got; i++) {
            /* Any torn or out-of-order byte fails here. */
            TEST_ASSERT_EQUAL_UINT8((uint8_t)expect, out[i]);
            expect++;
        }
        total += got;
        if (got == 0) {
            taskYIELD();
        }
    }

    TEST_ASSERT_EQUAL(ESP_OK, ctx.write_err);
    TEST_ASSERT_EQUAL(SMP_TOTAL, total);
    TEST_ASSERT_EQUAL(0, overruns);
}

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
