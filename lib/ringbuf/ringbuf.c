#include "ringbuf.h"
#include <string.h>

/* Distinguishes an initialised control block from zeroed or stale memory. */
#define RINGBUF_MAGIC 0x52494E47u /* "RING" */

static inline bool rb_valid(const ringbuf_t *rb)
{
    return rb != NULL && rb->magic == RINGBUF_MAGIC;
}

/* Copy `len` bytes starting at logical position `pos`, handling the wrap. */
static void rb_copy_out(const ringbuf_t *rb, uint32_t pos, uint8_t *dst, uint32_t len)
{
    uint32_t off = pos % rb->capacity;
    uint32_t first = rb->capacity - off;
    if (first >= len) {
        memcpy(dst, rb->storage + off, len);
    } else {
        memcpy(dst, rb->storage + off, first);
        memcpy(dst + first, rb->storage, len - first);
    }
}

static void rb_copy_in(ringbuf_t *rb, uint32_t pos, const uint8_t *src, uint32_t len)
{
    uint32_t off = pos % rb->capacity;
    uint32_t first = rb->capacity - off;
    if (first >= len) {
        memcpy(rb->storage + off, src, len);
    } else {
        memcpy(rb->storage + off, src, first);
        memcpy(rb->storage, src + first, len - first);
    }
}

esp_err_t ringbuf_init(ringbuf_t *rb, void *storage, size_t capacity)
{
    if (rb == NULL || storage == NULL || capacity == 0 || capacity > RINGBUF_MAX_CAPACITY) {
        return ESP_ERR_INVALID_ARG;
    }
    rb->storage = (uint8_t *)storage;
    rb->capacity = (uint32_t)capacity;
    rb->bytes_written = 0;
    atomic_store_explicit(&rb->write_pos, 0u, memory_order_relaxed);
    rb->magic = RINGBUF_MAGIC;
    return ESP_OK;
}

size_t ringbuf_capacity(const ringbuf_t *rb)
{
    return rb_valid(rb) ? rb->capacity : 0;
}

esp_err_t ringbuf_write(ringbuf_t *rb, const void *data, size_t len)
{
    if (!rb_valid(rb) || (data == NULL && len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > rb->capacity) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (len == 0) {
        return ESP_OK;
    }

    /* Single writer: a relaxed load of our own last store is sufficient. */
    uint32_t w = atomic_load_explicit(&rb->write_pos, memory_order_relaxed);
    rb_copy_in(rb, w, (const uint8_t *)data, (uint32_t)len);
    rb->bytes_written += len;

    /* Release: readers that acquire the new position see the bytes above. */
    atomic_store_explicit(&rb->write_pos, w + (uint32_t)len, memory_order_release);
    return ESP_OK;
}

esp_err_t ringbuf_reader_open(ringbuf_t *rb, ringbuf_reader_t *r)
{
    if (!rb_valid(rb) || r == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    r->rb = rb;
    r->pos = atomic_load_explicit(&rb->write_pos, memory_order_acquire);
    r->overruns = 0;
    r->bytes_read = 0;
    r->bytes_lost = 0;
    return ESP_OK;
}

size_t ringbuf_available(const ringbuf_reader_t *r)
{
    if (r == NULL || !rb_valid(r->rb)) {
        return 0;
    }
    uint32_t w = atomic_load_explicit(&r->rb->write_pos, memory_order_acquire);
    uint32_t dist = w - r->pos; /* Unsigned wrap arithmetic is the intent. */
    return dist > r->rb->capacity ? r->rb->capacity : dist;
}

/* Record an overrun and move the cursor to the oldest byte still guaranteed intact. */
static void rb_resync(ringbuf_reader_t *r, uint32_t w)
{
    uint32_t dist = w - r->pos;
    r->bytes_lost += (dist - r->rb->capacity);
    r->overruns++;
    r->pos = w - r->rb->capacity;
}

esp_err_t ringbuf_read(ringbuf_reader_t *r, void *dst, size_t len, size_t *out_len)
{
    if (out_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_len = 0;
    if (r == NULL || !rb_valid(r->rb) || (dst == NULL && len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    ringbuf_t *rb = r->rb;
    const uint32_t cap = rb->capacity;

    /* Acquire: everything the writer published before this position is visible. */
    uint32_t w = atomic_load_explicit(&rb->write_pos, memory_order_acquire);
    uint32_t dist = w - r->pos;

    if (dist > cap) {
        /* Overrun detected before copying anything. */
        rb_resync(r, w);
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t n = dist < len ? dist : (uint32_t)len;
    if (n == 0) {
        return ESP_OK;
    }

    uint32_t start = r->pos;
    rb_copy_out(rb, start, (uint8_t *)dst, n);

    /*
     * Validate after the copy: if the writer lapped `start` while we were copying,
     * some of what we copied was overwritten under us and is torn. Treat as overrun.
     * The acquire here orders the writer's data stores before our reload.
     */
    uint32_t w2 = atomic_load_explicit(&rb->write_pos, memory_order_acquire);
    if ((uint32_t)(w2 - start) > cap) {
        rb_resync(r, w2);
        return ESP_ERR_INVALID_STATE;
    }

    r->pos = start + n;
    r->bytes_read += n;
    *out_len = n;
    return ESP_OK;
}

esp_err_t ringbuf_reader_skip_to_newest(ringbuf_reader_t *r)
{
    if (r == NULL || !rb_valid(r->rb)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t w = atomic_load_explicit(&r->rb->write_pos, memory_order_acquire);
    uint32_t dist = w - r->pos;
    if (dist > r->rb->capacity) {
        /* Skipping past an overrun still counts as one: the reader lost data. */
        r->bytes_lost += dist - r->rb->capacity;
        r->overruns++;
        dist = r->rb->capacity;
    }
    r->bytes_lost += dist;
    r->pos = w;
    return ESP_OK;
}

esp_err_t ringbuf_stats(const ringbuf_t *rb, ringbuf_stats_t *out)
{
    if (!rb_valid(rb) || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    out->capacity = rb->capacity;
    out->bytes_written = rb->bytes_written;
    return ESP_OK;
}

esp_err_t ringbuf_reader_stats(const ringbuf_reader_t *r, ringbuf_reader_stats_t *out)
{
    if (r == NULL || !rb_valid(r->rb) || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    out->overruns = r->overruns;
    out->bytes_read = r->bytes_read;
    out->bytes_lost = r->bytes_lost;
    out->backlog = (uint32_t)ringbuf_available(r);
    return ESP_OK;
}
