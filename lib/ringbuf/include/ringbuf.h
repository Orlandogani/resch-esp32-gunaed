/**
 * @file ringbuf.h
 * @brief Single-writer, multi-reader byte ring buffer with independent read cursors.
 *
 * Pure logic: no hardware, no RTOS, no heap. The only ESP-IDF header used is
 * `esp_err.h`, which is header-only for the codes returned here and is shimmed in
 * host builds. Realises FW-AUD-004..006 and ADR-014.
 *
 * ## Model
 *
 * One writer appends bytes and advances a monotonic write position. Any number of
 * readers each hold their own cursor into the same storage. The writer never waits
 * for a reader: a reader that falls more than `capacity` bytes behind is **overrun**,
 * is told so on its next read (`ESP_ERR_INVALID_STATE`), has its cursor resynchronised
 * to the oldest surviving byte, and can carry on. Readers never affect each other.
 *
 * ## Concurrency
 *
 * - Exactly **one** writer. Concurrent `ringbuf_write()` calls are a programming error.
 * - Any number of readers, each owning its `ringbuf_reader_t`, may run concurrently
 *   with the writer and with each other, on any core.
 * - Lock-free: the write position is a C11 atomic with release/acquire ordering.
 * - ISR-safe: every function is callable from interrupt context. None blocks.
 *
 * ## Storage
 *
 * The caller provides both the control block and the byte storage; the module
 * allocates nothing (ADR-005). The struct fields are visible only so callers can
 * allocate them statically — they are private and must not be accessed directly.
 * This is the deliberate `lib/` exception to ADR-004's opaque-handle rule: a pure
 * logic module compiled with its consumer has no ABI boundary to protect.
 *
 * ## Limits
 *
 * Positions are 32-bit with wrap-around arithmetic. `capacity` must be
 * <= RINGBUF_MAX_CAPACITY, and a reader that is more than 2^31 bytes stale cannot
 * have its loss counted exactly (it is still reported as overrun). At audio rates
 * this means a reader idle for hours — a broken reader by any definition.
 */
#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Largest permitted capacity, keeping wrap arithmetic unambiguous. */
#define RINGBUF_MAX_CAPACITY (1u << 30)

/**
 * @brief Ring buffer control block. Caller-allocated. Fields are private.
 */
typedef struct ringbuf {
    uint8_t         *storage;        /**< Private. Caller-provided byte storage.        */
    uint32_t         capacity;       /**< Private. Bytes in `storage`.                  */
    _Atomic uint32_t write_pos;      /**< Private. Monotonic; wraps at 2^32.            */
    uint64_t         bytes_written;  /**< Private. Stats only; writer-owned.            */
    uint32_t         magic;          /**< Private. Guards use before init.              */
} ringbuf_t;

/**
 * @brief One reader's cursor. Caller-allocated, one per consumer. Fields are private.
 */
typedef struct ringbuf_reader {
    ringbuf_t *rb;                   /**< Private. Ring this cursor belongs to.         */
    uint32_t   pos;                  /**< Private. Next byte to read.                   */
    uint32_t   overruns;             /**< Private. Times this reader was overrun.       */
    uint64_t   bytes_read;           /**< Private. Stats.                               */
    uint64_t   bytes_lost;           /**< Private. Bytes skipped due to overrun.        */
} ringbuf_reader_t;

/** Writer-side statistics snapshot. */
typedef struct {
    uint32_t capacity;        /**< Bytes of storage.                                     */
    uint64_t bytes_written;   /**< Total bytes ever written.                             */
} ringbuf_stats_t;

/** Reader-side statistics snapshot. */
typedef struct {
    uint32_t overruns;        /**< Times `ringbuf_read()` reported ESP_ERR_INVALID_STATE. */
    uint64_t bytes_read;      /**< Total bytes delivered to this reader.                 */
    uint64_t bytes_lost;      /**< Total bytes this reader never saw.                    */
    uint32_t backlog;         /**< Bytes currently readable (<= capacity).               */
} ringbuf_reader_stats_t;

/**
 * @brief Initialise a ring over caller-provided storage.
 *
 * @param[out] rb        Control block to initialise. Any previous state is discarded;
 *                       readers opened on it become invalid.
 * @param[in]  storage   Byte storage of at least `capacity` bytes. Must outlive `rb`.
 *                       For DMA-fed rings, allocate it DMA-capable and internal (DES-ACAP-002).
 * @param[in]  capacity  Bytes in `storage`. 1..RINGBUF_MAX_CAPACITY.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_ARG   if `rb` or `storage` is NULL, or `capacity` is out of range
 *
 * @note Thread-safety: not safe concurrently with any other call on the same `rb`.
 * @note ISR-safety: safe.
 * @note Blocking: never.
 */
esp_err_t ringbuf_init(ringbuf_t *rb, void *storage, size_t capacity);

/**
 * @brief Capacity in bytes, or 0 if `rb` is NULL or uninitialised.
 *
 * @note Thread-safety: safe. ISR-safety: safe. Blocking: never.
 */
size_t ringbuf_capacity(const ringbuf_t *rb);

/**
 * @brief Append bytes. Never blocks, never fails for lack of space — it overwrites
 *        the oldest data, and lagging readers discover that on their next read.
 *
 * @param[in] rb    Initialised ring.
 * @param[in] data  Bytes to copy in.
 * @param[in] len   Byte count. 0 is permitted and is a no-op.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_ARG    if `rb` is NULL/uninitialised, or `data` is NULL with `len` > 0
 *   - ESP_ERR_INVALID_SIZE   if `len` > capacity (a single write that could never be read whole)
 *
 * @note Thread-safety: **single writer only**. Safe concurrently with readers.
 * @note ISR-safety: safe.
 * @note Blocking: never.
 */
esp_err_t ringbuf_write(ringbuf_t *rb, const void *data, size_t len);

/**
 * @brief Open a reader positioned at the current write position (newest data).
 *
 * Opening at "now" rather than at the oldest byte is deliberate: a consumer that
 * starts late must not begin by replaying stale audio (FW-AUD-026, DES-AUD-005).
 *
 * @param[in]  rb   Initialised ring.
 * @param[out] r    Cursor to initialise. Caller-owned; must outlive its use.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_ARG   if either pointer is NULL or `rb` is uninitialised
 *
 * @note Thread-safety: safe. ISR-safety: safe. Blocking: never.
 */
esp_err_t ringbuf_reader_open(ringbuf_t *rb, ringbuf_reader_t *r);

/**
 * @brief Bytes this reader could read right now, saturating at capacity.
 *
 * A value equal to capacity means the reader is at or past the overrun threshold;
 * the next `ringbuf_read()` will report it.
 *
 * @note Thread-safety: safe. ISR-safety: safe. Blocking: never.
 */
size_t ringbuf_available(const ringbuf_reader_t *r);

/**
 * @brief Copy up to `len` bytes out at this reader's cursor and advance it.
 *
 * Returns fewer bytes than requested when less is available; that is not an error.
 * Returns zero bytes and ESP_ERR_INVALID_STATE when the reader was overrun — the
 * cursor has already been resynchronised to the oldest surviving byte and the loss
 * is recorded, so the caller simply calls again to resume.
 *
 * @param[in]  r        Open reader.
 * @param[out] dst      Destination buffer of at least `len` bytes.
 * @param[in]  len      Maximum bytes to copy.
 * @param[out] out_len  Bytes actually copied. Always written (0 on error).
 *
 * @return
 *   - ESP_OK                  `*out_len` bytes copied (possibly 0 if nothing available)
 *   - ESP_ERR_INVALID_STATE   overrun: reader resynchronised, nothing copied
 *   - ESP_ERR_INVALID_ARG     NULL `r`/`out_len`, or NULL `dst` with `len` > 0
 *
 * @note Thread-safety: one caller per reader; safe concurrently with the writer and
 *       other readers.
 * @note ISR-safety: safe.
 * @note Blocking: never.
 */
esp_err_t ringbuf_read(ringbuf_reader_t *r, void *dst, size_t len, size_t *out_len);

/**
 * @brief Discard everything readable and move the cursor to the write position.
 *
 * @note Thread-safety: one caller per reader. ISR-safety: safe. Blocking: never.
 */
esp_err_t ringbuf_reader_skip_to_newest(ringbuf_reader_t *r);

/**
 * @brief Writer-side statistics snapshot. `bytes_written` may be torn if read
 *        concurrently with a write on a 32-bit core; it is for diagnostics only.
 *
 * @note Thread-safety: safe. ISR-safety: safe. Blocking: never.
 */
esp_err_t ringbuf_stats(const ringbuf_t *rb, ringbuf_stats_t *out);

/**
 * @brief Reader-side statistics snapshot.
 *
 * @note Thread-safety: safe if no concurrent `ringbuf_read()` on the same reader.
 *       ISR-safety: safe. Blocking: never.
 */
esp_err_t ringbuf_reader_stats(const ringbuf_reader_t *r, ringbuf_reader_stats_t *out);

#ifdef __cplusplus
}
#endif
