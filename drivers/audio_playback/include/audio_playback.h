/**
 * @file audio_playback.h
 * @brief I2S speaker playback: standard I2S output, DMA double-buffering, and
 *        consumption of PCM from a caller-owned `ringbuf` at the sample clock's cadence.
 *
 * Realises FW-AUD-030..043 and the driver contract of SDD §15.3 (DR1-DR10). The peer
 * of `drivers/audio_capture`: capture *owns* a ring and writes it, playback *reads* a
 * ring the caller owns. That asymmetry is deliberate — the producer of playback audio
 * is application policy (a USB OUT stream, a wireless profile, a local tone), and the
 * driver must not care which (ADR-013, ADR-014).
 *
 * ## Lifecycle (DR1)
 *
 *     audio_playback_init(cfg) -> audio_playback_start() -> audio_playback_stop() -> audio_playback_deinit()
 *                                          ^                       |
 *                                          +----- repeatable ------+
 *
 * `start()`/`stop()` enable and disable the channel without reallocating (DR2).
 *
 * ## Data path
 *
 *     ring -> playback task -> i2s_channel_write -> driver DMA buffers -> GDMA -> I2S -> DAC
 *
 * The task is paced by the DMA: `i2s_channel_write()` blocks until a descriptor frees,
 * which the I2S "sent" interrupt signals. This module's own ISR callbacks only count
 * (FW-AUD-036, DR5).
 *
 * ## Prefill and underrun
 *
 * After `start()` — and after any complete starvation — the driver emits silence until
 * `prefill_ms` of audio has accumulated in the ring. That hysteresis is what stops a
 * source running at exactly nominal rate from oscillating between "one frame ready" and
 * "nothing ready" and clicking once per frame. A partially filled frame is padded with
 * silence and counted; playback never stalls and never repeats stale audio
 * (FW-AUD-037, FW-AUD-038).
 *
 * ## Sample format expected in the ring
 *
 * Little-endian PCM, `channels` interleaved, `bits_per_sample` wide, exactly as the I2S
 * driver wants it (16 -> 2 bytes; 32 -> 4 bytes per sample per channel). Use
 * `audio_playback_get_format()` rather than assuming; it is the contract the ring's
 * writer must meet. Writing a different format is not detectable by the driver and
 * will simply sound wrong.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit/start/stop` are not safe concurrently with each other. `get_format()`,
 * `stats()` and `flush()` are safe from any task. Nothing is ISR-safe.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Which I2S slot a mono stream is placed in (mirrors `audio_capture_slot_t`). */
typedef enum {
    AUDIO_PLAYBACK_SLOT_LEFT = 0,
    AUDIO_PLAYBACK_SLOT_RIGHT,
    AUDIO_PLAYBACK_SLOT_BOTH,     /**< Mono duplicated into both slots.                   */
} audio_playback_slot_t;

typedef struct {
    ringbuf_t *source;            /**< PCM source. Caller-owned, must outlive the driver. */
    uint32_t sample_rate_hz;      /**< 8000..48000.                                       */
    uint8_t  channels;            /**< 1 or 2.                                            */
    uint8_t  bits_per_sample;     /**< 16 or 32.                                          */
    audio_playback_slot_t slot;   /**< Mono only: which slot(s) carry the stream.         */
    int8_t   port;                /**< I2S controller. -1 selects any free one.           */
    uint8_t  dma_frame_ms;        /**< Per DMA descriptor. 0 selects Kconfig (2 ms).      */
    uint8_t  dma_frame_count;     /**< Descriptors, >= 2. 0 selects Kconfig (4).          */
    uint16_t prefill_ms;          /**< Audio to accumulate before playing. 0 selects
                                       Kconfig (10 ms).                                   */
    struct {
        int bclk;                 /**< I2S BCLK. Required.                                */
        int ws;                   /**< I2S WS/LRCLK. Required.                            */
        int dout;                 /**< Serial data out. Required.                         */
        int mclk;                 /**< I2S MCLK out, or -1 if the DAC needs none.         */
    } pins;
} audio_playback_config_t;

typedef struct {
    uint32_t sample_rate_hz;
    uint8_t  channels;
    uint8_t  bits_per_sample;
    uint8_t  bytes_per_sample;    /**< Storage width per sample per channel in the ring.  */
    uint16_t bytes_per_ms;        /**< Convenience: rate x channels x bytes / 1000.       */
    uint16_t frame_bytes;         /**< One DMA descriptor's payload; the read granule.    */
    uint32_t prefill_bytes;       /**< Ring backlog required before playback resumes.     */
} audio_playback_format_t;

typedef struct {
    uint64_t bytes_played;        /**< Bytes handed to the I2S driver since start.        */
    uint64_t bytes_silence;       /**< Of those, silence emitted for want of source data. */
    uint32_t dma_frames;          /**< Descriptors written.                               */
    uint32_t underruns;           /**< Frames that were short of source data.             */
    uint32_t starvations;         /**< Frames with no source data at all; forces prefill. */
    uint32_t ring_overruns;       /**< Our reader fell behind the ring's writer.          */
    uint32_t write_timeouts;      /**< `i2s_channel_write()` timed out; a frame was lost. */
    uint32_t prefills;            /**< Times the driver entered the prefill state.        */
    uint32_t task_stack_free_min; /**< Words. From uxTaskGetStackHighWaterMark().         */
    bool     running;
    bool     primed;              /**< False while accumulating prefill.                  */
} audio_playback_stats_t;

/**
 * @brief Configure the I2S TX channel, allocate DMA and frame storage (internal SRAM,
 *        DES-APB-002), open a read cursor on `cfg->source`, create the playback task,
 *        and take a `pm_policy` lock named `"audio_out"` for use while running.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  already initialised
 *   - ESP_ERR_INVALID_ARG    NULL cfg or source, unsupported rate/channels/width
 *                            combination, missing required pin, dma_frame_count < 2,
 *                            a DMA descriptor over 4092 bytes, or a source ring too
 *                            small to hold the prefill plus one frame
 *   - ESP_ERR_NO_MEM         frame buffer allocation or task creation failed
 *   - (propagated)           I2S driver errors
 *
 * @note Thread-safety: not safe concurrently with any other lifecycle call.
 */
esp_err_t audio_playback_init(const audio_playback_config_t *cfg);

/** @brief Stop if running, delete the channel and task, free all storage. */
esp_err_t audio_playback_deinit(void);

/**
 * @brief Enable the channel and start consuming. Acquires the `"audio_out"` sleep lock.
 *
 * Starts in the prefill state: the read cursor is moved to the newest byte in the ring
 * (stale audio is never replayed, matching `ringbuf_reader_open()`'s rationale) and
 * silence is emitted until `prefill_ms` has accumulated.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not initialised or already running,
 *         or a propagated I2S error.
 */
esp_err_t audio_playback_start(void);

/** @brief Disable the channel and release the sleep lock. Idempotent. */
esp_err_t audio_playback_stop(void);

/**
 * @brief Discard whatever is buffered in the source ring and re-enter prefill.
 *
 * The hand-off operation: a profile that has just taken ownership of the ring calls
 * this so it does not play the previous owner's tail (ADR-013).
 *
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if not initialised.
 *
 * @note Thread-safety: safe from any task, including while running.
 */
esp_err_t audio_playback_flush(void);

/** @brief Format the ring's writer must produce. ESP_ERR_INVALID_STATE before init. */
esp_err_t audio_playback_get_format(audio_playback_format_t *out);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t audio_playback_stats(audio_playback_stats_t *out);

/** @brief Zero the counters. */
esp_err_t audio_playback_stats_reset(void);

#ifdef __cplusplus
}
#endif
