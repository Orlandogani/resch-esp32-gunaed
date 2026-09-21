/**
 * @file audio_capture.h
 * @brief I2S microphone capture: standard I2S or PDM input, DMA double-buffering,
 *        and publication into a multi-reader `ringbuf` at the sample clock's cadence.
 *
 * Realises FW-AUD-001..011 and the driver contract of SDD §15.3 (DR1–DR10). This is
 * the only module that owns an I2S peripheral. Consumers — `usb_audio`, the
 * application — read the ring returned by `audio_capture_get_ring()` as peers; the
 * driver never knows who they are (ADR-014, FW-AUD-004).
 *
 * ## Lifecycle (DR1)
 *
 *     audio_capture_init(cfg) ─► audio_capture_start() ─► audio_capture_stop() ─► audio_capture_deinit()
 *                                        ▲                       │
 *                                        └───── repeatable ──────┘
 *
 * `start()`/`stop()` enable and disable the channel without reallocating (DR2).
 *
 * ## Data path
 *
 *     I2S ─► GDMA ─► driver DMA buffers ─(ISR: notify only)─► capture task ─► ring
 *
 * The ISR does nothing but wake the task (FW-AUD-007, DR5). The task, pinned at the
 * highest SDK priority (DES-ACAP-004), copies one DMA frame per wake-up into the
 * ring. A DMA frame that arrives while the previous one is still unread is an
 * overrun: counted, rate-limited to the log, never fatal (FW-AUD-010).
 *
 * ## Sample format in the ring
 *
 * Bytes exactly as the I2S driver delivers them: little-endian PCM, `channels`
 * interleaved, `bits_per_sample` wide (16 → 2 bytes; 24 and 32 → 4 bytes per
 * sample as ESP-IDF's I2S packs them). Consumers use `audio_capture_get_format()`
 * rather than assuming.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit/start/stop` are not safe concurrently with each other. `get_ring()`,
 * `get_format()` and `stats()` are safe from any task. Nothing is ISR-safe.
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

typedef enum {
    AUDIO_CAPTURE_IF_I2S_STD = 0,  /**< Standard I2S (Philips) slave microphone or ADC. */
    AUDIO_CAPTURE_IF_PDM,          /**< PDM microphone; the S3 decimates to PCM in hardware. */
} audio_capture_interface_t;

/** Which I2S slot a mono standard-I2S microphone occupies (set by its L/R pin). */
typedef enum {
    AUDIO_CAPTURE_SLOT_LEFT = 0,
    AUDIO_CAPTURE_SLOT_RIGHT,
} audio_capture_slot_t;

typedef struct {
    audio_capture_interface_t interface;
    uint32_t sample_rate_hz;      /**< 8000..48000.                                       */
    uint8_t  channels;            /**< 1 or 2.                                            */
    uint8_t  bits_per_sample;     /**< 16 or 32 for I2S standard; 16 for PDM.             */
    audio_capture_slot_t slot;    /**< Mono standard I2S only: which slot carries data.   */
    uint8_t  dma_frame_ms;        /**< Per DMA descriptor. 0 selects Kconfig (2 ms).      */
    uint8_t  dma_frame_count;     /**< Descriptors, >= 2. 0 selects Kconfig (4).          */
    uint16_t ring_ms;             /**< Ring depth. 0 selects Kconfig (40 ms).             */
    struct {
        int clk;                  /**< I2S BCLK, or PDM CLK. Required.                    */
        int ws;                   /**< I2S WS/LRCLK. Ignored for PDM.                     */
        int din;                  /**< Serial data in. Required.                          */
        int mclk;                 /**< I2S MCLK out, or -1 if the microphone needs none.  */
    } pins;
} audio_capture_config_t;

typedef struct {
    uint32_t sample_rate_hz;
    uint8_t  channels;
    uint8_t  bits_per_sample;
    uint8_t  bytes_per_sample;    /**< Storage width per sample per channel in the ring. */
    uint16_t bytes_per_ms;        /**< Convenience: rate × channels × bytes / 1000.     */
} audio_capture_format_t;

typedef struct {
    uint64_t bytes_captured;      /**< Bytes written into the ring since start.           */
    uint32_t dma_frames;          /**< DMA descriptors consumed.                           */
    uint32_t dma_overruns;        /**< Driver queue overflowed: a frame was lost in DMA.   */
    uint32_t short_reads;         /**< Task got fewer bytes than a full descriptor.        */
    uint32_t ring_write_failures; /**< Should be 0; means the ring is smaller than a frame.*/
    uint32_t task_stack_free_min; /**< Words. From uxTaskGetStackHighWaterMark().          */
    bool     running;
} audio_capture_stats_t;

/**
 * @brief Configure the I2S channel, allocate DMA and ring storage (internal SRAM,
 *        DES-ACAP-002), create the capture task, and take a `pm_policy` lock named
 *        `"audio"` for use while running.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  already initialised
 *   - ESP_ERR_INVALID_ARG    NULL cfg, unsupported rate/channels/width combination,
 *                            missing required pin, or dma_frame_count < 2
 *   - ESP_ERR_NO_MEM         DMA or ring allocation failed, or task creation failed
 *   - (propagated)           I2S driver errors
 *
 * @note Thread-safety: not safe concurrently with any other lifecycle call.
 */
esp_err_t audio_capture_init(const audio_capture_config_t *cfg);

/** @brief Stop if running, delete the channel and task, free all storage. */
esp_err_t audio_capture_deinit(void);

/**
 * @brief Enable the channel and start publishing. Acquires the `"audio"` sleep lock.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not initialised or already running,
 *         or a propagated I2S error.
 */
esp_err_t audio_capture_start(void);

/** @brief Disable the channel and release the sleep lock. Idempotent. */
esp_err_t audio_capture_stop(void);

/**
 * @brief The ring consumers read from. Valid from `init()` until `deinit()`.
 * @return NULL if not initialised.
 */
ringbuf_t *audio_capture_get_ring(void);

/** @brief Effective format of the bytes in the ring. ESP_ERR_INVALID_STATE before init. */
esp_err_t audio_capture_get_format(audio_capture_format_t *out);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t audio_capture_stats(audio_capture_stats_t *out);

/** @brief Zero the counters. */
esp_err_t audio_capture_stats_reset(void);

#ifdef __cplusplus
}
#endif
