/**
 * @file usb_audio.h
 * @brief USB Audio Class 2.0 microphone function, streaming PCM from a `ringbuf`
 *        over an isochronous IN endpoint at one packet per 1 ms frame.
 *
 * Realises FW-AUD-020..028. Registers itself with `usb_device` (ADR-007) and reads
 * audio as an ordinary `ringbuf` reader — it has no privileged access to the capture
 * driver and is a peer of any application consumer (DES-AUD-001, FW-AUD-028). The
 * ring is supplied by the caller; typically `audio_capture_get_ring()`.
 *
 * ## Clock-drift compensation (FW-AUD-024)
 *
 * The I2S sample clock and the host's SOF clock are unrelated. This function is an
 * *asynchronous source*: TinyUSB's IN flow control sizes each packet at nominal,
 * nominal−1 or nominal+1 samples according to its FIFO fill level, and the host
 * adapts. No feedback endpoint is needed, which saves one from the CON-03 budget.
 * `usb_audio_stats_t` exposes the ±1 counts so a soak test can see the servo work.
 *
 * ## Data path
 *
 *     ring ──(feeder task, 1 ms esp_timer)──► TinyUSB SW FIFO ──(ISR, per SOF)──► host
 *
 * The feeder runs only while the host has selected the streaming alternate
 * setting. On alternate 0 it stops, the ring reader is closed, and the FIFO is
 * cleared, so resuming never replays stale audio (FW-AUD-026, DES-AUD-005).
 *
 * ## Limits in this version
 *
 * - One channel (mono). TinyUSB 0.21 ships descriptor macros for 1 and 4 channels
 *   only; stereo needs a hand-built descriptor and is deferred.
 * - One fixed sample rate, advertised as a single-value range. The host may not
 *   change it.
 * - 16-, 24- or 32-bit PCM, matching whatever the ring holds. The module copies
 *   bytes; it does not convert.
 *
 * ## Thread and ISR safety
 *
 * `usb_audio_init()`/`deinit()` are not safe concurrently with anything. `stats()`
 * is safe from any task. Callbacks run in the USB device task. Nothing is ISR-safe.
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

/**
 * @brief Host changed mute or volume on the feature unit. USB device task context.
 *        Volume is in UAC2 units: 1/256 dB, signed. The SDK applies neither; the
 *        application decides what they mean (ADR-013).
 */
typedef void (*usb_audio_control_cb_t)(bool mute, int16_t volume_db256, void *ctx);

typedef struct {
    ringbuf_t *ring;                    /**< Required. Source of PCM bytes. Must outlive this module. */
    uint32_t   sample_rate_hz;          /**< Required. 8000..48000.                            */
    uint8_t    channels;                /**< Must be 1 in this version.                        */
    uint8_t    bits_per_sample;         /**< 16, 24 or 32.                                     */
    usb_audio_control_cb_t on_control;  /**< Optional.                                         */
    void      *ctx;
} usb_audio_config_t;

typedef struct {
    uint32_t packets;             /**< Isochronous IN packets completed.                  */
    uint32_t packets_nominal;     /**< Exactly nominal size.                              */
    uint32_t packets_plus_one;    /**< Nominal + 1 sample: servo draining a surplus.      */
    uint32_t packets_minus_one;   /**< Nominal − 1 sample: servo covering a deficit.      */
    uint32_t zero_length_packets; /**< FIFO empty at SOF: an underrun on the wire.        */
    uint32_t ring_overruns;       /**< Feeder fell behind the ring writer.                */
    uint32_t fifo_overflows;      /**< Feeder wrote more than the FIFO accepted (should be 0). */
    uint64_t bytes_streamed;      /**< Total bytes handed to the USB FIFO.                */
    uint16_t fifo_fill_bytes;     /**< Instantaneous SW FIFO fill.                        */
    uint16_t nominal_packet_bytes;/**< For interpreting the counters above.               */
    bool     streaming;           /**< Alternate setting 1 selected.                      */
    bool     mute;                /**< Last value set by the host.                        */
    int16_t  volume_db256;        /**< Last value set by the host.                        */
} usb_audio_stats_t;

/**
 * @brief Register the UAC2 microphone function with `usb_device`. Call after
 *        `usb_device_init()` and before `usb_device_start()`.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  already initialised, or `usb_device` not in the
 *                            registration window
 *   - ESP_ERR_INVALID_ARG    NULL cfg/ring, rate outside 8000..48000, or
 *                            bits_per_sample not 16/24/32
 *   - ESP_ERR_NOT_SUPPORTED  channels != 1 (see "Limits")
 *   - ESP_ERR_INVALID_SIZE   the required packet exceeds
 *                            CONFIG_TINYUSB_AUDIO_EP_IN_SZ_MAX; raise that symbol
 *   - ESP_ERR_NO_MEM         `usb_device` refused the endpoint budget, or the feeder
 *                            task could not be created
 *
 * @note Thread-safety: not safe concurrently with itself or `usb_audio_deinit()`.
 */
esp_err_t usb_audio_init(const usb_audio_config_t *cfg);

/** @brief Stop the feeder and forget the configuration. */
esp_err_t usb_audio_deinit(void);

/** @brief Whether the host currently has the streaming alternate setting selected. */
bool usb_audio_is_streaming(void);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t usb_audio_stats(usb_audio_stats_t *out);

/** @brief Zero the counters (not the mute/volume state). */
esp_err_t usb_audio_stats_reset(void);

#ifdef __cplusplus
}
#endif
