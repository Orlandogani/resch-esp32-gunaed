/**
 * @file usb_audio.h
 * @brief USB Audio Class 2.0 function: a microphone, a speaker, or a duplex headset,
 *        over one isochronous IN and/or one isochronous OUT endpoint at one packet
 *        per 1 ms frame.
 *
 * Realises FW-AUD-020..028 (microphone) and FW-AUD-050..061 (speaker). Registers
 * itself with `usb_device` (ADR-007) and moves PCM through caller-owned `ringbuf`s —
 * it has no privileged access to `audio_capture` or `audio_playback` and is a peer
 * of any application reader or writer (DES-AUD-001, FW-AUD-028, FW-AUD-051).
 *
 * ## The two clock problems, and why they have different answers
 *
 * The I2S sample clock and the host's SOF clock are unrelated in both directions,
 * but the device's role differs, so the fix does too.
 *
 * - **Microphone (IN): asynchronous source.** The device decides how much to send.
 *   TinyUSB's IN flow control sizes each packet at nominal, nominal−1 or nominal+1
 *   samples from its FIFO fill, and the host adapts. No feedback endpoint
 *   (FW-AUD-024, DES-AUD-003).
 * - **Speaker (OUT): asynchronous sink.** The *host* decides how much to send, so
 *   the device must ask. An explicit feedback endpoint reports, in 16.16 format,
 *   the samples per frame the host should send. The value comes from the playback
 *   ring's backlog measured against a setpoint — not from TinyUSB's OUT FIFO, which
 *   this module drains every millisecond and which therefore carries jitter, not
 *   drift. See `ADR-021` and `FW-AUD-056`.
 *
 * Because only the application knows who drains the playback ring, it supplies the
 * backlog through `speaker.backlog_cb`. With `drivers/audio_playback` that is one
 * line: `audio_playback_stats(&st); return st.source_backlog_bytes;` (ADR-013).
 *
 * ## Data paths
 *
 *     mic:      ring ──(feeder task, 1 ms)──► TinyUSB IN FIFO ──(ISR, per SOF)──► host
 *     speaker:  host ──(ISR, per SOF)──► TinyUSB OUT FIFO ──(feeder task, 1 ms)──► ring
 *
 * One task and one 1 ms `esp_timer` serve both directions. Each runs only while the
 * host has selected its streaming alternate setting; on alternate 0 that direction
 * stops, its cursor is closed and its FIFO cleared, so resuming never replays stale
 * audio (FW-AUD-026, FW-AUD-057, DES-AUD-005).
 *
 * ## Build-time gating
 *
 * The speaker direction needs TinyUSB's OUT path and feedback endpoint compiled in:
 * `CONFIG_TINYUSB_AUDIO_SPEAKER_ENABLED`. Without it, `usb_audio_init()` returns
 * `ESP_ERR_NOT_SUPPORTED` for any configuration that asks for a speaker, rather than
 * silently enumerating as a microphone.
 *
 * ## Limits in this version
 *
 * - 1 or 2 channels per direction. The descriptor is built here rather than taken
 *   from TinyUSB's templates, which cover neither stereo nor duplex.
 * - One fixed sample rate per direction, advertised as a single-value range. The
 *   host may not change it. The two directions may differ (a 16 kHz mic beside a
 *   48 kHz speaker is the expected headset case) — they are separate clock sources.
 * - 16-, 24- or 32-bit PCM, matching whatever the rings hold. The module copies
 *   bytes; it does not convert, resample or attenuate.
 *
 * ## Thread and ISR safety
 *
 * `usb_audio_init()`/`deinit()` are not safe concurrently with anything. `stats()`
 * and `is_streaming()` are safe from any task. Callbacks run in the USB device task.
 * Nothing is ISR-safe.
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

/** Which streaming direction a callback or statistic refers to. */
typedef enum {
    USB_AUDIO_STREAM_MIC = 0,     /**< Device to host. Isochronous IN.  */
    USB_AUDIO_STREAM_SPEAKER,     /**< Host to device. Isochronous OUT. */
} usb_audio_stream_t;

/** Which directions the function advertises. */
typedef enum {
    USB_AUDIO_DIR_MIC = 0,        /**< Microphone only: 1 AC + 1 AS interface.     */
    USB_AUDIO_DIR_SPEAKER,        /**< Speaker only: 1 AC + 1 AS interface.        */
    USB_AUDIO_DIR_HEADSET,        /**< Duplex: 1 AC + 2 AS interfaces, one device. */
} usb_audio_direction_t;

/**
 * @brief Host changed mute or volume on one direction's feature unit. USB device
 *        task context. Volume is in UAC2 units: 1/256 dB, signed. The SDK applies
 *        neither; the application decides what they mean (ADR-013).
 */
typedef void (*usb_audio_control_cb_t)(usb_audio_stream_t which, bool mute,
                                       int16_t volume_db256, void *ctx);

/** One direction's stream format and buffer. */
typedef struct {
    ringbuf_t *ring;              /**< Required. Mic: the source this module reads.
                                       Speaker: the sink this module writes, and which
                                       `audio_playback` (or any peer) drains.          */
    uint32_t   sample_rate_hz;    /**< Required. 8000..48000.                           */
    uint8_t    channels;          /**< 1 or 2.                                          */
    uint8_t    bits_per_sample;   /**< 16, 24 or 32.                                    */
} usb_audio_stream_config_t;

/**
 * @brief Bytes still queued for playback in the speaker ring.
 *
 * Called from the feeder task once per millisecond while the speaker streams. It is
 * the measurement the feedback servo regulates, so it must be cheap and must not
 * block. Returning a constant disables the servo in effect: the feedback value
 * stays nominal and the ring will drift.
 */
typedef uint32_t (*usb_audio_backlog_cb_t)(void *ctx);

typedef struct {
    usb_audio_direction_t direction;

    usb_audio_stream_config_t mic;      /**< Used unless `direction` is SPEAKER.   */
    usb_audio_stream_config_t speaker;  /**< Used unless `direction` is MIC.       */

    /** Speaker only. Required when `direction` is not MIC. */
    usb_audio_backlog_cb_t speaker_backlog_cb;

    /** Speaker only. Backlog the servo aims to hold, in milliseconds of audio.
     *  0 selects `CONFIG_USB_AUDIO_SPEAKER_TARGET_MS`. Should match, or slightly
     *  exceed, the playback driver's prefill so the two do not fight. */
    uint16_t speaker_target_ms;

    usb_audio_control_cb_t on_control;  /**< Optional.                             */
    void      *ctx;                     /**< Passed to both callbacks.             */
} usb_audio_config_t;

/** Microphone (IN) counters. */
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
} usb_audio_mic_stats_t;

/** Speaker (OUT) counters. The feedback servo is observable here. */
typedef struct {
    uint32_t packets;             /**< Isochronous OUT packets received.                  */
    uint32_t packets_nominal;     /**< Exactly nominal size: the host is tracking us.     */
    uint32_t packets_short;       /**< Below nominal: host slowing down, or a dropped frame. */
    uint32_t packets_long;        /**< Above nominal: host speeding up.                   */
    uint32_t zero_length_packets; /**< Host sent nothing this frame.                      */
    uint64_t bytes_received;      /**< Total bytes taken from the USB FIFO.               */
    uint32_t ring_write_failures; /**< Should be 0; means the ring is smaller than a packet. */
    uint32_t fifo_fill_bytes;     /**< Instantaneous OUT SW FIFO fill: jitter, not drift. */
    uint32_t backlog_bytes;       /**< Last backlog reported by `speaker_backlog_cb`.     */
    uint32_t target_bytes;        /**< Setpoint the servo regulates `backlog_bytes` to.   */
    uint32_t feedback_value;      /**< Last value given to the host, 16.16 samples/frame. */
    uint32_t feedback_updates;    /**< Times the value was recomputed and set.            */
    uint32_t feedback_clamped;    /**< Times the servo hit its ±1-sample authority limit. */
    uint16_t nominal_packet_bytes;
    bool     streaming;
    bool     mute;
    int16_t  volume_db256;
} usb_audio_speaker_stats_t;

typedef struct {
    usb_audio_mic_stats_t     mic;
    usb_audio_speaker_stats_t speaker;
} usb_audio_stats_t;

/**
 * @brief Register the UAC2 function with `usb_device`. Call after
 *        `usb_device_init()` and before `usb_device_start()`.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE   already initialised
 *   - ESP_ERR_INVALID_ARG     NULL cfg, a used direction with a NULL ring, an
 *                             unsupported rate/channel/width combination, a speaker
 *                             direction with no `speaker_backlog_cb`, or a speaker
 *                             ring too small to hold the target backlog
 *   - ESP_ERR_NOT_SUPPORTED   a speaker was asked for without
 *                             `CONFIG_TINYUSB_AUDIO_SPEAKER_ENABLED`
 *   - ESP_ERR_INVALID_SIZE    a direction needs a larger endpoint than its
 *                             `CONFIG_TINYUSB_AUDIO_EP_*_SZ_MAX` allows
 *   - (propagated)            `usb_device_register()` errors, including the
 *                             endpoint budget (`FW-USB-003`)
 */
esp_err_t usb_audio_init(const usb_audio_config_t *cfg);

/** @brief Stop both directions, unregister nothing (the descriptor is frozen), free tasks. */
esp_err_t usb_audio_deinit(void);

/** @brief True if the host has selected the streaming alternate setting for `which`. */
bool usb_audio_is_streaming(usb_audio_stream_t which);

/** @brief Snapshot both directions' counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t usb_audio_stats(usb_audio_stats_t *out);

/** @brief Zero the counters, keeping the derived constants. */
esp_err_t usb_audio_stats_reset(void);

#ifdef __cplusplus
}
#endif
