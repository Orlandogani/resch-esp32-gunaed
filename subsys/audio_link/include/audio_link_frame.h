/**
 * @file audio_link_frame.h
 * @brief On-air frame format of the wireless audio link, and its pack/parse functions.
 *
 * Realises FW-LNK-010..013. One frame is one GATT write or notification: the link
 * never fragments, so a frame is at most one ATT payload (`AUDIO_LINK_FRAME_MAX_LEN`).
 * The format is versioned so that a dongle and a headset on different firmware
 * detect the mismatch instead of decoding garbage.
 *
 *     byte 0      (version << 4) | type
 *     byte 1      format code — AUDIO only; see audio_link_format_code()
 *     bytes 2..3  sequence number, little-endian — AUDIO only; wraps at 2^16
 *     bytes 4..5  sender's receive backlog in sample frames (samples per channel),
 *                 little-endian, saturating; 0xFFFF = unknown
 *     bytes 6..   payload: an Opus packet (AUDIO), nothing (REPORT), or
 *                 application bytes (CONTROL)
 *
 * Every frame, of every type, carries the backlog field. That is how the receiving
 * end of a direction tells the sending end how much audio it is holding, which is
 * the measurement the sender's rate servo regulates (ADR-022).
 *
 * The functions here are pure — no state, no allocation — and usable before
 * `audio_link_init()`.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_LINK_FRAME_VERSION     1u
#define AUDIO_LINK_FRAME_HDR_LEN     6u
/** One ATT payload at the SDK's default MTU of 247 (MTU − 3). */
#define AUDIO_LINK_FRAME_MAX_LEN     244u
#define AUDIO_LINK_FRAME_MAX_PAYLOAD (AUDIO_LINK_FRAME_MAX_LEN - AUDIO_LINK_FRAME_HDR_LEN)
#define AUDIO_LINK_BACKLOG_UNKNOWN   0xFFFFu

typedef enum {
    AUDIO_LINK_FRAME_AUDIO   = 1,   /**< One codec frame of one direction's stream. */
    AUDIO_LINK_FRAME_REPORT  = 2,   /**< Backlog report only; no payload.           */
    AUDIO_LINK_FRAME_CONTROL = 3,   /**< Application bytes, opaque to the SDK.      */
} audio_link_frame_type_t;

typedef struct {
    audio_link_frame_type_t type;
    uint8_t        format;          /**< AUDIO: format code. Otherwise 0.            */
    uint16_t       seq;             /**< AUDIO: sequence number. Otherwise 0.        */
    uint16_t       backlog_frames;  /**< Sender's receive backlog, or UNKNOWN.       */
    const uint8_t *payload;         /**< Parse: points into the input buffer.        */
    uint16_t       payload_len;
} audio_link_frame_t;

/**
 * @brief Encode a stream format as the one-byte code carried by AUDIO frames.
 *
 *     bits 0..2  rate: 0 = 8 kHz, 1 = 12 kHz, 2 = 16 kHz, 3 = 24 kHz, 4 = 48 kHz
 *     bit  3     stereo
 *     bits 4..5  frame duration: 0 = 2.5 ms, 1 = 5 ms, 2 = 10 ms, 3 = 20 ms
 *
 * @param frame_us  2500, 5000, 10000 or 20000.
 * @return The code, or 0xFF for a combination Opus does not support.
 */
uint8_t audio_link_format_code(uint32_t sample_rate_hz, uint8_t channels, uint32_t frame_us);

/**
 * @brief Serialise `f` into `out`.
 * @return Bytes written, or 0 if `f` is invalid (unknown type, payload too long,
 *         NULL payload with a length) or `cap` is too small.
 */
size_t audio_link_frame_pack(const audio_link_frame_t *f, uint8_t *out, size_t cap);

/**
 * @brief Parse `len` bytes. `out->payload` points into `in`, so `in` must outlive it.
 * @return ESP_OK, ESP_ERR_INVALID_ARG (NULL), ESP_ERR_INVALID_SIZE (shorter than a
 *         header, or longer than AUDIO_LINK_FRAME_MAX_LEN),
 *         ESP_ERR_INVALID_VERSION (different frame version), or
 *         ESP_ERR_NOT_SUPPORTED (unknown type, or a REPORT with a payload).
 */
esp_err_t audio_link_frame_parse(const uint8_t *in, size_t len, audio_link_frame_t *out);

/**
 * @brief Signed distance from `expected` to `seq` on the 16-bit sequence circle:
 *        0 is the frame expected next, > 0 means frames were lost, < 0 means late.
 */
static inline int16_t audio_link_seq_delta(uint16_t seq, uint16_t expected)
{
    return (int16_t)(uint16_t)(seq - expected);
}

#ifdef __cplusplus
}
#endif
