/**
 * @file decimator.h
 * @brief Integer-factor FIR decimation of interleaved 16-bit PCM.
 *
 * Realises FW-DEC-001..006. A pure-logic module (`lib/`, SAD §3.3 D5): no peripheral, no
 * task, no allocation — the caller owns the state, so any number of instances can run.
 *
 * Why it exists: a codec whose ADC and DAC share one word clock delivers microphone audio at
 * the playback rate (48 kHz on ORGA v1), while the headset's microphone streams run at
 * 16 kHz. `drivers/audio_capture` uses this to deliver the lower rate into its ring
 * (`DES-DEC-001`).
 *
 * ## Filter
 *
 * A linear-phase windowed-sinc (Blackman) low-pass of `factor × DECIMATOR_TAPS_PER_PHASE`
 * taps, designed once at `decimator_init()` and quantised to Q15 with unity DC gain. The
 * −6 dB point sits at 0.875 × the output Nyquist frequency (7 kHz for 48 → 16 kHz), which
 * keeps the voice band flat and puts Blackman's ~74 dB stopband over what would alias.
 * Group delay is `(taps − 1) / 2` input samples (≈ 0.74 ms for 48 → 16 kHz).
 *
 * ## Streaming
 *
 * History and output phase carry across calls, so splitting the input into blocks of any
 * size produces exactly the same output as one call with the whole input (`FW-DEC-004`).
 *
 * ## Thread and ISR safety
 *
 * An instance may be used by one task at a time. Distinct instances are independent.
 * Nothing allocates or blocks, so calls are ISR-safe if the caller's buffers are.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DECIMATOR_MAX_FACTOR      6u   /**< 48 kHz → 8 kHz is the largest step in use.   */
#define DECIMATOR_MAX_CHANNELS    2u
#define DECIMATOR_TAPS_PER_PHASE  24u  /**< Taps per output sample per unit of factor.   */
#define DECIMATOR_MAX_TAPS        (DECIMATOR_MAX_FACTOR * DECIMATOR_TAPS_PER_PHASE)

/**
 * @brief Decimator state. Caller-owned; treat the fields as private.
 *
 * About 1.4 KiB. History is stored twice over (`2 × taps`) so the convolution always reads
 * one contiguous window and never wraps.
 */
typedef struct {
    uint8_t  factor;
    uint8_t  channels;
    uint16_t taps;
    uint16_t pos;                                       /* Next history slot, 0..taps-1. */
    uint8_t  phase;                                     /* Inputs since the last output.  */
    int16_t  coef[DECIMATOR_MAX_TAPS];
    int16_t  hist[DECIMATOR_MAX_CHANNELS][2 * DECIMATOR_MAX_TAPS];
} decimator_t;

/**
 * @brief Design the filter for `factor` and clear the history.
 *
 * @param factor    2..DECIMATOR_MAX_FACTOR. (1 is rejected: a caller that does not
 *                  decimate should not run a filter.)
 * @param channels  1 or 2, interleaved.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG for NULL or an out-of-range factor/channel count.
 */
esp_err_t decimator_init(decimator_t *d, uint8_t factor, uint8_t channels);

/** @brief Clear history and phase, keeping the filter. Use at a stream discontinuity. */
void decimator_reset(decimator_t *d);

/**
 * @brief Filter and decimate `in_frames` frames from `in` into `out`.
 *
 * In-place operation (`out == in`) is allowed: every output frame is written after the
 * input frames it depends on have been consumed.
 *
 * @param in         Interleaved input, `in_frames × channels` samples.
 * @param in_frames  Frames available. Any count, including 0.
 * @param out        Room for at least `decimator_max_out_frames(in_frames)` frames.
 * @return Frames written to `out` (0 if `d`, `in` or `out` is NULL).
 */
size_t decimator_process(decimator_t *d, const int16_t *in, size_t in_frames, int16_t *out);

/** @brief Upper bound on the frames `decimator_process()` writes for `in_frames` inputs. */
size_t decimator_max_out_frames(const decimator_t *d, size_t in_frames);

#ifdef __cplusplus
}
#endif
