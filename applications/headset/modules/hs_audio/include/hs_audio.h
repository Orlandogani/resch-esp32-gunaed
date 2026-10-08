/**
 * @file hs_audio.h
 * @brief The headset's audio path on whatever board it runs: speaker output (with the codec
 *        where there is one), microphone input chosen by the jack, and volume.
 *
 * Product policy (ADR-013), shared by every profile so none of them knows the board.
 * Before ORGA v1 each profile set up `audio_playback` and `audio_capture` itself from
 * Kconfig pins; this module does it once, from `board_desc()` (ADR-025).
 *
 * ## Microphone selection
 *
 * With a boom microphone plugged into the jack (`board.jack`), the microphone is the
 * codec's ADC (MIC1LP), captured on the RX half of the speaker's I2S controller in full
 * duplex (ADR-026) at the speaker rate and decimated to the profile's mic rate. Unplugged,
 * it is the board's own (PDM) microphone. A plug or unplug while streaming switches the
 * input live with `audio_capture_switch_input()`: the mic ring and every reader of it
 * survive, so neither the USB host nor the dongle sees the change as anything but audio.
 *
 * ## Volume
 *
 * The host's speaker volume (UAC2 feature unit, or relayed by the dongle) becomes the
 * codec's DAC attenuation, never less than `CONFIG_HS_AUDIO_MIN_ATTEN_HALF_DB` (the
 * hearing-safety cap). Boards without a codec ignore it, as before: the host attenuates.
 *
 * ## Threading
 *
 * `init` and `open/start/close` from the main task. `set_jack`, `set_host_volume` and the
 * queries are safe from any task.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ringbuf_t *playback_ring;     /**< Required. main/'s ring; the profile writes it.    */
    uint32_t   spk_rate_hz;
    uint8_t    spk_channels;
    uint32_t   mic_rate_hz;       /**< Mono 16-bit. 0: no microphone wanted.             */
} hs_audio_config_t;

/**
 * @brief Once at boot: bring up the codec, if the board has one (reset, probe, clocks).
 * @return ESP_OK, or the codec's error (the headset then runs without it — on ORGA that
 *         means no sound, which diag reports).
 */
esp_err_t hs_audio_init(void);

/** @brief Initialise playback and capture for a profile. Nothing runs until `start()`. */
esp_err_t hs_audio_open(const hs_audio_config_t *cfg);

/** @brief Start playback, then the codec outputs, then capture. */
esp_err_t hs_audio_start(void);

/** @brief Stop and release everything `open()` took; the codec is muted and powered down.
 *         Drops whatever was queued for playback. Idempotent. */
esp_err_t hs_audio_close(void);

/** @brief The microphone ring, or NULL when no microphone came up. Valid until close. */
ringbuf_t *hs_audio_mic_ring(void);

/** @brief Bytes queued for playback — the feedback servo's input (ADR-021, ADR-022). */
uint32_t hs_audio_playback_backlog(void);

/** @brief Drop audio queued for playback (DES-APB-007). */
void hs_audio_flush(void);

/** @brief The jack changed (or its state at boot). Switches the microphone live if open. */
void hs_audio_set_jack(bool plugged);

/**
 * @brief Host speaker volume in 1/256 dB (≤ 0) and mute, applied to the codec.
 * @return ESP_ERR_NOT_SUPPORTED on a board without a codec.
 */
esp_err_t hs_audio_set_host_volume(bool mute, int16_t volume_db256);

/** @brief True when the boom microphone is the active input. */
bool hs_audio_boom_active(void);

#ifdef __cplusplus
}
#endif
