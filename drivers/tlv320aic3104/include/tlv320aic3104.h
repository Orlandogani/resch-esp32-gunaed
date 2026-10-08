/**
 * @file tlv320aic3104.h
 * @brief TI TLV320AIC3104 stereo codec: control over I2C (clocking, routing, power, volume).
 *
 * Realises FW-CDC-001..012. A *control driver* (SDD §15.3): it owns the codec's I2C device
 * and its reset pin, not the audio data. Audio flows over I2S through `audio_playback`
 * (DAC direction) and `audio_capture` (ADC direction, full duplex on the same controller,
 * ADR-026). No task and no ISR: every call is a short, synchronous I2C sequence.
 *
 * ## What it configures (datasheet SLAS510G, §10)
 *
 * - **Clocking:** codec is I2S slave; fS(ref) = MCLK / (128 × Q) with the PLL off, then the
 *   ADC and DAC rate = fS(ref) / N, N ∈ {1, 1.5, … 6}. Only integer MCLK ratios are supported
 *   (`tlv320aic3104_clock_plan()`); ORGA v1 uses 12.288 MHz → 48 kHz (Q = 2, N = 1). The
 *   AIC3104 requires ADC fS = DAC fS (register 2 footnote).
 * - **Headphones:** DAC_L1 → HPLOUT and DAC_R1 → HPROUT at 0 dB analog gain. In the default
 *   `TLV320AIC3104_HP_DIFFERENTIAL` mode HPLCOM/HPRCOM drive the inverse of HPLOUT/HPROUT,
 *   so each speaker is bridge-tied across OUT and COM — never ground a COM pin or a speaker
 *   wire in that mode. Short-circuit protection is on, in current-limit mode.
 * - **Microphone:** MIC1LP single-ended into the left ADC through the PGA, with MICBIAS for
 *   an electret. The right ADC stays off; its I2S slot carries zeros.
 *
 * ## Lifecycle
 *
 *     init(cfg)  reset, probe, program clocks and interface; outputs off, DAC muted
 *     start()    power DACs and drivers with the pop-reduction ramp, then unmute
 *     stop()     soft-mute, power the drivers and DACs down
 *     deinit()   stop, hold the codec in reset, release the I2C device
 *
 * **Start the I2S clocks (MCLK, BCLK, WCLK) before `start()`**: every internal timer,
 * including the soft-stepping and the driver ramp, runs from MCLK.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit` are not safe concurrently with anything. Every other call is safe from any
 * task (internal mutex). Nothing is ISR-safe. Calls block for the I2C transfers; `start()`
 * blocks for the driver power-up (`CONFIG_TLV320AIC3104_POWER_UP_TIMEOUT_MS` at most).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TLV320AIC3104_I2C_ADDR 0x18   /**< Fixed 7-bit address (0011000). */

typedef enum {
    TLV320AIC3104_HP_DIFFERENTIAL = 0,  /**< Speaker across HPxOUT and HPxCOM (ORGA v1).   */
    TLV320AIC3104_HP_SINGLE_ENDED_VCM,  /**< HPxCOM is a constant VCM: cap-less, the
                                             speaker's return goes to COM, never ground.  */
} tlv320aic3104_hp_mode_t;

typedef enum {
    TLV320AIC3104_MIC_NONE = 0,         /**< No microphone input used.                     */
    TLV320AIC3104_MIC_MIC1LP_SE,        /**< MIC1LP single-ended into the left ADC.        */
} tlv320aic3104_mic_t;

typedef enum {
    TLV320AIC3104_MICBIAS_OFF = 0,
    TLV320AIC3104_MICBIAS_2V0,
    TLV320AIC3104_MICBIAS_2V5,
    TLV320AIC3104_MICBIAS_AVDD,
} tlv320aic3104_micbias_t;

typedef struct {
    i2c_master_bus_handle_t bus;     /**< Required. Shared bus, owned by the board.        */
    uint8_t  i2c_addr;               /**< 0 selects TLV320AIC3104_I2C_ADDR.                */
    uint32_t i2c_hz;                 /**< 0 selects Kconfig (400 kHz).                     */
    int      reset_gpio;             /**< RESET pin, or -1 (software reset only — the
                                          datasheet requires a hardware reset after power
                                          up, so -1 is for boards that do it themselves).  */
    uint32_t mclk_hz;                /**< Required. MCLK the S3 (or an oscillator) drives. */
    uint32_t sample_rate_hz;         /**< Required. ADC = DAC rate.                         */
    uint8_t  word_bits;              /**< 16, 20, 24 or 32. 0 selects 16.                  */
    tlv320aic3104_hp_mode_t hp_mode;
    tlv320aic3104_mic_t     mic;
    tlv320aic3104_micbias_t micbias; /**< Applied while the microphone is enabled.        */
} tlv320aic3104_config_t;

/** Clock plan for PLL-off operation. */
typedef struct {
    uint32_t fs_ref_hz;              /**< 48000 or 44100.                                  */
    uint8_t  q;                      /**< 2..17: fS(ref) = MCLK / (128 × Q).               */
    uint8_t  n_x2;                   /**< 2..12: fS = fS(ref) / (n_x2 / 2).                */
} tlv320aic3104_clock_plan_t;

typedef struct {
    uint32_t i2c_writes;
    uint32_t i2c_reads;
    uint32_t i2c_errors;             /**< Failed transfers (NACK, timeout).                */
    uint32_t power_up_timeouts;      /**< `start()` gave up waiting for the drivers.       */
    uint8_t  power_status;           /**< Register 94 at the end of the last start/stop.   */
    uint8_t  short_circuit;          /**< Register 95 at the end of the last start.        */
    bool     running;                /**< Outputs powered.                                 */
    bool     mic_enabled;
} tlv320aic3104_stats_t;

/**
 * @brief Reset the codec, check it answers, and program clocking, the serial interface,
 *        the DAC data path and the output stage — everything powered down and muted.
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  already initialised
 *   - ESP_ERR_INVALID_ARG    NULL cfg or bus, bad word length, bad reset pin
 *   - ESP_ERR_NOT_SUPPORTED  no PLL-off clock plan for this MCLK and rate
 *   - ESP_ERR_NOT_FOUND      nothing acknowledges at the address
 *   - ESP_ERR_INVALID_RESPONSE  it acknowledges but its reset defaults are wrong (not an
 *                            AIC3104, or the reset did not take)
 *   - (propagated)           I2C errors
 */
esp_err_t tlv320aic3104_init(const tlv320aic3104_config_t *cfg);

/** @brief Stop, hold the codec in reset (if a pin was given), release the I2C device. */
esp_err_t tlv320aic3104_deinit(void);

/**
 * @brief Power the DACs and headphone drivers up through the pop-reduction ramp, wait for
 *        them, then unmute to the current volume. The I2S clocks must already be running.
 * @return ESP_OK; ESP_ERR_INVALID_STATE (not initialised or already running);
 *         ESP_ERR_TIMEOUT if the drivers did not report powered (the outputs are then left
 *         as programmed — `stats()` has register 94 and 95 for diagnosis); I2C errors.
 */
esp_err_t tlv320aic3104_start(void);

/** @brief Soft-mute, then power the headphone drivers and DACs down. Idempotent. */
esp_err_t tlv320aic3104_stop(void);

/**
 * @brief DAC digital volume, both channels, in 0.5 dB steps of attenuation: 0 = 0 dB,
 *        127 = −63.5 dB. Soft-stepped by the codec. Kept across stop/start.
 */
esp_err_t tlv320aic3104_set_volume(uint8_t atten_half_db);

/** @brief Soft-mute or unmute both DACs without changing the volume. */
esp_err_t tlv320aic3104_set_mute(bool mute);

/** @brief Headphone driver output level, 0..9 dB (registers 51/58/65/72). Default 0. */
esp_err_t tlv320aic3104_set_hp_level(uint8_t db);

/**
 * @brief Enable or disable the configured microphone: MICBIAS, the input routing, the left
 *        ADC and its PGA. Independent of start/stop (the ADC needs only the clocks).
 * @return ESP_ERR_NOT_SUPPORTED when the config has `TLV320AIC3104_MIC_NONE`.
 */
esp_err_t tlv320aic3104_mic_enable(bool enable);

/** @brief Left-ADC PGA gain in 0.5 dB steps, 0..119 (0..59.5 dB). Kept across enables. */
esp_err_t tlv320aic3104_set_mic_gain(uint8_t gain_half_db);

/** @brief Raw page-0 register access, for diagnostics and bring-up. */
esp_err_t tlv320aic3104_reg_read(uint8_t reg, uint8_t *val);
esp_err_t tlv320aic3104_reg_write(uint8_t reg, uint8_t val);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t tlv320aic3104_stats(tlv320aic3104_stats_t *out);

/**
 * @brief Find a PLL-off clock plan: an fS(ref) of 48 or 44.1 kHz that MCLK reaches with an
 *        integer Q in 2..17, and a rate divider N in {1, 1.5, … 6} from it to `fs_hz`.
 *        Pure; used by `init()` and exposed so a board can check its choice at build time.
 * @return ESP_OK, ESP_ERR_INVALID_ARG (NULL), ESP_ERR_NOT_SUPPORTED (no plan).
 */
esp_err_t tlv320aic3104_clock_plan(uint32_t mclk_hz, uint32_t fs_hz, tlv320aic3104_clock_plan_t *out);

#ifdef __cplusplus
}
#endif
