/**
 * @file board.h
 * @brief The board contract: what is wired where, as data (ADR-025).
 *
 * Realises FW-BRD-001..006. Every board under `boards/<name>/board/` is a component named
 * `board` that implements this header; an application selects one at configure time with
 * `-D BOARD=<name>` and depends on `board` without knowing which it got. This is the role
 * Zephyr's devicetree plays, kept to plain C: a board *describes* its wiring, it does not
 * instantiate drivers. Which drivers run, and with what policy, stays the application's
 * decision (ADR-013).
 *
 * A pin that a board does not have is `BOARD_PIN_NONE`; a part it does not carry has
 * `present = false`. Consumers check before use — the devkit has no codec, no IMU, no
 * encoder, no LED, no battery and no power latch.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_PIN_NONE (-1)

/** What a switch is for. Buttons are listed by role, not by index, so the application
 *  never hard-codes a board's switch order. */
typedef enum {
    BOARD_BTN_ACTION = 0,   /**< Play/pause; long press switches profile.     */
    BOARD_BTN_VOL_UP,
    BOARD_BTN_VOL_DOWN,
    BOARD_BTN_POWER,        /**< Long press powers off (boards with a latch). */
    BOARD_BTN_ROLE_COUNT,
} board_button_role_t;

typedef struct {
    board_button_role_t role;
    int  gpio;
    bool active_low;
    bool pull_enable;            /**< Internal pull opposing the active level.       */
    bool wake_from_deep_sleep;   /**< RTC-capable pins only (S3: GPIO 0..21).         */
} board_button_t;

typedef struct {
    const char *name;

    /** Speaker path: I2S TX into a DAC, amplifier or codec. */
    struct {
        int8_t port;             /**< I2S controller, or -1 for any.                  */
        int mclk, bclk, ws, dout;
        uint32_t mclk_hz;        /**< 0: the DAC needs no MCLK.                       */
    } spk;

    /** Codec control. Its ADC data comes back on `codec.din`, the RX half of `spk.port`. */
    struct {
        bool    present;
        uint8_t i2c_addr;
        int     reset;
        int     din;             /**< Codec DOUT into the S3, or BOARD_PIN_NONE.      */
        bool    hp_differential; /**< Speakers across HPxOUT/HPxCOM (never grounded). */
        bool    has_mic_input;   /**< A microphone is wired to the codec (boom jack). */
    } codec;

    /** Built-in microphone(s) on the S3's own I2S RX. */
    struct {
        bool   present;
        bool   pdm;
        int8_t port;             /**< -1 any; PDM needs I2S0 on the S3.              */
        int    clk, ws, din;
        bool   right_slot;       /**< The voice microphone answers in the right slot. */
    } mic;

    struct {
        int      sda, scl;       /**< BOARD_PIN_NONE: no bus.                        */
        uint32_t hz;
        bool     internal_pullup;/**< The board has no external pull-ups.            */
    } i2c;

    struct {
        bool    present;
        uint8_t i2c_addr;
        int     int1;
    } imu;

    struct {
        int  gpio;               /**< Headset jack detect, or BOARD_PIN_NONE.         */
        bool plugged_high;       /**< The pin reads 1 with a plug in.                 */
    } jack;

    const board_button_t *buttons;
    uint8_t button_count;

    struct {
        int     a, b;            /**< BOARD_PIN_NONE: no encoder.                     */
        bool    pull_up;
        bool    reverse;
        uint8_t counts_per_detent;
    } encoder;

    struct {
        int      r, g, b;        /**< BOARD_PIN_NONE per absent channel.              */
        bool     active_low;
        uint16_t gain_permille[3];
    } led;

    struct {
        int      adc_gpio;       /**< BOARD_PIN_NONE: no battery measurement.         */
        uint16_t divider_num, divider_den;
        int      ext_power_gpio;
        bool     ext_power_active_low;
    } battery;

    struct {
        int  hold;               /**< BOARD_PIN_NONE: no latch (bus-powered).         */
        bool active_low;
    } power_latch;

    int debug_gpio;              /**< Free test point for timing, or BOARD_PIN_NONE.  */
} board_desc_t;

/** @brief The selected board's description. Static lifetime. */
const board_desc_t *board_desc(void);

/**
 * @brief Board-level setup that no driver owns: safe levels on reset lines and test points.
 *        Idempotent. Call first in `app_main`.
 */
esp_err_t board_init(void);

/**
 * @brief The board's I2C master bus, created on first call (thread-safe) and kept.
 * @return ESP_OK; ESP_ERR_NOT_SUPPORTED if the board has no bus; ESP_ERR_INVALID_ARG on NULL;
 *         propagated I2C errors.
 */
esp_err_t board_i2c_bus(i2c_master_bus_handle_t *out);

/**
 * @brief PDM oversampling (64 or 128, `audio_capture_config_t.pdm_oversample`) that puts
 *        the PDM clock inside the board microphone's valid bands at `rate_hz`, or 0 for the
 *        driver default. The bands belong to the microphone part, so the board answers.
 */
uint8_t board_mic_pdm_oversample(uint32_t rate_hz);

/**
 * @brief For board implementations only: create (once) and return the I2C bus `desc`
 *        describes. Boards implement `board_i2c_bus()` with this, so the lazy, thread-safe
 *        creation exists once.
 */
esp_err_t board_api_i2c_bus(const board_desc_t *desc, i2c_master_bus_handle_t *out);

#ifdef __cplusplus
}
#endif
