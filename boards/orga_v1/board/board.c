/*
 * Board: ORGA v1 headset main board (left cup).
 *
 * Every pin below is from the netlist, C:\ORLANDO\Lab\kicad\headset-orga\orga-v1\
 * generator\design.py, mapped through the ESP32-S3-WROOM-1 pin table (module pin -> GPIO).
 * The wiring is fixed by copper, so unlike the devkit nothing here is Kconfig.
 * See ../doc/index.md for the full map and the review notes.
 */
#include "board.h"

#include "driver/gpio.h"

static const board_button_t s_buttons[] = {
    /* MFB (EVQ-P7A01P, side): 10 k pull-up + 100 nF on the board. Not RTC-capable. */
    { .role = BOARD_BTN_ACTION, .gpio = 42, .active_low = true, .pull_enable = false },
    /* POWER: seen through Q1 (BTN_PWR, 10 k pull-up to 3V3), low while pressed. The
     * same press raises the regulator enable through the BAT54C — that is the latch. */
    { .role = BOARD_BTN_POWER,  .gpio = 41, .active_low = true, .pull_enable = false },
};

static const board_desc_t s_desc = {
    .name = "orga_v1",
    /* TLV320AIC3104 on I2S1; 22 R series on MCLK/BCLK/WS/DOUT. */
    .spk = { .port = 1, .mclk = 4, .bclk = 5, .ws = 6, .dout = 7, .mclk_hz = 12288000 },
    .codec = {
        .present = true, .i2c_addr = 0x18, .reset = 10, .din = 15,
        .hp_differential = true,     /* HPxOUT/HPxCOM to floating speaker connectors */
        .has_mic_input = true,       /* boom mic: jack TIP -> MIC1LP, MICBIAS via 2.2 k */
    },
    /* MK1 voice (LR = GND -> left) and MK2 reference (LR = VDD -> right) share DIN0
     * (GPIO17); MK3 inner-cup on DIN1 (GPIO18) is not fitted. PDM RX is I2S0-only. */
    .mic = { .present = true, .pdm = true, .port = 0, .clk = 16, .ws = BOARD_PIN_NONE, .din = 17,
             .right_slot = false },
    .i2c = { .sda = 8, .scl = 9, .hz = 400000, .internal_pullup = false },   /* 4.7 k on board */
    .imu = { .present = true, .i2c_addr = 0x6A, .int1 = 11 },               /* SA0 = GND */
    .jack = { .gpio = 12, .plugged_high = true },   /* ring switch opens on insertion */
    .buttons = s_buttons,
    .button_count = sizeof(s_buttons) / sizeof(s_buttons[0]),
    /* PEC09 placeholder: 24 pulses / 24 detents; 10 k pull-ups + 10 nF on the board. */
    .encoder = { .a = 13, .b = 14, .pull_up = false, .reverse = false, .counts_per_detent = 4 },
    /* Common anode at 3V3, cathodes through 470 R (R) and 68 R (G, B). G and B sit near
     * their forward voltage at 3.3 V; the gains are a starting point to tune on the
     * first build (TBD-017). */
    .led = { .r = 38, .g = 39, .b = 40, .active_low = true, .gain_permille = { 600, 1000, 1000 } },
    /* 1 M / 1 M divider, 100 nF; BQ24074 PGOOD open-drain with 10 k to 3V3. */
    .battery = { .adc_gpio = 1, .divider_num = 2, .divider_den = 1, .ext_power_gpio = 47,
                 .ext_power_active_low = true },
    /* PWR_HOLD into the BAT54C diode-OR, 100 k pull-down: any reset drops it. */
    .power_latch = { .hold = 21, .active_low = false },
    .debug_gpio = 2,   /* DBG_TIMING test point */
};

const board_desc_t *board_desc(void)
{
    return &s_desc;
}

esp_err_t board_init(void)
{
    /* The debug test point is a scope trigger: drive it low so it reads clean until
     * someone toggles it. The codec reset line has its own 100 k pull-down and stays
     * low until drivers/tlv320aic3104 releases it. */
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_desc.debug_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err == ESP_OK) {
        err = gpio_set_level(s_desc.debug_gpio, 0);
    }
    return err;
}

esp_err_t board_i2c_bus(i2c_master_bus_handle_t *out)
{
    return board_api_i2c_bus(&s_desc, out);
}

/* IM73D122V01 valid PDM clock bands (datasheet, "PDM clock frequency"), in kHz. */
static const struct { uint32_t lo, hi; } k_pdm_bands[] = {
    { 450, 850 }, { 1200, 1650 }, { 2000, 2600 }, { 2900, 3300 },
};

uint8_t board_mic_pdm_oversample(uint32_t rate_hz)
{
    /* Prefer 64 (lower power), take 128 if 64 falls outside every band. */
    static const uint8_t candidates[] = { 64, 128 };
    for (unsigned c = 0; c < 2; c++) {
        uint32_t khz = rate_hz * candidates[c] / 1000u;
        for (unsigned b = 0; b < sizeof(k_pdm_bands) / sizeof(k_pdm_bands[0]); b++) {
            if (khz >= k_pdm_bands[b].lo && khz <= k_pdm_bands[b].hi) {
                return candidates[c];
            }
        }
    }
    return 128;   /* no valid choice: the higher clock is the safer guess */
}
