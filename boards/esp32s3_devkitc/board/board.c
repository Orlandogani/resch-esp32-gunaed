/*
 * Board: ESP32-S3-DevKitC-1 (N16R8) with provisional wiring — the development stand-in
 * for the headset until ORGA v1 exists. Pins are Kconfig (BOARD_DEVKIT_*) because
 * whatever is plugged into a devkit's headers is not fixed; they moved here from
 * applications/headset/main/Kconfig.projbuild with their old defaults.
 */
#include "board.h"

#include "sdkconfig.h"

static const board_button_t s_buttons[] = {
    { .role = BOARD_BTN_VOL_DOWN, .gpio = CONFIG_BOARD_DEVKIT_BTN_VOL_DOWN, .active_low = true, .pull_enable = true },
    { .role = BOARD_BTN_VOL_UP,   .gpio = CONFIG_BOARD_DEVKIT_BTN_VOL_UP,   .active_low = true, .pull_enable = true },
    /* GPIO 0 is the BOOT button: the one control physically present on every devkit,
     * and RTC-capable, so it also serves as the deep-sleep wake source. */
    { .role = BOARD_BTN_ACTION,   .gpio = CONFIG_BOARD_DEVKIT_BTN_ACTION,   .active_low = true, .pull_enable = true,
      .wake_from_deep_sleep = true },
};

static const board_desc_t s_desc = {
    .name = "esp32s3_devkitc",
    .spk = {
        .port = -1,
        .mclk = CONFIG_BOARD_DEVKIT_SPK_MCLK, .bclk = CONFIG_BOARD_DEVKIT_SPK_BCLK,
        .ws = CONFIG_BOARD_DEVKIT_SPK_WS, .dout = CONFIG_BOARD_DEVKIT_SPK_DOUT,
        .mclk_hz = 0,
    },
    .codec = { .present = false, .reset = BOARD_PIN_NONE, .din = BOARD_PIN_NONE },
    .mic = {
#if CONFIG_BOARD_DEVKIT_MIC_PDM
        .present = true, .pdm = true, .port = 0,
#else
        .present = true, .pdm = false, .port = -1,
#endif
        .clk = CONFIG_BOARD_DEVKIT_MIC_CLK, .ws = CONFIG_BOARD_DEVKIT_MIC_WS,
        .din = CONFIG_BOARD_DEVKIT_MIC_DIN, .right_slot = false,
    },
    .i2c = { .sda = BOARD_PIN_NONE, .scl = BOARD_PIN_NONE },
    .imu = { .present = false, .int1 = BOARD_PIN_NONE },
    .jack = { .gpio = BOARD_PIN_NONE },
    .buttons = s_buttons,
    .button_count = sizeof(s_buttons) / sizeof(s_buttons[0]),
    .encoder = { .a = BOARD_PIN_NONE, .b = BOARD_PIN_NONE },
    .led = { .r = BOARD_PIN_NONE, .g = BOARD_PIN_NONE, .b = BOARD_PIN_NONE },
    .battery = { .adc_gpio = BOARD_PIN_NONE, .ext_power_gpio = BOARD_PIN_NONE },
    .power_latch = { .hold = BOARD_PIN_NONE },
    .debug_gpio = BOARD_PIN_NONE,
};

const board_desc_t *board_desc(void)
{
    return &s_desc;
}

esp_err_t board_init(void)
{
    return ESP_OK;   /* nothing board-level to set up on a devkit */
}

esp_err_t board_i2c_bus(i2c_master_bus_handle_t *out)
{
    return board_api_i2c_bus(&s_desc, out);
}

uint8_t board_mic_pdm_oversample(uint32_t rate_hz)
{
    (void)rate_hz;
    return 0;   /* microphone part unknown (TBD-002): driver default */
}
