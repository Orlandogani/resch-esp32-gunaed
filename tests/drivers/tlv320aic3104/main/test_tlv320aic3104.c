/**
 * On-target Unity tests for drivers/tlv320aic3104 (FW-CDC-001..012).
 *
 * On a devkit there is no codec: the suite checks the pure clock planning, argument
 * validation, and that an absent chip fails cleanly (ESP_ERR_NOT_FOUND, no state left
 * behind). Cases tagged [needs_orga] need the real board and run only when
 * CONFIG_TEST_ON_ORGA=y. I2C on GPIO 8/9 — the ORGA v1 wiring, free on a DevKitC.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "unity.h"

#include "tlv320aic3104.h"

#define PIN_SDA   8
#define PIN_SCL   9
#define PIN_RESET 10

static i2c_master_bus_handle_t s_bus;

static i2c_master_bus_handle_t bus(void)
{
    if (s_bus == NULL) {
        const i2c_master_bus_config_t c = {
            .i2c_port = -1,
            .sda_io_num = PIN_SDA,
            .scl_io_num = PIN_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags = { .enable_internal_pullup = true },   /* ORGA has 4.7 k; the devkit has none */
        };
        TEST_ASSERT_EQUAL(ESP_OK, i2c_new_master_bus(&c, &s_bus));
    }
    return s_bus;
}

static tlv320aic3104_config_t base_cfg(void)
{
    tlv320aic3104_config_t c = {
        .bus = bus(),
        .reset_gpio = PIN_RESET,
        .mclk_hz = 12288000,
        .sample_rate_hz = 48000,
        .mic = TLV320AIC3104_MIC_MIC1LP_SE,
        .micbias = TLV320AIC3104_MICBIAS_2V5,
    };
    return c;
}

TEST_CASE("clock plan: integer MCLK ratios only", "[tlv320aic3104]")
{
    tlv320aic3104_clock_plan_t p;
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_clock_plan(12288000, 48000, &p));
    TEST_ASSERT_EQUAL(48000, p.fs_ref_hz);
    TEST_ASSERT_EQUAL(2, p.q);
    TEST_ASSERT_EQUAL(2, p.n_x2);

    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_clock_plan(12288000, 16000, &p));
    TEST_ASSERT_EQUAL(6, p.n_x2);                      /* 48 kHz / 3 */
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_clock_plan(12288000, 32000, &p));
    TEST_ASSERT_EQUAL(3, p.n_x2);                      /* 48 kHz / 1.5 */
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_clock_plan(24576000, 48000, &p));
    TEST_ASSERT_EQUAL(4, p.q);
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_clock_plan(11289600, 44100, &p));
    TEST_ASSERT_EQUAL(44100, p.fs_ref_hz);

    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, tlv320aic3104_clock_plan(12000000, 48000, &p)); /* needs the PLL */
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, tlv320aic3104_clock_plan(12288000, 7000, &p));  /* N > 6 */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tlv320aic3104_clock_plan(12288000, 48000, NULL));
}

TEST_CASE("init validates before touching the bus", "[tlv320aic3104]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tlv320aic3104_init(NULL));
    tlv320aic3104_config_t c = base_cfg(); c.bus = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tlv320aic3104_init(&c));
    c = base_cfg(); c.word_bits = 18;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tlv320aic3104_init(&c));
    c = base_cfg(); c.reset_gpio = 46;                 /* input-only */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tlv320aic3104_init(&c));
    c = base_cfg(); c.mclk_hz = 12000000;
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, tlv320aic3104_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tlv320aic3104_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tlv320aic3104_set_volume(0));
    uint8_t v;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tlv320aic3104_reg_read(2, &v));
}

TEST_CASE("absent codec: NOT_FOUND, and nothing is left initialised", "[tlv320aic3104][no_codec]")
{
    tlv320aic3104_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, tlv320aic3104_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, tlv320aic3104_init(&c));   /* not INVALID_STATE */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tlv320aic3104_start());
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_deinit());
}

TEST_CASE("ORGA: init programs clocking and the output stage", "[tlv320aic3104][needs_orga]")
{
    tlv320aic3104_config_t c = base_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_init(&c));
    uint8_t v = 0;
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(2, &v));
    TEST_ASSERT_EQUAL_HEX8(0x00, v);                   /* ADC = DAC = fsref / 1 */
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(3, &v));
    TEST_ASSERT_EQUAL_HEX8(0x10, v);                   /* PLL off, Q = 2 */
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(7, &v));
    TEST_ASSERT_EQUAL_HEX8(0x0A, v);                   /* L->L, R->R */
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(101, &v));
    TEST_ASSERT_EQUAL_HEX8(0x01, v);
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(37, &v));
    TEST_ASSERT_EQUAL_HEX8(0x00, v);                   /* DACs off, HPLCOM differential */
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(43, &v));
    TEST_ASSERT_BITS_HIGH(0x80, v);                    /* muted while stopped */

    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_set_volume(10));
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(43, &v));
    TEST_ASSERT_BITS_HIGH(0x80, v);                    /* still muted: not running */

    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_mic_enable(true));
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(19, &v));
    TEST_ASSERT_EQUAL_HEX8(0x04, v);
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(25, &v));
    TEST_ASSERT_EQUAL_HEX8(0x80, v);                   /* MICBIAS 2.5 V */
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_mic_enable(false));
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_reg_read(19, &v));
    TEST_ASSERT_EQUAL_HEX8(0x78, v);

    tlv320aic3104_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_stats(&st));
    TEST_ASSERT_EQUAL(0, st.i2c_errors);
    TEST_ASSERT_EQUAL(ESP_OK, tlv320aic3104_deinit());
}

void app_main(void)
{
    /* Let a USB-Serial/JTAG capture attach first (.claude/BACKLOG.md). */
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
#if CONFIG_TEST_ON_ORGA
    unity_run_tests_by_tag("[no_codec]", true);
#else
    unity_run_tests_by_tag("[needs_orga]", true);
#endif
    UNITY_END();
}
