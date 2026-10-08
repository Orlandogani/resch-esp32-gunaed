/*
 * TLV320AIC3104 control. Register numbers and bit fields are from TI SLAS510G
 * ("TLV320AIC3104", rev. G, Feb 2021), §10.6 tables 10-6..10-109, page 0 only. Each
 * register write below cites its table.
 */
#include "tlv320aic3104.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "tlv320aic3104";

/* Page-0 registers used (SLAS510G table numbers in brackets). */
enum {
    REG_PAGE          = 0,    /* [10-6]  */
    REG_RESET         = 1,    /* [10-7]  */
    REG_RATE          = 2,    /* [10-8]  ADC fs D7-D4, DAC fs D3-D0             */
    REG_PLL_A         = 3,    /* [10-9]  PLL en D7, Q D6-D3, P D2-D0; reset 0x10  */
    REG_DATAPATH      = 7,    /* [10-13] fsref D7, L-DAC D4-D3, R-DAC D2-D1      */
    REG_IFACE_A       = 8,    /* [10-14] BCLK/WCLK direction                     */
    REG_IFACE_B       = 9,    /* [10-15] mode D7-D6, word length D5-D4           */
    REG_IFACE_C       = 10,   /* [10-16] data offset                             */
    REG_LADC_PGA      = 15,   /* [10-21] mute D7, gain D6-D0 (0.5 dB)            */
    REG_MIC1LP_LADC   = 19,   /* [10-25] diff D7, level D6-D3, power D2, step D1-D0 */
    REG_MICBIAS       = 25,   /* [10-31] level D7-D6                             */
    REG_DAC_POWER     = 37,   /* [10-43] L D7, R D6, HPLCOM cfg D5-D4            */
    REG_HP_DRIVER     = 38,   /* [10-44] HPRCOM cfg D5-D3, SC prot D2, SC mode D1 */
    REG_HP_STAGE      = 40,   /* [10-46] CM D7-D6, soft-step D1-D0               */
    REG_DAC_SWITCH    = 41,   /* [10-47] DAC_L1/R1 path select                   */
    REG_POP           = 42,   /* [10-48] power-on delay D7-D4, ramp D3-D2, CM src D1 */
    REG_LDAC_VOL      = 43,   /* [10-49] mute D7, attenuation D6-D0 (0.5 dB)     */
    REG_RDAC_VOL      = 44,   /* [10-50]                                         */
    REG_DACL1_HPLOUT  = 47,   /* [10-54] route D7, analog volume D6-D0           */
    REG_HPLOUT        = 51,   /* [10-58] level D7-D4, unmute D3, HiZ-off D2, power D0 */
    REG_HPLCOM        = 58,   /* [10-65] same layout                             */
    REG_DACR1_HPROUT  = 64,   /* [10-71]                                         */
    REG_HPROUT        = 65,   /* [10-72]                                         */
    REG_HPRCOM        = 72,   /* [10-79]                                         */
    REG_POWER_STATUS  = 94,   /* [10-96] L-DAC D7, R-DAC D6, HPLOUT D2, HPROUT D1 */
    REG_SC_STATUS     = 95,   /* [10-97]                                         */
    REG_CLOCK         = 101,  /* [10-101] CODEC_CLKIN = CLKDIV_OUT when D0 = 1   */
    REG_CLOCK_GEN     = 102,  /* [10-102] CLKDIV_IN source D7-D6; D3-D0 must be 0010; reset 0x02 */
};

#define PLL_A_RESET      0x10
#define CLOCK_GEN_RESET  0x02
#define I2C_TIMEOUT_MS   50

/* Output-level registers: level 0 dB, unmuted (D3), high-Z when powered down (D2),
 * powered (D0) — or the same with power and unmute cleared. [10-58] */
#define HP_ON(level_db)  ((uint8_t)(((level_db) << 4) | 0x08u | 0x04u | 0x01u))
#define HP_OFF           0x04u

static bool                    s_init;
static tlv320aic3104_config_t  s_cfg;
static i2c_master_dev_handle_t s_dev;
static SemaphoreHandle_t       s_mutex;
static tlv320aic3104_stats_t   s_stats;
static uint8_t                 s_atten = CONFIG_TLV320AIC3104_DEFAULT_ATTEN_HALF_DB;
static bool                    s_user_mute;   /* set_mute(); applies while running */
static uint8_t                 s_hp_level;
static uint8_t                 s_mic_gain = CONFIG_TLV320AIC3104_DEFAULT_MIC_GAIN_HALF_DB;

/* -------------------------------------------------------------------------- */
/* Pure                                                                        */
/* -------------------------------------------------------------------------- */

esp_err_t tlv320aic3104_clock_plan(uint32_t mclk_hz, uint32_t fs_hz, tlv320aic3104_clock_plan_t *out)
{
    if (out == NULL || fs_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    static const uint32_t refs[] = { 48000, 44100 };
    for (unsigned r = 0; r < 2; r++) {
        for (uint8_t q = 2; q <= 17; q++) {
            if ((uint64_t)refs[r] * 128u * q != mclk_hz) {
                continue;
            }
            /* N = fsref / fs must be one of 1, 1.5, ... 6, i.e. 2*fsref/fs an integer 2..12. */
            if ((2u * refs[r]) % fs_hz != 0) {
                continue;
            }
            uint32_t n_x2 = 2u * refs[r] / fs_hz;
            if (n_x2 < 2 || n_x2 > 12) {
                continue;
            }
            out->fs_ref_hz = refs[r];
            out->q = q;
            out->n_x2 = (uint8_t)n_x2;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_SUPPORTED;
}

/* -------------------------------------------------------------------------- */
/* Register access                                                             */
/* -------------------------------------------------------------------------- */

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    esp_err_t err = i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        s_stats.i2c_writes++;
    } else {
        s_stats.i2c_errors++;
        ESP_LOGW(TAG, "write r%u=0x%02x: %s", reg, val, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t rd(uint8_t reg, uint8_t *val)
{
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, val, 1, I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        s_stats.i2c_reads++;
    } else {
        s_stats.i2c_errors++;
    }
    return err;
}

/* Stop at the first failure; the caller reports it. */
#define TRY(x) do { esp_err_t e_ = (x); if (e_ != ESP_OK) { return e_; } } while (0)

/* The DACs are muted whenever the outputs are down, whatever the user asked. */
static esp_err_t write_volume_locked(bool mute)
{
    uint8_t v = (uint8_t)((mute ? 0x80u : 0x00u) | (s_atten & 0x7Fu));   /* [10-49/50] */
    TRY(wr(REG_LDAC_VOL, v));
    return wr(REG_RDAC_VOL, v);
}

static esp_err_t hp_regs_locked(uint8_t val)
{
    TRY(wr(REG_HPLOUT, val));
    TRY(wr(REG_HPROUT, val));
    if (s_cfg.hp_mode == TLV320AIC3104_HP_DIFFERENTIAL) {
        /* The COM drivers carry the inverted signal and need their own power (DES-CDC-004). */
        TRY(wr(REG_HPLCOM, val));
        TRY(wr(REG_HPRCOM, val));
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

static esp_err_t hw_reset(void)
{
    if (s_cfg.reset_gpio >= 0) {
        const gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_cfg.reset_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        TRY(gpio_config(&io));
        /* >= 10 ns low after the supplies are up (SLAS510G §10.3.1); 10 us is generous. */
        TRY(gpio_set_level(s_cfg.reset_gpio, 0));
        esp_rom_delay_us(10);
        TRY(gpio_set_level(s_cfg.reset_gpio, 1));
        esp_rom_delay_us(100);
    }
    return ESP_OK;
}

static esp_err_t program_locked(const tlv320aic3104_clock_plan_t *plan)
{
    uint8_t v = 0;
    /* Page 0, then a software reset so every register starts from its documented default
     * even without a reset pin. */
    TRY(wr(REG_PAGE, 0x00));
    TRY(wr(REG_RESET, 0x80));
    vTaskDelay(pdMS_TO_TICKS(1));

    /* Identity: there is no ID register, so check two reset defaults (DES-CDC-002). */
    TRY(rd(REG_PLL_A, &v));
    uint8_t g = 0;
    TRY(rd(REG_CLOCK_GEN, &g));
    if (v != PLL_A_RESET || g != CLOCK_GEN_RESET) {
        ESP_LOGE(TAG, "unexpected defaults r3=0x%02x r102=0x%02x (want 0x%02x 0x%02x)", v, g,
                 PLL_A_RESET, CLOCK_GEN_RESET);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Clocking (DES-CDC-003): PLL off, CODEC_CLKIN = CLKDIV_OUT, CLKDIV_IN = MCLK,
     * fsref = MCLK / (128 Q), ADC = DAC = fsref / N. Q code: 16 -> 0, 17 -> 1, else Q. */
    uint8_t qcode = (plan->q >= 16) ? (uint8_t)(plan->q - 16) : plan->q;
    TRY(wr(REG_PLL_A, (uint8_t)(qcode << 3)));                         /* [10-9]   */
    TRY(wr(REG_CLOCK_GEN, 0x02));                                      /* [10-102] */
    TRY(wr(REG_CLOCK, 0x01));                                          /* [10-101] */
    uint8_t ncode = (uint8_t)(plan->n_x2 - 2);                         /* 1 -> 0, 1.5 -> 1, ... 6 -> 10 */
    TRY(wr(REG_RATE, (uint8_t)((ncode << 4) | ncode)));                /* [10-8]: ADC fs must equal DAC fs */

    /* Data path: fsref flag for the AGC timers; left DAC plays left, right plays right. */
    TRY(wr(REG_DATAPATH, (uint8_t)((plan->fs_ref_hz == 44100 ? 0x80u : 0x00u) | 0x08u | 0x02u)));

    /* Serial interface: slave (BCLK, WCLK inputs), I2S, word length, no offset. */
    uint8_t wl = (s_cfg.word_bits == 20) ? 1 : (s_cfg.word_bits == 24) ? 2 : (s_cfg.word_bits == 32) ? 3 : 0;
    TRY(wr(REG_IFACE_A, 0x00));                                        /* [10-14] */
    TRY(wr(REG_IFACE_B, (uint8_t)(wl << 4)));                          /* [10-15] */
    TRY(wr(REG_IFACE_C, 0x00));                                        /* [10-16] */

    /* Output stage, all still powered down (DES-CDC-004). */
    uint8_t hpcom = (s_cfg.hp_mode == TLV320AIC3104_HP_DIFFERENTIAL) ? 0x00 : 0x10;  /* HPLCOM diff / VCM */
    TRY(wr(REG_DAC_POWER, hpcom));                                     /* [10-43] DACs off */
    uint8_t hprcom = (s_cfg.hp_mode == TLV320AIC3104_HP_DIFFERENTIAL) ? 0x00 : 0x08; /* HPRCOM diff / VCM */
    TRY(wr(REG_HP_DRIVER, (uint8_t)(hprcom | 0x04u)));                 /* [10-44] SC protection, current limit */
    TRY(wr(REG_HP_STAGE, (uint8_t)(CONFIG_TLV320AIC3104_HP_CM << 6))); /* [10-46] CM, soft-step 1/sample */
    TRY(wr(REG_DAC_SWITCH, 0x00));                                     /* [10-47] DAC_L1/R1 paths */
    /* Pop reduction [10-48]: power-on delay code, 1 ms ramp steps, CM from the band gap. */
    TRY(wr(REG_POP, (uint8_t)((CONFIG_TLV320AIC3104_POWER_ON_DELAY_CODE << 4) | 0x04u | 0x02u)));
    TRY(wr(REG_DACL1_HPLOUT, 0x80));                                   /* [10-54] route, 0 dB */
    TRY(wr(REG_DACR1_HPROUT, 0x80));                                   /* [10-71] route, 0 dB */
    TRY(hp_regs_locked(HP_OFF));

    TRY(write_volume_locked(true));

    /* Microphone path off until asked for. */
    TRY(wr(REG_MIC1LP_LADC, 0x78));                                    /* [10-25] not connected, ADC off */
    TRY(wr(REG_LADC_PGA, 0x80));                                       /* [10-21] PGA muted */
    TRY(wr(REG_MICBIAS, 0x00));                                        /* [10-31] off */
    return ESP_OK;
}

esp_err_t tlv320aic3104_init(const tlv320aic3104_config_t *cfg)
{
    if (s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL || cfg->bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->word_bits != 0 && cfg->word_bits != 16 && cfg->word_bits != 20 &&
        cfg->word_bits != 24 && cfg->word_bits != 32) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->reset_gpio >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO(cfg->reset_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    tlv320aic3104_clock_plan_t plan;
    esp_err_t err = tlv320aic3104_clock_plan(cfg->mclk_hz, cfg->sample_rate_hz, &plan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no PLL-off clock plan for MCLK %u Hz -> %u Hz", (unsigned)cfg->mclk_hz,
                 (unsigned)cfg->sample_rate_hz);
        return err;
    }

    s_cfg = *cfg;
    if (s_cfg.i2c_addr == 0) {
        s_cfg.i2c_addr = TLV320AIC3104_I2C_ADDR;
    }
    if (s_cfg.i2c_hz == 0) {
        s_cfg.i2c_hz = CONFIG_TLV320AIC3104_I2C_HZ;
    }
    if (s_cfg.word_bits == 0) {
        s_cfg.word_bits = 16;
    }
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();   /* Kept across deinit. */
        if (s_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    TRY(hw_reset());
    err = i2c_master_probe(s_cfg.bus, s_cfg.i2c_addr, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no ACK at 0x%02x: %s", s_cfg.i2c_addr, esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }
    const i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_cfg.i2c_addr,
        .scl_speed_hz = s_cfg.i2c_hz,
    };
    TRY(i2c_master_bus_add_device(s_cfg.bus, &dcfg, &s_dev));

    memset(&s_stats, 0, sizeof(s_stats));
    s_hp_level = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    err = program_locked(&plan);
    xSemaphoreGive(s_mutex);
    if (err != ESP_OK) {
        (void)i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }
    s_init = true;
    ESP_LOGI(TAG, "init: 0x%02x, MCLK %u Hz, fsref %u / Q %u, fs %u Hz (N %u.%u), %u-bit, HP %s, mic %s",
             s_cfg.i2c_addr, (unsigned)s_cfg.mclk_hz, (unsigned)plan.fs_ref_hz, plan.q,
             (unsigned)s_cfg.sample_rate_hz, plan.n_x2 / 2, (plan.n_x2 & 1) ? 5 : 0, s_cfg.word_bits,
             s_cfg.hp_mode == TLV320AIC3104_HP_DIFFERENTIAL ? "differential" : "single-ended",
             s_cfg.mic == TLV320AIC3104_MIC_NONE ? "none" : "MIC1LP");
    return ESP_OK;
}

esp_err_t tlv320aic3104_deinit(void)
{
    if (!s_init) {
        return ESP_OK;
    }
    (void)tlv320aic3104_mic_enable(false);
    (void)tlv320aic3104_stop();
    (void)i2c_master_bus_rm_device(s_dev);
    s_dev = NULL;
    if (s_cfg.reset_gpio >= 0) {
        (void)gpio_set_level(s_cfg.reset_gpio, 0);   /* held in reset: lowest supply current */
    }
    s_init = false;
    return ESP_OK;
}

esp_err_t tlv320aic3104_start(void)
{
    if (!s_init || s_stats.running) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    /* DACs first, still muted, then the drivers, whose power-on delay and ramp
     * (register 42) are the pop reduction (DES-CDC-005). */
    uint8_t hpcom = (s_cfg.hp_mode == TLV320AIC3104_HP_DIFFERENTIAL) ? 0x00 : 0x10;
    if ((err = wr(REG_DAC_POWER, (uint8_t)(0xC0u | hpcom))) == ESP_OK) {
        err = hp_regs_locked(HP_ON(s_hp_level));
    }
    uint8_t ps = 0;
    if (err == ESP_OK) {
        TickType_t t0 = xTaskGetTickCount();
        err = ESP_ERR_TIMEOUT;
        while (xTaskGetTickCount() - t0 < pdMS_TO_TICKS(CONFIG_TLV320AIC3104_POWER_UP_TIMEOUT_MS)) {
            if (rd(REG_POWER_STATUS, &ps) == ESP_OK && (ps & 0xC6u) == 0xC6u) {   /* DACs + HPL/HPROUT */
                err = ESP_OK;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        s_stats.power_status = ps;
        (void)rd(REG_SC_STATUS, &s_stats.short_circuit);
        if (err == ESP_ERR_TIMEOUT) {
            s_stats.power_up_timeouts++;
            ESP_LOGW(TAG, "drivers not up after %d ms (r94=0x%02x r95=0x%02x); is MCLK running?",
                     CONFIG_TLV320AIC3104_POWER_UP_TIMEOUT_MS, ps, s_stats.short_circuit);
        }
    }
    if (err == ESP_OK) {
        err = write_volume_locked(s_user_mute);
    }
    s_stats.running = (err == ESP_OK);
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t tlv320aic3104_stop(void)
{
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_stats.running) {
        return ESP_OK;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    /* Soft-mute: one 0.5 dB step per sample, at most 127 steps — under 3 ms at 48 kHz. */
    esp_err_t err = write_volume_locked(true);
    vTaskDelay(pdMS_TO_TICKS(5));
    esp_err_t e2 = hp_regs_locked(HP_OFF);
    uint8_t hpcom = (s_cfg.hp_mode == TLV320AIC3104_HP_DIFFERENTIAL) ? 0x00 : 0x10;
    esp_err_t e3 = wr(REG_DAC_POWER, hpcom);
    (void)rd(REG_POWER_STATUS, &s_stats.power_status);
    s_stats.running = false;
    xSemaphoreGive(s_mutex);
    return (err != ESP_OK) ? err : (e2 != ESP_OK) ? e2 : e3;
}

esp_err_t tlv320aic3104_set_volume(uint8_t atten_half_db)
{
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_atten = (atten_half_db > 127) ? 127 : atten_half_db;
    esp_err_t err = s_stats.running ? write_volume_locked(s_user_mute) : ESP_OK;
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t tlv320aic3104_set_mute(bool mute)
{
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    /* While stopped the DACs stay muted; the request takes effect at start(). */
    s_user_mute = mute;
    esp_err_t err = s_stats.running ? write_volume_locked(mute) : ESP_OK;
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t tlv320aic3104_set_hp_level(uint8_t db)
{
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (db > 9) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_hp_level = db;
    esp_err_t err = s_stats.running ? hp_regs_locked(HP_ON(db)) : ESP_OK;
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t tlv320aic3104_mic_enable(bool enable)
{
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_cfg.mic == TLV320AIC3104_MIC_NONE) {
        return enable ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err;
    if (enable) {
        /* Bias first so the electret has settled by the time the ADC listens. */
        static const uint8_t bias[] = { 0x00, 0x40, 0x80, 0xC0 };
        err = wr(REG_MICBIAS, bias[s_cfg.micbias & 3]);                   /* [10-31] */
        if (err == ESP_OK) {
            /* MIC1LP single-ended, 0 dB input level, left ADC on, soft-step 1/sample. */
            err = wr(REG_MIC1LP_LADC, 0x04);                              /* [10-25] */
        }
        if (err == ESP_OK) {
            err = wr(REG_LADC_PGA, (uint8_t)(s_mic_gain & 0x7Fu));         /* [10-21] unmuted */
        }
    } else {
        err = wr(REG_LADC_PGA, 0x80);
        esp_err_t e2 = wr(REG_MIC1LP_LADC, 0x78);
        esp_err_t e3 = wr(REG_MICBIAS, 0x00);
        if (err == ESP_OK) {
            err = (e2 != ESP_OK) ? e2 : e3;
        }
    }
    s_stats.mic_enabled = enable && err == ESP_OK;
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t tlv320aic3104_set_mic_gain(uint8_t gain_half_db)
{
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (gain_half_db > 119) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_mic_gain = gain_half_db;
    esp_err_t err = s_stats.mic_enabled ? wr(REG_LADC_PGA, gain_half_db) : ESP_OK;
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t tlv320aic3104_reg_read(uint8_t reg, uint8_t *val)
{
    if (val == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = rd(reg, val);
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t tlv320aic3104_reg_write(uint8_t reg, uint8_t val)
{
    if (!s_init) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = wr(reg, val);
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t tlv320aic3104_stats(tlv320aic3104_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    return ESP_OK;
}
