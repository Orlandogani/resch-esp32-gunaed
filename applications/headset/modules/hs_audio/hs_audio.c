/**
 * The headset audio path, built from the board description (ADR-025). Policy only: the
 * mechanisms are drivers/audio_playback, drivers/audio_capture (ADR-026 duplex, decimation,
 * live input switch) and drivers/tlv320aic3104.
 */
#include "hs_audio.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "audio_capture.h"
#include "audio_playback.h"
#include "board.h"
#include "tlv320aic3104.h"

static const char *TAG = "hs_audio";

static struct {
    bool              codec_up;      /* tlv320aic3104_init() succeeded            */
    bool              open;
    bool              playback_up;
    bool              capture_up;
    bool              started;
    bool              jack;          /* boom plugged, as last reported            */
    bool              boom;          /* boom is the capture input now             */
    hs_audio_config_t cfg;
    SemaphoreHandle_t mutex;         /* open/close vs. set_jack from another task */
} s_a;

static audio_capture_port_t cap_port(int8_t p)
{
    return p == 0 ? AUDIO_CAPTURE_PORT_I2S0 : p == 1 ? AUDIO_CAPTURE_PORT_I2S1 : AUDIO_CAPTURE_PORT_AUTO;
}

/* Can the boom microphone feed a ring at this rate? It comes through the codec's ADC at
 * the speaker rate (ADC fs must equal DAC fs on the AIC3104), so only integer ratios. */
static uint8_t boom_decimation(void)
{
    const board_desc_t *b = board_desc();
    if (!s_a.codec_up || !b->codec.has_mic_input || b->codec.din == BOARD_PIN_NONE ||
        s_a.cfg.mic_rate_hz == 0 || s_a.cfg.spk_rate_hz % s_a.cfg.mic_rate_hz != 0) {
        return 0;
    }
    uint32_t d = s_a.cfg.spk_rate_hz / s_a.cfg.mic_rate_hz;
    return (d >= 1 && d <= 6) ? (uint8_t)d : 0;
}

static audio_capture_config_t mic_cfg(bool boom)
{
    const board_desc_t *b = board_desc();
    audio_capture_config_t c = {
        .sample_rate_hz = s_a.cfg.mic_rate_hz,
        .channels = 1,
        .bits_per_sample = 16,
    };
    if (boom) {
        /* RX half of the speaker controller, clocked by the TX (ADR-026). */
        c.interface = AUDIO_CAPTURE_IF_I2S_STD;
        c.port = cap_port(b->spk.port);
        c.bus_slave = true;
        c.slot = AUDIO_CAPTURE_SLOT_LEFT;           /* MIC1LP -> left ADC */
        c.decimation = boom_decimation();
        c.pins.clk = b->spk.bclk;
        c.pins.ws = b->spk.ws;
        c.pins.din = b->codec.din;
        c.pins.mclk = -1;
    } else {
        c.interface = b->mic.pdm ? AUDIO_CAPTURE_IF_PDM : AUDIO_CAPTURE_IF_I2S_STD;
        c.port = cap_port(b->mic.port);
        c.slot = b->mic.right_slot ? AUDIO_CAPTURE_SLOT_RIGHT : AUDIO_CAPTURE_SLOT_LEFT;
        c.pdm_oversample = b->mic.pdm ? board_mic_pdm_oversample(s_a.cfg.mic_rate_hz) : 0;
        c.pins.clk = b->mic.clk;
        c.pins.ws = b->mic.ws;
        c.pins.din = b->mic.din;
        c.pins.mclk = -1;
    }
    return c;
}

static bool want_boom(void)
{
    return s_a.jack && boom_decimation() != 0;
}

/* -------------------------------------------------------------------------- */

esp_err_t hs_audio_init(void)
{
    if (s_a.mutex == NULL) {
        s_a.mutex = xSemaphoreCreateMutex();
        if (s_a.mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    const board_desc_t *b = board_desc();
    if (!b->codec.present || s_a.codec_up) {
        return ESP_OK;
    }
    i2c_master_bus_handle_t bus;
    esp_err_t err = board_i2c_bus(&bus);
    if (err != ESP_OK) {
        return err;
    }
    const tlv320aic3104_config_t cc = {
        .bus = bus,
        .i2c_addr = b->codec.i2c_addr,
        .reset_gpio = b->codec.reset,
        .mclk_hz = b->spk.mclk_hz,
        .sample_rate_hz = CONFIG_HS_AUDIO_CODEC_RATE_HZ,
        .hp_mode = b->codec.hp_differential ? TLV320AIC3104_HP_DIFFERENTIAL : TLV320AIC3104_HP_SINGLE_ENDED_VCM,
        .mic = b->codec.has_mic_input ? TLV320AIC3104_MIC_MIC1LP_SE : TLV320AIC3104_MIC_NONE,
        .micbias = TLV320AIC3104_MICBIAS_2V5,
    };
    err = tlv320aic3104_init(&cc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "codec: %s — no speaker output on this board", esp_err_to_name(err));
        return err;
    }
    (void)tlv320aic3104_set_volume(CONFIG_HS_AUDIO_INITIAL_ATTEN_HALF_DB);
    (void)tlv320aic3104_set_mic_gain(CONFIG_HS_AUDIO_BOOM_GAIN_HALF_DB);
    s_a.codec_up = true;
    return ESP_OK;
}

esp_err_t hs_audio_open(const hs_audio_config_t *cfg)
{
    if (cfg == NULL || cfg->playback_ring == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_a.open) {
        return ESP_ERR_INVALID_STATE;
    }
    const board_desc_t *b = board_desc();
    if (b->codec.present && s_a.codec_up && cfg->spk_rate_hz != CONFIG_HS_AUDIO_CODEC_RATE_HZ) {
        ESP_LOGE(TAG, "speaker %u Hz, but the codec runs at %d Hz", (unsigned)cfg->spk_rate_hz,
                 CONFIG_HS_AUDIO_CODEC_RATE_HZ);
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_a.mutex, portMAX_DELAY);
    s_a.cfg = *cfg;
    const audio_playback_config_t pb = {
        .source = cfg->playback_ring,
        .sample_rate_hz = cfg->spk_rate_hz,
        .channels = cfg->spk_channels,
        .bits_per_sample = 16,
        .slot = AUDIO_PLAYBACK_SLOT_BOTH,
        .port = b->spk.port,
        .pins = { .bclk = b->spk.bclk, .ws = b->spk.ws, .dout = b->spk.dout, .mclk = b->spk.mclk },
    };
    esp_err_t err = audio_playback_init(&pb);
    if (err != ESP_OK) {
        xSemaphoreGive(s_a.mutex);
        ESP_LOGE(TAG, "audio_playback_init: %s", esp_err_to_name(err));
        return err;   /* without a sink there is no headset */
    }
    s_a.playback_up = true;
    s_a.open = true;

    /* Microphone: optional. Playback first, so a boom capture finds the TX it slaves to. */
    if (cfg->mic_rate_hz != 0 && (b->mic.present || boom_decimation() != 0)) {
        s_a.boom = want_boom() || !b->mic.present;
        if (s_a.boom && s_a.codec_up) {
            (void)tlv320aic3104_mic_enable(true);
        }
        audio_capture_config_t cc = mic_cfg(s_a.boom);
        err = audio_capture_init(&cc);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "no microphone (%s); speaker only", esp_err_to_name(err));
            if (s_a.boom && s_a.codec_up) {
                (void)tlv320aic3104_mic_enable(false);
            }
        } else {
            s_a.capture_up = true;
        }
    }
    xSemaphoreGive(s_a.mutex);
    ESP_LOGI(TAG, "open: speaker %u Hz x%u%s, mic %s", (unsigned)cfg->spk_rate_hz, cfg->spk_channels,
             s_a.codec_up ? " via codec" : "", !s_a.capture_up ? "none" : s_a.boom ? "boom (codec ADC)" : "built-in");
    return ESP_OK;
}

esp_err_t hs_audio_start(void)
{
    if (!s_a.open) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_a.mutex, portMAX_DELAY);
    esp_err_t err = audio_playback_start();
    if (err == ESP_OK && s_a.codec_up) {
        /* MCLK and BCLK run now; the codec's ramp and soft-step need them (FW-CDC-006). */
        esp_err_t cerr = tlv320aic3104_start();
        if (cerr != ESP_OK) {
            ESP_LOGW(TAG, "codec start: %s", esp_err_to_name(cerr));
        }
    }
    if (err == ESP_OK && s_a.capture_up) {
        esp_err_t cerr = audio_capture_start();
        if (cerr != ESP_OK) {
            ESP_LOGW(TAG, "audio_capture_start: %s; speaker only", esp_err_to_name(cerr));
        }
    }
    s_a.started = (err == ESP_OK);
    xSemaphoreGive(s_a.mutex);
    return err;
}

esp_err_t hs_audio_close(void)
{
    if (s_a.mutex == NULL) {
        return ESP_OK;
    }
    xSemaphoreTake(s_a.mutex, portMAX_DELAY);
    if (s_a.codec_up) {
        (void)tlv320aic3104_stop();               /* mute before the clocks go */
        (void)tlv320aic3104_mic_enable(false);
    }
    if (s_a.capture_up) {
        (void)audio_capture_stop();
        (void)audio_capture_deinit();
        s_a.capture_up = false;
    }
    if (s_a.playback_up) {
        (void)audio_playback_stop();
        (void)audio_playback_flush();
        (void)audio_playback_deinit();
        s_a.playback_up = false;
    }
    s_a.open = false;
    s_a.started = false;
    s_a.boom = false;
    xSemaphoreGive(s_a.mutex);
    return ESP_OK;
}

ringbuf_t *hs_audio_mic_ring(void)
{
    return s_a.capture_up ? audio_capture_get_ring() : NULL;
}

uint32_t hs_audio_playback_backlog(void)
{
    audio_playback_stats_t st;
    return (audio_playback_stats(&st) == ESP_OK) ? st.source_backlog_bytes : 0;
}

void hs_audio_flush(void)
{
    (void)audio_playback_flush();
}

void hs_audio_set_jack(bool plugged)
{
    if (s_a.mutex == NULL) {
        s_a.jack = plugged;
        return;
    }
    xSemaphoreTake(s_a.mutex, portMAX_DELAY);
    s_a.jack = plugged;
    const board_desc_t *b = board_desc();
    bool boom = want_boom() || (s_a.capture_up && !b->mic.present);
    if (s_a.capture_up && boom != s_a.boom) {
        if (boom) {
            (void)tlv320aic3104_mic_enable(true);   /* bias settles while we switch */
        }
        audio_capture_config_t cc = mic_cfg(boom);
        esp_err_t err = audio_capture_switch_input(&cc);
        if (err == ESP_OK) {
            s_a.boom = boom;
            if (!boom) {
                (void)tlv320aic3104_mic_enable(false);
            }
            ESP_LOGI(TAG, "microphone: %s", boom ? "boom (codec ADC)" : "built-in");
        } else {
            ESP_LOGW(TAG, "microphone switch failed: %s", esp_err_to_name(err));
            if (boom) {
                (void)tlv320aic3104_mic_enable(false);
            }
        }
    }
    xSemaphoreGive(s_a.mutex);
}

esp_err_t hs_audio_set_host_volume(bool mute, int16_t volume_db256)
{
    if (!s_a.codec_up) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    /* 1/256 dB -> 0.5 dB steps of attenuation, clamped to the codec's 63.5 dB and to the
     * hearing-safety floor (DES-HSA-003). */
    int32_t atten = (volume_db256 >= 0) ? 0 : (-(int32_t)volume_db256 + 64) / 128;
    if (atten < CONFIG_HS_AUDIO_MIN_ATTEN_HALF_DB) {
        atten = CONFIG_HS_AUDIO_MIN_ATTEN_HALF_DB;
    }
    if (atten > 127) {
        atten = 127;
    }
    esp_err_t err = tlv320aic3104_set_volume((uint8_t)atten);
    if (err == ESP_OK) {
        err = tlv320aic3104_set_mute(mute);
    }
    return err;
}

bool hs_audio_boom_active(void)
{
    return s_a.boom && s_a.capture_up;
}
