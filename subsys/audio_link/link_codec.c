/* Opus wrapper (DES-LNK-004, ADR-024). The only file that sees libopus. */
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "opus.h"
#include "link_priv.h"

static const char *TAG = "audio_link";

/* Codec state is touched on every frame: internal RAM, never PSRAM (ADR-014). */
#define CODEC_CAPS (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

esp_err_t link_enc_create(link_codec_t *c, uint32_t rate, uint8_t channels, uint32_t bitrate, uint32_t frame_us)
{
    memset(c, 0, sizeof(*c));
    int size = opus_encoder_get_size(channels);
    if (size <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    OpusEncoder *enc = heap_caps_malloc((size_t)size, CODEC_CAPS);
    if (enc == NULL) {
        ESP_LOGE(TAG, "encoder state: %d B internal unavailable", size);
        return ESP_ERR_NO_MEM;
    }
    /* RESTRICTED_LOWDELAY: CELT only, 2.5 ms look-ahead — the lowest algorithmic
     * delay Opus offers, and the right trade for a link with a latency budget. */
    int err = opus_encoder_init(enc, (opus_int32)rate, channels, OPUS_APPLICATION_RESTRICTED_LOWDELAY);
    if (err == OPUS_OK) {
        err = opus_encoder_ctl(enc, OPUS_SET_BITRATE((opus_int32)bitrate));
    }
    if (err == OPUS_OK) {
        /* CBR: every frame costs the same airtime, which is what the link's
         * connection interval is sized for (FW-LNK-016). */
        err = opus_encoder_ctl(enc, OPUS_SET_VBR(0));
    }
    if (err == OPUS_OK) {
        err = opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(CONFIG_AUDIO_LINK_OPUS_COMPLEXITY));
    }
    if (err != OPUS_OK) {
        ESP_LOGE(TAG, "encoder init %lu Hz x%u %lu bps: %s", (unsigned long)rate, channels,
                 (unsigned long)bitrate, opus_strerror(err));
        heap_caps_free(enc);
        return ESP_ERR_INVALID_ARG;
    }
    c->state = enc;
    c->state_bytes = (size_t)size;
    c->channels = channels;
    c->frame_samples = (uint16_t)((uint64_t)rate * frame_us / 1000000u);
    return ESP_OK;
}

esp_err_t link_dec_create(link_codec_t *c, uint32_t rate, uint8_t channels, uint32_t frame_us)
{
    memset(c, 0, sizeof(*c));
    int size = opus_decoder_get_size(channels);
    if (size <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    OpusDecoder *dec = heap_caps_malloc((size_t)size, CODEC_CAPS);
    if (dec == NULL) {
        ESP_LOGE(TAG, "decoder state: %d B internal unavailable", size);
        return ESP_ERR_NO_MEM;
    }
    int err = opus_decoder_init(dec, (opus_int32)rate, channels);
    if (err != OPUS_OK) {
        ESP_LOGE(TAG, "decoder init %lu Hz x%u: %s", (unsigned long)rate, channels, opus_strerror(err));
        heap_caps_free(dec);
        return ESP_ERR_INVALID_ARG;
    }
    c->state = dec;
    c->state_bytes = (size_t)size;
    c->channels = channels;
    c->frame_samples = (uint16_t)((uint64_t)rate * frame_us / 1000000u);
    return ESP_OK;
}

void link_codec_destroy(link_codec_t *c)
{
    if (c->state != NULL) {
        heap_caps_free(c->state);
    }
    memset(c, 0, sizeof(*c));
}

int link_encode(link_codec_t *c, const int16_t *pcm, uint8_t *out, size_t cap)
{
    return opus_encode((OpusEncoder *)c->state, pcm, c->frame_samples, out, (opus_int32)cap);
}

int link_decode(link_codec_t *c, const uint8_t *in, size_t len, int16_t *pcm)
{
    return opus_decode((OpusDecoder *)c->state, in, in ? (opus_int32)len : 0, pcm, c->frame_samples, 0);
}
