#include "audio_link_frame.h"
#include <string.h>

uint8_t audio_link_format_code(uint32_t sample_rate_hz, uint8_t channels, uint32_t frame_us)
{
    uint8_t rate;
    switch (sample_rate_hz) {
    case 8000:  rate = 0; break;
    case 12000: rate = 1; break;
    case 16000: rate = 2; break;
    case 24000: rate = 3; break;
    case 48000: rate = 4; break;
    default:    return 0xFF;
    }
    uint8_t dur;
    switch (frame_us) {
    case 2500:  dur = 0; break;
    case 5000:  dur = 1; break;
    case 10000: dur = 2; break;
    case 20000: dur = 3; break;
    default:    return 0xFF;
    }
    if (channels != 1 && channels != 2) {
        return 0xFF;
    }
    return (uint8_t)(rate | (channels == 2 ? 0x08u : 0u) | (uint8_t)(dur << 4));
}

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

size_t audio_link_frame_pack(const audio_link_frame_t *f, uint8_t *out, size_t cap)
{
    if (f == NULL || out == NULL) {
        return 0;
    }
    if (f->type != AUDIO_LINK_FRAME_AUDIO && f->type != AUDIO_LINK_FRAME_REPORT &&
        f->type != AUDIO_LINK_FRAME_CONTROL) {
        return 0;
    }
    if (f->payload_len > AUDIO_LINK_FRAME_MAX_PAYLOAD || (f->payload_len > 0 && f->payload == NULL)) {
        return 0;
    }
    if (f->type == AUDIO_LINK_FRAME_REPORT && f->payload_len != 0) {
        return 0;
    }
    size_t total = AUDIO_LINK_FRAME_HDR_LEN + f->payload_len;
    if (cap < total) {
        return 0;
    }
    bool audio = f->type == AUDIO_LINK_FRAME_AUDIO;
    out[0] = (uint8_t)((AUDIO_LINK_FRAME_VERSION << 4) | ((uint8_t)f->type & 0x0Fu));
    out[1] = audio ? f->format : 0;
    put_le16(&out[2], audio ? f->seq : 0);
    put_le16(&out[4], f->backlog_frames);
    if (f->payload_len > 0) {
        memcpy(&out[AUDIO_LINK_FRAME_HDR_LEN], f->payload, f->payload_len);
    }
    return total;
}

esp_err_t audio_link_frame_parse(const uint8_t *in, size_t len, audio_link_frame_t *out)
{
    if (in == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len < AUDIO_LINK_FRAME_HDR_LEN || len > AUDIO_LINK_FRAME_MAX_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }
    if ((in[0] >> 4) != AUDIO_LINK_FRAME_VERSION) {
        return ESP_ERR_INVALID_VERSION;
    }
    uint8_t type = in[0] & 0x0Fu;
    if (type != AUDIO_LINK_FRAME_AUDIO && type != AUDIO_LINK_FRAME_REPORT && type != AUDIO_LINK_FRAME_CONTROL) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    uint16_t plen = (uint16_t)(len - AUDIO_LINK_FRAME_HDR_LEN);
    if (type == AUDIO_LINK_FRAME_REPORT && plen != 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    out->type = (audio_link_frame_type_t)type;
    out->format = in[1];
    out->seq = get_le16(&in[2]);
    out->backlog_frames = get_le16(&in[4]);
    out->payload = plen ? &in[AUDIO_LINK_FRAME_HDR_LEN] : NULL;
    out->payload_len = plen;
    return ESP_OK;
}
