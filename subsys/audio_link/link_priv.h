/* Private interfaces between the audio_link core, its codec wrapper and its
 * transports. Not installed; nothing outside subsys/audio_link includes this. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "audio_link.h"

/* ---- Link task queue (DES-LNK-003) --------------------------------------- */

typedef enum {
    ITEM_FRAME = 0,     /* A received frame, copied out of the transport's context. */
    ITEM_EVENT,         /* A transport event, handled on the link task.             */
    ITEM_START,
    ITEM_STOP,
    ITEM_EXIT,
} link_item_kind_t;

/* Transport event codes carried by ITEM_EVENT. BLE events use their ble_event_t
 * value; these extend the range for events the transport synthesises itself. */
#define LINK_EVT_PEER_SUBSCRIBED   0x80u   /* Peripheral: central enabled the uplink CCCD. */
#define LINK_EVT_PEER_UNSUBSCRIBED 0x81u

typedef struct {
    uint8_t  kind;
    uint8_t  evt;
    uint8_t  status;
    uint8_t  addr_type;
    uint8_t  peer[6];
    uint16_t handle;
    uint16_t len;
    uint8_t  data[AUDIO_LINK_FRAME_MAX_LEN];
} link_item_t;

/* Core services the transports call. Both post functions are safe from any task
 * (they only enqueue) and never block. */
void link_core_post_frame(const uint8_t *data, size_t len);
void link_core_post_event(uint8_t evt, uint8_t status, const uint8_t *peer, uint8_t addr_type, uint16_t handle);
/* Link task only. */
void link_core_set_up(bool up);
/* Largest frame either direction will put on the air, in bytes. */
size_t link_core_max_frame_len(void);

/* ---- BLE transport (DES-LNK-006) ------------------------------------------ */

esp_err_t link_ble_start(const audio_link_config_t *cfg);
void      link_ble_stop(void);
esp_err_t link_ble_send(const uint8_t *frame, size_t len);
void      link_ble_handle_event(const link_item_t *it);   /* link task */
void      link_ble_poll(void);                            /* link task, every frame tick */

/* ---- Codec (DES-LNK-004) --------------------------------------------------- */

typedef struct {
    void    *state;          /* OpusEncoder* / OpusDecoder* */
    size_t   state_bytes;
    uint16_t frame_samples;  /* per channel */
    uint8_t  channels;
} link_codec_t;

esp_err_t link_enc_create(link_codec_t *c, uint32_t rate, uint8_t channels, uint32_t bitrate, uint32_t frame_us);
esp_err_t link_dec_create(link_codec_t *c, uint32_t rate, uint8_t channels, uint32_t frame_us);
void      link_codec_destroy(link_codec_t *c);
/* Returns the packet length, or < 0 on error. */
int       link_encode(link_codec_t *c, const int16_t *pcm, uint8_t *out, size_t cap);
/* `in` NULL → packet-loss concealment for one frame. Returns samples per channel, or < 0. */
int       link_decode(link_codec_t *c, const uint8_t *in, size_t len, int16_t *pcm);
