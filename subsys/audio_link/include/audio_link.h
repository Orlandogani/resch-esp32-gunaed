/**
 * @file audio_link.h
 * @brief Wireless duplex audio link between two ESP32-S3s: PCM ring in, Opus over
 *        a transport, PCM ring out, in both directions at once.
 *
 * Realises FW-LNK-001..009 and FW-LNK-014..031 per ADR-022. The link is proprietary —
 * both ends run this module — because the S3 has neither BR/EDR (no A2DP/HFP) nor
 * BLE ISO channels (no LE Audio). It is the mechanism a dongle and a headset share
 * from opposite sides; which profile runs, what the audio is and what control
 * messages mean stay with the application (ADR-013).
 *
 * ## Shape
 *
 *     this end                                                 far end
 *     tx.ring ──► encode ──► frame ──► transport ═══════════► parse ──► decode ──► rx.ring
 *     rx.ring ◄── decode ◄── parse ◄── transport ◄═══════════ frame ◄── encode ◄── tx.ring
 *
 * PCM at every boundary; the codec never leaves this module, so callers never see
 * compressed frames. `rx.ring` is written by this module alone (it is the ring's
 * single writer); `tx.ring` is read through this module's own cursor, so it can be
 * shared with other readers.
 *
 * ## Roles and transports
 *
 * - `AUDIO_LINK_ROLE_CENTRAL` (the dongle) scans for the link service, connects,
 *   and is the GATT client: it writes downlink frames without response and
 *   receives uplink frames as notifications.
 * - `AUDIO_LINK_ROLE_PERIPHERAL` (the headset) advertises the link service and is
 *   the GATT server.
 * - `AUDIO_LINK_TRANSPORT_LOOPBACK` delivers every frame this end sends straight
 *   back to its own receive path. It exists so the whole pipeline — codec, framing,
 *   sequencing, concealment, reporting — can be verified on one board (SDD §16).
 *
 * Over BLE this module owns `subsys/ble` for its lifetime: it calls `ble_init()` in
 * `audio_link_start()` and `ble_deinit()` in `audio_link_stop()`.
 *
 * ## Loss, lateness and concealment
 *
 * Every audio frame carries a sequence number. A gap is concealed with the codec's
 * packet-loss concealment, one frame per missing frame (up to
 * `CONFIG_AUDIO_LINK_MAX_CONCEAL_FRAMES`, beyond which the stream resynchronises);
 * a frame older than the next expected one is dropped. When frames stop arriving
 * altogether and the downstream backlog falls under
 * `CONFIG_AUDIO_LINK_CONCEAL_LOW_WATER_MS` (or, with no `rx_backlog_cb`, a frame is
 * two and a half periods overdue), a frame is concealed on the frame clock so the
 * consumer never runs dry because the radio was busy retransmitting.
 *
 * ## Drift: the remote backlog report (ADR-022)
 *
 * The two ends run on different crystals, and the ESP32-S3 has no audio PLL to trim
 * an I2S clock. Rather than resample, the receiving end reports how much audio it is
 * holding downstream of `rx.ring` (`rx_backlog_cb`) in every frame it sends, and the
 * sending end exposes `audio_link_path_backlog_bytes()`: what it has not yet sent
 * plus what the far end last reported, in its own `tx` format. A dongle hands that to
 * `subsys/usb_audio` as the speaker backlog, and the existing asynchronous-sink
 * feedback servo (ADR-021) makes the host send at the headset's I2S rate.
 *
 * ## Thread and ISR safety
 *
 * `init/deinit/start/stop` are not safe concurrently with each other. `stats`,
 * `is_up`, `path_backlog_bytes`, `peer_backlog` and `send_control` are safe from any
 * task. Nothing is ISR-safe. Callbacks run on this module's task and must not block
 * or call the lifecycle functions.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "ringbuf.h"
#include "audio_link_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AUDIO_LINK_ROLE_CENTRAL = 0,    /**< Scans and connects; GATT client (dongle).     */
    AUDIO_LINK_ROLE_PERIPHERAL,     /**< Advertises; GATT server (headset).            */
} audio_link_role_t;

typedef enum {
    AUDIO_LINK_TRANSPORT_BLE = 0,   /**< Bluetooth LE via subsys/ble (ADR-022).        */
    AUDIO_LINK_TRANSPORT_LOOPBACK,  /**< Own frames back to own receive path (tests).  */
} audio_link_transport_t;

/** One direction's PCM format and ring. 16-bit interleaved PCM only. */
typedef struct {
    ringbuf_t *ring;                /**< NULL disables the direction at this end.      */
    uint32_t   sample_rate_hz;      /**< 8000, 12000, 16000, 24000 or 48000.           */
    uint8_t    channels;            /**< 1 or 2.                                       */
    uint32_t   bitrate_bps;         /**< tx only. 0 → CONFIG_AUDIO_LINK_BITRATE_STEREO /
                                         _MONO by channel count.                       */
} audio_link_stream_t;

typedef enum {
    AUDIO_LINK_EVT_UP = 0,          /**< Link ready; frames flow from now on.          */
    AUDIO_LINK_EVT_DOWN,            /**< Link lost or stopped.                          */
    AUDIO_LINK_EVT_FORMAT_MISMATCH, /**< Peer's audio format differs from `rx`; its
                                         frames are dropped. Reported once per link-up. */
} audio_link_event_t;

typedef void (*audio_link_event_cb_t)(audio_link_event_t evt, void *ctx);

/** CONTROL frame from the far end. `msg` is valid only during the call. */
typedef void (*audio_link_control_cb_t)(const uint8_t *msg, size_t len, void *ctx);

/**
 * @brief Bytes of this end's received audio still queued downstream of `rx.ring` —
 *        for a headset, `audio_playback_stats().source_backlog_bytes`. Called once
 *        per frame from this module's task; must be cheap and must not block.
 */
typedef uint32_t (*audio_link_backlog_cb_t)(void *ctx);

typedef struct {
    audio_link_role_t      role;
    audio_link_transport_t transport;
    audio_link_stream_t    tx;           /**< What this end encodes and sends.         */
    audio_link_stream_t    rx;           /**< What this end receives and decodes.      */
    audio_link_backlog_cb_t rx_backlog_cb; /**< Optional; enables reporting and
                                              low-water concealment. See above.        */
    audio_link_event_cb_t  on_event;     /**< Optional.                                 */
    audio_link_control_cb_t on_control;  /**< Optional.                                 */
    void                  *ctx;          /**< Passed to every callback.                 */
    const char            *device_name;  /**< BLE name; NULL → CONFIG_BLE_DEVICE_NAME.  */
    const uint8_t         *peer_addr;    /**< Central: connect only to this address (6
                                              bytes, static lifetime). NULL → the first
                                              advertiser of the link service.            */
} audio_link_config_t;

typedef struct {
    bool     up;
    uint32_t link_ups;
    uint32_t link_downs;

    uint32_t tx_frames;             /**< Audio frames sent.                             */
    uint32_t tx_bytes;              /**< Frame bytes sent, all types.                   */
    uint32_t tx_busy;               /**< Transport refused a frame (no buffer); dropped. */
    uint32_t tx_skipped;            /**< Frames discarded because tx backlog exceeded
                                         CONFIG_AUDIO_LINK_TX_MAX_BACKLOG_MS.           */
    uint32_t tx_reports;            /**< REPORT frames sent.                            */
    uint32_t tx_encode_errors;

    uint32_t rx_frames;             /**< Audio frames decoded.                          */
    uint32_t rx_bytes;
    uint32_t rx_lost;               /**< Frames missing from the sequence.              */
    uint32_t rx_late;               /**< Frames older than expected; dropped.           */
    uint32_t rx_concealed;          /**< Frames synthesised by packet-loss concealment. */
    uint32_t rx_resyncs;            /**< Gaps too long to conceal.                      */
    uint32_t rx_decode_errors;
    uint32_t rx_format_mismatch;
    uint32_t rx_bad_frames;         /**< Failed to parse (size, version, type).         */
    uint32_t rx_queue_overflow;     /**< Arrived faster than the task drained them.     */
    uint32_t rx_control;            /**< CONTROL frames delivered.                      */

    uint16_t peer_backlog_frames;   /**< Last backlog the far end reported, or UNKNOWN. */
    uint32_t peer_report_age_ms;    /**< Since that report. UINT32_MAX if never.        */

    uint32_t encode_us_max;         /**< Worst codec call since init.                   */
    uint32_t encode_us_avg;         /**< Exponential moving average, 1/16.              */
    uint32_t decode_us_max;
    uint32_t decode_us_avg;
    uint32_t codec_state_bytes;     /**< Heap held by encoder + decoder state.          */
    uint32_t task_stack_free_min;   /**< Stack high-water mark of the link task, bytes. */
} audio_link_stats_t;

/**
 * @brief Validate the configuration, create the codec states and the link task.
 *        Nothing is transmitted until `audio_link_start()`.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  already initialised
 *   - ESP_ERR_INVALID_ARG    NULL cfg, both directions disabled, an unsupported
 *                            rate/channel count, or a bitrate whose frame would not
 *                            fit AUDIO_LINK_FRAME_MAX_PAYLOAD
 *   - ESP_ERR_NO_MEM         codec state, buffers or task
 */
esp_err_t audio_link_init(const audio_link_config_t *cfg);

/**
 * @brief Bring the transport up (BLE: initialise, then advertise or scan) and start
 *        framing. `AUDIO_LINK_EVT_UP` follows when the far end is ready — at once
 *        for loopback.
 * @return ESP_OK, ESP_ERR_INVALID_STATE, or a propagated transport error.
 * @note Blocking: yes, for the BLE bring-up (hundreds of ms).
 */
esp_err_t audio_link_start(void);

/** @brief Stop framing and take the transport down. Idempotent. */
esp_err_t audio_link_stop(void);

/** @brief Stop if started, delete the task, free the codec states. */
esp_err_t audio_link_deinit(void);

/** @brief True between AUDIO_LINK_EVT_UP and AUDIO_LINK_EVT_DOWN. */
bool audio_link_is_up(void);

/**
 * @brief Send application bytes to the far end as one CONTROL frame.
 * @return ESP_OK, ESP_ERR_INVALID_STATE (link not up), ESP_ERR_INVALID_ARG,
 *         ESP_ERR_INVALID_SIZE (> AUDIO_LINK_FRAME_MAX_PAYLOAD), ESP_ERR_NO_MEM
 *         (transport busy: retry), or a propagated transport error.
 */
esp_err_t audio_link_send_control(const void *msg, size_t len);

/**
 * @brief The rate servo's measurement for the `tx` direction: bytes of `tx` audio
 *        this end has not yet encoded, plus the far end's last reported receive
 *        backlog converted to `tx` bytes. Unknown far-end backlog counts as zero.
 *        Cheap; safe from any task, including a 1 ms USB feeder.
 */
uint32_t audio_link_path_backlog_bytes(void);

/**
 * @brief Last receive backlog the far end reported, in sample frames, and its age.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_NOT_FOUND (no report yet).
 */
esp_err_t audio_link_peer_backlog(uint16_t *frames, uint32_t *age_ms);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL; zeros before init. */
esp_err_t audio_link_stats(audio_link_stats_t *out);

#if CONFIG_AUDIO_LINK_TEST_HOOKS
/**
 * @brief Test only: the loopback transport discards the next `n` audio frames it
 *        is given, so loss handling can be exercised deterministically.
 */
void audio_link_test_drop_next(uint32_t n);

/** @brief Test only: the loopback transport holds back the next frame and delivers
 *         it after the following one, i.e. out of order. */
void audio_link_test_swap_next(void);
#endif

#ifdef __cplusplus
}
#endif
