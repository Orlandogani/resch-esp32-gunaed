/**
 * @file host_port.h
 * @brief The dongle's host side: a duplex USB audio function with HID consumer
 *        control — or, in the console-safe build, an emulation of that host.
 *
 * `CONFIG_DONGLE_ENABLE_USB=y`: `subsys/usb_device` + `usb_audio` (UAC2 duplex,
 * asynchronous-sink feedback servo, ADR-021) + `usb_hid`. Starting it takes the
 * USB PHY from the USB-Serial/JTAG console (CLAUDE.md hard rule 1).
 *
 * `CONFIG_DONGLE_ENABLE_USB=n`: a 1 ms timer plays the host. It writes a 1 kHz stereo
 * tone into the speaker ring at the nominal rate trimmed by ±1 sample per
 * millisecond from the speaker-path backlog — the same authority and the same
 * measurement the real feedback endpoint gives a real host — and drains the mic
 * ring at the nominal rate, measuring its level. The whole dongle, radio included,
 * then runs with its console intact, which is how the link is developed and soaked.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ringbuf_t *speaker_ring;          /**< host → dongle. Written here (single writer).   */
    ringbuf_t *mic_ring;              /**< dongle → host. Read here, own cursor.          */
    uint32_t   speaker_rate_hz;
    uint8_t    speaker_channels;
    uint32_t   mic_rate_hz;           /**< Mono.                                           */
    uint16_t   speaker_target_ms;     /**< Servo setpoint for the speaker-path backlog.    */
    /** The speaker-path backlog in bytes, from the active backend. Every 1 ms. */
    uint32_t (*speaker_backlog_cb)(void *ctx);
    /** Host changed mute/volume (USB only). Optional. */
    void (*on_volume)(bool mic, bool mute, int16_t volume_db256, void *ctx);
    void      *ctx;
} host_port_config_t;

/** @brief Bring the host side up. The backend must already be reading the speaker ring. */
esp_err_t host_port_start(const host_port_config_t *cfg);

/** @brief Take it down. Idempotent. */
esp_err_t host_port_stop(void);

/** @brief One HID consumer tap (press, then release). Safe from any task; dropped if
 *         the host has not enumerated. */
void host_port_hid_tap(uint8_t bits);

/** @brief Whether this build drives real USB. */
bool host_port_is_usb(void);

/** @brief One log line of host-side counters. */
void host_port_log_stats(void);

#ifdef __cplusplus
}
#endif
