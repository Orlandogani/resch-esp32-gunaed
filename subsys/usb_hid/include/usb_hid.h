/**
 * @file usb_hid.h
 * @brief USB HID class function: caller-supplied report descriptor, non-blocking
 *        input-report queue serviced every 1 ms frame, output reports delivered to
 *        the application in task context.
 *
 * Realises FW-HID-001..011. Registers itself with `usb_device` (ADR-007); it never
 * touches TinyUSB's device API directly. The interrupt IN endpoint is declared with
 * `bInterval = 1` — one service opportunity per full-speed frame, 1000 Hz, which is
 * the hardware floor on the ESP32-S3 (CON-02, SYS-HID-001).
 *
 * ## Report path
 *
 *     usb_hid_report_send() ──► queue (CONFIG_USB_HID_REPORT_QUEUE_LEN) ──► IN endpoint
 *                                                                          │
 *                        next report armed on completion ◄─────────────────┘
 *
 * `usb_hid_report_send()` never blocks: a full queue returns ESP_ERR_NO_MEM and the
 * report is counted as dropped. When the queue is empty the endpoint NAKs; a stale
 * report is never re-sent (FW-HID-006).
 *
 * ## Policy stays with the application (ADR-013)
 *
 * The SDK has no idea what a report means. The application supplies the report
 * descriptor, decides when to send, and interprets output reports.
 *
 * ## Thread and ISR safety
 *
 * `usb_hid_report_send()` and `usb_hid_stats()` are safe from any task; none of this
 * module is ISR-safe. Callbacks run in the USB device task.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Full-speed interrupt endpoint maximum, and therefore the largest report. */
#define USB_HID_EP_SIZE_MAX 64u

/** HID report types, numerically identical to the HID 1.11 specification. */
typedef enum {
    USB_HID_REPORT_INPUT   = 1,
    USB_HID_REPORT_OUTPUT  = 2,
    USB_HID_REPORT_FEATURE = 3,
} usb_hid_report_type_t;

/** Boot-interface protocol advertised in the HID interface descriptor. */
typedef enum {
    USB_HID_PROTOCOL_NONE     = 0,
    USB_HID_PROTOCOL_KEYBOARD = 1,
    USB_HID_PROTOCOL_MOUSE    = 2,
} usb_hid_boot_protocol_t;

/**
 * @brief Host → device report (SET_REPORT or OUT data). USB device task context;
 *        return quickly. `data` is valid only for the duration of the call.
 */
typedef void (*usb_hid_output_cb_t)(uint8_t report_id, usb_hid_report_type_t type,
                                    const uint8_t *data, size_t len, void *ctx);

/**
 * @brief Host GET_REPORT request. Fill `buf` (capacity `reqlen`) and return the
 *        number of bytes written, or 0 to stall the request. USB device task context.
 */
typedef uint16_t (*usb_hid_get_report_cb_t)(uint8_t report_id, usb_hid_report_type_t type,
                                            uint8_t *buf, uint16_t reqlen, void *ctx);

typedef struct {
    const uint8_t          *report_descriptor;      /**< Required. Static lifetime (DES-HID-001). */
    size_t                  report_descriptor_len;  /**< Required. */
    uint8_t                 ep_size;                /**< 1..64. 0 selects `CONFIG_USB_HID_EP_SIZE`. */
    uint8_t                 poll_interval_ms;       /**< bInterval, 1..255. 0 selects Kconfig (1). */
    usb_hid_boot_protocol_t boot_protocol;          /**< Usually NONE unless BIOS boot support is wanted. */
    usb_hid_output_cb_t     on_output_report;       /**< Optional. */
    usb_hid_get_report_cb_t on_get_report;          /**< Optional; absent → GET_REPORT stalls. */
    void                   *ctx;                    /**< Passed to both callbacks. */
} usb_hid_config_t;

typedef struct {
    uint32_t reports_sent;        /**< Completed IN transfers.                           */
    uint32_t reports_dropped;     /**< `usb_hid_report_send()` refused: queue full.      */
    uint32_t reports_rejected;    /**< Refused: not mounted or interface not active.     */
    uint32_t output_reports;      /**< Host → device reports delivered to the callback. */
    uint8_t  queue_high_water;    /**< Deepest the queue has been since reset.          */
    uint8_t  queue_depth;         /**< Current occupancy.                                */
} usb_hid_stats_t;

/**
 * @brief Register the HID function with `usb_device`. Call after `usb_device_init()`
 *        and before `usb_device_start()`.
 *
 * @param[in] cfg  Required. The report descriptor pointer is stored, not copied.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  already initialised, or `usb_device` not initialised /
 *                            already started
 *   - ESP_ERR_INVALID_ARG    NULL cfg or descriptor, zero length, `ep_size` > 64,
 *                            or `poll_interval_ms` out of 1..255
 *   - ESP_ERR_NO_MEM         `usb_device` refused the endpoint budget (FW-USB-003)
 *
 * @note Thread-safety: not safe concurrently with itself or `usb_hid_deinit()`.
 */
esp_err_t usb_hid_init(const usb_hid_config_t *cfg);

/**
 * @brief Drop the queue and forget the configuration. `usb_device` keeps its
 *        registration until `usb_device_deinit()`; call that first for a clean
 *        rebuild of the composite device.
 */
esp_err_t usb_hid_deinit(void);

/**
 * @brief Queue an input report for the next IN token. Never blocks (DES-HID-003).
 *
 * If the endpoint is idle the report is armed immediately; otherwise it is sent
 * when the in-flight one completes. Latency to the wire is bounded by one poll
 * interval plus one frame (SYS-HID-003).
 *
 * @param[in] report_id  Report ID, or 0 if the descriptor uses none.
 * @param[in] data       Report payload.
 * @param[in] len        1..ep_size bytes. (With a non-zero report ID the ID byte is
 *                       prepended on the wire, so `len` must be <= ep_size - 1.)
 *
 * @return
 *   - ESP_OK                  queued
 *   - ESP_ERR_INVALID_STATE   not initialised, not mounted, or interface not active;
 *                             counted in `reports_rejected`
 *   - ESP_ERR_INVALID_ARG     NULL `data` or `len` == 0
 *   - ESP_ERR_INVALID_SIZE    `len` exceeds the endpoint size
 *   - ESP_ERR_NO_MEM          queue full; counted in `reports_dropped`
 *
 * @note Thread-safety: safe from any task; concurrent sends never interleave a
 *       report's bytes (FW-HID-011). ISR-safety: no. Blocking: never.
 */
esp_err_t usb_hid_report_send(uint8_t report_id, const void *data, size_t len);

/** @brief Whether the host has configured the device and the HID interface is active. */
bool usb_hid_ready(void);

/** @brief Snapshot the counters. ESP_ERR_INVALID_ARG on NULL. */
esp_err_t usb_hid_stats(usb_hid_stats_t *out);

/** @brief Zero the counters. */
esp_err_t usb_hid_stats_reset(void);

#ifdef __cplusplus
}
#endif
