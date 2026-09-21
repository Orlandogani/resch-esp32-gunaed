/**
 * @file usb_device.h
 * @brief The one owner of the USB device stack: class-function registry, composite
 *        descriptor assembly, endpoint/FIFO budget enforcement, enumeration events,
 *        and the sleep lock that keeps the chip awake while a host is attached.
 *
 * Realises FW-USB-001..010, FW-USB-020..023 and ADR-007. Class functions
 * (`usb_hid`, `usb_audio`) never touch TinyUSB's device or descriptor API; they
 * register a `usb_function_t` here before `usb_device_start()` and implement
 * TinyUSB's *class* callbacks (`tud_hid_*`, `tud_audio_*`). This module implements
 * everything at the device level.
 *
 * ## Lifecycle
 *
 *     usb_device_init(cfg)
 *       └► usb_device_register(fn)   ← each class function, any order
 *            └► usb_device_start()   ← descriptor frozen, stack started, bus attached
 *                 └► usb_device_stop()
 *                      └► usb_device_deinit()
 *
 * Registration after `usb_device_start()` is refused; the descriptor is immutable
 * once the host has seen it.
 *
 * ## Hardware budget (CON-03, from TinyUSB's ESP32-S3 port)
 *
 * | Resource                      | Limit          |
 * | ----------------------------- | -------------- |
 * | Non-zero IN endpoints         | 4 (dedicated TX FIFOs) |
 * | Non-zero OUT endpoints        | 6              |
 * | Shared FIFO RAM               | 1024 bytes, holding the RX FIFO plus every TX FIFO |
 *
 * `usb_device_register()` rejects a function that would exceed these
 * (`FW-USB-003`), so a composite that cannot work is caught at init, not by an
 * assertion inside the driver at enumeration time.
 *
 * ## Console warning
 *
 * On the ESP32-S3 the OTG PHY and the built-in USB-Serial/JTAG share D+/D-.
 * `usb_device_start()` switches the PHY to OTG; a USJ console on the same connector
 * disappears and does not return until the board is re-plugged. Boards whose only
 * console is USJ must not start this module without a second path in.
 *
 * ## Thread and ISR safety
 *
 * All functions are safe from any task; none is ISR-safe. Event callbacks run in the
 * USB device task on `CONFIG_USB_DEVICE_TASK_CORE`; keep them short and non-blocking.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Hardware endpoint budget for the ESP32-S3 full-speed OTG device core. */
#define USB_DEVICE_MAX_IN_EP    4u
#define USB_DEVICE_MAX_OUT_EP   6u
#define USB_DEVICE_FIFO_BYTES   1024u
/** Bytes of FIFO RAM the core keeps for EP0 and the shared RX FIFO. */
#define USB_DEVICE_FIFO_RESERVED (64u + 256u)

/** Maximum interfaces a single composite configuration may declare. */
#define USB_DEVICE_MAX_INTERFACES 8u

typedef struct {
    uint16_t    vid;              /**< Vendor ID. 0 selects `CONFIG_USB_DEVICE_VID`.   */
    uint16_t    pid;              /**< Product ID. 0 selects `CONFIG_USB_DEVICE_PID`.  */
    uint16_t    bcd_device;       /**< Device release, e.g. 0x0100. 0 selects 0x0100. */
    const char *manufacturer;     /**< Static lifetime. NULL selects Kconfig default.  */
    const char *product;          /**< Static lifetime. NULL selects Kconfig default.  */
    const char *serial;           /**< Static lifetime. NULL derives from the eFuse MAC. */
    uint16_t    max_power_ma;     /**< bMaxPower. 0 selects 100 mA.                    */
    bool        self_powered;     /**< Sets the descriptor attribute and enables VBUS monitoring. */
    int         vbus_monitor_gpio;/**< GPIO sensing VBUS when self-powered; -1 if none. */
    bool        remote_wakeup;    /**< Advertise remote-wakeup capability.             */
} usb_device_config_t;

typedef enum {
    USB_DEVICE_EVT_MOUNTED = 0,   /**< Host configured the device (SET_CONFIGURATION). */
    USB_DEVICE_EVT_UNMOUNTED,     /**< Host detached, reset, or deconfigured.          */
    USB_DEVICE_EVT_SUSPENDED,     /**< Bus idle > 3 ms. Sleep lock released.           */
    USB_DEVICE_EVT_RESUMED,       /**< Resume signalling. Sleep lock re-acquired.      */
} usb_device_event_t;

typedef void (*usb_device_event_cb_t)(usb_device_event_t evt, void *ctx);

/** What a class function needs from the endpoint budget. */
typedef struct {
    uint8_t  in_endpoints;    /**< Non-zero IN endpoints.                         */
    uint8_t  out_endpoints;   /**< Non-zero OUT endpoints.                        */
    uint16_t tx_fifo_bytes;   /**< Sum of IN endpoint max packet sizes (FIFO RAM). */
} usb_ep_budget_t;

/**
 * @brief The class-function contract (DES-USB-001). One instance per registered
 *        function, with static lifetime — the pointer is stored, not copied.
 *
 * `descriptor_write()` is called once, during `usb_device_start()`, with the
 * interface number and endpoint addresses this function has been assigned. It must
 * write exactly `descriptor_len()` bytes. Endpoint addresses are handed out by the
 * device core so functions can never collide.
 */
typedef struct usb_function {
    const char *name;             /**< For logs. Static lifetime.                    */
    uint8_t     interface_count;  /**< Interfaces this function's descriptor declares. */

    /** Endpoint needs, evaluated at registration. */
    usb_ep_budget_t (*endpoint_request)(void *ctx);

    /** Exact byte length `descriptor_write()` will produce. */
    size_t (*descriptor_len)(void *ctx);

    /**
     * @param buf        Destination with at least `descriptor_len()` bytes free.
     * @param itf_base   First interface number assigned to this function.
     * @param ep_in      Array of `in_endpoints` IN addresses (0x8N), in order.
     * @param ep_out     Array of `out_endpoints` OUT addresses (0x0N), in order.
     * @return bytes written; must equal `descriptor_len()`.
     */
    size_t (*descriptor_write)(void *ctx, uint8_t *buf, uint8_t itf_base,
                               const uint8_t *ep_in, const uint8_t *ep_out);

    void (*on_mount)(void *ctx);                        /**< Optional. USB task context. */
    void (*on_unmount)(void *ctx);                      /**< Optional. Discard in-flight state. */
    void (*on_suspend)(void *ctx, bool remote_wakeup_en);/**< Optional. */
    void (*on_resume)(void *ctx);                       /**< Optional. */

    void *ctx;                                          /**< Passed to every callback. */
} usb_function_t;

/**
 * @brief Prepare the device core. Nothing touches the bus until `usb_device_start()`.
 *
 * Idempotent while not started. Creates the `"usb"` `pm_policy` lock.
 *
 * @param[in] cfg  May be NULL for all-Kconfig defaults.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  already started
 *   - (propagated)           `pm_policy_lock_create()` failure
 *
 * @note Thread-safety: not safe concurrently with itself, start, stop or deinit.
 */
esp_err_t usb_device_init(const usb_device_config_t *cfg);

/**
 * @brief Register a class function. Must precede `usb_device_start()`.
 *
 * @param[in] fn  Static-lifetime function descriptor. Stored by pointer.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before init, or after start
 *   - ESP_ERR_INVALID_ARG    NULL `fn`, missing mandatory callbacks, or
 *                            `interface_count` of 0
 *   - ESP_ERR_NO_MEM         would exceed the endpoint, FIFO, interface, or
 *                            function-count budget; logged at ERROR with the numbers
 *
 * @note Thread-safety: safe. ISR-safety: no. Blocking: never.
 */
esp_err_t usb_device_register(const usb_function_t *fn);

/**
 * @brief Assemble the composite descriptor, start the TinyUSB task, and attach to
 *        the bus. After this the descriptor set is frozen.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  before init, or already started
 *   - ESP_ERR_INVALID_SIZE   a function wrote a descriptor length different from
 *                            what it declared (programming error in that function)
 *   - (propagated)           `tinyusb_driver_install()` failure
 *
 * @note Thread-safety: not safe concurrently with init/stop/deinit.
 * @note Blocking: briefly (task creation).
 */
esp_err_t usb_device_start(void);

/**
 * @brief Detach from the bus and stop the stack. Registered functions remain
 *        registered; `usb_device_start()` may be called again.
 *
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if not started.
 */
esp_err_t usb_device_stop(void);

/**
 * @brief Stop if started, forget every registered function, release the sleep lock
 *        count, and return to the uninitialised state.
 *
 * @return ESP_OK, also if never initialised.
 */
esp_err_t usb_device_deinit(void);

/** @brief Whether the host has configured the device. false when not started. */
bool usb_device_is_mounted(void);

/**
 * @brief Install the application's event callback. May be called before or after
 *        start; NULL removes it.
 */
esp_err_t usb_device_set_event_cb(usb_device_event_cb_t cb, void *ctx);

/**
 * @brief Signal remote wakeup to a suspended host that enabled it.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not started or not suspended, or
 *         ESP_ERR_NOT_SUPPORTED if the host did not enable remote wakeup.
 */
esp_err_t usb_device_remote_wakeup(void);

/**
 * @brief The composite configuration descriptor as the host will see it.
 *
 * Before `usb_device_start()` this assembles it from the functions registered so
 * far (idempotent, no bus side effects), which makes the composite verifiable on
 * target without attaching to a host. After start it returns the frozen copy.
 *
 * @param[out] desc  Receives a pointer into module-owned storage. Valid until the
 *                   next registration or start.
 * @param[out] len   Total length in bytes.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE before init, ESP_ERR_INVALID_ARG on NULL,
 *         or the assembly errors documented for `usb_device_start()`.
 */
esp_err_t usb_device_config_descriptor(const uint8_t **desc, size_t *len);

/**
 * @brief Budget accounting for diagnostics: what registered functions have claimed.
 *
 * @param[out] used  May be NULL.
 * @param[out] max   May be NULL. The hardware limits.
 */
esp_err_t usb_device_budget(usb_ep_budget_t *used, usb_ep_budget_t *max);

#ifdef __cplusplus
}
#endif
