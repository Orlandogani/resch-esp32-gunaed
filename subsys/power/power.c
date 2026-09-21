#include "power.h"
#include <inttypes.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_system.h"
#include "driver/rtc_io.h"
#if SOC_USB_SERIAL_JTAG_SUPPORTED && !CONFIG_POWER_LIGHT_SLEEP_ALLOW_WITH_USJ
#include "driver/usb_serial_jtag.h"
#endif

#if CONFIG_PM_ENABLE && CONFIG_POWER_ENABLE_DFS
#include "esp_pm.h"
#endif

static const char *TAG = "power";

/* -------------------------------------------------------------------------- */
/* Retained block (DES-PWR-007). Lives in RTC slow memory; survives deep sleep. */
/* -------------------------------------------------------------------------- */

#define POWER_RETAINED_MAGIC   0x52544E53u /* "RTNS" */
#define POWER_RETAINED_VERSION 1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t boot_count;
    uint32_t payload_len;
    uint8_t  payload[POWER_RETAINED_PAYLOAD_BYTES];
    uint32_t crc32;   /* Over every field above, in order. Must stay last. */
} power_retained_t;

RTC_DATA_ATTR static power_retained_t s_retained;

/* -------------------------------------------------------------------------- */
/* Module state — ordinary RAM, reset every boot.                              */
/* -------------------------------------------------------------------------- */

static bool s_initialised;
static bool s_dfs_active;
static bool s_retained_valid;
static uint64_t s_ext1_mask;   /* Pins we enabled, so deinit can undo exactly those. */
static bool s_timer_armed;     /* esp_sleep logs an error if we disable an unarmed timer. */

static void timer_wake_set(uint64_t sleep_us)
{
    if (sleep_us > 0) {
        esp_sleep_enable_timer_wakeup(sleep_us);
        s_timer_armed = true;
    } else if (s_timer_armed) {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        s_timer_armed = false;
    }
}

static uint32_t retained_crc(const power_retained_t *b)
{
    return esp_rom_crc32_le(0, (const uint8_t *)b, offsetof(power_retained_t, crc32));
}

static void retained_validate_on_boot(void)
{
    esp_reset_reason_t reason = esp_reset_reason();

    /* A power-on or brownout reset means RTC memory content is not trustworthy
     * even if the magic happens to match (SyRS §3.6: invalid on cold boot). */
    if (reason == ESP_RST_POWERON || reason == ESP_RST_BROWNOUT) {
        s_retained_valid = false;
        memset(&s_retained, 0, sizeof(s_retained));
        return;
    }

    s_retained_valid = (s_retained.magic == POWER_RETAINED_MAGIC) &&
                       (s_retained.version == POWER_RETAINED_VERSION) &&
                       (s_retained.payload_len <= POWER_RETAINED_PAYLOAD_BYTES) &&
                       (retained_crc(&s_retained) == s_retained.crc32);

    if (s_retained_valid) {
        s_retained.boot_count++;
        s_retained.crc32 = retained_crc(&s_retained);
    } else if (s_retained.magic == POWER_RETAINED_MAGIC) {
        /* Magic present but CRC wrong: real corruption, worth a line in the log. */
        ESP_LOGW(TAG, "retained block failed CRC; discarding");
        memset(&s_retained, 0, sizeof(s_retained));
    }
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t power_init(void)
{
    if (s_initialised) {
        return ESP_OK;
    }

    esp_err_t err = ESP_OK;
    uint32_t causes = power_get_wakeup_causes();
    ESP_LOGI(TAG, "init: reset_reason=%d wakeup_causes=0x%08" PRIx32,
             (int)esp_reset_reason(), causes);

    retained_validate_on_boot();
    if (s_retained_valid) {
        ESP_LOGI(TAG, "retained block valid, boot_count=%" PRIu32, s_retained.boot_count);
    }

#if CONFIG_PM_ENABLE && CONFIG_POWER_ENABLE_DFS
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_POWER_MAX_FREQ_MHZ,
        .min_freq_mhz = CONFIG_POWER_MIN_FREQ_MHZ,
#if CONFIG_POWER_LIGHT_SLEEP_AUTO
        .light_sleep_enable = true,
#else
        .light_sleep_enable = false,
#endif
    };
    err = esp_pm_configure(&pm_config);
    if (err != ESP_OK) {
        /* Degraded, not fatal: the device runs at a fixed frequency (FW-SYS-025). */
        ESP_LOGW(TAG, "esp_pm_configure failed: %s (running at fixed frequency)",
                 esp_err_to_name(err));
    } else {
        s_dfs_active = true;
        ESP_LOGI(TAG, "DFS active: %u-%u MHz, auto light sleep %s",
                 CONFIG_POWER_MIN_FREQ_MHZ, CONFIG_POWER_MAX_FREQ_MHZ,
                 pm_config.light_sleep_enable ? "on" : "off");
    }
#endif

    s_ext1_mask = 0;
    s_initialised = true;
    return err;
}

esp_err_t power_deinit(void)
{
    if (!s_initialised) {
        return ESP_OK;
    }

    if (s_ext1_mask != 0) {
        esp_sleep_disable_ext1_wakeup_io(s_ext1_mask);
        s_ext1_mask = 0;
    }
    timer_wake_set(0);

#if CONFIG_PM_ENABLE && CONFIG_POWER_ENABLE_DFS
    if (s_dfs_active) {
        esp_pm_config_t fixed = {
            .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
            .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
            .light_sleep_enable = false,
        };
        esp_err_t err = esp_pm_configure(&fixed);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "restoring fixed frequency failed: %s", esp_err_to_name(err));
        }
        s_dfs_active = false;
    }
#endif

    s_initialised = false;
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Sleep                                                                       */
/* -------------------------------------------------------------------------- */

void power_enter_deep_sleep(uint64_t sleep_us)
{
    if (sleep_us > 0) {
        ESP_LOGI(TAG, "deep sleep for %" PRIu64 " us", sleep_us);
    } else {
        ESP_LOGI(TAG, "deep sleep, no timer (ext1 mask 0x%016" PRIx64 ")", s_ext1_mask);
    }
    timer_wake_set(sleep_us);
    esp_deep_sleep_start();
}

esp_err_t power_enter_light_sleep(uint64_t sleep_us, uint32_t *wakeup_causes)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
#if SOC_USB_SERIAL_JTAG_SUPPORTED && !CONFIG_POWER_LIGHT_SLEEP_ALLOW_WITH_USJ
    /* On the ESP32-S3 the USB-Serial/JTAG link does not survive light sleep: the pad
     * is disabled on entry and the host never re-enumerates. A developer whose only
     * console is USJ would lose the board until a physical replug. Refuse by default;
     * CONFIG_POWER_LIGHT_SLEEP_ALLOW_WITH_USJ opts in for production builds. */
    if (usb_serial_jtag_is_connected()) {
        ESP_LOGW(TAG, "light sleep refused: USB-Serial/JTAG host is connected "
                      "(set CONFIG_POWER_LIGHT_SLEEP_ALLOW_WITH_USJ to override)");
        return ESP_ERR_NOT_ALLOWED;
    }
#endif
    timer_wake_set(sleep_us);

    esp_err_t err = esp_light_sleep_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "light sleep rejected: %s", esp_err_to_name(err));
        return err;
    }
    if (wakeup_causes != NULL) {
        *wakeup_causes = power_get_wakeup_causes();
    }
    return ESP_OK;
}

uint32_t power_get_wakeup_causes(void)
{
    /* ESP-IDF sets BIT(ESP_SLEEP_WAKEUP_UNDEFINED) when the boot was not a wake
     * from sleep. Strip it so that zero means exactly "cold boot" (SYS-SYS-003). */
    return esp_sleep_get_wakeup_causes() & ~(1u << ESP_SLEEP_WAKEUP_UNDEFINED);
}

/* -------------------------------------------------------------------------- */
/* Wake sources                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t power_enable_gpio_wakeup(gpio_num_t pin, bool wake_on_high)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!rtc_gpio_is_valid_gpio(pin)) {
        ESP_LOGW(TAG, "GPIO %d is not RTC-capable; cannot be a deep-sleep wake source", (int)pin);
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t bit = 1ULL << pin;
    esp_err_t err = esp_sleep_enable_ext1_wakeup_io(
        bit, wake_on_high ? ESP_EXT1_WAKEUP_ANY_HIGH : ESP_EXT1_WAKEUP_ANY_LOW);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "enable gpio wakeup on pin %d failed: %s", (int)pin, esp_err_to_name(err));
        return err;
    }
    s_ext1_mask |= bit;
    ESP_LOGD(TAG, "ext1 wake: pin %d %s (mask 0x%016" PRIx64 ")",
             (int)pin, wake_on_high ? "high" : "low", s_ext1_mask);
    return ESP_OK;
}

esp_err_t power_disable_gpio_wakeup(gpio_num_t pin)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t bit = 1ULL << pin;
    esp_err_t err = esp_sleep_disable_ext1_wakeup_io(bit);
    if (err == ESP_OK) {
        s_ext1_mask &= ~bit;
    }
    return err;
}

/* -------------------------------------------------------------------------- */
/* Retained state                                                              */
/* -------------------------------------------------------------------------- */

bool power_retained_is_valid(void)
{
    return s_initialised && s_retained_valid;
}

uint32_t power_retained_boot_count(void)
{
    return power_retained_is_valid() ? s_retained.boot_count : 0;
}

esp_err_t power_retained_read(void *dst, size_t len)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dst == NULL || len > POWER_RETAINED_PAYLOAD_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_retained_valid) {
        return ESP_ERR_INVALID_CRC;
    }
    memcpy(dst, s_retained.payload, len);
    return ESP_OK;
}

esp_err_t power_retained_write(const void *src, size_t len)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((src == NULL && len > 0) || len > POWER_RETAINED_PAYLOAD_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }

    /* A fresh write starts a new boot-count epoch unless continuing a valid block. */
    uint32_t boot_count = s_retained_valid ? s_retained.boot_count : 0;

    memset(s_retained.payload, 0, sizeof(s_retained.payload));
    if (len > 0) {
        memcpy(s_retained.payload, src, len);
    }
    s_retained.magic = POWER_RETAINED_MAGIC;
    s_retained.version = POWER_RETAINED_VERSION;
    s_retained.boot_count = boot_count;
    s_retained.payload_len = (uint32_t)len;
    s_retained.crc32 = retained_crc(&s_retained);
    s_retained_valid = true;
    return ESP_OK;
}

esp_err_t power_retained_clear(void)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(&s_retained, 0, sizeof(s_retained));
    s_retained_valid = false;
    return ESP_OK;
}
