/**
 * On-target Unity tests for subsys/power (SDD §16.2).
 *
 * Deep-sleep entry is not tested here because it resets the chip. Light sleep is
 * exercised with a short timer; the retained-state block is tested for its
 * in-RAM semantics (validity, boot counter epoch, CRC-on-write). Survival across
 * an actual deep-sleep cycle is a manual verification item (SYS-TST-003).
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#include "driver/usb_serial_jtag.h"
#endif
#include "power.h"

static void fresh(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, power_deinit());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
}

/* ------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* ------------------------------------------------------------------------- */

TEST_CASE("calls before init return ESP_ERR_INVALID_STATE", "[power]")
{
    TEST_ASSERT_EQUAL(ESP_OK, power_deinit());
    uint8_t buf[4];
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, power_enable_gpio_wakeup(GPIO_NUM_0, false));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, power_disable_gpio_wakeup(GPIO_NUM_0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, power_enter_light_sleep(1000, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, power_retained_read(buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, power_retained_write(buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, power_retained_clear());
    TEST_ASSERT_FALSE(power_retained_is_valid());
    TEST_ASSERT_EQUAL(0, power_retained_boot_count());
}

TEST_CASE("init is idempotent; deinit then init works", "[power]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, power_deinit());
    TEST_ASSERT_EQUAL(ESP_OK, power_deinit()); /* also idempotent */
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
}

TEST_CASE("wakeup causes is a pure read and zero on a non-sleep boot", "[power]")
{
    fresh();
    /* The test runner is flashed and reset by the host, never woken from sleep. */
    TEST_ASSERT_EQUAL(0, power_get_wakeup_causes());
    TEST_ASSERT_EQUAL(power_get_wakeup_causes(), power_get_wakeup_causes());
}

/* ------------------------------------------------------------------------- */
/* Wake sources                                                               */
/* ------------------------------------------------------------------------- */

TEST_CASE("gpio wake: RTC-capable pins accepted, others rejected, additive mask", "[power]")
{
    fresh();
    /* On the ESP32-S3, GPIO0..21 are RTC-capable; GPIO38 is not. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, power_enable_gpio_wakeup(GPIO_NUM_38, false));

    TEST_ASSERT_EQUAL(ESP_OK, power_enable_gpio_wakeup(GPIO_NUM_1, false));
    TEST_ASSERT_EQUAL(ESP_OK, power_enable_gpio_wakeup(GPIO_NUM_2, false));
    /* Disabling one must not fail and must not disturb the other. */
    TEST_ASSERT_EQUAL(ESP_OK, power_disable_gpio_wakeup(GPIO_NUM_1));
    TEST_ASSERT_EQUAL(ESP_OK, power_disable_gpio_wakeup(GPIO_NUM_2));
    /* Deinit clears whatever remains without error. */
    TEST_ASSERT_EQUAL(ESP_OK, power_enable_gpio_wakeup(GPIO_NUM_3, true));
    TEST_ASSERT_EQUAL(ESP_OK, power_deinit());
}

/* ------------------------------------------------------------------------- */
/* Light sleep                                                                */
/* ------------------------------------------------------------------------- */

TEST_CASE("light sleep is refused while a USB-Serial/JTAG host is connected", "[power]")
{
    fresh();
#if SOC_USB_SERIAL_JTAG_SUPPORTED && !CONFIG_POWER_LIGHT_SLEEP_ALLOW_WITH_USJ
    if (usb_serial_jtag_is_connected()) {
        uint32_t causes = 0xFFFFFFFFu;
        TEST_ASSERT_EQUAL(ESP_ERR_NOT_ALLOWED, power_enter_light_sleep(50 * 1000, &causes));
        TEST_ASSERT_EQUAL(0xFFFFFFFFu, causes); /* not written on failure */
        return;
    }
#endif
    TEST_IGNORE_MESSAGE("no USJ host connected; guard not exercisable here");
}

TEST_CASE("light sleep with a timer returns after roughly that long", "[power][needs_uart_console]")
{
    fresh();
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    /* Light sleep kills the USB-Serial/JTAG link (see power.h). Only run this when
     * the console is a real UART, or the board is lost until physically re-plugged. */
    if (usb_serial_jtag_is_connected()) {
        TEST_IGNORE_MESSAGE("USB-Serial/JTAG host connected; light sleep would sever it");
    }
#endif
    const uint64_t want_us = 50 * 1000;
    uint32_t causes = 0;

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = power_enter_light_sleep(want_us, &causes);
    int64_t dt = esp_timer_get_time() - t0;

    /* Some boards/configs refuse light sleep (e.g. an active console UART). That
     * is a propagated, documented failure, not a test failure of this module. */
    if (err == ESP_OK) {
        TEST_ASSERT_TRUE(dt >= (int64_t)want_us - 5000);
        TEST_ASSERT_TRUE(dt < (int64_t)want_us + 100000);
        TEST_ASSERT_TRUE(causes & (1u << ESP_SLEEP_WAKEUP_TIMER));
    } else {
        TEST_IGNORE_MESSAGE("light sleep rejected on this configuration");
    }
}

/* ------------------------------------------------------------------------- */
/* Retained state                                                             */
/* ------------------------------------------------------------------------- */

TEST_CASE("retained block: invalid until written, then readable, then clearable", "[power]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, power_retained_clear());
    TEST_ASSERT_FALSE(power_retained_is_valid());

    uint8_t out[8] = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_CRC, power_retained_read(out, sizeof(out)));

    const uint8_t in[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    TEST_ASSERT_EQUAL(ESP_OK, power_retained_write(in, sizeof(in)));
    TEST_ASSERT_TRUE(power_retained_is_valid());
    TEST_ASSERT_EQUAL(0, power_retained_boot_count()); /* fresh epoch */

    TEST_ASSERT_EQUAL(ESP_OK, power_retained_read(out, sizeof(out)));
    TEST_ASSERT_EQUAL_MEMORY(in, out, sizeof(in));

    /* Bytes beyond what was written read as zero. */
    uint8_t wide[16];
    memset(wide, 0xAA, sizeof(wide));
    TEST_ASSERT_EQUAL(ESP_OK, power_retained_read(wide, sizeof(wide)));
    TEST_ASSERT_EQUAL_MEMORY(in, wide, 8);
    for (int i = 8; i < 16; i++) {
        TEST_ASSERT_EQUAL_UINT8(0, wide[i]);
    }

    TEST_ASSERT_EQUAL(ESP_OK, power_retained_clear());
    TEST_ASSERT_FALSE(power_retained_is_valid());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_CRC, power_retained_read(out, sizeof(out)));
}

TEST_CASE("retained block: argument validation", "[power]")
{
    fresh();
    uint8_t buf[POWER_RETAINED_PAYLOAD_BYTES + 1];
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, power_retained_read(NULL, 4));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, power_retained_read(buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, power_retained_write(NULL, 4));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, power_retained_write(buf, sizeof(buf)));
    /* NULL with len 0 is an all-zero payload, which is legal. */
    TEST_ASSERT_EQUAL(ESP_OK, power_retained_write(NULL, 0));
    TEST_ASSERT_TRUE(power_retained_is_valid());
    /* Full-size payload is legal. */
    TEST_ASSERT_EQUAL(ESP_OK, power_retained_write(buf, POWER_RETAINED_PAYLOAD_BYTES));
}

TEST_CASE("retained block: rewrite while valid keeps the boot-count epoch", "[power]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, power_retained_clear());
    uint32_t v = 1;
    TEST_ASSERT_EQUAL(ESP_OK, power_retained_write(&v, sizeof(v)));
    uint32_t epoch = power_retained_boot_count();
    v = 2;
    TEST_ASSERT_EQUAL(ESP_OK, power_retained_write(&v, sizeof(v)));
    TEST_ASSERT_EQUAL(epoch, power_retained_boot_count());
    uint32_t back = 0;
    TEST_ASSERT_EQUAL(ESP_OK, power_retained_read(&back, sizeof(back)));
    TEST_ASSERT_EQUAL(2, back);
}

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
