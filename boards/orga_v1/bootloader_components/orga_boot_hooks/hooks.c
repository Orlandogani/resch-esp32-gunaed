/*
 * ORGA v1 bootloader hook: assert PWR_HOLD (GPIO21) as the very first thing.
 *
 * The board's 3.3 V regulator is enabled through a diode-OR of the POWER button and
 * PWR_HOLD; PWR_HOLD has a 100 k pull-down. At power-on the user is holding the button,
 * and the application's drivers/power_latch would take ~0.5-1 s (PSRAM test included) to
 * assert the pin. Driving it here, before the bootloader has even initialised, shrinks the
 * window the user must hold the button to tens of milliseconds (DES-LAT-003).
 *
 * After a warm reset (panic, watchdog, OTA reboot) the application's pad hold is still
 * latched, these writes do not reach the pad, and nothing changes.
 *
 * Only header-only LL calls: BSS, flash and most of the system are not initialised yet.
 */
#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"

#define ORGA_PWR_HOLD_GPIO 21

/* Referenced so the linker keeps this file: the bootloader's hooks are weak. */
void bootloader_hooks_include(void)
{
}

void bootloader_before_init(void)
{
    gpio_ll_set_level(&GPIO, ORGA_PWR_HOLD_GPIO, 1);
    esp_rom_gpio_pad_select_gpio(ORGA_PWR_HOLD_GPIO);
    gpio_ll_pulldown_dis(&GPIO, ORGA_PWR_HOLD_GPIO);
    gpio_ll_output_enable(&GPIO, ORGA_PWR_HOLD_GPIO);
    gpio_ll_set_level(&GPIO, ORGA_PWR_HOLD_GPIO, 1);
}
