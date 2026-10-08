/**
 * Headset user interface: board inputs to events, and the LED pattern engine.
 */
#include "hs_ui.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "board.h"
#include "buttons.h"
#include "encoder.h"
#include "rgb_led.h"

static const char *TAG = "hs_ui";

#if CONFIG_HS_UI_WHEEL_REVERSE
#define WHEEL_REVERSE true
#else
#define WHEEL_REVERSE false
#endif

#define ROLE_JACK BOARD_BTN_ROLE_COUNT   /* the jack rides drivers/buttons as one more input */
#define MAX_INPUTS 8

static struct {
    hs_input_cb_t cb;
    void         *ctx;
    uint8_t       role_of[MAX_INPUTS];     /* buttons index -> role (or ROLE_JACK)  */
    int8_t        power_index;             /* -1: no power switch                   */
    bool          power_long;              /* this press reached the long press     */
    bool          power_fired;             /* HS_IN_POWER_HOLD sent for this press  */
    bool          have_led;
    hs_led_t      steady;
    hs_led_t      shown;
    hs_led_t      flash;
    bool          phase_on;
    int64_t       flash_until_us;
    esp_timer_handle_t timer;
    portMUX_TYPE  lock;
} s_ui = { .power_index = -1, .lock = portMUX_INITIALIZER_UNLOCKED };

/* -------------------------------------------------------------------------- */
/* Inputs                                                                      */
/* -------------------------------------------------------------------------- */

static void emit(hs_input_t in)
{
    if (s_ui.cb != NULL) {
        s_ui.cb(in, s_ui.ctx);
    }
}

static void on_button(buttons_event_t evt, const buttons_event_info_t *info, void *ctx)
{
    (void)ctx;
    if (info->index >= MAX_INPUTS) {
        return;
    }
    switch (s_ui.role_of[info->index]) {
    case BOARD_BTN_ACTION:
        if (evt == BUTTONS_EVENT_PRESSED) {
            emit(HS_IN_ACTION_PRESS);
        } else if (evt == BUTTONS_EVENT_LONG_PRESS) {
            emit(HS_IN_ACTION_LONG);
        }
        break;
    case BOARD_BTN_VOL_UP:
        if (evt == BUTTONS_EVENT_PRESSED || evt == BUTTONS_EVENT_REPEAT) {
            emit(HS_IN_VOL_UP);
        }
        break;
    case BOARD_BTN_VOL_DOWN:
        if (evt == BUTTONS_EVENT_PRESSED || evt == BUTTONS_EVENT_REPEAT) {
            emit(HS_IN_VOL_DOWN);
        }
        break;
    case BOARD_BTN_POWER:
        if (evt == BUTTONS_EVENT_PRESSED) {
            s_ui.power_long = false;
            s_ui.power_fired = false;
        } else if (evt == BUTTONS_EVENT_LONG_PRESS || evt == BUTTONS_EVENT_REPEAT) {
            s_ui.power_long = true;
            if (!s_ui.power_fired && info->held_ms >= CONFIG_HS_UI_POWER_OFF_HOLD_MS) {
                s_ui.power_fired = true;
                emit(HS_IN_POWER_HOLD);
            }
        } else if (evt == BUTTONS_EVENT_RELEASED && !s_ui.power_long) {
            emit(HS_IN_POWER_SHORT);
        }
        break;
    case ROLE_JACK:
        if (evt == BUTTONS_EVENT_PRESSED) {
            emit(HS_IN_JACK_IN);
        } else if (evt == BUTTONS_EVENT_RELEASED) {
            emit(HS_IN_JACK_OUT);
        }
        break;
    default:
        break;
    }
}

static void on_wheel(int32_t steps, void *ctx)
{
    (void)ctx;
    hs_input_t in = steps > 0 ? HS_IN_VOL_UP : HS_IN_VOL_DOWN;
    int32_t n = steps > 0 ? steps : -steps;
    if (n > CONFIG_HS_UI_WHEEL_MAX_STEPS) {
        n = CONFIG_HS_UI_WHEEL_MAX_STEPS;   /* a flick is a few steps, not a jump to mute */
    }
    while (n-- > 0) {
        emit(in);
    }
}

/* -------------------------------------------------------------------------- */
/* LED (DES-HSU-003)                                                           */
/* -------------------------------------------------------------------------- */

typedef struct {
    rgb_led_color_t color;
    uint16_t on_ms;      /* 0: steady */
    uint16_t off_ms;
    uint16_t fade_ms;
} pattern_t;

static const pattern_t k_patterns[HS_LED_PATTERN_COUNT] = {
    [HS_LED_OFF]         = { { 0, 0, 0 },       0,    0,    150 },
    [HS_LED_BOOT]        = { { 120, 120, 120 }, 0,    0,    400 },
    [HS_LED_IDLE]        = { { 0, 60, 0 },      80,   4000, 40 },
    [HS_LED_USB]         = { { 50, 50, 50 },    0,    0,    300 },
    [HS_LED_ADVERTISING] = { { 0, 0, 200 },     150,  850,  60 },
    [HS_LED_LINKED]      = { { 0, 0, 70 },      0,    0,    300 },
    [HS_LED_CHARGING]    = { { 200, 90, 0 },    1200, 1200, 1000 },
    [HS_LED_CHARGED]     = { { 0, 120, 0 },     0,    0,    300 },
    [HS_LED_LOW_BATTERY] = { { 220, 0, 0 },     150,  1850, 60 },
    [HS_LED_LEVEL_HIGH]  = { { 0, 200, 0 },     0,    0,    100 },
    [HS_LED_LEVEL_MID]   = { { 200, 110, 0 },   0,    0,    100 },
    [HS_LED_LEVEL_LOW]   = { { 220, 0, 0 },     0,    0,    100 },
    [HS_LED_POWER_OFF]   = { { 220, 0, 0 },     0,    0,    100 },
};

/* esp_timer task context. Renders the current phase and arms the next timer, if any.
 * A flash overrides the steady pattern until `flash_until_us`; a steady (on_ms = 0)
 * pattern needs no timer at all unless a flash is still to end. */
static void led_tick(void *arg)
{
    (void)arg;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_ui.lock);
    hs_led_t p = s_ui.steady;
    int64_t flash_left_us = 0;
    if (s_ui.flash_until_us != 0) {
        if (now < s_ui.flash_until_us) {
            p = s_ui.flash;
            flash_left_us = s_ui.flash_until_us - now;
        } else {
            s_ui.flash_until_us = 0;
        }
    }
    bool changed = (p != s_ui.shown);
    s_ui.shown = p;
    s_ui.phase_on = changed ? true : !s_ui.phase_on;
    bool on = s_ui.phase_on;
    portEXIT_CRITICAL(&s_ui.lock);

    const pattern_t *pt = &k_patterns[p];
    int64_t next_us;
    if (pt->on_ms == 0) {
        if (changed) {
            (void)rgb_led_fade(pt->color, pt->fade_ms);
        }
        next_us = flash_left_us;   /* 0: nothing to do until someone changes the pattern */
    } else {
        (void)rgb_led_fade(on ? pt->color : RGB_LED_OFF, pt->fade_ms);
        next_us = (int64_t)(on ? pt->on_ms : pt->off_ms) * 1000;
        if (flash_left_us > 0 && flash_left_us < next_us) {
            next_us = flash_left_us;
        }
    }
    if (next_us > 0) {
        (void)esp_timer_start_once(s_ui.timer, (uint64_t)next_us);
    }
}

static void led_kick(void)
{
    if (!s_ui.have_led) {
        return;
    }
    (void)esp_timer_stop(s_ui.timer);
    led_tick(NULL);
}

void hs_ui_led(hs_led_t pattern)
{
    if (pattern >= HS_LED_PATTERN_COUNT) {
        return;
    }
    portENTER_CRITICAL(&s_ui.lock);
    bool same = (s_ui.steady == pattern);
    s_ui.steady = pattern;
    portEXIT_CRITICAL(&s_ui.lock);
    if (!same) {
        led_kick();
    }
}

void hs_ui_led_flash(hs_led_t pattern, uint32_t ms)
{
    if (pattern >= HS_LED_PATTERN_COUNT) {
        return;
    }
    portENTER_CRITICAL(&s_ui.lock);
    s_ui.flash = pattern;
    s_ui.flash_until_us = esp_timer_get_time() + (int64_t)ms * 1000;
    portEXIT_CRITICAL(&s_ui.lock);
    led_kick();
}

/* -------------------------------------------------------------------------- */

esp_err_t hs_ui_init(hs_input_cb_t cb, void *ctx)
{
    const board_desc_t *b = board_desc();
    s_ui.cb = cb;
    s_ui.ctx = ctx;

    /* Switches by role, then the jack as one more debounced input. */
    buttons_pin_config_t pins[MAX_INPUTS];
    uint8_t n = 0;
    for (uint8_t i = 0; i < b->button_count && n < MAX_INPUTS - 1; i++, n++) {
        const board_button_t *bt = &b->buttons[i];
        pins[n] = (buttons_pin_config_t){ .gpio = bt->gpio, .active_low = bt->active_low,
                                          .pull_enable = bt->pull_enable,
                                          .wake_from_deep_sleep = bt->wake_from_deep_sleep };
        s_ui.role_of[n] = (uint8_t)bt->role;
        if (bt->role == BOARD_BTN_POWER) {
            s_ui.power_index = (int8_t)n;
        }
    }
    if (b->jack.gpio != BOARD_PIN_NONE && n < MAX_INPUTS) {
        pins[n] = (buttons_pin_config_t){ .gpio = b->jack.gpio, .active_low = !b->jack.plugged_high };
        s_ui.role_of[n] = ROLE_JACK;
        n++;
    }
    esp_err_t first = ESP_OK;
    if (n > 0) {
        const buttons_config_t bc = {
            .pins = pins, .count = n,
            .auto_repeat = true,          /* volume ramps while held; power-off hold timing */
            .cb = on_button,
        };
        esp_err_t err = buttons_init(&bc);
        if (err == ESP_OK) {
            err = buttons_start();
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "buttons: %s", esp_err_to_name(err));
            first = err;
        }
    }

    if (b->encoder.a != BOARD_PIN_NONE) {
        const encoder_config_t ec = {
            .gpio_a = b->encoder.a, .gpio_b = b->encoder.b, .pull_up = b->encoder.pull_up,
            .reverse = b->encoder.reverse != WHEEL_REVERSE,
            .counts_per_detent = b->encoder.counts_per_detent, .cb = on_wheel,
        };
        esp_err_t err = encoder_init(&ec);
        if (err == ESP_OK) {
            err = encoder_start();
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "encoder: %s", esp_err_to_name(err));
            if (first == ESP_OK) {
                first = err;
            }
        }
    }

    if (b->led.r != BOARD_PIN_NONE || b->led.g != BOARD_PIN_NONE || b->led.b != BOARD_PIN_NONE) {
        const rgb_led_config_t lc = {
            .gpio_r = b->led.r, .gpio_g = b->led.g, .gpio_b = b->led.b, .active_low = b->led.active_low,
            .gain_permille = { b->led.gain_permille[0], b->led.gain_permille[1], b->led.gain_permille[2] },
        };
        const esp_timer_create_args_t ta = { .callback = led_tick, .name = "hs_led" };
        if (rgb_led_init(&lc) == ESP_OK && esp_timer_create(&ta, &s_ui.timer) == ESP_OK) {
            s_ui.have_led = true;
            s_ui.shown = HS_LED_PATTERN_COUNT;   /* force the first render */
            s_ui.steady = HS_LED_BOOT;
            led_kick();
        } else {
            ESP_LOGW(TAG, "no status LED");
        }
    }
    ESP_LOGI(TAG, "inputs: %u switch(es)%s%s, LED %s", b->button_count,
             b->jack.gpio != BOARD_PIN_NONE ? " + jack" : "", b->encoder.a != BOARD_PIN_NONE ? " + wheel" : "",
             s_ui.have_led ? "yes" : "no");
    return first;
}

bool hs_ui_power_pressed(void)
{
    bool pressed = false;
    if (s_ui.power_index >= 0) {
        (void)buttons_is_pressed((uint8_t)s_ui.power_index, &pressed);
    }
    return pressed;
}
