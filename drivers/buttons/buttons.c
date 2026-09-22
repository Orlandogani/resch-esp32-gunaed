#include "buttons.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "diag.h"
#include "pm_policy.h"
#include "power.h"

static const char *TAG = "buttons";

typedef enum { ST_UNINIT = 0, ST_READY, ST_RUNNING } state_t;

typedef struct {
    buttons_pin_config_t cfg;
    bool     raw;            /* Last raw sample, as "pressed".                 */
    bool     stable;         /* Debounced state.                               */
    uint32_t raw_since_ms;   /* When the raw level last changed.               */
    uint32_t press_start_ms; /* Debounced press time; valid while stable.      */
    bool     long_fired;
    uint32_t next_repeat_ms;
} button_t;

static state_t                 s_state;
static button_t                s_btn[CONFIG_BUTTONS_MAX];
static uint8_t                 s_count;
static uint16_t                s_debounce_ms;
static uint16_t                s_long_press_ms;
static uint16_t                s_repeat_ms;
static bool                    s_auto_repeat;
static buttons_event_cb_t      s_cb;
static void                   *s_cb_ctx;
static portMUX_TYPE            s_cb_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t            s_task;
static SemaphoreHandle_t       s_mutex;          /* Recursive: the callback may call stop(). */
static StaticSemaphore_t       s_mutex_storage;
static pm_policy_lock_handle_t s_lock;
static bool                    s_active;         /* Lock held: a press is in progress. */
static bool                    s_isr_service_ours;
static buttons_stats_t         s_stats;

/* -------------------------------------------------------------------------- */
/* ISR: notify only (FW-BTN-007, DR5)                                          */
/* -------------------------------------------------------------------------- */

static void IRAM_ATTR on_edge_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    if (s_task != NULL) {
        vTaskNotifyGiveFromISR(s_task, &woken);
    }
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* -------------------------------------------------------------------------- */
/* Debounce state machine (DES-BTN-004)                                        */
/* -------------------------------------------------------------------------- */

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void emit(buttons_event_t evt, uint8_t index, uint32_t held_ms)
{
    buttons_event_cb_t cb;
    void *ctx;
    portENTER_CRITICAL(&s_cb_mux);
    cb  = s_cb;
    ctx = s_cb_ctx;
    portEXIT_CRITICAL(&s_cb_mux);

    switch (evt) {
    case BUTTONS_EVENT_PRESSED:    s_stats.presses++;      break;
    case BUTTONS_EVENT_RELEASED:   s_stats.releases++;     break;
    case BUTTONS_EVENT_LONG_PRESS: s_stats.long_presses++; break;
    case BUTTONS_EVENT_REPEAT:     s_stats.repeats++;      break;
    }
    if (cb == NULL) {
        s_stats.events_dropped++;
        return;
    }
    buttons_event_info_t info = {
        .index   = index,
        .gpio    = s_btn[index].cfg.gpio,
        .held_ms = held_ms,
    };
    cb(evt, &info, ctx);
}

static bool read_pressed(const button_t *b)
{
    int level = gpio_get_level((gpio_num_t)b->cfg.gpio);
    return b->cfg.active_low ? (level == 0) : (level != 0);
}

/* Returns true while this button still needs the fast sampling loop. */
static bool process(button_t *b, uint8_t index, uint32_t now)
{
    bool raw = read_pressed(b);

    if (raw != b->raw) {
        if (b->raw != b->stable) {
            /* Reversed before the previous change was accepted: a bounce. */
            s_stats.bounces_rejected++;
        }
        b->raw = raw;
        b->raw_since_ms = now;
    } else if (raw != b->stable && (uint32_t)(now - b->raw_since_ms) >= s_debounce_ms) {
        b->stable = raw;
        if (raw) {
            b->press_start_ms = now;
            b->long_fired = false;
            emit(BUTTONS_EVENT_PRESSED, index, 0);
        } else {
            emit(BUTTONS_EVENT_RELEASED, index, now - b->press_start_ms);
        }
    }

    if (b->stable) {
        uint32_t held = now - b->press_start_ms;
        if (!b->long_fired && held >= s_long_press_ms) {
            b->long_fired = true;
            b->next_repeat_ms = now + s_repeat_ms;
            emit(BUTTONS_EVENT_LONG_PRESS, index, held);
        } else if (b->long_fired && s_auto_repeat && (int32_t)(now - b->next_repeat_ms) >= 0) {
            b->next_repeat_ms += s_repeat_ms;
            emit(BUTTONS_EVENT_REPEAT, index, held);
        }
    }

    /* Held, or a release still being debounced: keep sampling fast. */
    return b->raw || b->stable;
}

static void reset_states(void)
{
    for (uint8_t i = 0; i < s_count; i++) {
        s_btn[i].raw = false;
        s_btn[i].stable = false;
        s_btn[i].raw_since_ms = 0;
        s_btn[i].press_start_ms = 0;
        s_btn[i].long_fired = false;
        s_btn[i].next_repeat_ms = 0;
    }
}

/* Caller holds s_mutex. The pm_policy call stays inside it on purpose: the task and
 * stop() both drive this transition, and an acquire/release pair split across the
 * mutex boundary can reorder into a permanent sleep veto. pm_policy is a leaf
 * platform service (SAD §3.3 D6), so no lock-order cycle is possible. */
static void set_active(bool active)
{
    if (active == s_active) {
        return;
    }
    s_active = active;
    if (active) {
        pm_policy_lock_acquire(s_lock);
    } else {
        pm_policy_lock_release(s_lock);
    }
}

/* -------------------------------------------------------------------------- */
/* Task (DES-BTN-003, DES-BTN-008)                                             */
/* -------------------------------------------------------------------------- */

static void buttons_task(void *arg)
{
    for (;;) {
        TickType_t wait = pdMS_TO_TICKS(s_active ? CONFIG_BUTTONS_SAMPLE_MS : CONFIG_BUTTONS_IDLE_POLL_MS);
        if (ulTaskNotifyTake(pdTRUE, wait) > 0) {
            s_stats.isr_wakeups++;
        }

        xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY);
        if (s_state == ST_RUNNING) {
            uint32_t now = now_ms();
            bool any_active = false;
            for (uint8_t i = 0; i < s_count; i++) {
                any_active |= process(&s_btn[i], i, now);
            }
            /* A callback may have called stop(), which already dropped the lock. */
            if (s_state == ST_RUNNING) {
                set_active(any_active);
            }
        }
        xSemaphoreGiveRecursive(s_mutex);

        UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
        if (s_stats.task_stack_free_min == 0 || hw < s_stats.task_stack_free_min) {
            s_stats.task_stack_free_min = (uint32_t)hw;
        }
        diag_task_feed();
    }
}

/* -------------------------------------------------------------------------- */
/* Configuration helpers                                                       */
/* -------------------------------------------------------------------------- */

static esp_err_t validate(const buttons_config_t *cfg)
{
    if (cfg == NULL || cfg->pins == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->count == 0 || cfg->count > CONFIG_BUTTONS_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint8_t i = 0; i < cfg->count; i++) {
        const buttons_pin_config_t *p = &cfg->pins[i];
        if (!GPIO_IS_VALID_GPIO(p->gpio)) {
            return ESP_ERR_INVALID_ARG;
        }
        if (p->wake_from_deep_sleep && !rtc_gpio_is_valid_gpio((gpio_num_t)p->gpio)) {
            return ESP_ERR_INVALID_ARG;
        }
        for (uint8_t j = 0; j < i; j++) {
            if (cfg->pins[j].gpio == p->gpio) {
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    return ESP_OK;
}

static esp_err_t configure_pin(const buttons_pin_config_t *p)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << p->gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = (p->pull_enable && p->active_low)  ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE,
        .pull_down_en = (p->pull_enable && !p->active_low) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_isr_handler_add((gpio_num_t)p->gpio, on_edge_isr, NULL);
    if (err != ESP_OK) {
        gpio_reset_pin((gpio_num_t)p->gpio);
        return err;
    }
    /* gpio_config() and gpio_isr_handler_add() both enable the interrupt; start() does. */
    gpio_intr_disable((gpio_num_t)p->gpio);
    return ESP_OK;
}

static void release_pin(const buttons_pin_config_t *p)
{
    gpio_intr_disable((gpio_num_t)p->gpio);
    gpio_isr_handler_remove((gpio_num_t)p->gpio);
    gpio_reset_pin((gpio_num_t)p->gpio);
    if (p->wake_from_deep_sleep) {
        power_disable_gpio_wakeup((gpio_num_t)p->gpio);
    }
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t buttons_init(const buttons_config_t *cfg)
{
    if (s_state != ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = validate(cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = pm_policy_lock_create("buttons", &s_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lock: %s", esp_err_to_name(err));
        return err;
    }
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_mutex_storage);
    }

    s_count         = cfg->count;
    s_debounce_ms   = cfg->debounce_ms   ? cfg->debounce_ms   : CONFIG_BUTTONS_DEBOUNCE_MS;
    s_long_press_ms = cfg->long_press_ms ? cfg->long_press_ms : CONFIG_BUTTONS_LONG_PRESS_MS;
    s_repeat_ms     = cfg->repeat_ms     ? cfg->repeat_ms     : CONFIG_BUTTONS_REPEAT_MS;
    s_auto_repeat   = cfg->auto_repeat;
    s_cb            = cfg->cb;
    s_cb_ctx        = cfg->ctx;
    for (uint8_t i = 0; i < s_count; i++) {
        s_btn[i].cfg = cfg->pins[i];
    }
    reset_states();

    err = gpio_install_isr_service(0);
    if (err == ESP_OK) {
        s_isr_service_ours = true;
    } else if (err != ESP_ERR_INVALID_STATE) {   /* INVALID_STATE: someone else installed it. */
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t configured = 0;
    for (; configured < s_count; configured++) {
        const buttons_pin_config_t *p = &s_btn[configured].cfg;
        err = configure_pin(p);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio %d: %s", p->gpio, esp_err_to_name(err));
            goto fail;
        }
        if (p->wake_from_deep_sleep) {
            err = power_enable_gpio_wakeup((gpio_num_t)p->gpio, !p->active_low);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "gpio %d wake: %s", p->gpio, esp_err_to_name(err));
                gpio_isr_handler_remove((gpio_num_t)p->gpio);
                gpio_reset_pin((gpio_num_t)p->gpio);
                goto fail;
            }
        }
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_active = false;
    s_state = ST_READY; /* Task checks this; set before creating it. */

    BaseType_t ok = xTaskCreatePinnedToCore(buttons_task, "buttons",
                                            CONFIG_BUTTONS_TASK_STACK, NULL,
                                            CONFIG_BUTTONS_TASK_PRIORITY, &s_task,
                                            CONFIG_BUTTONS_TASK_CORE);
    if (ok != pdPASS) {
        err = ESP_ERR_NO_MEM;
        s_state = ST_UNINIT;
        goto fail;
    }

    ESP_LOGI(TAG, "init: %u button(s), debounce %u ms, long %u ms, repeat %s%u ms, task prio %d core %d",
             s_count, s_debounce_ms, s_long_press_ms, s_auto_repeat ? "" : "off/",
             s_repeat_ms, CONFIG_BUTTONS_TASK_PRIORITY, CONFIG_BUTTONS_TASK_CORE);
    return ESP_OK;

fail:
    for (uint8_t i = 0; i < configured; i++) {
        release_pin(&s_btn[i].cfg);
    }
    if (s_isr_service_ours) {
        gpio_uninstall_isr_service();
        s_isr_service_ours = false;
    }
    s_count = 0;
    return err;
}

esp_err_t buttons_deinit(void)
{
    if (s_state == ST_UNINIT) {
        return ESP_OK;
    }
    buttons_stop();

    if (s_task != NULL) {
        TaskHandle_t t = s_task;
        s_task = NULL;          /* ISR checks this before notifying. */
        vTaskDelete(t);
    }
    for (uint8_t i = 0; i < s_count; i++) {
        release_pin(&s_btn[i].cfg);
    }
    if (s_isr_service_ours) {
        gpio_uninstall_isr_service();
        s_isr_service_ours = false;
    }
    s_count = 0;
    s_cb = NULL;
    s_cb_ctx = NULL;
    s_state = ST_UNINIT;
    /* s_lock is kept: pm_policy locks are never destroyed. */
    ESP_LOGI(TAG, "deinit");
    return ESP_OK;
}

esp_err_t buttons_start(void)
{
    if (s_state != ST_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY);
    reset_states();
    s_state = ST_RUNNING;
    xSemaphoreGiveRecursive(s_mutex);

    for (uint8_t i = 0; i < s_count; i++) {
        gpio_intr_enable((gpio_num_t)s_btn[i].cfg.gpio);
    }
    diag_task_register(s_task);   /* Best effort: ESP_ERR_INVALID_STATE without diag_init(). */
    s_stats.running = true;
    xTaskNotifyGive(s_task);      /* Sample immediately: a button may already be held (wake). */
    ESP_LOGI(TAG, "started");
    return ESP_OK;
}

esp_err_t buttons_stop(void)
{
    if (s_state != ST_RUNNING) {
        return s_state == ST_UNINIT ? ESP_ERR_INVALID_STATE : ESP_OK;
    }
    for (uint8_t i = 0; i < s_count; i++) {
        gpio_intr_disable((gpio_num_t)s_btn[i].cfg.gpio);
    }
    xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY);
    s_state = ST_READY;
    set_active(false);
    reset_states();
    xSemaphoreGiveRecursive(s_mutex);

    diag_task_unregister(s_task);
    s_stats.running = false;
    ESP_LOGI(TAG, "stopped");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Queries                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t buttons_set_event_cb(buttons_event_cb_t cb, void *ctx)
{
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&s_cb_mux);
    s_cb     = cb;
    s_cb_ctx = ctx;
    portEXIT_CRITICAL(&s_cb_mux);
    return ESP_OK;
}

esp_err_t buttons_is_pressed(uint8_t index, bool *pressed)
{
    if (pressed == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state == ST_UNINIT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (index >= s_count) {
        return ESP_ERR_INVALID_ARG;
    }
    *pressed = s_btn[index].stable;
    return ESP_OK;
}

esp_err_t buttons_stats(buttons_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_stats;
    out->running = (s_state == ST_RUNNING);
    return ESP_OK;
}

esp_err_t buttons_stats_reset(void)
{
    bool running = s_stats.running;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.running = running;
    return ESP_OK;
}
