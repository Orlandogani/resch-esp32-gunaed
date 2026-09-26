/**
 * Dongle: ring ownership, backend arbitration and wiring. No backend or host-side
 * logic lives here (SAD §2.2, ADR-013) — main/ decides *which* backend runs, never
 * *how* it behaves (applications/dongle/docs/design.md).
 *
 *     host side (host_port)          main/ owns              backend
 *     USB OUT / emulated tone ──►  speaker ring  ──► (own cursor) wlink_central | tone
 *     USB IN  / emulated sink ◄──  mic ring      ◄── (single writer)
 *
 * The host side is fixed for the life of the image: the dongle exists because it is
 * plugged into a PC. The backend is switchable: a long press of the action button
 * stops one and starts the other on the same two rings. The host side's servo always
 * asks the *active* backend for the speaker-path backlog, through one indirection
 * that stays valid across a switch.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "buttons.h"
#include "cfg.h"
#include "diag.h"
#include "dongle_backend.h"
#include "host_port.h"
#include "pm_policy.h"
#include "power.h"
#include "ringbuf.h"
#include "tone_backend.h"
#include "wlink_central.h"

static const char *TAG = "dongle";

#define APP_FAULT_MANDATORY_INIT 0x0101
#define APP_FAULT_NO_RING        0x0102

#define REQ_SWITCH_BACKEND (1u << 0)

static struct {
    ringbuf_t                 speaker_ring;
    ringbuf_t                 mic_ring;
    void                     *speaker_storage;
    void                     *mic_storage;
    const dongle_backend_t   *volatile active;
    TaskHandle_t              main_task;
} s_d;

static const buttons_pin_config_t s_button_pins[] = {
    { .gpio = CONFIG_DONGLE_BTN_PIN_ACTION, .active_low = true, .pull_enable = true },
};

/* -------------------------------------------------------------------------- */
/* Indirections the host side calls — any task                                 */
/* -------------------------------------------------------------------------- */

/* 1 ms, from the USB feeder task (or the emulated host's timer). During a backend
 * switch there is briefly no backend and this reports zero: the servo asks the host
 * for one extra sample per millisecond for at most one period, which is harmless. */
static uint32_t speaker_backlog(void *ctx)
{
    (void)ctx;
    const dongle_backend_t *b = s_d.active;
    return b != NULL ? b->speaker_backlog(b->ctx) : 0;
}

static void on_host_volume(bool mic, bool mute, int16_t volume_db256, void *ctx)
{
    (void)ctx;
    const dongle_backend_t *b = s_d.active;
    if (b != NULL && b->on_host_volume != NULL) {
        b->on_host_volume(mic, mute, volume_db256, b->ctx);
    }
}

static void on_button(buttons_event_t evt, const buttons_event_info_t *info, void *ctx)
{
    (void)ctx;
    (void)info;
    if (evt == BUTTONS_EVENT_LONG_PRESS) {
        xTaskNotify(s_d.main_task, REQ_SWITCH_BACKEND, eSetBits);
    }
}

static void on_fault(uint32_t code, const char *detail, void *ctx)
{
    (void)ctx;
    ESP_LOGE(TAG, "system entered FAULT (0x%08" PRIx32 "): %s", code, detail);
}

/* -------------------------------------------------------------------------- */
/* Backend arbitration — main task only                                        */
/* -------------------------------------------------------------------------- */

static void backend_enter(const dongle_backend_t *next)
{
    const dongle_backend_t *prev = s_d.active;
    if (prev == next) {
        return;
    }
    if (prev != NULL) {
        ESP_LOGI(TAG, "%s -> stopping", prev->name);
        s_d.active = NULL;
        (void)prev->stop(prev->ctx);
    }
    const dongle_backend_resources_t res = {
        .speaker_ring = &s_d.speaker_ring,
        .mic_ring     = &s_d.mic_ring,
        .format = {
            .speaker_rate_hz  = CONFIG_DONGLE_SPEAKER_SAMPLE_RATE_HZ,
            .speaker_channels = CONFIG_DONGLE_SPEAKER_CHANNELS,
            .mic_rate_hz      = CONFIG_DONGLE_MIC_SAMPLE_RATE_HZ,
        },
        .hid_tap = host_port_hid_tap,
    };
    esp_err_t err = next->start(&res, next->ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed to start: %s", next->name, esp_err_to_name(err));
        return;
    }
    s_d.active = next;
    ESP_LOGI(TAG, "backend = %s", next->name);
}

/* -------------------------------------------------------------------------- */
/* Boot                                                                        */
/* -------------------------------------------------------------------------- */

static void mandatory(esp_err_t err, const char *what)
{
    if (err != ESP_OK) {
        diag_fault(APP_FAULT_MANDATORY_INIT, what);
    }
}

static esp_err_t ring_alloc(ringbuf_t *rb, void **storage, uint32_t rate, uint32_t ch, uint32_t ms)
{
    size_t bytes = (size_t)rate * ch * 2u / 1000u * ms;
    /* Internal RAM: both rings are on the 1 ms USB path (ADR-014). */
    *storage = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (*storage == NULL) {
        ESP_LOGE(TAG, "ring: %u bytes internal unavailable", (unsigned)bytes);
        return ESP_ERR_NO_MEM;
    }
    return ringbuf_init(rb, *storage, bytes);
}

void app_main(void)
{
    s_d.main_task = xTaskGetCurrentTaskHandle();

    mandatory(cfg_init(), "cfg_init");
    diag_config_t diag_cfg = { .fault_cb = on_fault };
    mandatory(diag_init(&diag_cfg), "diag_init");
    mandatory(power_init(), "power_init");
    mandatory(pm_policy_init(), "pm_policy_init");

    if (ring_alloc(&s_d.speaker_ring, &s_d.speaker_storage, CONFIG_DONGLE_SPEAKER_SAMPLE_RATE_HZ,
                   CONFIG_DONGLE_SPEAKER_CHANNELS, CONFIG_DONGLE_SPEAKER_RING_MS) != ESP_OK ||
        ring_alloc(&s_d.mic_ring, &s_d.mic_storage, CONFIG_DONGLE_MIC_SAMPLE_RATE_HZ, 1,
                   CONFIG_DONGLE_MIC_RING_MS) != ESP_OK) {
        diag_fault(APP_FAULT_NO_RING, "ring allocation");
        return;
    }

    const buttons_config_t btn = {
        .pins  = s_button_pins,
        .count = sizeof(s_button_pins) / sizeof(s_button_pins[0]),
        .cb    = on_button,
    };
    if (buttons_init(&btn) == ESP_OK) {
        (void)buttons_start();
    }

    /* Backend first: it must hold its cursor on the speaker ring before the host
     * side starts writing, and the mic ring must have its writer. */
#if CONFIG_DONGLE_BACKEND_TONE
    backend_enter(tone_backend());
#else
    backend_enter(wlink_central_backend());
#endif

    const host_port_config_t host = {
        .speaker_ring       = &s_d.speaker_ring,
        .mic_ring           = &s_d.mic_ring,
        .speaker_rate_hz    = CONFIG_DONGLE_SPEAKER_SAMPLE_RATE_HZ,
        .speaker_channels   = CONFIG_DONGLE_SPEAKER_CHANNELS,
        .mic_rate_hz        = CONFIG_DONGLE_MIC_SAMPLE_RATE_HZ,
        .speaker_target_ms  = CONFIG_DONGLE_PATH_TARGET_MS,
        .speaker_backlog_cb = speaker_backlog,
        .on_volume          = on_host_volume,
    };
    esp_err_t err = host_port_start(&host);
    if (err != ESP_OK) {
        diag_fault(APP_FAULT_MANDATORY_INIT, "host side");
    }

    if (diag_state() != DIAG_STATE_FAULT) {
        diag_mark_running();
    }

    while (true) {
        uint32_t req = 0;
        (void)xTaskNotifyWait(0, UINT32_MAX, &req, pdMS_TO_TICKS(CONFIG_DONGLE_STATS_PERIOD_S * 1000));
        if (req & REQ_SWITCH_BACKEND) {
            const dongle_backend_t *cur = s_d.active;
            backend_enter(cur == tone_backend() ? wlink_central_backend() : tone_backend());
            continue;
        }
        const dongle_backend_t *b = s_d.active;
        ESP_LOGI(TAG, "backend %s, %s, heap %u", b ? b->name : "none",
                 (b && b->is_linked(b->ctx)) ? "linked" : "not linked",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        host_port_log_stats();
        if (b != NULL && b->log_stats != NULL) {
            b->log_stats(b->ctx);
        }
    }
}
