/**
 * On-target Unity tests for the USB composite: usb_device + usb_hid + usb_audio.
 *
 * Everything tagged [usb] verifies registration, budget enforcement and the
 * assembled composite descriptor WITHOUT calling usb_device_start(): the bus is
 * never touched, so these are safe on a board whose only console is the built-in
 * USB-Serial/JTAG. The single test tagged [needs_usb_host] starts the stack and
 * requires a separate console; app_main() excludes it by default.
 */
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "power.h"
#include "pm_policy.h"
#include "ringbuf.h"
#include "usb_device.h"
#include "usb_hid.h"
#include "usb_audio.h"

/* Minimal vendor-defined HID report descriptor: 8-byte input, 8-byte output. */
static const uint8_t s_hid_report_desc[] = {
    0x06, 0x00, 0xFF,  /* Usage Page (Vendor 0xFF00) */
    0x09, 0x01,        /* Usage (0x01)               */
    0xA1, 0x01,        /* Collection (Application)   */
    0x15, 0x00,        /*   Logical Minimum (0)      */
    0x26, 0xFF, 0x00,  /*   Logical Maximum (255)    */
    0x75, 0x08,        /*   Report Size (8)          */
    0x95, 0x08,        /*   Report Count (8)         */
    0x09, 0x01,        /*   Usage (0x01)             */
    0x81, 0x02,        /*   Input (Data,Var,Abs)     */
    0x09, 0x01,        /*   Usage (0x01)             */
    0x91, 0x02,        /*   Output (Data,Var,Abs)    */
    0xC0,              /* End Collection             */
};

static uint8_t   s_ring_storage[4096];
static ringbuf_t s_ring;

static void fresh(void)
{
    usb_audio_deinit();
    usb_hid_deinit();
    usb_device_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, power_init());
    TEST_ASSERT_EQUAL(ESP_OK, pm_policy_init());
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&s_ring, s_ring_storage, sizeof(s_ring_storage)));
}

/* ------------------------------------------------------------------------- */
/* Descriptor walking helpers                                                 */
/* ------------------------------------------------------------------------- */

#define DT_CONFIG    0x02
#define DT_INTERFACE 0x04
#define DT_ENDPOINT  0x05
#define DT_IAD       0x0B
#define DT_HID       0x21

typedef struct {
    uint8_t  addr;
    uint8_t  attr;        /* Full bmAttributes. */
    uint16_t mps;
    uint8_t  interval;
} ep_info_t;

typedef struct {
    uint8_t interfaces;
    uint8_t iads;
    uint8_t endpoints;
    uint8_t ep_addrs[16];
    ep_info_t eps[16];
    uint8_t hid_ep_interval;
    uint8_t iso_ep_interval;
    uint16_t iso_ep_size;
    uint8_t hid_count;
    uint8_t audio_itf_descs;    /* bInterfaceClass 0x01, one per alternate setting */
    uint8_t audio_interfaces;   /* Distinct bInterfaceNumber among those           */
    uint32_t audio_itf_mask;    /* Private to walk().                              */
} desc_summary_t;

/* UAC2 endpoint usage, bmAttributes bits 5:4 — 00 data, 01 feedback. */
#define EP_USAGE(attr)   (((attr) >> 4) & 0x03)
#define EP_IS_FEEDBACK(attr) (EP_USAGE(attr) == 1)

static void walk(const uint8_t *d, size_t len, desc_summary_t *s)
{
    memset(s, 0, sizeof(*s));
    size_t off = 0;
    uint8_t cur_class = 0;
    while (off + 2 <= len) {
        uint8_t blen = d[off];
        uint8_t type = d[off + 1];
        TEST_ASSERT_TRUE_MESSAGE(blen >= 2 && off + blen <= len, "malformed descriptor block");
        switch (type) {
        case DT_INTERFACE:
            s->interfaces++;
            cur_class = d[off + 5];                 /* bInterfaceClass */
            if (cur_class == 0x03) {                /* HID */
                s->hid_count++;
            }
            if (cur_class == 0x01) {                /* AUDIO */
                /* Alternate settings repeat the interface descriptor, so count
                 * distinct interface numbers rather than descriptors. */
                s->audio_itf_descs++;
                uint8_t num = d[off + 2];           /* bInterfaceNumber */
                if (num < 32 && !(s->audio_itf_mask & (1u << num))) {
                    s->audio_itf_mask |= (1u << num);
                    s->audio_interfaces++;
                }
            }
            break;
        case DT_IAD:
            s->iads++;
            break;
        case DT_ENDPOINT: {
            uint8_t addr = d[off + 2];
            uint8_t attr = d[off + 3] & 0x03;      /* transfer type */
            uint16_t mps = (uint16_t)(d[off + 4] | (d[off + 5] << 8));
            uint8_t interval = d[off + 6];
            TEST_ASSERT_TRUE_MESSAGE(s->endpoints < 16, "too many endpoints");
            s->eps[s->endpoints].addr = addr;
            s->eps[s->endpoints].attr = d[off + 3];
            s->eps[s->endpoints].mps = mps;
            s->eps[s->endpoints].interval = interval;
            s->ep_addrs[s->endpoints++] = addr;
            if (attr == 0x03 && cur_class == 0x03) { /* interrupt, HID */
                s->hid_ep_interval = interval;
            }
            if (attr == 0x01) {                      /* isochronous */
                s->iso_ep_interval = interval;
                s->iso_ep_size = mps & 0x07FF;
            }
            break;
        }
        default:
            break;
        }
        off += blen;
    }
    TEST_ASSERT_EQUAL_MESSAGE(len, off, "descriptor blocks do not tile the total length");
}

/* ------------------------------------------------------------------------- */
/* Lifecycle and budget                                                       */
/* ------------------------------------------------------------------------- */

TEST_CASE("device core: calls before init fail, init is idempotent", "[usb]")
{
    usb_device_deinit();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, usb_device_register(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, usb_device_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, usb_device_stop());
    TEST_ASSERT_FALSE(usb_device_is_mounted());
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_device_register(NULL));
}

/* Synthetic function factory for budget tests. */
static uint8_t s_fake_in, s_fake_out;
static uint16_t s_fake_fifo;
static usb_ep_budget_t fake_req(void *ctx) { return (usb_ep_budget_t){ s_fake_in, s_fake_out, s_fake_fifo }; }
static size_t fake_len(void *ctx) { return 9; }
static size_t fake_write(void *ctx, uint8_t *buf, uint8_t itf, const uint8_t *in, const uint8_t *out)
{
    /* A bare interface descriptor so the composite still tiles. */
    const uint8_t itfd[9] = { 9, DT_INTERFACE, itf, 0, 0, 0xFF, 0, 0, 0 };
    memcpy(buf, itfd, 9);
    return 9;
}
static usb_function_t s_fake = {
    .name = "fake", .interface_count = 1,
    .endpoint_request = fake_req, .descriptor_len = fake_len, .descriptor_write = fake_write,
};

TEST_CASE("device core: endpoint budget is enforced at registration (CON-03)", "[usb]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));

    usb_ep_budget_t max;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_budget(NULL, &max));
    TEST_ASSERT_EQUAL(4, max.in_endpoints);

    /* Claim all four IN endpoints in one go: accepted. */
    s_fake_in = 4; s_fake_out = 0; s_fake_fifo = 64;
    static usb_function_t f1;
    f1 = s_fake; f1.name = "f1";
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_register(&f1));

    /* One more IN endpoint: refused, budget unchanged. */
    s_fake_in = 1;
    static usb_function_t f2;
    f2 = s_fake; f2.name = "f2";
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, usb_device_register(&f2));
    usb_ep_budget_t used;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_budget(&used, NULL));
    TEST_ASSERT_EQUAL(4, used.in_endpoints);

    /* OUT-only function still fits. */
    s_fake_in = 0; s_fake_out = 2; s_fake_fifo = 0;
    static usb_function_t f3;
    f3 = s_fake; f3.name = "f3";
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_register(&f3));
}

TEST_CASE("device core: FIFO RAM budget is enforced", "[usb]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    /* One IN endpoint asking for more TX FIFO than the core has after reservations. */
    s_fake_in = 1; s_fake_out = 0; s_fake_fifo = USB_DEVICE_FIFO_BYTES;
    static usb_function_t big;
    big = s_fake; big.name = "big";
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, usb_device_register(&big));
    usb_ep_budget_t used;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_budget(&used, NULL));
    TEST_ASSERT_EQUAL(0, used.in_endpoints);
    TEST_ASSERT_EQUAL(0, used.tx_fifo_bytes);
}

TEST_CASE("device core: a function that lies about its descriptor length is caught", "[usb]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    s_fake_in = 0; s_fake_out = 0; s_fake_fifo = 0;
    static usb_function_t liar;
    liar = s_fake; liar.name = "liar";
    liar.descriptor_len = NULL; /* invalid: mandatory */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_device_register(&liar));
    liar.descriptor_len = fake_len;
    liar.interface_count = 0;   /* invalid */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_device_register(&liar));
}

/* ------------------------------------------------------------------------- */
/* Class functions                                                            */
/* ------------------------------------------------------------------------- */

TEST_CASE("hid: init validates configuration and requires the device core", "[usb]")
{
    fresh();
    usb_hid_config_t cfg = {
        .report_descriptor = s_hid_report_desc,
        .report_descriptor_len = sizeof(s_hid_report_desc),
    };
    /* Device core not initialised: refused. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, usb_hid_init(&cfg));

    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_hid_init(NULL));
    usb_hid_config_t bad = cfg; bad.report_descriptor = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_hid_init(&bad));
    bad = cfg; bad.ep_size = 65;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_hid_init(&bad));

    TEST_ASSERT_EQUAL(ESP_OK, usb_hid_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, usb_hid_init(&cfg)); /* singleton */

    /* Not mounted: sends are rejected and counted, never queued. */
    uint8_t rep[8] = {0};
    TEST_ASSERT_FALSE(usb_hid_ready());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, usb_hid_report_send(0, rep, sizeof(rep)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_hid_report_send(0, NULL, 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, usb_hid_report_send(0, rep, 65));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, usb_hid_report_send(1, rep, 64)); /* id byte + 64 > 64 */
    usb_hid_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, usb_hid_stats(&st));
    TEST_ASSERT_EQUAL(1, st.reports_rejected);
    TEST_ASSERT_EQUAL(0, st.queue_depth);
}

TEST_CASE("audio: init validates configuration and sizes the endpoint", "[usb]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    usb_audio_config_t cfg = {
        .direction = USB_AUDIO_DIR_MIC,
        .mic = { .ring = &s_ring, .sample_rate_hz = 16000, .channels = 1, .bits_per_sample = 16 },
    };
    usb_audio_config_t bad;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(NULL));
    bad = cfg; bad.mic.ring = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    bad = cfg; bad.mic.sample_rate_hz = 96000;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    bad = cfg; bad.mic.bits_per_sample = 12;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    bad = cfg; bad.mic.channels = 3;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    /* A speaker is refused either way, but for different reasons: without
     * CONFIG_TINYUSB_AUDIO_SPEAKER_ENABLED there is no OUT path to enumerate at
     * all; with it, this configuration still has no speaker ring or backlog
     * source. Neither may silently enumerate as a microphone. */
    bad = cfg; bad.direction = USB_AUDIO_DIR_HEADSET;
#if CONFIG_TINYUSB_AUDIO_SPEAKER_ENABLED
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
#else
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, usb_audio_init(&bad));
#endif

    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_init(&cfg));
    TEST_ASSERT_FALSE(usb_audio_is_streaming(USB_AUDIO_STREAM_MIC));
    usb_audio_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_stats(&st));
    TEST_ASSERT_EQUAL(32, st.mic.nominal_packet_bytes); /* 16 samples/ms × 2 B */
    TEST_ASSERT_EQUAL(0, st.mic.packets);
}

TEST_CASE("composite: HID + UAC2 mic descriptor is well-formed with 1 ms intervals", "[usb]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));

    usb_hid_config_t hid = {
        .report_descriptor = s_hid_report_desc,
        .report_descriptor_len = sizeof(s_hid_report_desc),
    };
    usb_audio_config_t uac = {
        .direction = USB_AUDIO_DIR_MIC,
        .mic = { .ring = &s_ring, .sample_rate_hz = 48000, .channels = 1, .bits_per_sample = 16 },
    };
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_init(&uac));
    TEST_ASSERT_EQUAL(ESP_OK, usb_hid_init(&hid));

    usb_ep_budget_t used;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_budget(&used, NULL));
    TEST_ASSERT_EQUAL(2, used.in_endpoints);   /* iso IN + interrupt IN */
    TEST_ASSERT_EQUAL(0, used.out_endpoints);

    const uint8_t *d = NULL;
    size_t len = 0;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_config_descriptor(&d, &len));
    TEST_ASSERT_NOT_NULL(d);

    /* Configuration header: type, wTotalLength, bNumInterfaces. */
    TEST_ASSERT_EQUAL(DT_CONFIG, d[1]);
    TEST_ASSERT_EQUAL(len, (size_t)(d[2] | (d[3] << 8)));
    TEST_ASSERT_EQUAL(3, d[4]);                /* AC + AS + HID */

    desc_summary_t s;
    walk(d, len, &s);
    TEST_ASSERT_EQUAL(1, s.iads);              /* the UAC2 function */
    TEST_ASSERT_EQUAL(1, s.hid_count);
    /* Interface descriptors: AC, AS alt0, AS alt1, HID = 4 blocks over 3 numbers. */
    TEST_ASSERT_EQUAL(4, s.interfaces);
    TEST_ASSERT_EQUAL(2, s.endpoints);

    /* Both endpoints IN, distinct addresses, both serviced every frame. */
    TEST_ASSERT_TRUE(s.ep_addrs[0] & 0x80);
    TEST_ASSERT_TRUE(s.ep_addrs[1] & 0x80);
    TEST_ASSERT_NOT_EQUAL(s.ep_addrs[0], s.ep_addrs[1]);
    TEST_ASSERT_EQUAL_MESSAGE(1, s.hid_ep_interval, "HID bInterval must be 1 (SYS-HID-001)");
    TEST_ASSERT_EQUAL_MESSAGE(1, s.iso_ep_interval, "isochronous bInterval must be 1 (FW-AUD-021)");
    /* 48 kHz mono 16-bit: (48 + 1) × 2 = 98 bytes, under the FS limit of 1023. */
    TEST_ASSERT_EQUAL(98, s.iso_ep_size);
}

TEST_CASE("composite: registration after start is refused; stop/deinit are clean", "[usb][needs_usb_host]")
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    usb_hid_config_t hid = {
        .report_descriptor = s_hid_report_desc,
        .report_descriptor_len = sizeof(s_hid_report_desc),
    };
    TEST_ASSERT_EQUAL(ESP_OK, usb_hid_init(&hid));

    /* Starts the stack: on a USJ-console board this severs the console. */
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, usb_device_start());
    static usb_function_t late;
    late = s_fake; late.name = "late";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, usb_device_register(&late));

    vTaskDelay(pdMS_TO_TICKS(1500));
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_stop());
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_deinit());
}

/* ------------------------------------------------------------------------- */
/* Duplex headset: only built when the speaker direction is compiled in.      */
/* Run with -D SDKCONFIG_DEFAULTS="...;sdkconfig.headset.defaults".           */
/* ------------------------------------------------------------------------- */
#if CONFIG_TINYUSB_AUDIO_SPEAKER_ENABLED

static uint8_t   s_spk_storage[8192];
static ringbuf_t s_spk_ring;
static uint32_t  s_fake_backlog;

static uint32_t fake_backlog_cb(void *ctx)
{
    return s_fake_backlog;
}

static usb_audio_config_t headset_cfg(void)
{
    usb_audio_config_t c = {
        .direction = USB_AUDIO_DIR_HEADSET,
        .mic     = { .ring = &s_ring,     .sample_rate_hz = 16000, .channels = 1, .bits_per_sample = 16 },
        .speaker = { .ring = &s_spk_ring, .sample_rate_hz = 48000, .channels = 2, .bits_per_sample = 16 },
        .speaker_backlog_cb = fake_backlog_cb,
        .speaker_target_ms = 12,
    };
    return c;
}

static void fresh_headset(void)
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&s_spk_ring, s_spk_storage, sizeof(s_spk_storage)));
    s_fake_backlog = 0;
}

TEST_CASE("headset: init validates the speaker direction's extra requirements", "[usb]")
{
    fresh_headset();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    usb_audio_config_t cfg = headset_cfg();
    usb_audio_config_t bad;

    /* The feedback servo has nothing to regulate without a backlog source. */
    bad = cfg; bad.speaker_backlog_cb = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    bad = cfg; bad.speaker.ring = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    bad = cfg; bad.speaker.channels = 3;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    /* 12 ms at 48 kHz stereo 16-bit is 2304 B; a ring that small cannot also hold
     * a packet, so the servo could never reach its setpoint. */
    static uint8_t tiny[2048];
    static ringbuf_t tiny_ring;
    TEST_ASSERT_EQUAL(ESP_OK, ringbuf_init(&tiny_ring, tiny, sizeof(tiny)));
    bad = cfg; bad.speaker.ring = &tiny_ring;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));

    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_init(&cfg));
    usb_audio_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_stats(&st));
    TEST_ASSERT_EQUAL(32, st.mic.nominal_packet_bytes);        /* 16 smp/ms x 1ch x 2 B */
    TEST_ASSERT_EQUAL(192, st.speaker.nominal_packet_bytes);   /* 48 smp/ms x 2ch x 2 B */
    TEST_ASSERT_EQUAL(12 * 48 * 4, st.speaker.target_bytes);   /* 2304 B */
    TEST_ASSERT_FALSE(usb_audio_is_streaming(USB_AUDIO_STREAM_MIC));
    TEST_ASSERT_FALSE(usb_audio_is_streaming(USB_AUDIO_STREAM_SPEAKER));
}

TEST_CASE("headset: the duplex descriptor is one function with three interfaces", "[usb]")
{
    fresh_headset();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    usb_audio_config_t cfg = headset_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_init(&cfg));

    const uint8_t *d = NULL;
    size_t len = 0;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_config_descriptor(&d, &len));
    desc_summary_t sum;
    walk(d, len, &sum);

    /* One IAD covering AudioControl + two AudioStreaming interfaces: the host
     * sees a single headset, not a microphone and a speaker. */
    TEST_ASSERT_EQUAL_MESSAGE(1, sum.iads, "duplex audio must be one interface association");
    TEST_ASSERT_EQUAL_MESSAGE(3, sum.audio_interfaces, "expected AC + 2 AS interfaces");
    /* Each AS interface must offer a zero-bandwidth alternate 0 as well as the
     * streaming alternate 1, or the host can never release the bandwidth
     * reservation (FW-AUD-026, FW-AUD-057). AC has no alternates. */
    TEST_ASSERT_EQUAL_MESSAGE(5, sum.audio_itf_descs, "each AS interface needs alt 0 and alt 1");
    TEST_ASSERT_EQUAL(3, d[4]);                       /* bNumInterfaces */
    TEST_ASSERT_EQUAL(len, (size_t)(d[2] | (d[3] << 8)));

    /* Three endpoints: iso IN (mic), iso OUT (speaker), iso IN (feedback). */
    TEST_ASSERT_EQUAL(3, sum.endpoints);
    int iso_in = 0, iso_out = 0, fb = 0;
    for (int i = 0; i < sum.endpoints; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(0x01, sum.eps[i].attr & 0x03, "audio endpoints must be isochronous");
        TEST_ASSERT_EQUAL_MESSAGE(1, sum.eps[i].interval, "every audio endpoint is 1 ms");
        if (EP_IS_FEEDBACK(sum.eps[i].attr)) {
            fb++;
            TEST_ASSERT_EQUAL_MESSAGE(0x80, sum.eps[i].addr & 0x80, "feedback must be an IN endpoint");
            TEST_ASSERT_EQUAL_MESSAGE(4, sum.eps[i].mps, "UAC2 feedback packets are 4 bytes");
        } else if (sum.eps[i].addr & 0x80) {
            iso_in++;
            TEST_ASSERT_EQUAL(34, sum.eps[i].mps);    /* (16 + 1) smp x 1ch x 2 B */
        } else {
            iso_out++;
            TEST_ASSERT_EQUAL(196, sum.eps[i].mps);   /* (48 + 1) smp x 2ch x 2 B */
        }
    }
    TEST_ASSERT_EQUAL_MESSAGE(1, iso_in, "exactly one mic data endpoint");
    TEST_ASSERT_EQUAL_MESSAGE(1, iso_out, "exactly one speaker data endpoint");
    TEST_ASSERT_EQUAL_MESSAGE(1, fb, "exactly one feedback endpoint");

    usb_ep_budget_t used;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_budget(&used, NULL));
    TEST_ASSERT_EQUAL(2, used.in_endpoints);   /* mic + feedback */
    TEST_ASSERT_EQUAL(1, used.out_endpoints);  /* speaker */
}

TEST_CASE("headset: a mono speaker and a stereo mic also assemble", "[usb]")
{
    /* Channel count changes the feature unit's length, which is the one part of
     * the hand-built descriptor whose size is not constant. walk() asserts that
     * the emitted blocks tile the declared total exactly, so a length computed
     * differently from the bytes written fails right here. */
    fresh_headset();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    usb_audio_config_t cfg = headset_cfg();
    cfg.mic.channels = 2;
    cfg.speaker.channels = 1;
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_init(&cfg));

    const uint8_t *d = NULL;
    size_t len = 0;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_config_descriptor(&d, &len));
    desc_summary_t sum;
    walk(d, len, &sum);
    TEST_ASSERT_EQUAL(3, sum.audio_interfaces);
    TEST_ASSERT_EQUAL(3, sum.endpoints);
}

TEST_CASE("headset: speaker-only is a two-interface function with a feedback endpoint", "[usb]")
{
    fresh_headset();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    usb_audio_config_t cfg = headset_cfg();
    cfg.direction = USB_AUDIO_DIR_SPEAKER;
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_init(&cfg));

    const uint8_t *d = NULL;
    size_t len = 0;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_config_descriptor(&d, &len));
    desc_summary_t sum;
    walk(d, len, &sum);
    TEST_ASSERT_EQUAL(2, sum.audio_interfaces);   /* AC + 1 AS */
    TEST_ASSERT_EQUAL(3, sum.audio_itf_descs);    /* AC, AS alt 0, AS alt 1 */
    TEST_ASSERT_EQUAL(2, sum.endpoints);          /* iso OUT + feedback IN */

    int fb = 0, out = 0;
    for (int i = 0; i < sum.endpoints; i++) {
        if (EP_IS_FEEDBACK(sum.eps[i].attr)) { fb++; } else { out++; }
    }
    TEST_ASSERT_EQUAL(1, fb);
    TEST_ASSERT_EQUAL(1, out);

    usb_ep_budget_t used;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_budget(&used, NULL));
    TEST_ASSERT_EQUAL(1, used.in_endpoints);      /* feedback only */
    TEST_ASSERT_EQUAL(1, used.out_endpoints);
}

TEST_CASE("headset: composite with HID still fits the endpoint budget", "[usb]")
{
    fresh_headset();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    usb_audio_config_t cfg = headset_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_init(&cfg));
    usb_hid_config_t hid = {
        .report_descriptor = s_hid_report_desc,
        .report_descriptor_len = sizeof(s_hid_report_desc),
    };
    TEST_ASSERT_EQUAL(ESP_OK, usb_hid_init(&hid));

    usb_ep_budget_t used, limit;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_budget(&used, &limit));
    TEST_ASSERT_EQUAL(3, used.in_endpoints);      /* mic + feedback + HID */
    TEST_ASSERT_EQUAL(1, used.out_endpoints);
    TEST_ASSERT_TRUE_MESSAGE(used.in_endpoints <= limit.in_endpoints, "IN endpoints over budget");
    TEST_ASSERT_TRUE_MESSAGE(used.tx_fifo_bytes <= limit.tx_fifo_bytes, "TX FIFO over budget");

    const uint8_t *d = NULL;
    size_t len = 0;
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_config_descriptor(&d, &len));
    desc_summary_t sum;
    walk(d, len, &sum);
    TEST_ASSERT_EQUAL(4, d[4]);                   /* AC + 2 AS + HID */
    TEST_ASSERT_EQUAL(4, sum.endpoints);
    TEST_ASSERT_EQUAL(1, sum.hid_ep_interval);
}

#endif /* CONFIG_TINYUSB_AUDIO_SPEAKER_ENABLED */

#if CONFIG_TINYUSB_AUDIO_SPEAKER_ENABLED
TEST_CASE("headset: enumerate the duplex descriptor against a real host", "[usb][needs_usb_host]")
{
    /* Starts the stack, so the USJ console dies here and takes Unity's output with
     * it. The result is read from the HOST instead - on Windows:
     *
     *     Get-PnpDevice -Class MEDIA | Where-Object FriendlyName -match 'TinyUSB|ESP'
     *     Get-PnpDevice -Status ERROR
     *
     * A descriptor the host rejects appears with a problem code rather than as an
     * audio device; a descriptor it accepts appears once, with both a capture and a
     * render endpoint, because FW-AUD-052 makes it one function. The device is held
     * up for 90 s so there is time to look. */
    fresh_headset();
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_init(NULL));
    usb_audio_config_t uac = headset_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_init(&uac));
    usb_hid_config_t hid = {
        .report_descriptor = s_hid_report_desc,
        .report_descriptor_len = sizeof(s_hid_report_desc),
    };
    TEST_ASSERT_EQUAL(ESP_OK, usb_hid_init(&hid));

    TEST_ASSERT_EQUAL(ESP_OK, usb_device_start());

    /* Report a backlog sitting exactly on the setpoint, so if the host does start
     * streaming the servo has a sane starting point rather than slamming a clamp. */
    usb_audio_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_stats(&st));
    s_fake_backlog = st.speaker.target_bytes;

    for (int i = 0; i < 90; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    TEST_ASSERT_TRUE_MESSAGE(usb_device_is_mounted(), "host never enumerated the device");
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_stop());
    TEST_ASSERT_EQUAL(ESP_OK, usb_device_deinit());
}
#endif

void app_main(void)
{
    UNITY_BEGIN();
#if CONFIG_TEST_USB_HOST_ATTACHED
    /* Opt-in build (sdkconfig.hostattach.defaults): ONLY the bus-touching cases.
     * This severs a USB-Serial/JTAG console, so the result is read from the host. */
    unity_run_tests_by_tag("[needs_usb_host]", false);
#else
    /* Bus-touching tests are opt-in: they need a console that is not USB-Serial/JTAG. */
    unity_run_tests_by_tag("[needs_usb_host]", true);
#endif
    UNITY_END();
}
