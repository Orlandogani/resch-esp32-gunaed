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
    uint8_t interfaces;
    uint8_t iads;
    uint8_t endpoints;
    uint8_t ep_addrs[16];
    uint8_t hid_ep_interval;
    uint8_t iso_ep_interval;
    uint16_t iso_ep_size;
    uint8_t hid_count;
} desc_summary_t;

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
        .ring = &s_ring, .sample_rate_hz = 16000, .channels = 1, .bits_per_sample = 16,
    };
    usb_audio_config_t bad;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(NULL));
    bad = cfg; bad.ring = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    bad = cfg; bad.sample_rate_hz = 96000;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    bad = cfg; bad.bits_per_sample = 12;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, usb_audio_init(&bad));
    bad = cfg; bad.channels = 2;
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, usb_audio_init(&bad));

    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_init(&cfg));
    TEST_ASSERT_FALSE(usb_audio_is_streaming());
    usb_audio_stats_t st;
    TEST_ASSERT_EQUAL(ESP_OK, usb_audio_stats(&st));
    TEST_ASSERT_EQUAL(32, st.nominal_packet_bytes); /* 16 samples/ms × 2 B */
    TEST_ASSERT_EQUAL(0, st.packets);
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
        .ring = &s_ring, .sample_rate_hz = 48000, .channels = 1, .bits_per_sample = 16,
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

void app_main(void)
{
    UNITY_BEGIN();
    /* Bus-touching tests are opt-in: they need a console that is not USB-Serial/JTAG. */
    unity_run_tests_by_tag("[needs_usb_host]", true);
    UNITY_END();
}
