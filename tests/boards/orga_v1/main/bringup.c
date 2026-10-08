/**
 * ORGA v1 bring-up console.
 *
 * For the first powered boards: one command per part, so each IC can be proven alive on
 * its own before the headset firmware runs on top of them. The console is a REPL on UART0
 * (the programming pads), so it survives anything the USB-C does.
 *
 *   help                    list commands
 *   i2c_scan                probe 0x08..0x77 (expect 0x18 codec, 0x6A IMU)
 *   codec init|start|stop|regs|vol <0..127>|mic on|off
 *   tone <hz> [seconds]     sine to both drivers through the codec
 *   mic pdm|boom [seconds]  capture 16 kHz mono, print RMS dBFS every 100 ms
 *   imu accel|pose [seconds]
 *   led <r> <g> <b>         0..255 each
 *   enc [seconds]           print wheel steps
 *   btn [seconds]           print switch and jack events
 *   bat                     battery mV, %, external power
 *   latch off               release PWR_HOLD (powers the board off once POWER is released)
 *
 * Build: idf.py -C tests/boards/orga_v1 build (the board is forced to orga_v1). Flash over
 * the UART pads or USB; never to a devkit (it drives GPIO21/10).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_console.h"
#include "esp_log.h"

#include "audio_capture.h"
#include "audio_playback.h"
#include "battery.h"
#include "board.h"
#include "buttons.h"
#include "cfg.h"
#include "diag.h"
#include "encoder.h"
#include "lsm6dsv16x.h"
#include "pm_policy.h"
#include "power.h"
#include "power_latch.h"
#include "rgb_led.h"
#include "ringbuf.h"
#include "tlv320aic3104.h"

#define SPK_RATE   48000
#define MIC_RATE   16000
#define PI_F       3.14159265358979f

static const board_desc_t *B;
static bool s_codec, s_led, s_imu, s_bat, s_btn, s_enc;
static ringbuf_t s_ring;
static uint8_t  s_ring_mem[SPK_RATE * 2 * 2 / 1000 * 60];   /* 60 ms stereo */

static int secs_arg(int argc, char **argv, int i, int def)
{
    return (argc > i) ? atoi(argv[i]) : def;
}

/* -------------------------------------------------------------------------- */

static int cmd_i2c_scan(int argc, char **argv)
{
    i2c_master_bus_handle_t bus;
    if (board_i2c_bus(&bus) != ESP_OK) {
        printf("no I2C bus\n");
        return 1;
    }
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(bus, a, 20) == ESP_OK) {
            printf("  0x%02x%s\n", a, a == 0x18 ? "  TLV320AIC3104" : a == 0x6A ? "  LSM6DSV16X" : "");
        }
    }
    return 0;
}

static esp_err_t codec_init(void)
{
    if (s_codec) {
        return ESP_OK;
    }
    i2c_master_bus_handle_t bus;
    esp_err_t err = board_i2c_bus(&bus);
    if (err != ESP_OK) {
        return err;
    }
    const tlv320aic3104_config_t c = {
        .bus = bus, .i2c_addr = B->codec.i2c_addr, .reset_gpio = B->codec.reset,
        .mclk_hz = B->spk.mclk_hz, .sample_rate_hz = SPK_RATE,
        .mic = TLV320AIC3104_MIC_MIC1LP_SE, .micbias = TLV320AIC3104_MICBIAS_2V5,
    };
    err = tlv320aic3104_init(&c);
    s_codec = (err == ESP_OK);
    return err;
}

static esp_err_t playback_up(void)
{
    const audio_playback_config_t pb = {
        .source = &s_ring, .sample_rate_hz = SPK_RATE, .channels = 2, .bits_per_sample = 16,
        .port = B->spk.port,
        .pins = { .bclk = B->spk.bclk, .ws = B->spk.ws, .dout = B->spk.dout, .mclk = B->spk.mclk },
    };
    esp_err_t err = audio_playback_init(&pb);
    if (err == ESP_OK) {
        err = audio_playback_start();   /* MCLK runs from here: the codec needs it */
    }
    return err;
}

static int cmd_codec(int argc, char **argv)
{
    if (argc < 2) {
        printf("codec init|start|stop|regs|vol <n>|mic on|off\n");
        return 1;
    }
    esp_err_t err = ESP_OK;
    if (strcmp(argv[1], "init") == 0) {
        err = codec_init();
    } else if (!s_codec) {
        printf("codec init first\n");
        return 1;
    } else if (strcmp(argv[1], "start") == 0) {
        err = tlv320aic3104_start();
    } else if (strcmp(argv[1], "stop") == 0) {
        err = tlv320aic3104_stop();
    } else if (strcmp(argv[1], "vol") == 0 && argc > 2) {
        err = tlv320aic3104_set_volume((uint8_t)atoi(argv[2]));
    } else if (strcmp(argv[1], "mic") == 0 && argc > 2) {
        err = tlv320aic3104_mic_enable(strcmp(argv[2], "on") == 0);
    } else if (strcmp(argv[1], "regs") == 0) {
        static const uint8_t regs[] = { 2, 3, 7, 8, 9, 15, 19, 25, 37, 38, 40, 42, 43, 44, 47, 51, 58, 64, 65, 72, 94, 95, 101, 102 };
        for (unsigned i = 0; i < sizeof(regs); i++) {
            uint8_t v = 0;
            err = tlv320aic3104_reg_read(regs[i], &v);
            printf("  r%-3u = 0x%02x%s\n", regs[i], v, err == ESP_OK ? "" : " (read failed)");
        }
    }
    tlv320aic3104_stats_t st;
    (void)tlv320aic3104_stats(&st);
    printf("%s; i2c errors %u, r94 0x%02x, r95 0x%02x\n", esp_err_to_name(err), (unsigned)st.i2c_errors,
           st.power_status, st.short_circuit);
    return err == ESP_OK ? 0 : 1;
}

static int cmd_tone(int argc, char **argv)
{
    float hz = (argc > 1) ? (float)atof(argv[1]) : 1000.0f;
    int secs = secs_arg(argc, argv, 2, 3);
    if (codec_init() != ESP_OK) {
        printf("codec not answering\n");
        return 1;
    }
    (void)ringbuf_init(&s_ring, s_ring_mem, sizeof(s_ring_mem));
    if (playback_up() != ESP_OK) {
        printf("playback failed\n");
        return 1;
    }
    esp_err_t err = tlv320aic3104_start();
    printf("codec start: %s; %.0f Hz for %d s\n", esp_err_to_name(err), (double)hz, secs);
    int16_t frame[96 * 2];   /* 2 ms */
    float ph = 0.0f;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(secs * 1000);
    while (xTaskGetTickCount() < end) {
        audio_playback_stats_t st;
        (void)audio_playback_stats(&st);
        if (st.source_backlog_bytes > sizeof(s_ring_mem) / 2) {
            vTaskDelay(1);
            continue;
        }
        for (int i = 0; i < 96; i++) {
            int16_t v = (int16_t)(8000.0f * sinf(ph));
            frame[2 * i] = v;
            frame[2 * i + 1] = v;
            ph += 2.0f * PI_F * hz / SPK_RATE;
            if (ph > 2.0f * PI_F) {
                ph -= 2.0f * PI_F;
            }
        }
        (void)ringbuf_write(&s_ring, (const uint8_t *)frame, sizeof(frame));
    }
    (void)tlv320aic3104_stop();
    (void)audio_playback_deinit();
    return 0;
}

static int cmd_mic(int argc, char **argv)
{
    bool boom = (argc > 1 && strcmp(argv[1], "boom") == 0);
    int secs = secs_arg(argc, argv, 2, 3);
    audio_capture_config_t c = { .sample_rate_hz = MIC_RATE, .channels = 1, .bits_per_sample = 16 };
    if (boom) {
        if (codec_init() != ESP_OK) {
            printf("codec not answering\n");
            return 1;
        }
        (void)ringbuf_init(&s_ring, s_ring_mem, sizeof(s_ring_mem));
        if (playback_up() != ESP_OK) {   /* the boom's clocks come from the playback TX */
            printf("playback (clock master) failed\n");
            return 1;
        }
        (void)tlv320aic3104_mic_enable(true);
        c.interface = AUDIO_CAPTURE_IF_I2S_STD;
        c.port = B->spk.port == 1 ? AUDIO_CAPTURE_PORT_I2S1 : AUDIO_CAPTURE_PORT_I2S0;
        c.bus_slave = true;
        c.decimation = SPK_RATE / MIC_RATE;
        c.pins.clk = B->spk.bclk;
        c.pins.ws = B->spk.ws;
        c.pins.din = B->codec.din;
        c.pins.mclk = -1;
    } else {
        c.interface = AUDIO_CAPTURE_IF_PDM;
        c.port = AUDIO_CAPTURE_PORT_I2S0;
        c.pdm_oversample = board_mic_pdm_oversample(MIC_RATE);
        c.slot = B->mic.right_slot ? AUDIO_CAPTURE_SLOT_RIGHT : AUDIO_CAPTURE_SLOT_LEFT;
        c.pins.clk = B->mic.clk;
        c.pins.ws = -1;
        c.pins.din = B->mic.din;
        c.pins.mclk = -1;
    }
    esp_err_t err = audio_capture_init(&c);
    ringbuf_reader_t r;
    if (err == ESP_OK) {
        err = ringbuf_reader_open(audio_capture_get_ring(), &r);
    }
    if (err == ESP_OK) {
        err = audio_capture_start();
    }
    if (err != ESP_OK) {
        printf("capture: %s\n", esp_err_to_name(err));
    } else {
        for (int t = 0; t < secs * 10; t++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            int16_t buf[1600];
            size_t got = 0;
            (void)ringbuf_read(&r, (uint8_t *)buf, sizeof(buf), &got);
            double acc = 0.0;
            size_t n = got / 2;
            for (size_t i = 0; i < n; i++) {
                acc += (double)buf[i] * buf[i];
            }
            double rms = n ? sqrt(acc / (double)n) : 0.0;
            printf("  %s %6.1f dBFS (%u samples)\n", boom ? "boom" : "pdm ", 20.0 * log10(rms / 32768.0 + 1e-9),
                   (unsigned)n);
        }
    }
    (void)audio_capture_deinit();
    if (boom) {
        (void)tlv320aic3104_mic_enable(false);
        (void)audio_playback_deinit();
    }
    return err == ESP_OK ? 0 : 1;
}

static void on_pose(const lsm6dsv16x_quat_t *q, void *ctx)
{
    static uint32_t n;
    if (++n % 15 == 0) {
        printf("  q = %.3f %.3f %.3f %.3f\n", (double)q->w, (double)q->x, (double)q->y, (double)q->z);
    }
}

static void on_motion(bool moving, void *ctx)
{
    printf("  IMU: %s\n", moving ? "moving" : "still");
}

static int cmd_imu(int argc, char **argv)
{
    if (!s_imu) {
        i2c_master_bus_handle_t bus;
        const lsm6dsv16x_config_t c = { .bus = NULL, .int1_gpio = B->imu.int1, .on_pose = on_pose, .on_motion = on_motion };
        lsm6dsv16x_config_t cc = c;
        if (board_i2c_bus(&bus) != ESP_OK) {
            return 1;
        }
        cc.bus = bus;
        cc.i2c_addr = B->imu.i2c_addr;
        esp_err_t err = lsm6dsv16x_init(&cc);
        if (err == ESP_OK) {
            err = lsm6dsv16x_start();
        }
        printf("imu: %s\n", esp_err_to_name(err));
        s_imu = (err == ESP_OK);
        if (!s_imu) {
            return 1;
        }
    }
    int secs = secs_arg(argc, argv, 2, 3);
    if (argc > 1 && strcmp(argv[1], "pose") == 0) {
        (void)lsm6dsv16x_pose_enable(true);
        vTaskDelay(pdMS_TO_TICKS(secs * 1000));
        (void)lsm6dsv16x_pose_enable(false);
    } else {
        for (int i = 0; i < secs * 4; i++) {
            float a[3];
            if (lsm6dsv16x_read_accel_mg(a) == ESP_OK) {
                printf("  accel %6.0f %6.0f %6.0f mg\n", (double)a[0], (double)a[1], (double)a[2]);
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
    lsm6dsv16x_stats_t st;
    (void)lsm6dsv16x_stats(&st);
    printf("  i2c errors %u, poses %u, motion events %u\n", (unsigned)st.i2c_errors, (unsigned)st.pose_samples,
           (unsigned)st.motion_events);
    return 0;
}

static int cmd_led(int argc, char **argv)
{
    if (!s_led) {
        const rgb_led_config_t c = { .gpio_r = B->led.r, .gpio_g = B->led.g, .gpio_b = B->led.b,
                                     .active_low = B->led.active_low,
                                     .gain_permille = { B->led.gain_permille[0], B->led.gain_permille[1],
                                                        B->led.gain_permille[2] } };
        s_led = (rgb_led_init(&c) == ESP_OK);
    }
    rgb_led_color_t col = { (uint8_t)(argc > 1 ? atoi(argv[1]) : 0), (uint8_t)(argc > 2 ? atoi(argv[2]) : 0),
                            (uint8_t)(argc > 3 ? atoi(argv[3]) : 0) };
    return rgb_led_set(col) == ESP_OK ? 0 : 1;
}

static void on_step(int32_t steps, void *ctx)
{
    printf("  wheel %+ld (position %ld)\n", (long)steps, (long)encoder_position());
}

static int cmd_enc(int argc, char **argv)
{
    if (!s_enc) {
        const encoder_config_t c = { .gpio_a = B->encoder.a, .gpio_b = B->encoder.b,
                                     .counts_per_detent = B->encoder.counts_per_detent, .cb = on_step };
        s_enc = (encoder_init(&c) == ESP_OK && encoder_start() == ESP_OK);
    }
    vTaskDelay(pdMS_TO_TICKS(secs_arg(argc, argv, 1, 10) * 1000));
    return 0;
}

static void on_btn(buttons_event_t evt, const buttons_event_info_t *info, void *ctx)
{
    static const char *const ev[] = { "pressed", "released", "long", "repeat" };
    const char *what = info->gpio == B->jack.gpio ? "jack" : info->gpio == 41 ? "POWER" : info->gpio == 42 ? "MFB" : "?";
    printf("  %s (GPIO%d) %s, held %u ms\n", what, info->gpio, ev[evt], (unsigned)info->held_ms);
}

static int cmd_btn(int argc, char **argv)
{
    if (!s_btn) {
        buttons_pin_config_t pins[4];
        uint8_t n = 0;
        for (uint8_t i = 0; i < B->button_count; i++) {
            pins[n++] = (buttons_pin_config_t){ .gpio = B->buttons[i].gpio, .active_low = B->buttons[i].active_low };
        }
        pins[n++] = (buttons_pin_config_t){ .gpio = B->jack.gpio, .active_low = !B->jack.plugged_high };
        const buttons_config_t c = { .pins = pins, .count = n, .cb = on_btn };
        s_btn = (buttons_init(&c) == ESP_OK && buttons_start() == ESP_OK);
    }
    vTaskDelay(pdMS_TO_TICKS(secs_arg(argc, argv, 1, 10) * 1000));
    return 0;
}

static int cmd_bat(int argc, char **argv)
{
    if (!s_bat) {
        const battery_config_t c = { .adc_gpio = B->battery.adc_gpio, .divider_num = B->battery.divider_num,
                                     .divider_den = B->battery.divider_den, .ext_power_gpio = B->battery.ext_power_gpio,
                                     .ext_power_active_low = B->battery.ext_power_active_low };
        s_bat = (battery_init(&c) == ESP_OK);
    }
    battery_status_t st;
    if (!s_bat || battery_sample_now(&st) != ESP_OK) {
        printf("battery: no reading\n");
        return 1;
    }
    battery_stats_t bs;
    (void)battery_stats(&bs);
    printf("  %u mV, %u%%, external power %s%s\n", st.mv, st.percent, st.ext_power ? "yes" : "no",
           bs.calibrated ? "" : " (ADC uncalibrated)");
    return 0;
}

static int cmd_latch(int argc, char **argv)
{
    if (argc < 2 || strcmp(argv[1], "off") != 0) {
        printf("latch off\n");
        return 1;
    }
    printf("releasing PWR_HOLD; release the POWER button\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_err_t err = power_latch_release();
    printf("still powered: %s\n", esp_err_to_name(err));
    return 1;
}

static void reg(const char *name, const char *help, esp_console_cmd_func_t fn)
{
    const esp_console_cmd_t c = { .command = name, .help = help, .func = fn };
    ESP_ERROR_CHECK(esp_console_cmd_register(&c));   /* a typo here is a build-time bug */
}

void app_main(void)
{
    B = board_desc();
    const power_latch_config_t lc = { .hold_gpio = B->power_latch.hold, .active_low = B->power_latch.active_low };
    (void)power_latch_init(&lc);   /* first: keep the power on */
    (void)board_init();
    (void)cfg_init();
    (void)diag_init(NULL);
    (void)power_init();
    (void)pm_policy_init();

    printf("\nORGA v1 bring-up. Board: %s. Type 'help'.\n", B->name);
    reg("i2c_scan", "Probe the I2C bus", cmd_i2c_scan);
    reg("codec", "codec init|start|stop|regs|vol <0..127>|mic on|off", cmd_codec);
    reg("tone", "tone <hz> [s]: sine through the codec", cmd_tone);
    reg("mic", "mic pdm|boom [s]: print levels", cmd_mic);
    reg("imu", "imu accel|pose [s]", cmd_imu);
    reg("led", "led <r> <g> <b>", cmd_led);
    reg("enc", "enc [s]: print wheel steps", cmd_enc);
    reg("btn", "btn [s]: print switch and jack events", cmd_btn);
    reg("bat", "battery voltage and external power", cmd_bat);
    reg("latch", "latch off: power down", cmd_latch);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt = "orga> ";
    esp_console_dev_uart_config_t uc = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_register_help_command());
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uc, &rc, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
