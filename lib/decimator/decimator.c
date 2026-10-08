#include "decimator.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f

/* -6 dB point as a fraction of the output Nyquist frequency (DES-DEC-002). */
#define CUTOFF_OF_OUTPUT_NYQUIST 0.875f

static void design(decimator_t *d)
{
    const uint16_t n = d->taps;
    const float fc = (0.5f / (float)d->factor) * CUTOFF_OF_OUTPUT_NYQUIST;  /* cycles/input sample */
    const float mid = (float)(n - 1) / 2.0f;
    float h[DECIMATOR_MAX_TAPS];
    float sum = 0.0f;

    for (uint16_t i = 0; i < n; i++) {
        float x = (float)i - mid;
        float sinc = (x == 0.0f) ? 2.0f * fc : sinf(2.0f * PI_F * fc * x) / (PI_F * x);
        float w = 0.42f - 0.5f * cosf(2.0f * PI_F * (float)i / (float)(n - 1))
                + 0.08f * cosf(4.0f * PI_F * (float)i / (float)(n - 1));
        h[i] = sinc * w;
        sum += h[i];
    }

    /* Q15 with unity DC gain. Rounding leaves the taps summing to within a few LSB of
     * 32768; the residue goes into the centre tap so DC passes exactly (FW-DEC-003). */
    int32_t qsum = 0;
    for (uint16_t i = 0; i < n; i++) {
        float q = h[i] / sum * 32768.0f;
        int32_t v = (int32_t)lroundf(q);
        if (v > 32767) {
            v = 32767;
        } else if (v < -32768) {
            v = -32768;
        }
        d->coef[i] = (int16_t)v;
        qsum += v;
    }
    d->coef[n / 2] = (int16_t)(d->coef[n / 2] + (32768 - qsum));
}

esp_err_t decimator_init(decimator_t *d, uint8_t factor, uint8_t channels)
{
    if (d == NULL || factor < 2 || factor > DECIMATOR_MAX_FACTOR ||
        channels < 1 || channels > DECIMATOR_MAX_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(d, 0, sizeof(*d));
    d->factor = factor;
    d->channels = channels;
    d->taps = (uint16_t)(factor * DECIMATOR_TAPS_PER_PHASE);
    design(d);
    return ESP_OK;
}

void decimator_reset(decimator_t *d)
{
    if (d == NULL) {
        return;
    }
    memset(d->hist, 0, sizeof(d->hist));
    d->pos = 0;
    d->phase = 0;
}

size_t decimator_max_out_frames(const decimator_t *d, size_t in_frames)
{
    if (d == NULL || d->factor == 0) {
        return 0;
    }
    return in_frames / d->factor + 1;
}

/* One output sample: the newest `taps` inputs of channel `c` against the filter. The
 * double-stored history makes hist[pos .. pos+taps-1] the window, oldest first. The
 * filter is symmetric, so tap order does not matter. Worst-case |sum| is
 * sum(|coef|) x 32768 ≈ 1.3e9, inside int32 (DES-DEC-003). */
static int16_t convolve(const decimator_t *d, unsigned c)
{
    const int16_t *x = &d->hist[c][d->pos];
    int32_t acc = 1 << 14;   /* round half up on the >> 15 */
    for (uint16_t i = 0; i < d->taps; i++) {
        acc += (int32_t)d->coef[i] * x[i];
    }
    acc >>= 15;
    if (acc > 32767) {
        acc = 32767;
    } else if (acc < -32768) {
        acc = -32768;
    }
    return (int16_t)acc;
}

size_t decimator_process(decimator_t *d, const int16_t *in, size_t in_frames, int16_t *out)
{
    if (d == NULL || in == NULL || out == NULL || d->factor == 0) {
        return 0;
    }

    const unsigned ch = d->channels;
    size_t produced = 0;

    for (size_t f = 0; f < in_frames; f++) {
        for (unsigned c = 0; c < ch; c++) {
            int16_t s = in[f * ch + c];
            d->hist[c][d->pos] = s;
            d->hist[c][d->pos + d->taps] = s;
        }
        /* After this, hist[pos .. pos+taps-1] is the window ending at the newest sample. */
        d->pos = (uint16_t)((d->pos + 1u == d->taps) ? 0u : d->pos + 1u);

        if (++d->phase == d->factor) {
            d->phase = 0;
            /* out[produced] never overtakes in[f]: produced <= f / factor, so in-place
             * use is safe. */
            for (unsigned c = 0; c < ch; c++) {
                out[produced * ch + c] = convolve(d, c);
            }
            produced++;
        }
    }
    return produced;
}
