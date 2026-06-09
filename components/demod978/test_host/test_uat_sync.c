/* Host unit test for the UAT FSK demod core (../demod978.c, pure part) wired to
 * the RS FEC (../uat_fec.c). It SYNTHESIZES a 2-FSK I/Q waveform at 2.4 Msps for
 * a known message — sync word + RS-encoded payload — then runs the production
 * discriminator + uat_core_process over it and asserts the decoded payload equals
 * the original. This exercises: discriminator sign, fractional-sample bit timing,
 * 36-bit sync correlation, and the short/long/uplink dispatch into the FEC.
 *
 * Compiled with -DUAT_HOST_TEST so demod978.c excludes its FreeRTOS task shell and
 * exposes only the pure core. NOT part of the firmware build. */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#define UAT_HOST_TEST 1
#include "demod978_internal.h"
#include "uat_fec.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); g_fail++; } else { printf("ok  : %s\n", msg); } } while (0)

/* ── Synthetic 2-FSK modulator ───────────────────────────────────────────────
 * Generate interleaved unsigned 8-bit I/Q for a bit stream at the UAT bit rate
 * over a sample rate of fs. A "1" advances the carrier phase at +dev, a "0" at
 * -dev (continuous phase). The receiver only cares about the SIGN of the
 * instantaneous frequency, so exact deviation/amplitude are not critical; we use
 * a healthy deviation and full-scale amplitude. */
static uint32_t fsk_modulate(const uint8_t *bits, int nbits, double fs,
                             uint8_t *iq, uint32_t iq_cap)
{
    const double bitrate = (double)UAT_BIT_RATE_HZ;
    const double samp_per_bit = fs / bitrate;
    const double dev = 312500.0;                 /* ±312.5 kHz nominal */
    const double two_pi = 6.283185307179586;
    double phase = 0.0;
    double t = 0.0;                              /* sample counter */
    uint32_t w = 0;

    /* Walk samples; the bit index is floor(sample / samp_per_bit). We emit a few
     * extra bits of run-out past the last data bit so the demod's integrate-and-
     * dump window around the FINAL bit centre stays inside the buffer (a real
     * burst always has signal after its last bit; without this the last bit's
     * window falls off the end and the frame is dropped at the boundary). */
    double total_samples = samp_per_bit * (double)(nbits + 4);
    for (uint32_t s = 0; s < (uint32_t)total_samples && w + 1 < iq_cap; s++) {
        int bi = (int)((double)s / samp_per_bit);
        if (bi >= nbits) {
            bi = nbits - 1;   /* hold the last bit during the run-out. */
        }
        double f = (bits[bi] ? +dev : -dev);
        phase += two_pi * f / fs;
        /* Wrap phase to keep cos/sin well-conditioned. */
        if (phase > two_pi)  phase -= two_pi;
        if (phase < -two_pi) phase += two_pi;

        double i = cos(phase);
        double q = sin(phase);
        /* Scale to offset-binary 8-bit around 127. */
        int ii = (int)lround(127.0 + 120.0 * i);
        int qq = (int)lround(127.0 + 120.0 * q);
        if (ii < 0)   { ii = 0;   }
        if (ii > 255) { ii = 255; }
        if (qq < 0)   { qq = 0;   }
        if (qq > 255) { qq = 255; }
        iq[2 * w]     = (uint8_t)ii;
        iq[2 * w + 1] = (uint8_t)qq;
        w++;
        (void)t;
    }
    return w;   /* IQ pairs produced */
}

/* Expand a 36-bit sync word + coded bytes into an MSB-first bit array. */
static int build_bits(uint64_t sync, const uint8_t *coded, int coded_len,
                      uint8_t *bits)
{
    int n = 0;
    for (int i = UAT_SYNC_BITS - 1; i >= 0; i--) {
        bits[n++] = (uint8_t)((sync >> i) & 1);
    }
    for (int b = 0; b < coded_len; b++) {
        for (int i = 7; i >= 0; i--) {
            bits[n++] = (uint8_t)((coded[b] >> i) & 1);
        }
    }
    return n;
}

/* Emit collector for uat_core_process. */
typedef struct { uat_decoded_msg_t last; int count; } collector_t;
static void collect(const uat_decoded_msg_t *m, void *user)
{
    collector_t *c = (collector_t *)user;
    c->last = *m;
    c->count++;
}

int main(void)
{
    uat_fec_init();
    const double fs = 2400000.0;
    uint64_t fp_per_bit = ((uint64_t)2400000u << UAT_FP_SHIFT) / UAT_BIT_RATE_HZ;

    /* Buffers sized for the largest frame (uplink: 36 + 552*8 bits). */
    static uint8_t bits[36 + 552 * 8 + 16];
    static uint8_t iq[1u << 20];                 /* plenty for any single frame */
    static int16_t disc[1u << 19];

    /* 1) Basic ADS-B (18-byte payload, RS(30,18)) round-trip through the waveform. */
    {
        uint8_t cw[30];
        for (int i = 0; i < 18; i++) cw[i] = (uint8_t)(i * 11 + 5);
        cw[0] &= 0x07;                           /* message type 0 => short. */
        uat_fec_rs_encode(cw, 30, 18);

        int nbits = build_bits(UAT_ADSB_SYNC_WORD, cw, 30, bits);
        /* Prefix some idle samples so sync is not at index 0 (more realistic). */
        memset(iq, 127, 2 * 200);
        uint32_t pre = 200;
        uint32_t got = fsk_modulate(bits, nbits, fs, &iq[2 * pre], (sizeof(iq)/2) - pre);
        uint32_t npairs = pre + got;

        uint32_t nd = uat_discriminator(iq, npairs, disc);
        collector_t col = {0};
        uint64_t a = 0, u = 0, f = 0;
        int n = uat_core_process(disc, nd, fp_per_bit, 4, collect, &col, &a, &u, &f);

        CHECK(n >= 1 && col.count >= 1, "basic ADS-B: a frame is recovered");
        CHECK(col.last.kind == UAT_MSG_ADSB_SHORT, "basic ADS-B: kind is short");
        CHECK(col.last.len == 18 && memcmp(col.last.data, cw, 18) == 0,
              "basic ADS-B: 18-byte payload matches original");
    }

    /* 2) Long ADS-B (34-byte payload, RS(48,34)). Message type != 0 => long. */
    {
        uint8_t cw[48];
        for (int i = 0; i < 34; i++) cw[i] = (uint8_t)(i * 7 + 1);
        cw[0] = (uint8_t)((1 << 3) | (cw[0] & 0x07));   /* message type 1 => long. */
        uat_fec_rs_encode(cw, 48, 34);

        int nbits = build_bits(UAT_ADSB_SYNC_WORD, cw, 48, bits);
        memset(iq, 127, 2 * 200);
        uint32_t pre = 200;
        uint32_t got = fsk_modulate(bits, nbits, fs, &iq[2 * pre], (sizeof(iq)/2) - pre);
        uint32_t npairs = pre + got;

        uint32_t nd = uat_discriminator(iq, npairs, disc);
        collector_t col = {0};
        uint64_t a = 0, u = 0, f = 0;
        int n = uat_core_process(disc, nd, fp_per_bit, 4, collect, &col, &a, &u, &f);

        CHECK(n >= 1 && col.last.kind == UAT_MSG_ADSB_LONG, "long ADS-B: kind is long");
        CHECK(col.last.len == 34 && memcmp(col.last.data, cw, 34) == 0,
              "long ADS-B: 34-byte payload matches original");
    }

    /* 3) Uplink frame (432-byte payload, 6× RS(92,72) interleaved). */
    {
        uint8_t blocks[6][92];
        uint8_t expect[432];
        for (int b = 0; b < 6; b++) {
            for (int i = 0; i < 72; i++) blocks[b][i] = (uint8_t)(b * 31 + i * 3 + 7);
            uat_fec_rs_encode(blocks[b], 92, 72);
            memcpy(&expect[b * 72], blocks[b], 72);
        }
        /* Interleave on the air: frame[i*6 + b] = blocks[b][i]. */
        uint8_t frame[552];
        for (int b = 0; b < 6; b++)
            for (int i = 0; i < 92; i++)
                frame[i * 6 + b] = blocks[b][i];

        int nbits = build_bits(UAT_UPLINK_SYNC_WORD, frame, 552, bits);
        memset(iq, 127, 2 * 200);
        uint32_t pre = 200;
        uint32_t got = fsk_modulate(bits, nbits, fs, &iq[2 * pre], (sizeof(iq)/2) - pre);
        uint32_t npairs = pre + got;

        uint32_t nd = uat_discriminator(iq, npairs, disc);
        collector_t col = {0};
        uint64_t a = 0, u = 0, f = 0;
        int n = uat_core_process(disc, nd, fp_per_bit, 4, collect, &col, &a, &u, &f);

        CHECK(n >= 1 && u >= 1, "uplink: an uplink sync hit and a frame");
        CHECK(col.last.kind == UAT_MSG_UPLINK, "uplink: kind is uplink");
        CHECK(col.last.len == 432 && memcmp(col.last.data, expect, 432) == 0,
              "uplink: 432-byte payload matches original");
    }

    /* 4) Pure noise (no sync) yields no frames. */
    {
        for (uint32_t i = 0; i < 20000; i++) {
            /* deterministic pseudo-noise around mid-scale */
            iq[i] = (uint8_t)(120 + ((i * 37) % 16));
        }
        uint32_t nd = uat_discriminator(iq, 10000, disc);
        collector_t col = {0};
        int n = uat_core_process(disc, nd, fp_per_bit, 4, collect, &col, NULL, NULL, NULL);
        CHECK(n == 0 && col.count == 0, "noise: no false frames");
    }

    printf("\n%s (%d failures)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
