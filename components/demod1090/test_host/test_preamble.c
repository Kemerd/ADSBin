/* Host unit test for the 1090ES demod core (../demod1090.c, pure part). It
 * SYNTHESIZES Mode-S bursts at 2.4 Msps — 8 µs preamble + PPM data for known
 * frame bytes — at CONTROLLED SUB-SAMPLE ARRIVAL PHASES, pushes the interleaved
 * 8-bit I/Q through the production magnitude LUT + preamble correlator + PPM
 * slicer, and asserts the emitted candidate bytes match the transmitted frame.
 *
 * This is the test that never existed for the flight build: it exercises
 * exactly the stage the bench "+INJECT" path bypasses. The phase sweep is the
 * critical part — a point-sampling single-template detector passes only a
 * narrow slice of arrival phases and fails this suite immediately.
 *
 * Compiled with -DDEMOD1090_HOST_TEST so demod1090.c excludes its FreeRTOS task
 * shell and exposes the pure core. NOT part of the firmware build. */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define DEMOD1090_HOST_TEST 1
#include "demod1090_internal.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); g_fail++; } else { printf("ok  : %s\n", msg); } } while (0)

/* ── Emitted-frame capture (the hook demod1090.c calls in host mode) ─────────*/
#define CAP_MAX 256
static modes_frame_t g_cap[CAP_MAX];
static int           g_ncap = 0;

void demod1090_host_capture(const modes_frame_t *frame)
{
    if (g_ncap < CAP_MAX) {
        g_cap[g_ncap++] = *frame;
    }
}

static void cap_reset(void) { g_ncap = 0; }

/* Did ANY captured frame carry exactly these bytes? */
static int cap_has_frame(const uint8_t *bytes, int len)
{
    for (int i = 0; i < g_ncap; ++i) {
        if (g_cap[i].len_bytes == len && memcmp(g_cap[i].data, bytes, (size_t)len) == 0) {
            return 1;
        }
    }
    return 0;
}

/* Smallest bit-distance between the truth and any captured same-length frame
 * (INT_MAX-ish sentinel when none were captured). Separates DETECTION quality
 * from per-bit noise: a frame that arrives with 1-2 flipped bits still reached
 * the CRC stage — the demod did its job even though the exact-match test says
 * no. */
static int cap_best_hamming(const uint8_t *bytes, int len)
{
    int best = 1 << 20;
    for (int i = 0; i < g_ncap; ++i) {
        if (g_cap[i].len_bytes != len) {
            continue;
        }
        int d = 0;
        for (int b = 0; b < len; ++b) {
            uint8_t x = (uint8_t)(g_cap[i].data[b] ^ bytes[b]);
            while (x) { d += (x & 1); x >>= 1; }
        }
        if (d < best) best = d;
    }
    return best;
}

/* ── Gaussian noise (Box–Muller over rand(); fixed seed => deterministic) ────*/
static double gauss(void)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

/* ── Mode-S burst synthesizer ────────────────────────────────────────────────
 * Envelope model: the four 0.5 µs preamble pulses (leading edges 0/1/3.5/4.5 µs)
 * then one 1 µs PPM symbol per data bit starting at 8 µs — energy in the first
 * half-bit for a 1, the second half for a 0. env_at() returns the ideal
 * rectangle; the sampler below applies a one-pole smoothing so pulse edges have
 * realistic rise/fall (the RTL2832U output filter is ~rate/2 wide; crisp
 * rectangles would flatter a point-sampling detector). */
static int env_at(const uint8_t *bytes, int nbits, double t_us)
{
    /* Preamble pulses. */
    if ((t_us >= 0.0 && t_us < 0.5) || (t_us >= 1.0 && t_us < 1.5) ||
        (t_us >= 3.5 && t_us < 4.0) || (t_us >= 4.5 && t_us < 5.0)) {
        return 1;
    }
    /* Data PPM symbols: bit b occupies [8+b, 8+b+1) µs. */
    if (t_us >= 8.0 && t_us < 8.0 + (double)nbits) {
        const int    b    = (int)(t_us - 8.0);
        const double frac = (t_us - 8.0) - (double)b;
        const int    bit  = (bytes[b >> 3] >> (7 - (b & 7))) & 1;
        return bit ? (frac < 0.5) : (frac >= 0.5);
    }
    return 0;
}

/*
 * Fill `iq` (interleaved unsigned 8-bit I/Q around the 127.4 offset-binary
 * bias) with noise plus one burst whose PREAMBLE STARTS at the continuous
 * sample position `t0_samples` (the fractional part is the arrival phase under
 * test). Amplitude `amp`, per-component Gaussian noise `sigma`, random carrier
 * phase per call (envelope detection must not care).
 */
static void synth_burst(uint8_t *iq, uint32_t n_samples,
                        const uint8_t *bytes, int nbits,
                        double t0_samples, double amp, double sigma)
{
    const double sp    = 2.4;                       /* samples per µs           */
    const double theta = 6.283185307179586 * ((double)rand() / (double)RAND_MAX);
    const double ci = cos(theta), cq = sin(theta);

    double env_f = 0.0;                             /* one-pole smoothed env    */
    for (uint32_t s = 0; s < n_samples; ++s) {
        const double t_us = ((double)s - t0_samples) / sp;
        const double e    = (double)env_at(bytes, nbits, t_us);
        /* alpha 0.75: ~1-sample rise time, comparable to the dongle's output
         * low-pass. Enough ISI to punish edge-sampling without being cruel.    */
        env_f += 0.75 * (e - env_f);

        const double a  = amp * env_f;
        const double vi = 127.4 + a * ci + sigma * gauss();
        const double vq = 127.4 + a * cq + sigma * gauss();
        int ii = (int)lround(vi), qq = (int)lround(vq);
        if (ii < 0) ii = 0; if (ii > 255) ii = 255;
        if (qq < 0) qq = 0; if (qq > 255) qq = 255;
        iq[2 * s]     = (uint8_t)ii;
        iq[2 * s + 1] = (uint8_t)qq;
    }
}

/* Known frames. The demod does NO CRC (that is modes_decode's job), so any bit
 * pattern with the right DF works; we use a real-shaped DF17 and a DF11. */
static const uint8_t k_df17[14] = { 0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3,
                                    0x71, 0xC3, 0x2C, 0xE0, 0x57, 0x60, 0x98 };
static const uint8_t k_df11[7]  = { 0x5D, 0x48, 0x40, 0xD6, 0xB8, 0xF9, 0xC6 };

#define BUF_SAMPLES 1024u
#define BURST_AT    300.0   /* integer part of the burst position (samples)    */

/* Run `trials` bursts of the given frame across a uniform sweep of sub-sample
 * arrival phases; return how many decoded to the exact transmitted bytes. */
static int run_phase_sweep(const uint8_t *bytes, int nbits, int len_bytes,
                           int trials, double amp, double sigma)
{
    uint8_t iq[2 * BUF_SAMPLES];
    int hits = 0;
    for (int i = 0; i < trials; ++i) {
        /* Deterministic phase grid over [0,1): every fraction gets tested.     */
        const double phase = (double)i / (double)trials;
        synth_burst(iq, BUF_SAMPLES, bytes, nbits, BURST_AT + phase, amp, sigma);
        cap_reset();
        demod1090_host_process_iq(iq, sizeof(iq), 0);
        if (cap_has_frame(bytes, len_bytes)) {
            ++hits;
        }
    }
    return hits;
}

int main(void)
{
    srand(0xADB5);   /* fixed seed — the suite must be reproducible.            */

    CHECK(demod1090_host_setup(2400000u, 0) == ESP_OK, "host setup (LUT + geometry)");

    /* ── 1. Strong-signal phase sweep, long frame (THE regression test) ──────
     * Every sub-sample arrival phase must decode. The pre-fix detector point-
     * sampled one template and lost a large slice of phases outright.          */
    {
        const int trials = 240;
        const int hits = run_phase_sweep(k_df17, 112, 14, trials, 80.0, 3.0);
        printf("info: DF17 phase sweep: %d/%d exact decodes\n", hits, trials);
        CHECK(hits >= (trials * 97) / 100, "DF17 decodes at >=97% of arrival phases");
    }

    /* ── 2. Short frame (56-bit DF11) across the same sweep ─────────────────*/
    {
        const int trials = 120;
        const int hits = run_phase_sweep(k_df11, 56, 7, trials, 80.0, 3.0);
        printf("info: DF11 phase sweep: %d/%d exact decodes\n", hits, trials);
        CHECK(hits >= (trials * 95) / 100, "DF11 decodes at >=95% of arrival phases");
    }

    /* ── 3. Weak signal: graceful degradation, not a cliff ──────────────────*/
    {
        const int trials = 120;
        const int hits = run_phase_sweep(k_df17, 112, 14, trials, 30.0, 6.0);
        printf("info: weak DF17 (SNR ~14 dB): %d/%d exact decodes\n", hits, trials);
        CHECK(hits >= (trials * 60) / 100, "weak DF17 decodes at >=60% of phases");
    }

    /* ── 4. Pure noise: the gate + correlator must reject nearly everything ──
     * Realistic hot front end (49.6 dB gain => fat noise floor). False frames
     * here are harmless (CRC kills them downstream) but their RATE bounds the
     * Core-0/Core-1 garbage load, so keep it demonstrably low.                 */
    {
        enum { NBLK = 8, NS = 16384 };
        static uint8_t iq[2 * NS];
        uint64_t emitted = 0;
        demod1090_stats_t st0, st1;
        demod1090_get_stats(&st0);
        for (int b = 0; b < NBLK; ++b) {
            for (uint32_t s = 0; s < NS; ++s) {
                int ii = (int)lround(127.4 + 20.0 * gauss());
                int qq = (int)lround(127.4 + 20.0 * gauss());
                if (ii < 0) ii = 0; if (ii > 255) ii = 255;
                if (qq < 0) qq = 0; if (qq > 255) qq = 255;
                iq[2 * s] = (uint8_t)ii; iq[2 * s + 1] = (uint8_t)qq;
            }
            cap_reset();
            demod1090_host_process_iq(iq, sizeof(iq), 0);
        }
        demod1090_get_stats(&st1);
        emitted = st1.frames_emitted - st0.frames_emitted;
        printf("info: noise-only: %llu candidate frames from %d blocks (%.2f/blk)\n",
               (unsigned long long)emitted, NBLK, (double)emitted / NBLK);
        CHECK(emitted < (uint64_t)(NBLK * 150), "noise false-candidate rate bounded");
    }

    /* ── 5. Two bursts in one block both decode (advance logic sanity) ──────*/
    {
        uint8_t iq[2 * (2 * BUF_SAMPLES)];
        uint8_t tmp[2 * BUF_SAMPLES];
        synth_burst(tmp, BUF_SAMPLES, k_df17, 112, BURST_AT + 0.37, 80.0, 3.0);
        memcpy(iq, tmp, sizeof(tmp));
        synth_burst(tmp, BUF_SAMPLES, k_df11, 56, 200.0 + 0.81, 80.0, 3.0);
        memcpy(iq + sizeof(tmp), tmp, sizeof(tmp));
        cap_reset();
        demod1090_host_process_iq(iq, sizeof(iq), 0);
        CHECK(cap_has_frame(k_df17, 14) && cap_has_frame(k_df11, 7),
              "two bursts in one block both decode");
    }

    demod1090_host_teardown();

    printf(g_fail ? "\n%d FAILURE(S)\n" : "\nall tests passed\n", g_fail);
    return g_fail ? 1 : 0;
}
