/**
 * @file    demod1090.c
 * @brief   1090ES Mode-S demodulator front end (plan S4.2) — Core-0 DSP.
 *
 * @details
 *   This is the real-time signal-processing stage of ADSBin. It pulls raw 8-bit
 *   I/Q blocks from usb_rtlsdr's ring, turns them into a magnitude envelope via
 *   a precomputed look-up table, hunts for the 8 µs Mode-S sync preamble,
 *   PPM-slices the bits that follow into a 56- or 112-bit candidate frame, and
 *   ships that frame BY VALUE to modes_decode. It does NO CRC, no DF parsing,
 *   no CPR — those live one stage downstream (plan S4.3).
 *
 *   WHY A LUT + CORRELATOR. At 2.4 Msps the magnitude of every sample would be
 *   a sqrt; precomputing all 65 536 (I,Q) results into a table turns the hot
 *   loop into a single load. The preamble detector and Manchester/PPM slicer
 *   are adapted from Salvatore Sanfilippo's (antirez) dump1090, whose BSD
 *   2-clause notice is reproduced below — but generalised so they work at our
 *   2.4 Msps rate (fractional samples-per-bit) rather than dump1090's classic
 *   2.0 Msps where one bit is exactly two samples.
 *
 *   REAL-TIME CONTRACT. This task runs on Core 0 alongside the USB ingest and
 *   MUST NEVER block: it pops the ring with a bounded timeout, and when the
 *   output queue is full it DROPS the candidate frame and bumps a counter
 *   instead of waiting. Every ring item is released the instant we finish
 *   reading it; we never retain the borrowed sample pointer.
 *
 *  ─────────────────────────────────────────────────────────────────────────
 *   Portions adapted from dump1090 by Salvatore Sanfilippo (antirez):
 *
 *   Copyright (c) 2012, Salvatore Sanfilippo <antirez@gmail.com>
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
 *   COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *   EXEMPLARY, OR CONSEQUENTIAL DAMAGES HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE.
 *  ─────────────────────────────────────────────────────────────────────────
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

/* The host unit test builds this file with -DDEMOD1090_HOST_TEST=1: no RTOS, no
 * ESP heap/log — plain libc stand-ins below keep the PURE DSP core identical
 * between target and host so the test exercises the exact shipping code path
 * (LUT → preamble scan → PPM slice → emit). Same arrangement as demod978. */
#ifndef DEMOD1090_HOST_TEST
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#else
#include <stdio.h>
#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "[%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) printf("[%s] " fmt "\n", tag, ##__VA_ARGS__)
#define heap_caps_malloc(sz, caps)     malloc(sz)
#define heap_caps_realloc(p, sz, caps) realloc((p), (sz))
#define heap_caps_free(p)              free(p)
#define MALLOC_CAP_INTERNAL 0
#define MALLOC_CAP_8BIT     0
#endif

#include "demod1090.h"
#include "demod1090_internal.h"
#include "adsbin_types.h"
#ifndef DEMOD1090_HOST_TEST
#include "adsbin_err.h"
#endif

/* Logging tag for this component. */
static const char *TAG = "demod1090";

/* The one and only demodulator instance. Zero-initialised => inited == false. */
static demod1090_ctx_t s_ctx;

/* RTL2832U offset-binary mid-scale. The dongle centres an "absolute zero" RF  */
/* sample at code 127.4 on both I and Q; we subtract it before computing power. */
#define DEMOD_DC_BIAS  127.4

/* How long the task waits on the ring before looping to re-check `running`.    */
/* Bounded so stop() is responsive; long enough that we rarely spin idle.       */
#define DEMOD_RING_WAIT_MS  20

/* ── Optional hot-path cycle profiler (OFF by default) ───────────────────────
 * Define DEMOD1090_PROFILE=1 (e.g. via a build flag) to log, once per second, the
 * average CPU cycles spent in block_to_magnitude() vs process_magnitude() per IQ
 * block. This is the measurement tool to decide whether the magnitude transform
 * is worth hand-vectorizing with the P4's PIE 128-bit SIMD: if process_magnitude
 * dwarfs block_to_magnitude (expected after the coarse pre-gate), SIMD on the
 * magnitude buys little and the effort belongs elsewhere. Reading the RISC-V
 * cycle CSR is a couple of instructions, so the probe is near-free even when on;
 * compiled out entirely when off. Never enable in a shipping build. */
#ifndef DEMOD1090_PROFILE
#define DEMOD1090_PROFILE 0
#endif

#if DEMOD1090_PROFILE
#include "esp_cpu.h"   /* esp_cpu_get_cycle_count() */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Stats helpers — every counter touch goes through the mutex so get_stats and
 *  reset_stats see a coherent picture. The hot loop only ever *increments*, so
 *  contention is negligible (the readers are the slow status task).
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef DEMOD1090_HOST_TEST
/** @brief Take the stats lock (never from ISR; only task context here). */
static inline void stats_lock(void)
{
    if (s_ctx.stats_mux) {
        xSemaphoreTake(s_ctx.stats_mux, portMAX_DELAY);
    }
}

/** @brief Release the stats lock. */
static inline void stats_unlock(void)
{
    if (s_ctx.stats_mux) {
        xSemaphoreGive(s_ctx.stats_mux);
    }
}
#else
/* Host build is single-threaded: the counters need no lock. */
static inline void stats_lock(void)   {}
static inline void stats_unlock(void) {}
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Magnitude LUT.
 *
 *  Build a 256×256 table mapping the raw (I,Q) byte pair to a 16-bit magnitude.
 *  We index it as lut[(I << 8) | Q]. The DC bias is removed first; the result
 *  is scaled so the largest achievable magnitude (a corner of the I/Q square)
 *  maps to ~65535, giving the correlation sums the widest dynamic range without
 *  overflowing a uint16 per sample.
 * ═══════════════════════════════════════════════════════════════════════════ */
static esp_err_t build_mag_lut(void)
{
    /* 64 KiB — allocate from internal RAM so the Core-0 hot loop never eats a  */
    /* PSRAM cache miss on a per-sample lookup.                                 */
    s_ctx.mag_lut = (uint16_t *)heap_caps_malloc(DEMOD_MAG_LUT_SIZE * sizeof(uint16_t),
                                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_ctx.mag_lut) {
        ESP_LOGE(TAG, "mag LUT alloc failed (%u bytes)",
                 (unsigned)(DEMOD_MAG_LUT_SIZE * sizeof(uint16_t)));
        return ESP_ERR_NO_MEM;
    }

    /* The maximum centred magnitude: the farthest corner from the DC bias. We  */
    /* normalise against it so a strong burst sits near full-scale uint16.      */
    const double max_i = (255.0 - DEMOD_DC_BIAS);
    const double max_q = (255.0 - DEMOD_DC_BIAS);
    const double max_mag = sqrt(max_i * max_i + max_q * max_q);
    const double scale  = 65535.0 / max_mag;

    /* Fill every (I,Q) combination once. This is the only sqrt-per-sample we   */
    /* will ever pay — the hot loop is pure table loads after this.             */
    for (int i = 0; i < 256; ++i) {
        const double fi = (double)i - DEMOD_DC_BIAS;     /* centred I component */
        for (int q = 0; q < 256; ++q) {
            const double fq = (double)q - DEMOD_DC_BIAS; /* centred Q component */
            double m = sqrt(fi * fi + fq * fq) * scale;  /* scaled magnitude    */
            if (m > 65535.0) m = 65535.0;                /* clamp to uint16     */
            s_ctx.mag_lut[(i << 8) | q] = (uint16_t)(m + 0.5);
        }
    }
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Magnitude conversion for one IQ block.
 *
 *  Walk the interleaved I,Q,I,Q… bytes and emit one magnitude per IQ pair into
 *  the reusable scratch buffer. The scratch grows on demand and is kept across
 *  blocks so steady-state runs allocate nothing.
 *
 *  @return number of magnitude samples written, or 0 on alloc failure.
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint32_t block_to_magnitude(const uint8_t *iq, uint32_t n_bytes)
{
    /* Each IQ pair (two bytes) yields exactly one magnitude sample. Odd tail   */
    /* bytes (should never happen) are ignored.                                 */
    const uint32_t n_samples = n_bytes >> 1;
    if (n_samples == 0) {
        return 0;
    }

    /* Grow the scratch buffer if this block is larger than any we have seen.   */
    if (n_samples > s_ctx.mag_cap) {
        uint16_t *grown = (uint16_t *)heap_caps_realloc(
            s_ctx.mag, n_samples * sizeof(uint16_t),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!grown) {
            ESP_LOGE(TAG, "mag scratch realloc failed (%u samples)", (unsigned)n_samples);
            return 0;
        }
        s_ctx.mag     = grown;
        s_ctx.mag_cap = n_samples;
    }

    /* The hot transform: two byte loads + one LUT load per output sample. The  */
    /* compiler unrolls this nicely; we keep locals to avoid repeated globals.  */
    const uint16_t *lut = s_ctx.mag_lut;
    uint16_t       *out = s_ctx.mag;
    for (uint32_t s = 0; s < n_samples; ++s) {
        const uint8_t i_byte = iq[(s << 1)];        /* I sample */
        const uint8_t q_byte = iq[(s << 1) + 1];    /* Q sample */
        out[s] = lut[((uint32_t)i_byte << 8) | q_byte];
    }
    return n_samples;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  DSP geometry — computed ONCE at init.
 *
 *  Everything the scan loop needs (pulse/valley sample offsets per phase
 *  template, bit strides, frame spans) is derived here with double math and
 *  frozen into integers / 32.32 fixed-point. The P4's FPU is single-precision
 *  only, so a double op in the per-sample path is a soft-float LIBRARY CALL —
 *  the old per-sample index math cost ~20 such calls per scanned position and
 *  put the demod 3–9× over the Core-0 real-time budget (the bench showed 14 of
 *  the required 146 blocks/s being consumed). Init-time is the only place
 *  doubles are allowed in this file.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void build_geometry(void)
{
    const double sp = (double)s_ctx.sample_rate_hz / 1e6;   /* samples per µs   */

    /* Fixed-point strides for the PPM slicer. One bit = 1 µs; the half-bit is  */
    /* the PPM decision window.                                                 */
    s_ctx.fp_samples_per_us = (uint64_t)(sp * (double)DEMOD_FP_ONE);
    s_ctx.fp_bit            = s_ctx.fp_samples_per_us;      /* DEMOD_BIT_US = 1 */
    s_ctx.fp_half           = s_ctx.fp_bit >> 1;
    s_ctx.fp_data_start     = (uint64_t)(DEMOD_PREAMBLE_US * sp * (double)DEMOD_FP_ONE);

    /* Integer sample spans (ceil), with the same +4 tail margin the scan logic */
    /* has always used for the emit/skip distances.                             */
    const uint32_t rate = s_ctx.sample_rate_hz;
    s_ctx.span_preamble = (uint32_t)(((uint64_t)8u  * rate + 999999u) / 1000000u);
    s_ctx.span_short    = (uint32_t)(((uint64_t)(8u + MODES_SHORT_BITS) * rate + 999999u) / 1000000u) + 4u;
    s_ctx.span_full     = (uint32_t)(((uint64_t)(8u + MODES_LONG_BITS)  * rate + 999999u) / 1000000u) + 4u;

    /* Preamble slot centres, physical constants of the 1090 downlink: pulses   */
    /* peak a quarter-bit past their leading edges (0/1/3.5/4.5 µs); the gaps at */
    /* 2.0/3.0/5.5/6.5/7.5 µs MUST be quiet in a genuine preamble.              */
    static const double pulse_us[DEMOD_PRE_PULSES] = {
        DEMOD_PULSE0_US + 0.25, DEMOD_PULSE1_US + 0.25,
        DEMOD_PULSE2_US + 0.25, DEMOD_PULSE3_US + 0.25,
    };
    static const double valley_us[DEMOD_PRE_VALLEYS] = { 2.0, 3.0, 5.5, 6.5, 7.5 };

    /* Build one integer index template per assumed sub-sample arrival phase.   */
    /* For phase h (fraction of a sample) a slot at t µs truly sits at h + t*sp */
    /* samples past the candidate index; we snap that to the nearest integer.   */
    uint32_t max_off = 0;
    uint32_t g_lo = UINT32_MAX, g_hi = 0;
    for (int p = 0; p < DEMOD_PRE_PHASES; ++p) {
        const double h = (double)p / (double)DEMOD_PRE_PHASES;
        s_ctx.pre_phase_fp[p] =
            ((uint64_t)p * DEMOD_FP_ONE) / (uint64_t)DEMOD_PRE_PHASES;

        for (int k = 0; k < DEMOD_PRE_PULSES; ++k) {
            const uint16_t off = (uint16_t)(pulse_us[k] * sp + h + 0.5);
            s_ctx.pre_tpl[p].pulse[k] = off;
            if (off > max_off) max_off = off;
        }
        for (int k = 0; k < DEMOD_PRE_VALLEYS; ++k) {
            const uint16_t off = (uint16_t)(valley_us[k] * sp + h + 0.5);
            s_ctx.pre_tpl[p].valley[k] = off;
            if (off > max_off) max_off = off;
        }

        /* Track where pulse 0 lands across the templates: the coarse pre-gate  */
        /* probes exactly these offsets, so it can never out-reject the full    */
        /* correlator.                                                          */
        const uint32_t p0_off = s_ctx.pre_tpl[p].pulse[0];
        if (p0_off < g_lo) g_lo = p0_off;
        if (p0_off > g_hi) g_hi = p0_off;
    }
    s_ctx.pre_window  = max_off + 2u;      /* bounds slack past the last slot   */
    s_ctx.gate_idx_lo = g_lo;
    s_ctx.gate_idx_hi = g_hi;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Preamble correlation — integer-only, phase-template bank.
 *
 *  The Mode-S preamble is four 0.5 µs pulses with leading edges at 0, 1, 3.5
 *  and 4.5 µs inside an 8 µs window. We score a candidate start index by
 *  comparing the four "pulse" slots against the guaranteed-quiet "valley"
 *  slots that separate them. A genuine preamble has high pulses and deep
 *  valleys; noise and continuous carriers do not.
 *
 *  At 2.4 Msps a 0.5 µs pulse is only 1.2 samples wide, so a SINGLE index
 *  template only catches arrivals whose sub-sample phase puts its point
 *  samples on the plateaus — much real traffic would never be detectable. We
 *  therefore try DEMOD_PRE_PHASES precomputed templates (one per assumed
 *  arrival phase) and accept on the best-scoring one; the winner's phase then
 *  seeds the data slicer so its hypothesis sweep starts on target.
 *
 *  @param m          magnitude buffer
 *  @param n          number of valid magnitude samples
 *  @param start      candidate preamble start (sample index)
 *  @param out_score  0..255 correlation quality on success
 *  @param out_level  representative burst magnitude (proxy RSSI)
 *  @param out_phase  index into pre_phase_fp[] of the winning template
 *  @return true if @p start looks like a valid preamble.
 * ═══════════════════════════════════════════════════════════════════════════ */
static bool detect_preamble(const uint16_t *m, uint32_t n, uint32_t start,
                            uint8_t *out_score, uint16_t *out_level,
                            uint32_t *out_phase)
{
    /* The widest template window must fit entirely inside the buffer.          */
    if (start + s_ctx.pre_window >= n) {
        return false;
    }

    uint32_t best_score = 0;
    uint32_t best_level = 0;
    uint32_t best_phase = 0;
    bool     hit        = false;

    for (uint32_t p = 0; p < DEMOD_PRE_PHASES; ++p) {
        const demod_pre_tpl_t *t = &s_ctx.pre_tpl[p];

        /* The four pulse slots for this phase hypothesis.                      */
        const uint32_t p0 = m[start + t->pulse[0]];
        const uint32_t p1 = m[start + t->pulse[1]];
        const uint32_t p2 = m[start + t->pulse[2]];
        const uint32_t p3 = m[start + t->pulse[3]];

        /* Weakest pulse — every valley must stay strictly below it (the        */
        /* dump1090-style dominance check). Early-out on the first hot valley:  */
        /* on noise that usually kills the template within one or two loads.    */
        uint32_t min_pulse = p0;
        if (p1 < min_pulse) min_pulse = p1;
        if (p2 < min_pulse) min_pulse = p2;
        if (p3 < min_pulse) min_pulse = p3;

        uint32_t valley_sum = 0;
        bool     dominated  = true;
        for (int k = 0; k < DEMOD_PRE_VALLEYS; ++k) {
            const uint32_t v = m[start + t->valley[k]];
            if (v >= min_pulse) {
                dominated = false;
                break;
            }
            valley_sum += v;
        }
        if (!dominated) {
            continue;
        }

        /* Mean pulse vs mean valley: need genuine separation, not a flat blob. */
        const uint32_t pulse_mean  = (p0 + p1 + p2 + p3) / 4u;
        const uint32_t valley_mean = valley_sum / DEMOD_PRE_VALLEYS;
        if (pulse_mean <= valley_mean) {
            continue;
        }

        /* Score = how cleanly the pulses dominate, mapped to 0..255 via the    */
        /* ratio (pulse-valley)/pulse: a perfect preamble (silent gaps) → 255.  */
        const uint32_t margin  = pulse_mean - valley_mean;
        uint32_t score32 = (margin * 255u) / pulse_mean;
        if (score32 > 255u) score32 = 255u;

        /* Keep the best-matching phase template.                               */
        if (!hit || score32 > best_score) {
            hit        = true;
            best_score = score32;
            best_level = pulse_mean;
            best_phase = p;
        }
    }

    if (!hit) {
        return false;
    }

    *out_score = (uint8_t)best_score;
    *out_level = (uint16_t)(best_level > 65535u ? 65535u : best_level);
    *out_phase = best_phase;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PPM bit slicing at a chosen sub-sample phase (matched-filter, single phase).
 *
 *  After a valid preamble the data begins ~8 µs after `start`, but at 2.4 Msps the
 *  exact sub-sample phase of the bit grid relative to our integer sample stream is
 *  unknown — the preamble detector only locks to an integer `start`. This routine
 *  slices the data assuming a GIVEN sub-sample phase offset @p phase_fp (a
 *  fixed-point fraction of a sample, 0..DEMOD_FP_ONE) added to the nominal data
 *  start. The caller (process_magnitude) sweeps several phase hypotheses and keeps
 *  the best — that sweep is the multi-phase matched filter (see CITATIONS.md §B).
 *
 *  Each bit is a 1 µs PPM symbol: energy in the first half-bit ⇒ 1, in the second
 *  ⇒ 0. We sample each half-bit at its CENTRE (¼ and ¾ bit) and decide by which is
 *  louder. We also accumulate a CONFIDENCE score = Σ|m_first − m_second| over all
 *  bits: at the correct phase the two half-bit samples sit cleanly on a pulse vs a
 *  gap so the per-bit margin is large; at a wrong phase both samples straddle the
 *  symbol edge and the margins collapse toward zero. Maximising this confidence is
 *  exactly choosing the matched-filter-optimal sampling phase, with no CRC needed
 *  (so demod1090 stays decoupled from modes_decode).
 *
 *  @param phase_off  SIGNED sub-sample offset (fixed-point) added to the data
 *                    start; the caller centres its sweep on the phase the
 *                    preamble detector locked, so this can be negative.
 *  @param out_conf   if non-NULL, receives the summed PPM confidence for this phase.
 *  @return number of bits actually sliced (≤ want_bits).
 * ═══════════════════════════════════════════════════════════════════════════ */
static int slice_bits(const uint16_t *m, uint32_t n, uint32_t start,
                      int want_bits, uint8_t *out_data,
                      int64_t phase_off, uint64_t *out_conf)
{
    /* All strides precomputed at init — no floating point on this path.        */
    const uint64_t bit_fp  = s_ctx.fp_bit;                    /* 1 µs PPM bit   */
    const uint64_t half_fp = s_ctx.fp_half;                   /* ½-bit window   */

    /* Data starts 8 µs after the preamble start, shifted by the trial sub-sample
     * phase. Carry it as a fixed-point absolute sample position. The signed
     * offset is at most ±⅓ sample against a ≥19-sample base, so the unsigned
     * result cannot underflow. */
    uint64_t pos = (uint64_t)((int64_t)(((uint64_t)start << DEMOD_FP_SHIFT) +
                                        s_ctx.fp_data_start) + phase_off);

    /* Clear the destination so unused long-frame bytes are deterministic when  */
    /* the caller only keeps the short-frame prefix.                            */
    memset(out_data, 0, MODES_LONG_BYTES);

    uint64_t conf = 0;          /* Σ|first−second| confidence for this phase.    */
    int bits_done = 0;
    for (int b = 0; b < want_bits; ++b) {

        /* Sample each half-bit at its centre (¼ bit and ¾ bit in). Round to    */
        /* nearest by adding ½ a sample before the truncating shift.            */
        const uint64_t first_fp  = pos + (half_fp >> 1) + (DEMOD_FP_ONE >> 1);  /* ¼ bit */
        const uint64_t second_fp = pos + half_fp + (half_fp >> 1) + (DEMOD_FP_ONE >> 1); /* ¾ bit */
        const uint32_t i_first   = (uint32_t)(first_fp  >> DEMOD_FP_SHIFT);
        const uint32_t i_second  = (uint32_t)(second_fp >> DEMOD_FP_SHIFT);

        /* Bounds check — if the long frame runs off the end of this block we   */
        /* stop here and let the caller treat what we have as a short frame.    */
        if (i_second >= n) {
            break;
        }

        const uint16_t a = m[i_first];
        const uint16_t c = m[i_second];

        /* PPM decision: first-half-louder ⇒ 1. Ties resolve to 0.             */
        if (a > c) {
            out_data[b >> 3] |= (uint8_t)(1u << (7 - (b & 7)));
        }

        /* Accumulate |first − second| as this bit's decision confidence.       */
        conf += (a > c) ? (uint64_t)(a - c) : (uint64_t)(c - a);

        /* Advance one whole bit period and count it.                          */
        pos += bit_fp;
        ++bits_done;
    }

    if (out_conf) {
        *out_conf = conf;
    }
    return bits_done;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Emit a candidate frame to the output queue (non-blocking, drop-on-full).
 * ═══════════════════════════════════════════════════════════════════════════ */
static void emit_frame(const uint8_t *data, int len_bytes, uint8_t score,
                       uint16_t level, int64_t rx_time_us)
{
    /* Assemble the POD frame on the stack; it is copied by value into the queue.*/
    modes_frame_t f;
    memset(&f, 0, sizeof(f));
    memcpy(f.data, data, (size_t)len_bytes);
    f.len_bytes      = (uint8_t)len_bytes;
    f.df             = (uint8_t)(data[0] >> 3);   /* precomputed downlink format */
    f.preamble_score = score;
    f.signal_level   = level;
    f.rx_time_us     = rx_time_us;

#ifndef DEMOD1090_HOST_TEST
    /* Non-blocking send: on Core 0 we MUST NOT wait. A full queue means
       modes_decode is behind, so we drop and count rather than stall ingest.   */
    if (xQueueSend(s_ctx.out_queue, &f, 0) == pdTRUE) {
        stats_lock();
        s_ctx.stats.frames_emitted++;
        if (len_bytes == MODES_SHORT_BYTES) s_ctx.stats.frames_56bit++;
        else                                s_ctx.stats.frames_112bit++;
        s_ctx.stats.last_signal_level = level;
        stats_unlock();
    } else {
        stats_lock();
        s_ctx.stats.queue_overflows++;
        stats_unlock();
    }
#else
    /* Host build: hand the candidate straight to the test's capture hook.      */
    s_ctx.stats.frames_emitted++;
    if (len_bytes == MODES_SHORT_BYTES) s_ctx.stats.frames_56bit++;
    else                                s_ctx.stats.frames_112bit++;
    s_ctx.stats.last_signal_level = level;
    demod1090_host_capture(&f);
#endif
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Process one magnitude buffer end to end: scan for preambles, slice, emit.
 *
 *  @param m            magnitude samples for the block
 *  @param n            number of magnitude samples
 *  @param block_t_us   capture time of the block's first sample
 *  @param rate_hz      sample rate (to map a sample offset back to a timestamp)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void process_magnitude(const uint16_t *m, uint32_t n,
                              int64_t block_t_us, uint32_t rate_hz)
{
    /* Magnitude samples spanned by a full preamble+long frame, by just the     */
    /* preamble, and by a short (56-bit) frame — precomputed at init (integer). */
    const uint32_t full_span     = s_ctx.span_full;
    const uint32_t preamble_span = s_ctx.span_preamble;
    const uint32_t short_span    = s_ctx.span_short;

    /* We only need room for the SHORTEST emittable frame to begin scanning at a */
    /* position. A long candidate that runs off the block tail is handled        */
    /* gracefully: slice_bits() bounds-checks and returns fewer bits, which we   */
    /* then classify as short or skip. Limiting the window to the long span      */
    /* instead would silently drop every short frame in the last ~110 µs of      */
    /* every block — a real recall loss on continuous air.                       */
    if (n <= short_span) {
        /* Block too small to contain even one short frame; nothing to do.      */
        return;
    }

    /* Local counters batched so we touch the mutex once per block, not per hit.*/
    uint64_t local_preambles = 0;

    /* Scan window: leave room for at least a short frame after every candidate. */
    /* (Longer candidates that overrun the tail are truncated by slice_bits.)    */
    const uint32_t scan_end = n - short_span;

    /* ── COARSE PRE-GATE (performance) ───────────────────────────────────────
     * detect_preamble() is integer-only now, but it still costs ~10–30 loads and
     * compares per position; a one-load gate in front of it decides whether the
     * position is even worth that.
     *
     * The gate probes the PULSE-0 sample offsets used by the phase templates
     * (gate_idx_lo..gate_idx_hi — a 1-sample window at 2.4 Msps, since pulse 0
     * rounds to the same offset for every phase) and demands the strongest of
     * them exceed 1.5× the block's mean magnitude. Justification: on Rayleigh
     * noise (real antenna air) P(sample > 1.5×mean) ≈ 17%, so ~5/6 of positions
     * skip the correlator; while any preamble the correlator itself would pass
     * needs each pulse to top the MAX of five noise valleys — whose median on
     * noise is ≈1.5×mean — so a burst gated out here had essentially no chance
     * of surviving the full dominance test anyway. (The old mean/4 "provably
     * permissive" floor passed >99.9% of noise positions and was the single
     * biggest contributor to the demod running 3–9× over real time.)
     */
    uint64_t mag_sum = 0;
    for (uint32_t k = 0; k < n; ++k) {
        mag_sum += m[k];
    }
    const uint32_t mag_mean = (uint32_t)(mag_sum / (n ? n : 1u));
    uint32_t gate_hi        = mag_mean + (mag_mean >> 1);     /* 1.5 × mean     */
    /* Clamp below the uint16 magnitude ceiling: past a mean of ~43690 the raw
     * 1.5× threshold exceeds 65535 and NOTHING could pass the strict '>' — a
     * rail-amplitude interferer would blind the demod completely instead of
     * just degrading it. Clamped, a saturated pulse (65535) always gates in.  */
    if (gate_hi > 65534u) {
        gate_hi = 65534u;
    }
    const uint32_t g_lo     = s_ctx.gate_idx_lo;
    const uint32_t g_hi     = s_ctx.gate_idx_hi;

    uint32_t j = 0;
    while (j < scan_end) {

        uint8_t  score = 0;
        uint16_t level = 0;
        uint32_t phase = 0;

        /* Coarse pre-gate: pulse 0 must already stand clear of the noise floor
         * at (at least) one of the template offsets, or no template can accept
         * this position. One load + one compare in the common case.            */
        uint32_t gate_peak = 0;
        for (uint32_t g = g_lo; g <= g_hi; ++g) {
            if (m[j + g] > gate_peak) gate_peak = m[j + g];
        }
        if (gate_peak <= gate_hi) {
            ++j;
            continue;
        }

        /* Full preamble correlation. On a miss, step one sample and keep hunting.*/
        if (!detect_preamble(m, n, j, &score, &level, &phase)) {
            ++j;
            continue;
        }

        /* Threshold the correlation quality against the configured minimum.    */
        if (score < s_ctx.preamble_threshold) {
            ++j;
            continue;
        }

        ++local_preambles;

        /* Timestamp this frame: block start plus the preamble's sample offset  */
        /* (integer µs; one 64-bit divide per ACCEPTED candidate, not per sample).*/
        const int64_t rx_us =
            block_t_us + (int64_t)(((uint64_t)j * 1000000ull) / rate_hz);

        /* ── MULTI-PHASE SLICE ───────────────────────────────────────────────
         * The winning preamble template pins the burst's sub-sample phase to
         * ±1/6 sample (±1/2 if the scorer picked a neighbouring class on a noisy
         * burst). Slice the data at DEMOD_PHASE_STEPS hypotheses spaced 1/6 of a
         * sample apart and CENTRED on that detected phase, keeping the one with
         * the highest summed PPM confidence (Σ|first−second|). The confidence
         * peaks at the matched-filter-optimal phase where each half-bit sample
         * sits cleanly on a pulse vs a gap; wrong phases straddle symbol edges
         * and score low. See CITATIONS.md §B for the derivation. */
        const int64_t phase_centre = (int64_t)s_ctx.pre_phase_fp[phase];

        uint8_t data[MODES_LONG_BYTES];
        uint8_t best_data[MODES_LONG_BYTES];
        uint64_t best_conf = 0;
        int got = 0;
        for (int p = 0; p < DEMOD_PHASE_STEPS; ++p) {
            /* Hypotheses at centre + {-2,-1,0,+1,+2} × (1/6 sample).            */
            const int64_t phase_off = phase_centre +
                ((int64_t)p - DEMOD_PHASE_STEPS / 2) * (int64_t)DEMOD_PHASE_STEP_FP;

            uint64_t conf = 0;
            int got_p = slice_bits(m, n, j, MODES_LONG_BITS, data, phase_off, &conf);

            /* Keep the highest-confidence phase. The first phase always seeds    */
            /* best_* so we never emit an uninitialised buffer.                   */
            if (p == 0 || conf > best_conf) {
                best_conf = conf;
                got       = got_p;
                memcpy(best_data, data, sizeof(best_data));
            }
        }
        /* Decode/emit from the winning phase's bits. */
        memcpy(data, best_data, sizeof(data));

        /* Default skip: at minimum step past the preamble so we never re-detect */
        /* the same sync burst, even if this candidate slices to nothing usable. */
        uint32_t advance = preamble_span;

        /* Decide short vs long from the downlink format (data[0] >> 3). DF 0,  */
        /* 4, 5, 11 are 56-bit; everything else (16,17,18,19,20,21,24) is 112.  */
        /* This mirrors the Mode-S spec and lets the decoder skip re-checking.  */
        if (got >= MODES_SHORT_BITS) {
            const uint8_t df = (uint8_t)(data[0] >> 3);

            /* Is this a 56-bit downlink format?                                */
            bool is_short_df = (df == 0 || df == 4 || df == 5 || df == 11);

            if (is_short_df) {
                /* A genuine short frame: bytes [0..6] are the whole thing.     */
                emit_frame(data, MODES_SHORT_BYTES, score, level, rx_us);
                advance = short_span;
            } else if (got >= MODES_LONG_BITS) {
                /* A long-format frame and we sliced all 112 bits — emit it.    */
                emit_frame(data, MODES_LONG_BYTES, score, level, rx_us);
                advance = full_span;
            } else {
                /* A long-format candidate truncated by the block tail. Its     */
                /* first 7 bytes are NOT a valid short frame, so we DROP it      */
                /* rather than emit garbage; it will not be re-acquired from this*/
                /* block. We still step past the preamble we consumed.          */
                advance = short_span;
            }
        }

        /* Jump past this frame's own samples so its PPM data is never mistaken  */
        /* for the next preamble. advance is always >= one preamble.             */
        j += advance;
    }

    /* Fold the per-block preamble tally into the shared stats once.            */
    if (local_preambles) {
        stats_lock();
        s_ctx.stats.preambles_detected += local_preambles;
        stats_unlock();
    }
}

#ifndef DEMOD1090_HOST_TEST
/* ═══════════════════════════════════════════════════════════════════════════
 *  The Core-0 demod task.
 *
 *  Loops: pop an IQ block (bounded wait), magnitude-convert it, scan it, then
 *  IMMEDIATELY return the ring item. Never blocks on output. Exits cleanly when
 *  demod1090_stop() clears `running`.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void demod_task(void *arg)
{
    (void)arg;
    s_ctx.task_alive = true;
    ESP_LOGI(TAG, "demod task up on core %d", (int)xPortGetCoreID());

    while (s_ctx.running) {

        /* Pop the next ring item. RingbufferReturnItem must follow on success.*/
        size_t item_size = 0;
        void  *item = xRingbufferReceive(s_ctx.iq_ring, &item_size,
                                         pdMS_TO_TICKS(DEMOD_RING_WAIT_MS));
        if (!item) {
            /* Timed out with no data — loop to re-check `running`. Not an error.*/
            continue;
        }

        /* The ring item IS an iq_block_t whose .samples points just past it in */
        /* the same allocation (per the usb_rtlsdr contract).                   */
        const iq_block_t *blk = (const iq_block_t *)item;
        const uint8_t *samples = blk->samples;
        const uint32_t n_bytes = blk->n_bytes;
        const int64_t  t_cap   = blk->t_capture_us;
        const uint32_t blk_seq = blk->seq;

        /* Detect a sequence gap: usb_rtlsdr bumps seq monotonically, so a jump */
        /* greater than 1 means it dropped block(s) into the ring overflow.     */
        /* Track that as iq_blocks_dropped on our side too. State lives in the  */
        /* context (not function statics) so a stop()/start() cycle starts fresh*/
        /* and never reports a phantom gap across the restart boundary.         */
        uint64_t dropped = 0;
        if (s_ctx.have_seq && blk_seq > s_ctx.last_seq + 1) {
            dropped = (uint64_t)(blk_seq - s_ctx.last_seq - 1);
        }
        s_ctx.last_seq = blk_seq;
        s_ctx.have_seq = true;

        /* Convert to magnitude. On alloc failure we still must return the item.*/
#if DEMOD1090_PROFILE
        uint32_t prof_c0 = esp_cpu_get_cycle_count();
#endif
        uint32_t n_samples = 0;
        if (samples && n_bytes >= 2) {
            n_samples = block_to_magnitude(samples, n_bytes);
        }
#if DEMOD1090_PROFILE
        uint32_t prof_c1 = esp_cpu_get_cycle_count();
#endif

        /* Return the ring item NOW — we have copied everything we need into the*/
        /* scratch magnitude buffer and never retain the borrowed samples.      */
        vRingbufferReturnItem(s_ctx.iq_ring, item);

        /* Roll the consumed/dropped/sample counters in one locked section.     */
        stats_lock();
        s_ctx.stats.iq_blocks_consumed++;
        s_ctx.stats.iq_blocks_dropped += dropped;
        s_ctx.stats.samples_processed += n_samples;
        stats_unlock();

        /* Run the detector over the magnitude envelope (lock-free hot path).   */
#if DEMOD1090_PROFILE
        uint32_t prof_c2 = esp_cpu_get_cycle_count();
#endif
        if (n_samples) {
            process_magnitude(s_ctx.mag, n_samples, t_cap, s_ctx.sample_rate_hz);
        }
#if DEMOD1090_PROFILE
        uint32_t prof_c3 = esp_cpu_get_cycle_count();
        /* Accumulate and report once per second so the log is readable. The
         * cycle CSR is per-core and monotonic; unsigned subtraction handles wrap. */
        static uint64_t s_mag_cyc = 0, s_scan_cyc = 0, s_blocks = 0;
        s_mag_cyc  += (uint32_t)(prof_c1 - prof_c0);
        s_scan_cyc += (uint32_t)(prof_c3 - prof_c2);
        if (++s_blocks >= 73) {   /* ~1 s of blocks at 2.4 Msps / 32 KiB URBs */
            ESP_LOGI(TAG, "PROFILE/blk: magnitude=%llu cyc  preamble-scan=%llu cyc  (n=%llu)",
                     (unsigned long long)(s_mag_cyc / s_blocks),
                     (unsigned long long)(s_scan_cyc / s_blocks),
                     (unsigned long long)s_blocks);
            s_mag_cyc = s_scan_cyc = s_blocks = 0;
        }
#endif
    }

    /* Clean exit: signal the joiner and self-delete.                          */
    ESP_LOGI(TAG, "demod task exiting");
    s_ctx.task_alive = false;
    s_ctx.task = NULL;
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t demod1090_init(const demod1090_config_t *cfg)
{
    /* Reject double-init; deinit first if you want fresh config.              */
    if (s_ctx.inited) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Start from a clean slate so a previous failed init leaves no debris.    */
    memset(&s_ctx, 0, sizeof(s_ctx));

    /* Resolve configuration, applying documented defaults for zeroed fields.  */
    s_ctx.sample_rate_hz     = (cfg && cfg->sample_rate_hz)   ? cfg->sample_rate_hz
                                                              : ADSB_SAMPLE_RATE_HZ;
    s_ctx.task_core_id       = (cfg) ? cfg->task_core_id      : ADSBIN_CORE_DSP;
    s_ctx.task_priority      = (cfg && cfg->task_priority)    ? cfg->task_priority
                                                              : DEMOD_DEFAULT_PRIORITY;
    s_ctx.task_stack_size    = (cfg && cfg->task_stack_size)  ? cfg->task_stack_size
                                                              : DEMOD_DEFAULT_STACK;
    s_ctx.preamble_threshold = (cfg && cfg->preamble_threshold) ? cfg->preamble_threshold
                                                                : DEMOD_DEFAULT_PREAMBLE;

    /* Validate the sample rate: we need at least ~2 samples per µs to recover  */
    /* the 0.5 µs half-bit PPM symbols at all.                                  */
    if (s_ctx.sample_rate_hz < 2000000u) {
        ESP_LOGE(TAG, "sample rate %u too low for PPM (need >= 2 Msps)",
                 (unsigned)s_ctx.sample_rate_hz);
        return ESP_ERR_INVALID_ARG;
    }

    /* Precompute the DSP geometry (fixed-point strides, integer spans, and the */
    /* per-phase preamble index templates). The only double math this component */
    /* ever runs happens inside this one init-time call.                        */
    build_geometry();

    /* Stats mutex first so any later failure path can still be torn down.      */
    s_ctx.stats_mux = xSemaphoreCreateMutex();
    if (!s_ctx.stats_mux) {
        ESP_LOGE(TAG, "stats mutex alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* Build the 64 KiB magnitude LUT.                                          */
    esp_err_t err = build_mag_lut();
    if (err != ESP_OK) {
        vSemaphoreDelete(s_ctx.stats_mux);
        s_ctx.stats_mux = NULL;
        return err;
    }

    /* Counters start clean.                                                    */
    memset(&s_ctx.stats, 0, sizeof(s_ctx.stats));

    s_ctx.inited = true;
    ESP_LOGI(TAG, "init ok: %u sps, %u phase templates, threshold %u",
             (unsigned)s_ctx.sample_rate_hz, (unsigned)DEMOD_PRE_PHASES,
             (unsigned)s_ctx.preamble_threshold);
    return ESP_OK;
}

esp_err_t demod1090_start(RingbufHandle_t iq_ring, QueueHandle_t out_frame_queue)
{
    /* Must be initialised, have valid handles, and not already running.        */
    if (!s_ctx.inited || !iq_ring || !out_frame_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx.running || s_ctx.task) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Stash the borrowed IPC handles (owned by usb_rtlsdr / main respectively).*/
    s_ctx.iq_ring   = iq_ring;
    s_ctx.out_queue = out_frame_queue;
    s_ctx.running   = true;

    /* Reset ring-sequence tracking so the first block of this run never looks  */
    /* like it followed a gap.                                                  */
    s_ctx.last_seq  = 0;
    s_ctx.have_seq  = false;

    /* Pin the task to the configured core. We are the producer of frames and a */
    /* hard-real-time consumer of IQ, so we live on ADSBIN_CORE_DSP in prod.    */
    BaseType_t ok = xTaskCreatePinnedToCore(
        demod_task, "demod1090",
        s_ctx.task_stack_size / sizeof(StackType_t),
        NULL, s_ctx.task_priority, &s_ctx.task,
        s_ctx.task_core_id);

    if (ok != pdPASS) {
        /* Roll back so a retry is clean.                                       */
        s_ctx.running   = false;
        s_ctx.iq_ring   = NULL;
        s_ctx.out_queue = NULL;
        s_ctx.task      = NULL;
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "started (core %u, prio %u, stack %u)",
             (unsigned)s_ctx.task_core_id, (unsigned)s_ctx.task_priority,
             (unsigned)s_ctx.task_stack_size);
    return ESP_OK;
}

esp_err_t demod1090_stop(void)
{
    /* Nothing running ⇒ trivially stopped.                                     */
    if (!s_ctx.running && !s_ctx.task) {
        return ESP_OK;
    }

    /* Ask the task to drain and exit, then wait for it to actually leave its   */
    /* loop. The task self-deletes and clears task_alive on the way out.        */
    s_ctx.running = false;

    /* Bounded join: poll task_alive. The task wakes at least every            */
    /* DEMOD_RING_WAIT_MS, so this resolves quickly without a join primitive.   */
    const TickType_t poll = pdMS_TO_TICKS(2);
    int guard = 0;
    while (s_ctx.task_alive && guard++ < 2000 /* ~4 s ceiling */) {
        vTaskDelay(poll);
    }

    s_ctx.iq_ring   = NULL;
    s_ctx.out_queue = NULL;
    s_ctx.task      = NULL;

    ESP_LOGI(TAG, "stopped");
    return ESP_OK;
}

void demod1090_deinit(void)
{
    /* Always stop the task first so nothing touches the buffers we free.       */
    if (s_ctx.running || s_ctx.task) {
        (void)demod1090_stop();
    }

    /* Release the magnitude scratch and LUT.                                   */
    if (s_ctx.mag) {
        heap_caps_free(s_ctx.mag);
        s_ctx.mag     = NULL;
        s_ctx.mag_cap = 0;
    }
    if (s_ctx.mag_lut) {
        heap_caps_free(s_ctx.mag_lut);
        s_ctx.mag_lut = NULL;
    }

    /* Tear down the stats mutex last.                                          */
    if (s_ctx.stats_mux) {
        vSemaphoreDelete(s_ctx.stats_mux);
        s_ctx.stats_mux = NULL;
    }

    s_ctx.inited = false;
    ESP_LOGI(TAG, "deinit complete");
}

#else /* DEMOD1090_HOST_TEST */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Host-test shims — no RTOS, no ring, no task. These drive the exact same
 *  pure pipeline the firmware task runs: build_geometry + LUT, then
 *  block_to_magnitude → process_magnitude → emit (captured by the test's
 *  demod1090_host_capture hook). See test_host/.
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t demod1090_host_setup(uint32_t sample_rate_hz, uint8_t preamble_threshold)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.sample_rate_hz     = sample_rate_hz ? sample_rate_hz : ADSB_SAMPLE_RATE_HZ;
    s_ctx.preamble_threshold = preamble_threshold ? preamble_threshold
                                                  : DEMOD_DEFAULT_PREAMBLE;
    build_geometry();

    esp_err_t err = build_mag_lut();
    if (err != ESP_OK) {
        return err;
    }
    s_ctx.inited = true;
    return ESP_OK;
}

uint32_t demod1090_host_process_iq(const uint8_t *iq, uint32_t n_bytes, int64_t t_us)
{
    if (!s_ctx.inited || !iq) {
        return 0;
    }
    /* Same two hot-path stages the firmware task runs per ring block.          */
    const uint32_t n_samples = block_to_magnitude(iq, n_bytes);
    if (n_samples) {
        process_magnitude(s_ctx.mag, n_samples, t_us, s_ctx.sample_rate_hz);
    }
    return n_samples;
}

void demod1090_host_teardown(void)
{
    free(s_ctx.mag);
    free(s_ctx.mag_lut);
    memset(&s_ctx, 0, sizeof(s_ctx));
}

#endif /* !DEMOD1090_HOST_TEST */

void demod1090_get_stats(demod1090_stats_t *out)
{
    if (!out) {
        return;
    }
    /* Coherent snapshot under the same lock the hot loop uses to mutate.       */
    stats_lock();
    *out = s_ctx.stats;
    stats_unlock();
}

void demod1090_reset_stats(void)
{
    stats_lock();
    memset(&s_ctx.stats, 0, sizeof(s_ctx.stats));
    stats_unlock();
}
