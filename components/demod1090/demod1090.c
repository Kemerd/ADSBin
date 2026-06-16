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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "demod1090.h"
#include "demod1090_internal.h"
#include "adsbin_types.h"
#include "adsbin_err.h"

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
 *  Preamble correlation.
 *
 *  The Mode-S preamble is four 0.5 µs pulses with leading edges at 0, 1, 3.5
 *  and 4.5 µs inside an 8 µs window. We score a candidate start index by
 *  comparing the energy in the four "pulse" slots against the energy in the
 *  guaranteed-quiet "valley" slots that separate them. A genuine preamble has
 *  high pulses and deep valleys; noise and continuous signals do not.
 *
 *  This is the dump1090 idea, but the slot centres are computed from the
 *  fractional samples-per-µs so it is correct at 2.4 Msps.
 *
 *  @param m      magnitude buffer
 *  @param n      number of valid magnitude samples
 *  @param start  candidate preamble start (sample index)
 *  @param out_score  0..255 correlation quality on success
 *  @param out_level  representative burst magnitude (proxy RSSI)
 *  @return true if @p start looks like a valid preamble.
 * ═══════════════════════════════════════════════════════════════════════════ */
static bool detect_preamble(const uint16_t *m, uint32_t n, uint32_t start,
                            uint8_t *out_score, uint16_t *out_level)
{
    const uint64_t sp_us = s_ctx.fp_samples_per_us;   /* samples/µs, fixed pt   */

    /* The preamble window must fit entirely inside the buffer.                 */
    const uint32_t window = (uint32_t)((DEMOD_PREAMBLE_US * (double)sp_us) /
                                       (double)DEMOD_FP_ONE) + 2;
    if (start + window >= n) {
        return false;
    }

    /* Helper: magnitude at a µs offset from `start`, rounded to nearest sample.*/
    /* We inline the fixed-point conversion to dodge any float in the loop body */
    /* after this point.                                                        */
    #define MAG_AT_US(us) \
        (m[start + (uint32_t)(((uint64_t)((us) * (double)sp_us) + (DEMOD_FP_ONE/2)) >> DEMOD_FP_SHIFT)])

    /* The four pulse peaks: sample each pulse near its centre (leading edge +  */
    /* a quarter-bit) so we sit on the plateau, not the rising edge.           */
    const uint32_t p0 = MAG_AT_US(DEMOD_PULSE0_US + 0.25);
    const uint32_t p1 = MAG_AT_US(DEMOD_PULSE1_US + 0.25);
    const uint32_t p2 = MAG_AT_US(DEMOD_PULSE2_US + 0.25);
    const uint32_t p3 = MAG_AT_US(DEMOD_PULSE3_US + 0.25);

    /* The valleys: points that MUST be quiet in a real preamble. These sit in  */
    /* the gaps at 2.0, 3.0, 5.5, 6.5 and 7.5 µs.                               */
    const uint32_t v0 = MAG_AT_US(2.0);
    const uint32_t v1 = MAG_AT_US(3.0);
    const uint32_t v2 = MAG_AT_US(5.5);
    const uint32_t v3 = MAG_AT_US(6.5);
    const uint32_t v4 = MAG_AT_US(7.5);

    /* Sum of the four pulse magnitudes and the five valley magnitudes.         */
    const uint32_t pulse_sum  = p0 + p1 + p2 + p3;
    const uint32_t valley_sum = v0 + v1 + v2 + v3 + v4;

    /* Mean pulse vs mean valley. A real preamble's pulses tower over the gaps. */
    const uint32_t pulse_mean  = pulse_sum / 4u;
    const uint32_t valley_mean = valley_sum / 5u;

    /* Reject if any single pulse is weaker than the strongest valley — that is */
    /* the cheap dump1090-style edge check that kills most false starts.        */
    const uint32_t max_valley = (v0 > v1 ? v0 : v1);
    const uint32_t mv2 = (v2 > v3 ? v2 : v3);
    const uint32_t max_v = (max_valley > mv2 ? max_valley : mv2);
    const uint32_t max_v2 = (max_v > v4 ? max_v : v4);

    if (p0 <= max_v2 || p1 <= max_v2 || p2 <= max_v2 || p3 <= max_v2) {
        return false;
    }

    /* Need real separation between pulse and valley energy. If the valleys are */
    /* nearly as hot as the pulses this is not a clean PPM preamble.            */
    if (pulse_mean <= valley_mean) {
        return false;
    }

    /* Score = how cleanly the pulses dominate, mapped to 0..255. We use the    */
    /* ratio (pulse-valley)/pulse so a perfect preamble (zero valleys) → 255.   */
    uint32_t margin = pulse_mean - valley_mean;
    uint32_t score32 = (margin * 255u) / (pulse_mean ? pulse_mean : 1u);
    if (score32 > 255u) score32 = 255u;

    *out_score = (uint8_t)score32;

    /* Proxy RSSI: the mean pulse height, capped to 16 bits.                    */
    *out_level = (uint16_t)(pulse_mean > 65535u ? 65535u : pulse_mean);

    #undef MAG_AT_US
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PPM bit slicing.
 *
 *  After a valid preamble the data begins exactly 8 µs after `start`. Each bit
 *  is a 1 µs PPM symbol: more energy in the first half-bit than the second ⇒ 1,
 *  otherwise ⇒ 0. We advance the read position with a fixed-point phase
 *  accumulator so the half-bit sample points stay aligned across all 112 bits
 *  even though one bit is a non-integer 2.4 samples wide.
 *
 *  We always slice the full 112 bits if the buffer allows; the caller decides
 *  short vs long. Returns the number of bits actually sliced (≤ want_bits).
 * ═══════════════════════════════════════════════════════════════════════════ */
static int slice_bits(const uint16_t *m, uint32_t n, uint32_t start,
                      int want_bits, uint8_t *out_data)
{
    const uint64_t sp_us   = s_ctx.fp_samples_per_us;          /* samples/µs fp */
    const uint64_t bit_fp  = (uint64_t)(DEMOD_BIT_US     * (double)sp_us); /* 1 bit  */
    const uint64_t half_fp = (uint64_t)(DEMOD_HALFBIT_US * (double)sp_us); /* ½ bit  */

    /* Data starts 8 µs after the preamble start. Carry it as a fixed-point     */
    /* absolute sample position relative to the buffer origin.                  */
    uint64_t pos = ((uint64_t)start << DEMOD_FP_SHIFT) +
                   (uint64_t)(DEMOD_PREAMBLE_US * (double)sp_us);

    /* Clear the destination so unused long-frame bytes are deterministic when  */
    /* the caller only keeps the short-frame prefix.                            */
    memset(out_data, 0, MODES_LONG_BYTES);

    int bits_done = 0;
    for (int b = 0; b < want_bits; ++b) {

        /* Integer sample index of this bit's first and second half centres.    */
        /* Sampling a quarter-bit into each half lands squarely on the plateau.  */
        /* Round-to-nearest (add ½ a sample before the shift) is essential here: */
        /* at 2.4 samples/bit the centres land on fractional sample positions,   */
        /* and a plain truncation would bias us a full sample earlier — onto the */
        /* previous symbol's tail — and corrupt every bit.                       */
        const uint64_t first_fp  = pos + (half_fp >> 1) + (DEMOD_FP_ONE >> 1);  /* ¼ bit */
        const uint64_t second_fp = pos + half_fp + (half_fp >> 1) + (DEMOD_FP_ONE >> 1); /* ¾ bit */
        const uint32_t i_first   = (uint32_t)(first_fp  >> DEMOD_FP_SHIFT);
        const uint32_t i_second  = (uint32_t)(second_fp >> DEMOD_FP_SHIFT);

        /* Bounds check — if the long frame runs off the end of this block we   */
        /* stop here and let the caller treat what we have as a short frame.    */
        if (i_second >= n) {
            break;
        }

        /* PPM decision: first-half-louder ⇒ 1. Ties resolve to 0, matching the */
        /* dump1090 convention (rare, and a tie is almost certainly noise).     */
        if (m[i_first] > m[i_second]) {
            out_data[b >> 3] |= (uint8_t)(1u << (7 - (b & 7)));
        }

        /* Advance one whole bit period and count it.                          */
        pos += bit_fp;
        ++bits_done;
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
    /* preamble, and by a short (56-bit) frame. Precomputed once and reused for  */
    /* both the per-hit skip distance and the scan-window limit below.           */
    const uint32_t full_span =
        (uint32_t)(DEMOD_FULL_FRAME_US * (double)rate_hz / 1e6) + 4;
    const uint32_t preamble_span =
        (uint32_t)(DEMOD_PREAMBLE_US * (double)rate_hz / 1e6);
    const uint32_t short_span =
        (uint32_t)((DEMOD_PREAMBLE_US + (double)MODES_SHORT_BITS * DEMOD_BIT_US)
                   * (double)rate_hz / 1e6) + 4;

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

    /* Per-bit sample stride in microseconds, used to timestamp each frame at   */
    /* its true preamble position rather than the block start.                  */
    const double us_per_sample = 1e6 / (double)rate_hz;

    /* Scan window: leave room for at least a short frame after every candidate. */
    /* (Longer candidates that overrun the tail are truncated by slice_bits.)    */
    const uint32_t scan_end = n - short_span;

    /* ── COARSE PRE-GATE (performance) ───────────────────────────────────────
     * detect_preamble() is invoked at EVERY sample index, and each call does ~9
     * fixed-point index computes + 9 magnitude loads + several compares. On quiet
     * air the overwhelming majority of positions are noise and can be rejected by
     * a SINGLE cheap comparison before paying for the full correlation.
     *
     * The gate must be STRICTLY WEAKER than detect_preamble's own checks so it can
     * never reject a position the real detector would accept (zero recall loss).
     *
     * detect_preamble requires every pulse (including the first, p0) to exceed the
     * strongest valley, and the valleys are >= 0 — so a real preamble's first
     * pulse magnitude is well above the block's noise floor. p0 is sampled a
     * quarter-bit into PULSE0 (t=0), which at our ~2.4 samples/µs lands within one
     * sample of `start`. To stay provably permissive regardless of the exact
     * rounding, the gate inspects the SMALL NEIGHBOURHOOD around the start sample
     * (start..start+2) and keeps the position if ANY of those equals/exceeds the
     * floor. That window strictly contains wherever p0 actually samples, so the
     * gate can never discard a position detect_preamble would have accepted.
     *
     * The floor is a low fraction of the block's mean magnitude (mean/4): a genuine
     * pulse towers over the mean, so the gate only ever drops obviously-quiet air.
     */
    uint64_t mag_sum = 0;
    for (uint32_t k = 0; k < n; ++k) {
        mag_sum += m[k];
    }
    const uint16_t mag_mean  = (uint16_t)(mag_sum / (n ? n : 1u));
    const uint16_t gate_floor = (uint16_t)(mag_mean >> 2);   /* mean/4 */

    uint32_t j = 0;
    while (j < scan_end) {

        uint8_t  score = 0;
        uint16_t level = 0;

        /* Coarse pre-gate: a real preamble's first pulse sits within start..start+2.
         * If that whole neighbourhood is down in the noise, no preamble can begin
         * here — skip the expensive correlation. This single (≤3-load) compare
         * eliminates the vast majority of full detect_preamble() calls on quiet air.
         * The window strictly covers p0's sample point, so it is provably permissive.
         */
        if (m[j] <= gate_floor && m[j + 1] <= gate_floor && m[j + 2] <= gate_floor) {
            ++j;
            continue;
        }

        /* Full preamble correlation. On a miss, step one sample and keep hunting.*/
        if (!detect_preamble(m, n, j, &score, &level)) {
            ++j;
            continue;
        }

        /* Threshold the correlation quality against the configured minimum.    */
        if (score < s_ctx.preamble_threshold) {
            ++j;
            continue;
        }

        ++local_preambles;

        /* Timestamp this frame: block start plus the preamble's sample offset. */
        const int64_t rx_us = block_t_us + (int64_t)((double)j * us_per_sample + 0.5);

        /* Slice the maximum 112 bits; the DF tells us if it is really short.   */
        uint8_t data[MODES_LONG_BYTES];
        int got = slice_bits(m, n, j, MODES_LONG_BITS, data);

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

    /* Precompute the DSP geometry: samples-per-µs in 32.32 fixed point and a   */
    /* rounded integer copy for coarse span math.                              */
    const double sp_us = (double)s_ctx.sample_rate_hz / 1e6;
    s_ctx.fp_samples_per_us = (uint64_t)(sp_us * (double)DEMOD_FP_ONE);

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
    ESP_LOGI(TAG, "init ok: %u sps, %.3f samples/us, threshold %u",
             (unsigned)s_ctx.sample_rate_hz, sp_us,
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
