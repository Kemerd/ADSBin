/**
 * @file    demod1090_internal.h
 * @brief   Private state + DSP geometry for the 1090ES demodulator.
 *
 * @details
 *   This header is INTERNAL to components/demod1090. Nothing outside the
 *   component includes it. It keeps the public demod1090.h clean while giving
 *   the .c a single place to declare the module-private singleton, the
 *   compile-time DSP geometry derived from the sample rate, and the small
 *   helper inlines that both the task loop and any future host-side unit test
 *   would share.
 *
 *   The geometry here is the heart of why this demod works at the ADSBin
 *   sample rate (2.4 Msps) rather than the classic dump1090 2.0 Msps: every
 *   bit/half-bit/preamble-pulse offset is expressed in microseconds and then
 *   converted to a *fractional* sample position with a fixed-point phase
 *   accumulator, so we are not locked to an integer "samples-per-bit".
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
/* Host unit tests compile this component with -DDEMOD1090_HOST_TEST=1 (see
 * test_host/): the FreeRTOS task shell is excluded and only the pure DSP core
 * (LUT, preamble correlator, PPM slicer, scan loop) is built, exactly like
 * demod978's UAT_HOST_TEST arrangement. */
#ifndef DEMOD1090_HOST_TEST
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#endif
#include "adsbin_types.h"
#include "demod1090.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  Mode-S air-interface timing (the numbers the chip transmits, in µs).
 *
 *  These are physical constants of the 1090 MHz downlink and never change with
 *  our sample rate. The preamble is a fixed 8 µs sync burst with 0.5 µs pulses
 *  whose leading edges land at 0.0, 1.0, 3.5 and 4.5 µs. Each data bit is a
 *  1 µs PPM symbol: energy in the first half == 1, energy in the second == 0.
 * ─────────────────────────────────────────────────────────────────────────── */
#define DEMOD_PREAMBLE_US        8.0    /**< Full preamble window length.        */
#define DEMOD_BIT_US             1.0    /**< One PPM data-bit symbol period.     */
#define DEMOD_HALFBIT_US         0.5    /**< Half-bit (the PPM decision window).  */

/* The four preamble pulse leading edges, in microseconds from window start.    */
#define DEMOD_PULSE0_US          0.0
#define DEMOD_PULSE1_US          1.0
#define DEMOD_PULSE2_US          3.5
#define DEMOD_PULSE3_US          4.5

/* ───────────────────────────────────────────────────────────────────────────
 *  Fixed-point sample-phase math.
 *
 *  We never want a float divide on the Core-0 hot path, so a "sample position"
 *  is carried as a 32.32-style fixed-point in a uint64: the high 32 bits are an
 *  integer sample index, the low 32 bits a sub-sample fraction. DEMOD_FP_ONE is
 *  one whole sample. samples_per_us is precomputed once into the same format.
 * ─────────────────────────────────────────────────────────────────────────── */
#define DEMOD_FP_SHIFT           32
#define DEMOD_FP_ONE             (1ull << DEMOD_FP_SHIFT)

/* ───────────────────────────────────────────────────────────────────────────
 *  Demod working limits.
 *
 *  A 112-bit long frame plus its 8 µs preamble spans 120 µs. At the maximum
 *  sane rate that we support this is comfortably under DEMOD_MAX_FRAME_SAMPLES
 *  samples; the constant bounds how far past a detected preamble we are willing
 *  to read inside one IQ block before we give up and wait for the next one.
 * ─────────────────────────────────────────────────────────────────────────── */
#define DEMOD_FULL_FRAME_US      (DEMOD_PREAMBLE_US + (double)MODES_LONG_BITS * DEMOD_BIT_US)

/* Stack default if the caller passes 0. The hot loop keeps almost nothing on  */
/* the stack (the magnitude scratch lives in the heap-allocated state), so a   */
/* modest stack is plenty even with ESP-IDF's per-task overhead.               */
#define DEMOD_DEFAULT_STACK      4096
#define DEMOD_DEFAULT_PRIORITY   20      /**< High, but below the USB RX task.   */
#define DEMOD_DEFAULT_PREAMBLE   28      /**< Default min correlation score.     */

/* ───────────────────────────────────────────────────────────────────────────
 *  Multi-phase PREAMBLE DETECTION bank.
 *
 *  At 2.4 Msps the preamble's true start lands anywhere inside one sample
 *  period, and a single fixed sample-index template only "sees" arrivals whose
 *  phase happens to put its point samples on the pulse plateaus — a large slice
 *  of real arrivals is simply undetectable that way. So detection itself runs
 *  DEMOD_PRE_PHASES index templates, each precomputed (at init, in integers)
 *  for a different assumed sub-sample arrival phase; a candidate passes if ANY
 *  template accepts, and the winning template's phase seeds the data slicer.
 *  3 templates = ±1/6-sample worst-case phase error at detection, well inside
 *  the ±0.6-sample half-bit decision margin.
 * ─────────────────────────────────────────────────────────────────────────── */
#define DEMOD_PRE_PHASES         3       /**< Preamble phase templates tried.    */
#define DEMOD_PRE_PULSES         4       /**< Mode-S preamble pulse count.       */
#define DEMOD_PRE_VALLEYS        5       /**< Quiet points checked per template. */

/**
 * @brief One precomputed preamble index template for a given sub-sample phase.
 *
 * @details
 *   Offsets are integer samples RELATIVE to the candidate start index, chosen
 *   at init so each lands nearest the centre of its pulse / quiet slot for the
 *   template's assumed arrival phase. The hot loop is then pure integer loads
 *   and compares — no floating point of any kind (the P4 has no double FPU;
 *   soft-double per scanned sample was what put the demod 3–9× over budget).
 */
typedef struct {
    uint16_t pulse[DEMOD_PRE_PULSES];    /**< Pulse-centre sample offsets.       */
    uint16_t valley[DEMOD_PRE_VALLEYS];  /**< Quiet-slot sample offsets.         */
} demod_pre_tpl_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Multi-phase DATA slicing bank.
 *
 *  Detection pins the arrival phase to ±1/6 sample (one template class, above),
 *  or ±1/2 sample if the scorer picked a neighbouring class on a noisy burst.
 *  The data slicer therefore sweeps DEMOD_PHASE_STEPS hypotheses spaced
 *  DEMOD_PHASE_STEP_FP apart, CENTRED on the detected phase (covering ±1/3 of a
 *  sample), and keeps the phase whose per-bit PPM margins are largest (the
 *  matched-filter-optimal phase). See CITATIONS.md §B.
 * ─────────────────────────────────────────────────────────────────────────── */
#define DEMOD_PHASE_STEPS        5       /**< Sub-sample phase hypotheses tried. */
#define DEMOD_PHASE_STEP_FP      (DEMOD_FP_ONE / 6)  /**< Spacing: 1/6 sample.   */

/* ───────────────────────────────────────────────────────────────────────────
 *  Magnitude look-up table.
 *
 *  The RTL2832U delivers offset-binary unsigned 8-bit I and Q (mid-scale
 *  ~127.4 == 0). Magnitude = sqrt(I'² + Q'²) where I' = I-127.4. Computing that
 *  per sample at 2.4 Msps would be murder, so we precompute every (I,Q) → mag
 *  into a 256×256 = 64 KiB table of uint16. The result is scaled so a full-
 *  scale tone maps near 65535, preserving headroom for the correlation sums.
 * ─────────────────────────────────────────────────────────────────────────── */
#define DEMOD_MAG_LUT_SIZE       (256 * 256)

/* ───────────────────────────────────────────────────────────────────────────
 *  The module-private singleton. One demodulator per firmware image; the public
 *  API is stateless-looking but backed by this. All cross-thread access to the
 *  stats block is guarded by stats_mux so get/reset are a coherent snapshot.
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ---- configuration (resolved, post-defaults) ---- */
    uint32_t sample_rate_hz;        /**< Effective IQ sample rate.               */
    uint8_t  task_core_id;          /**< Core the task is pinned to.             */
    uint8_t  task_priority;         /**< Task priority.                          */
    uint32_t task_stack_size;       /**< Task stack in bytes.                    */
    uint8_t  preamble_threshold;    /**< Min preamble correlation score.         */

    /* ---- precomputed DSP geometry (set once in init; ALL hot-path index and
     *      span math reads these — the scan loop itself never touches floating
     *      point, because every double op is a soft-float library call on the
     *      P4's single-precision-only FPU) ---- */
    uint64_t fp_samples_per_us;     /**< samples/µs in 32.32 fixed-point.        */
    uint64_t fp_bit;                /**< One 1 µs PPM bit, fp samples.           */
    uint64_t fp_half;               /**< Half a bit (PPM decision window), fp.   */
    uint64_t fp_data_start;         /**< Preamble→data gap (8 µs), fp samples.   */
    uint32_t span_preamble;         /**< Whole preamble, integer samples (ceil). */
    uint32_t span_short;            /**< Preamble + 56 bits, samples (+margin).  */
    uint32_t span_full;             /**< Preamble + 112 bits, samples (+margin). */

    /* ---- preamble phase-template bank (see DEMOD_PRE_PHASES) ---- */
    demod_pre_tpl_t pre_tpl[DEMOD_PRE_PHASES]; /**< Integer index templates.     */
    uint64_t pre_phase_fp[DEMOD_PRE_PHASES];   /**< Each template's phase, fp.   */
    uint32_t pre_window;            /**< Max template offset + slack (bounds).   */
    uint32_t gate_idx_lo;           /**< First pulse-0 offset across templates.  */
    uint32_t gate_idx_hi;           /**< Last  pulse-0 offset across templates.  */

    /* ---- magnitude LUT (heap, 64 KiB) ---- */
    uint16_t *mag_lut;              /**< [256*256] (I<<8 | Q) → magnitude.        */

    /* ---- per-block magnitude scratch (heap, grows to fit) ---- */
    uint16_t *mag;                  /**< Magnitude buffer for the current block.  */
    uint32_t  mag_cap;             /**< Capacity of @c mag in samples.           */

#ifndef DEMOD1090_HOST_TEST
    /* ---- runtime handles (firmware only; the host test has no RTOS) ---- */
    RingbufHandle_t iq_ring;        /**< Borrowed source ring (from usb_rtlsdr).  */
    QueueHandle_t   out_queue;      /**< Borrowed destination frame queue.        */
    TaskHandle_t    task;           /**< The Core-0 demod task.                   */
    volatile bool   running;        /**< Task should keep looping while true.     */
    volatile bool   task_alive;     /**< Set by task on entry, cleared on exit.   */

    /* ---- ring sequence tracking (task-private, reset on start) ---- */
    uint32_t last_seq;              /**< Last iq_block_t.seq we consumed.         */
    bool     have_seq;             /**< false until the first block arrives.     */

    /* ---- stats lock (host build keeps bare counters, no readers race) ---- */
    SemaphoreHandle_t stats_mux;
#endif
    demod1090_stats_t stats;

    bool inited;                    /**< demod1090_init() succeeded.              */
} demod1090_ctx_t;

#ifdef DEMOD1090_HOST_TEST
/* ───────────────────────────────────────────────────────────────────────────
 *  Host-test surface. The test provides the capture hook; the shims below are
 *  implemented in demod1090.c under the same guard and drive the SAME pure
 *  pipeline the firmware task uses (LUT → scan → slice → emit).
 * ─────────────────────────────────────────────────────────────────────────── */
/** @brief Build the LUT + geometry for a host run (no task, no RTOS). */
esp_err_t demod1090_host_setup(uint32_t sample_rate_hz, uint8_t preamble_threshold);
/** @brief Run one interleaved-IQ buffer through magnitude + preamble scan.
 *  @return number of magnitude samples processed. */
uint32_t demod1090_host_process_iq(const uint8_t *iq, uint32_t n_bytes, int64_t t_us);
/** @brief Free host-run allocations. */
void demod1090_host_teardown(void);
/** @brief PROVIDED BY THE TEST: receives every emitted candidate frame. */
void demod1090_host_capture(const modes_frame_t *frame);
#endif

#ifdef __cplusplus
}
#endif
