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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
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

    /* ---- precomputed DSP geometry (set in init) ---- */
    uint64_t fp_samples_per_us;     /**< samples/µs in 32.32 fixed-point.        */

    /* ---- magnitude LUT (heap, 64 KiB) ---- */
    uint16_t *mag_lut;              /**< [256*256] (I<<8 | Q) → magnitude.        */

    /* ---- per-block magnitude scratch (heap, grows to fit) ---- */
    uint16_t *mag;                  /**< Magnitude buffer for the current block.  */
    uint32_t  mag_cap;             /**< Capacity of @c mag in samples.           */

    /* ---- runtime handles ---- */
    RingbufHandle_t iq_ring;        /**< Borrowed source ring (from usb_rtlsdr).  */
    QueueHandle_t   out_queue;      /**< Borrowed destination frame queue.        */
    TaskHandle_t    task;           /**< The Core-0 demod task.                   */
    volatile bool   running;        /**< Task should keep looping while true.     */
    volatile bool   task_alive;     /**< Set by task on entry, cleared on exit.   */

    /* ---- ring sequence tracking (task-private, reset on start) ---- */
    uint32_t last_seq;              /**< Last iq_block_t.seq we consumed.         */
    bool     have_seq;             /**< false until the first block arrives.     */

    /* ---- stats (guarded by stats_mux) ---- */
    SemaphoreHandle_t stats_mux;
    demod1090_stats_t stats;

    bool inited;                    /**< demod1090_init() succeeded.              */
} demod1090_ctx_t;

#ifdef __cplusplus
}
#endif
