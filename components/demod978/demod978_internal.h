/**
 * @file    demod978_internal.h
 * @brief   Private state + DSP geometry + the host-testable UAT core for demod978.
 *
 * @details
 *   INTERNAL to components/demod978. It holds the module-private singleton, the
 *   UAT air-interface constants (clean-room from the public DO-282B spec and the
 *   well-known sync words), and the pure bit-recovery / sync / frame-assembly
 *   functions. Those pure functions take a magnitude/discriminator buffer and
 *   produce decoded frames WITHOUT any FreeRTOS or ESP-IDF dependency, so the
 *   identical translation unit compiles into a host unit test that feeds it a
 *   synthetic FSK waveform.
 *
 *   WHY FSK, NOT PPM. UAT is binary continuous-phase FSK at 1.041667 Mbps: a "1"
 *   and "0" are two FM tones (~±312.5 kHz around 978 MHz). The demodulator is
 *   therefore a frequency discriminator (the sign of the instantaneous frequency
 *   gives the bit) followed by a 36-bit sync correlator — not the magnitude/
 *   preamble correlator the 1090 PPM path uses. No 64 KiB magnitude LUT is needed.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 *
 * Clean-room from the public DO-282B UAT spec + the published sync words. No code
 * adapted from dump978 / any GPL source.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "adsbin_types.h"   /* uat_frame_t, uat_uplink_t, UAT_* sizes */
#include "uat_fec.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  UAT air-interface constants (physical layer; independent of our sample rate).
 *
 *  Bit rate is 1.041667 Mbps. Every transmission opens with a 36-bit sync word;
 *  the ADS-B (air) and uplink (ground) sync words are bitwise complements of each
 *  other, which is how the demod tells the two message types apart. The frame
 *  geometry below is the ON-AIR coded length (sync excluded) the FEC layer needs.
 * ─────────────────────────────────────────────────────────────────────────── */
#define UAT_BIT_RATE_HZ          1041667u     /**< 1.041667 Mbps CPFSK.            */
#define UAT_SYNC_BITS            36            /**< Sync-word length in bits.       */

/* The two published 36-bit sync words (low 36 bits used). The uplink word is the
 * bitwise complement of the ADS-B word within 36 bits. */
#define UAT_ADSB_SYNC_WORD       0xEACDDA4E2ULL
#define UAT_UPLINK_SYNC_WORD     0x153225B1DULL
#define UAT_SYNC_MASK            ((1ULL << UAT_SYNC_BITS) - 1ULL)

/* Default Hamming tolerance for a 36-bit sync match (the spec receivers use 4). */
#define UAT_DEFAULT_SYNC_MAX_ERR 4

/* Coded on-air byte lengths AFTER the sync word (what the demod captures and the
 * FEC layer consumes). These come straight from the RS code geometry. */
#define UAT_ADSB_SHORT_CODED_BYTES  UAT_RS_ADSB_SHORT_N   /**< 30 (RS(30,18)).      */
#define UAT_ADSB_LONG_CODED_BYTES   UAT_RS_ADSB_LONG_N    /**< 48 (RS(48,34)).      */
#define UAT_UPLINK_CODED_BYTES      UAT_RS_UPLINK_FRAME_BYTES /**< 552 (6×92 interleaved).*/

/* Longest coded capture is the uplink frame (552 bytes). */
#define UAT_MAX_CODED_BYTES         UAT_UPLINK_CODED_BYTES

/* ───────────────────────────────────────────────────────────────────────────
 *  Fixed-point sample-phase math (shared idea with demod1090): a sample position
 *  is a 32.32 fixed-point in a uint64 so we walk fractional samples-per-bit
 *  (≈2.304 at 2.4 Msps) without a float divide on the hot path.
 * ─────────────────────────────────────────────────────────────────────────── */
#define UAT_FP_SHIFT             32
#define UAT_FP_ONE               (1ull << UAT_FP_SHIFT)

/* Default task knobs (mirror demod1090: high priority, below USB RX). */
#define UAT_DEFAULT_STACK        6144
#define UAT_DEFAULT_PRIORITY     20

/* ───────────────────────────────────────────────────────────────────────────
 *  Pure UAT core — HOST-TESTABLE (no FreeRTOS / ESP-IDF).
 *
 *  These operate on a discriminator buffer: one int16 per IQ sample giving the
 *  instantaneous frequency (sign = bit value). The task fills the buffer from the
 *  IQ block; the host test fills it from a synthesized FSK waveform. The result
 *  is delivered through a tiny callback so the same logic serves both the task
 *  (which enqueues) and the test (which asserts).
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief What kind of UAT frame the demod recovered. */
typedef enum {
    UAT_MSG_ADSB_SHORT = 0,   /**< 18-byte basic ADS-B (RS(30,18)).             */
    UAT_MSG_ADSB_LONG  = 1,   /**< 34-byte long ADS-B (RS(48,34)).              */
    UAT_MSG_UPLINK     = 2,   /**< 432-byte uplink / FIS-B (6× RS(92,72)).      */
} uat_msg_kind_t;

/**
 * @brief One recovered + FEC-corrected UAT message handed back by the core.
 *
 * @details
 *   For ADS-B kinds, @c data holds the corrected payload (18 or 34 bytes). For
 *   the uplink kind, @c data holds the 432-byte corrected payload. @c rs_errors
 *   is total symbols corrected; @c block_errors only meaningful for the uplink.
 */
typedef struct {
    uat_msg_kind_t kind;
    uint8_t        data[UAT_UPLINK_PAYLOAD_BYTES]; /**< Corrected payload.        */
    uint16_t       len;                            /**< 18 / 34 / 432.            */
    int            rs_errors;
    int            block_errors;
} uat_decoded_msg_t;

/** @brief Callback invoked for each successfully FEC-decoded message. */
typedef void (*uat_core_emit_fn)(const uat_decoded_msg_t *msg, void *user);

/**
 * @brief Recover the bit stream from a discriminator buffer, find sync words, and
 *        FEC-decode every framed message, invoking @p emit for each success.
 *
 * @param disc          Per-sample instantaneous-frequency buffer (sign = bit).
 * @param nsamps        Number of samples in @p disc.
 * @param fp_samp_per_bit  Samples-per-bit in 32.32 fixed-point.
 * @param sync_max_err  Max Hamming distance for a 36-bit sync match.
 * @param emit          Callback for each decoded message (may be NULL to count).
 * @param user          Opaque pointer passed to @p emit.
 * @param out_adsb_hits Optional: incremented per ADS-B sync match.
 * @param out_uplink_hits Optional: incremented per uplink sync match.
 * @param out_rs_fail   Optional: incremented per sync hit with uncorrectable FEC.
 * @return Number of messages emitted (FEC-decoded successfully).
 */
int uat_core_process(const int16_t *disc, uint32_t nsamps,
                     uint64_t fp_samp_per_bit, int sync_max_err,
                     uat_core_emit_fn emit, void *user,
                     uint64_t *out_adsb_hits, uint64_t *out_uplink_hits,
                     uint64_t *out_rs_fail);

/**
 * @brief Compute the per-sample FSK discriminator from interleaved 8-bit IQ.
 *
 * @details
 *   d[n] = sign/scale of the instantaneous frequency ≈ cross product of
 *   consecutive complex samples: q = I[n-1]*Q[n] - I[n]*Q[n-1] (after removing
 *   the RTL2832U offset-binary bias). The magnitude is not needed for a 2-FSK
 *   slicer, but we keep a scaled value so a soft slicer / sync correlator can use
 *   it. Writes @p nsamps-1 entries (the first sample has no predecessor).
 *
 * @param iq       Interleaved unsigned 8-bit I,Q,I,Q...
 * @param n_pairs  Number of IQ pairs in @p iq.
 * @param out      Receives n_pairs-1 discriminator values.
 * @return Number of discriminator samples written (n_pairs-1, or 0 if too short).
 */
uint32_t uat_discriminator(const uint8_t *iq, uint32_t n_pairs, int16_t *out);

/* ───────────────────────────────────────────────────────────────────────────
 *  Module-private singleton (the firmware side; not used by the host test).
 * ─────────────────────────────────────────────────────────────────────────── */
#ifndef UAT_HOST_TEST
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "demod978.h"

typedef struct {
    /* ---- configuration (resolved) ---- */
    uint32_t sample_rate_hz;
    uint8_t  task_core_id;
    uint8_t  task_priority;
    uint32_t task_stack_size;
    uint8_t  sync_max_errors;

    /* ---- precomputed geometry ---- */
    uint64_t fp_samp_per_bit;       /**< samples per UAT bit, 32.32 fixed-point.  */

    /* ---- per-block discriminator scratch (heap, grows to fit) ---- */
    int16_t *disc;                  /**< Discriminator buffer for current block.  */
    uint32_t disc_cap;              /**< Capacity in samples.                     */

    /* ---- runtime handles ---- */
    RingbufHandle_t iq_ring;        /**< Borrowed source ring (978-role).         */
    QueueHandle_t   out_frame_q;    /**< Borrowed uat_frame_t queue.              */
    RingbufHandle_t out_uplink_ring;/**< Borrowed uat_uplink_t ring.              */
    TaskHandle_t    task;
    volatile bool   running;
    volatile bool   task_alive;

    /* ---- ring sequence tracking ---- */
    uint32_t last_seq;
    bool     have_seq;

    /* ---- stats ---- */
    SemaphoreHandle_t stats_mux;
    demod978_stats_t  stats;

    bool inited;
} demod978_ctx_t;
#endif /* !UAT_HOST_TEST */

#ifdef __cplusplus
}
#endif
