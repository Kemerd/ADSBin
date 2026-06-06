/**
 * @file    modes_internal.h
 * @brief   Private glue shared between modes_decode.c and cpr.c.
 *
 * @details
 *   NOT a public header — it never leaves the component. It only collects the
 *   couple of constants and tiny bit-extraction helpers that both translation
 *   units want, so the CPR math and the frame parser agree on the exact field
 *   geometry (17-bit CPR words, the 2^17 normaliser, etc.) without either file
 *   re-deriving a magic number.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  CPR field geometry (ICAO Annex 10 Vol IV / RTCA DO-260B §2.2.3.2.3).
 *
 *  Each CPR latitude/longitude is a 17-bit fraction of one "zone". 2^17 is the
 *  normaliser that turns the raw integer back into a [0,1) fraction; both the
 *  global and local decoders lean on it, so it lives here once.
 * ─────────────────────────────────────────────────────────────────────────── */
#define CPR_BITS        17                       /**< Encoded field width.        */
#define CPR_MAX         131072.0                 /**< 2^17, the fraction divisor. */
#define CPR_NZ          15                       /**< Number of latitude zones NZ.*/

/* Airborne CPR zone size in latitude: 360/(4*NZ) for even, 360/(4*NZ-1) for odd.
 * Pre-computing the constants keeps the hot path free of redundant divides. */
#define CPR_DLAT_EVEN   (360.0 / (4.0 * CPR_NZ))         /* = 6.0 degrees.        */
#define CPR_DLAT_ODD    (360.0 / (4.0 * CPR_NZ - 1.0))   /* ≈ 6.101694915 degrees.*/

/* ───────────────────────────────────────────────────────────────────────────
 *  Bit helpers — read an arbitrary big-endian bit run out of an MSB-first frame.
 *
 *  Mode-S fields are specified by bit number counting from 1 at the MSB of the
 *  whole 56/112-bit frame; these helpers translate that "bit n, width w" spec
 *  straight into a value so the parser reads like the ICAO tables.
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Extract @p nbits starting at zero-based bit @p start_bit (MSB-first).
 *
 * @param data       Frame bytes, MSB-first (bit 0 == MSB of data[0]).
 * @param start_bit  Zero-based index of the first (most significant) bit to read.
 * @param nbits      How many bits to read (1..32).
 * @return The extracted field, right-justified in a uint32.
 */
static inline uint32_t modes_get_bits(const uint8_t *data,
                                      unsigned start_bit, unsigned nbits)
{
    uint32_t value = 0;

    // Walk the requested bits one at a time. This is not the fastest possible
    // extractor, but frames are tiny (≤112 bits) and decode runs off the hot
    // Core-0 path, so clarity wins over a shift-heavy byte-spanning version.
    for (unsigned i = 0; i < nbits; ++i) {
        unsigned bit = start_bit + i;
        unsigned byte_idx = bit >> 3;          // which byte holds this bit
        unsigned bit_in_byte = 7u - (bit & 7u);// MSB-first within the byte
        uint32_t b = (data[byte_idx] >> bit_in_byte) & 1u;
        value = (value << 1) | b;              // shift it into the accumulator
    }
    return value;
}

#ifdef __cplusplus
}
#endif
