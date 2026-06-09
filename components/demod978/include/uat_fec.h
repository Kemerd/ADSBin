/**
 * @file    uat_fec.h
 * @brief   Reed-Solomon forward-error-correction for the UAT (978 MHz) physical
 *          layer — public, host-unit-testable contract.
 *
 * @details
 *   UAT protects every on-air message with a Reed-Solomon code over GF(256). This
 *   module is a CLEAN-ROOM RS decoder (and a matching encoder, used only by the
 *   host tests to generate known-good codewords) written from the standard
 *   syndrome → Berlekamp-Massey → Chien → Forney algorithm and the PUBLIC UAT
 *   FEC parameters in RTCA DO-282B. NO code is lifted from libfec, dump978, or any
 *   GPL source — only the spec's numeric parameters are used:
 *
 *     - Field: GF(256) with primitive polynomial p(x) = x^8 + x^7 + x^2 + x + 1,
 *       i.e. 0x187 (the value passed as gfpoly to the reference init_rs_char).
 *     - First consecutive root (fcr) = 120; primitive element step (prim) = 1.
 *     - ADS-B "Basic" message : RS(30,18)  -> 12 parity bytes, corrects t = 6.
 *     - ADS-B "Long"  message : RS(48,34)  -> 14 parity bytes, corrects t = 7.
 *     - Uplink block          : RS(92,72)  -> 20 parity bytes, corrects t = 10.
 *
 *   The uplink frame is 6 RS(92,72) codewords BYTE-INTERLEAVED on the air; the
 *   deinterleave + per-block decode that reconstructs the 6 x 72 = 432-byte
 *   payload lives here too (::uat_fec_uplink_decode).
 *
 *   Everything here is allocation-free and free of ESP-IDF runtime calls so the
 *   identical translation unit links into a host unit test.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 *
 * Clean-room from the standard RS decoding algorithm + the public DO-282B UAT
 * FEC parameters. No code adapted from libfec / dump978 / any GPL source.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  UAT RS code geometry (DO-282B). n = codeword length, k = data length; the
 *  parity length is n - k and the decoder corrects up to (n - k)/2 byte errors.
 * ─────────────────────────────────────────────────────────────────────────── */
#define UAT_RS_ADSB_SHORT_N   30   /**< Basic ADS-B codeword bytes.              */
#define UAT_RS_ADSB_SHORT_K   18   /**< Basic ADS-B data bytes (RS(30,18)).      */
#define UAT_RS_ADSB_LONG_N    48   /**< Long ADS-B codeword bytes.               */
#define UAT_RS_ADSB_LONG_K    34   /**< Long ADS-B data bytes (RS(48,34)).       */
#define UAT_RS_UPLINK_N       92   /**< Uplink block codeword bytes.             */
#define UAT_RS_UPLINK_K       72   /**< Uplink block data bytes (RS(92,72)).     */

/* The uplink frame is six RS(92,72) codewords, byte-interleaved on the air. */
#define UAT_RS_UPLINK_BLOCKS  6                              /**< Interleave depth.*/
#define UAT_RS_UPLINK_FRAME_BYTES  (UAT_RS_UPLINK_N * UAT_RS_UPLINK_BLOCKS)   /**< 552 on-air.*/
#define UAT_RS_UPLINK_DATA_BYTES   (UAT_RS_UPLINK_K * UAT_RS_UPLINK_BLOCKS)   /**< 432 payload.*/

/* ───────────────────────────────────────────────────────────────────────────
 *  GF(256) tables — built ONCE, lazily, on first decode/encode call. Exposed so
 *  the host test can pin them against known field values.
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Force-build the GF(256) log/antilog tables (idempotent). */
void uat_fec_init(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Decode. Each takes a full codeword IN PLACE: on success the first k bytes are
 *  the corrected data. Returns the number of byte errors corrected (0..t), or
 *  -1 if the codeword is uncorrectable (more than t errors).
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Decode one RS codeword in place.
 *
 * @param data    Codeword of length @p n (data || parity); corrected in place.
 * @param n       Codeword length (30 / 48 / 92).
 * @param k       Data length (18 / 34 / 72). Parity = n - k.
 * @return Symbols corrected (>= 0), or -1 if uncorrectable.
 */
int uat_fec_rs_decode(uint8_t *data, int n, int k);

/** @brief Decode a Basic ADS-B codeword (RS(30,18)) in place. */
int uat_fec_adsb_short_decode(uint8_t *cw30);

/** @brief Decode a Long ADS-B codeword (RS(48,34)) in place. */
int uat_fec_adsb_long_decode(uint8_t *cw48);

/**
 * @brief Decode a full UAT uplink frame: deinterleave 6 blocks, RS-decode each,
 *        and concatenate the corrected 72-byte data parts.
 *
 * @param frame_in   ::UAT_RS_UPLINK_FRAME_BYTES (552) interleaved on-air bytes.
 * @param data_out   Receives ::UAT_RS_UPLINK_DATA_BYTES (432) corrected bytes.
 * @param total_corrected  Optional: total symbols corrected across all blocks.
 * @param block_errors     Optional: count of blocks that were UNCORRECTABLE.
 * @return true if EVERY block decoded (data_out fully valid); false if any block
 *         was uncorrectable (block_errors says how many; data_out is best-effort).
 */
bool uat_fec_uplink_decode(const uint8_t *frame_in, uint8_t *data_out,
                           int *total_corrected, int *block_errors);

/* ───────────────────────────────────────────────────────────────────────────
 *  Encode — used ONLY by the host tests (and any future TX path, which ADSBin
 *  does not have) to produce known-good codewords. Not on the firmware hot path.
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Systematic RS-encode: append (n - k) parity bytes after k data bytes.
 *
 * @param data   First @p k bytes are the message; bytes [k..n) are overwritten
 *               with the computed parity.
 * @param n      Codeword length. @param k Data length.
 */
void uat_fec_rs_encode(uint8_t *data, int n, int k);

#ifdef __cplusplus
}
#endif
