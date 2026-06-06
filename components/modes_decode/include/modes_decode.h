/**
 * @file    modes_decode.h
 * @brief   Mode-S / ADS-B decoder — public contract (plan S4.3).
 *
 * @details
 *   Validates and parses raw candidate frames from demod1090 and emits decoded
 *   ::adsb_msg_t to the traffic manager. Accepts DF17 (ADS-B) and DF18 (TIS-B).
 *
 *   POSITION RESOLUTION HAPPENS HERE (reconciled fork choice, plan S4.3): this
 *   component owns the per-ICAO CPR pairing cache and runs global even/odd
 *   pairing (no ownship needed) plus optional local decode against a reference.
 *   The emitted ::adsb_msg_t therefore already carries an ABSOLUTE lat/lon when
 *   @c has_position is set; the traffic manager never handles raw CPR.
 *
 *   API SHAPE: the Core-1 decode task calls only modes_decode_frame(); the
 *   granular extract_*/decode_* helpers are exposed so the Python bench harness
 *   and host unit tests can drive decode field-by-field on canned frames.
 *
 * @par Core affinity
 *   Core 1 (::ADSBIN_CORE_DECODE). A single decode task owns the instance; the
 *   CPR cache is not internally locked (document, don't enforce). get_stats does
 *   an atomic-ish snapshot copy safe to call from the status task.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "adsbin_types.h"   /* adsb_msg_t, adsb_emitter_category_t */
#include "ownship.h"        /* ownship_ref_t (for local CPR)        */
#include "modes_types.h"
#include "cpr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Allocate the CPR pairing cache, seed CRC tables, apply @p cfg.
 *  @param cfg NULL => defaults. */
esp_err_t modes_decode_init(const modes_decode_cfg_t *cfg);

/** @brief Free the CPR pairing cache and internal state. */
void modes_decode_deinit(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Top-level entry — this is all the Core-1 decode task needs
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Decode one raw candidate frame end-to-end into @p out_msg.
 *
 * CRC + DF gate, parse, and (for position frames) CPR resolution against the
 * internal pairing cache and @p ref. @p frame may be freed/reused immediately
 * after return (nothing beyond a small cache entry is retained).
 *
 * @param frame         Raw 7- or 14-byte Mode-S frame, MSB-first.
 * @param frame_len     7 or 14.
 * @param rx_time_us    adsbin_now_us() when the frame was captured.
 * @param ref           Ownship reference for local CPR; NULL or !valid => global only.
 * @param out_msg       Filled on MODES_OK; field guards say what is populated.
 * @return ::modes_result_t status.
 */
modes_result_t modes_decode_frame(const uint8_t *frame, size_t frame_len,
                                  int64_t rx_time_us, const ownship_ref_t *ref,
                                  adsb_msg_t *out_msg);

/* ───────────────────────────────────────────────────────────────────────────
 *  Granular helpers (exposed for the bench harness + host unit tests)
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Verify the 24-bit Mode-S parity; recover ICAO for DF17/18. */
modes_crc_t modes_decode_check_crc(const uint8_t *frame, size_t frame_len, uint32_t *out_icao);

/** @brief Cheap pre-CRC DF gate (byte 0 >> 3) classification. */
modes_df_t modes_decode_df(const uint8_t *frame, size_t frame_len);

/** @brief Extract the 24-bit ICAO (AA) without position/velocity work. */
modes_result_t modes_decode_extract_icao(const uint8_t *frame, size_t frame_len, uint32_t *out_icao);

/** @brief Map the 5-bit ADS-B type code to a high-level kind. */
modes_msg_kind_t modes_decode_typecode(const uint8_t *frame);

/** @brief Decode TC 1-4 identification: callsign (8+NUL) + emitter category. */
modes_result_t modes_decode_identification(const uint8_t *frame,
                                           char out_callsign[ADSB_CALLSIGN_LEN],
                                           adsb_emitter_category_t *out_category);

/** @brief Decode TC 9-18 airborne position to raw CPR + barometric altitude. */
modes_result_t modes_decode_airborne_position(const uint8_t *frame, modes_cpr_frame_t *out_cpr,
                                              int32_t *out_altitude_ft, bool *out_altitude_valid);

/** @brief Decode TC 19 airborne velocity (ground speed / track / vertical rate). */
modes_result_t modes_decode_velocity(const uint8_t *frame, adsb_velocity_t *out_vel);

/**
 * @brief Resolve an absolute position for @p icao using the pairing cache + @p ref.
 *
 * Looks up the cached opposite-parity frame: runs global decode if a fresh pair
 * exists, else local decode against @p ref when valid; updates the cache.
 * @return MODES_OK only when a position was produced.
 */
modes_result_t modes_decode_position_cpr(uint32_t icao, const modes_cpr_frame_t *frame,
                                         int64_t rx_time_us, const ownship_ref_t *ref,
                                         double *out_lat, double *out_lon);

/* ───────────────────────────────────────────────────────────────────────────
 *  Diagnostics
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Static human-readable string for a ::modes_result_t (for sink_debug). */
const char *modes_result_str(modes_result_t result);

/** @brief Copy a snapshot of the running decode counters. */
void modes_decode_get_stats(modes_decode_stats_t *out_stats);

#ifdef __cplusplus
}
#endif
