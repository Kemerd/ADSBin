/**
 * @file    cpr.h
 * @brief   Compact Position Reporting (CPR) — raw frame container + pure math.
 *
 * @details
 *   CPR is how ADS-B packs lat/lon into 17-bit fields that only become absolute
 *   when you combine an even+odd pair (global) or apply a known reference
 *   (local). These functions are PURE (no state, no I/O) so they are unit-
 *   testable on a host PC and reusable by the Python bench harness; the stateful
 *   per-ICAO pairing cache lives inside modes_decode.c, not here.
 *
 *   Reference: ICAO Annex 10 / RTCA DO-260B CPR; see also the dump1090 lineage.
 *   Implementation must be CLEAN-ROOM from the public spec unless the project's
 *   THIRD_PARTY.md license path says otherwise.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One frame's raw CPR components, as parsed from an airborne-position ME.
 *
 * Owned by modes_decode; this is the unit the pairing cache stores and the CPR
 * math consumes.
 */
typedef struct {
    bool     odd;            /**< CPR format bit F (false = even, true = odd).  */
    uint32_t lat_cpr;        /**< 17-bit encoded latitude.                     */
    uint32_t lon_cpr;        /**< 17-bit encoded longitude.                    */
    bool     surface;        /**< false = airborne (MVP decodes airborne only).*/
    int64_t  rx_time_us;     /**< adsbin_now_us() when the frame was received. */
} modes_cpr_frame_t;

/**
 * @brief CPR "number of longitude zones" (NL) for a latitude.
 *
 * The transcendental helper both global and local decode depend on.
 * @param lat  Latitude in degrees.
 * @return NL value (1..59).
 */
int cpr_nl(double lat);

/**
 * @brief Global CPR: resolve absolute lat/lon from a matched even+odd pair.
 *
 * No reference position required (plan S5.4). Caller guarantees the two frames
 * are a fresh pair; @p latest_is_odd selects which one's zone anchors the result.
 *
 * @param even         The even-format frame.
 * @param odd          The odd-format frame.
 * @param latest_is_odd Which frame is the most recently received.
 * @param out_lat      Absolute latitude out (degrees).
 * @param out_lon      Absolute longitude out (degrees).
 * @return 0 on success; negative on NL mismatch / out-of-range reject.
 */
int cpr_global_decode(const modes_cpr_frame_t *even, const modes_cpr_frame_t *odd,
                      bool latest_is_odd, double *out_lat, double *out_lon);

/**
 * @brief Local CPR: resolve a single frame relative to a known reference.
 *
 * Faster (single message) but requires a nearby reference lat/lon (ownship).
 * @param frame    The airborne-position frame.
 * @param ref_lat  Reference latitude (degrees).
 * @param ref_lon  Reference longitude (degrees).
 * @param out_lat  Absolute latitude out.
 * @param out_lon  Absolute longitude out.
 * @return 0 on success; negative on reject.
 */
int cpr_local_decode(const modes_cpr_frame_t *frame, double ref_lat, double ref_lon,
                     double *out_lat, double *out_lon);

#ifdef __cplusplus
}
#endif
