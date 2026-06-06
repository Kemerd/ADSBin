/**
 * @file    cpr.c
 * @brief   Compact Position Reporting (CPR) — pure, stateless decode math.
 *
 * @details
 *   CPR is the scheme ADS-B uses to squeeze a latitude/longitude into two
 *   17-bit fields. A single message is ambiguous (it only locates the aircraft
 *   inside one "zone"); resolving an absolute fix needs either a matched
 *   even+odd pair (GLOBAL decode, no prior position required) or a known nearby
 *   reference (LOCAL decode, single message). Both algorithms are reproduced
 *   here straight from the public specification — ICAO Annex 10 Vol IV and RTCA
 *   DO-260B §2.2.3.2.3 — so nothing in this file carries state or touches I/O.
 *
 *   Keeping the math pure makes it trivially host-unit-testable and lets the
 *   Python bench harness exercise it directly; the stateful per-ICAO pairing
 *   cache that *feeds* these functions lives in modes_decode.c, never here.
 *
 * @par Provenance / license
 *   Structure of the global/local decoders and the NL lookup is adapted from
 *   Salvatore Sanfilippo's (antirez) original dump1090, which is BSD-licensed
 *   and explicitly permitted by the project's source policy. The BSD copyright
 *   notice is preserved below. Number tables (NL boundaries) are independently
 *   keyed from the ICAO/DO-260B specification.
 *
 *   Original dump1090 portions:
 *     Copyright (c) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
 *     Redistribution and use in source and binary forms, with or without
 *     modification, are permitted provided that the BSD 3-clause conditions are
 *     met. THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "cpr.h"
#include "modes_internal.h"

#include <math.h>

/* ───────────────────────────────────────────────────────────────────────────
 *  cpr_nl — number of longitude zones (NL) for a given latitude.
 *
 *  NL drops from 59 at the equator to 1 near the poles. The canonical formula
 *  is a closed-form arccos expression, but it is sensitive to floating-point
 *  rounding right at the zone boundaries, which is exactly where a misclassed
 *  value corrupts a position. The DO-260B-blessed approach is therefore a
 *  precomputed boundary TABLE: NL(lat) is the count of table entries whose
 *  transition latitude the input has not yet exceeded. We key the table from
 *  the spec rather than copying any implementation's literal array.
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Latitude (degrees) at which NL steps down from N to N-1.
 *
 * Entry @c k is the transition latitude below which NL == (59 - k) holds, i.e.
 * the boundary between NL=59 and NL=58 sits at NLAT_BOUNDARY[0]. These are the
 * standard CPR zone boundaries derived from the arccos definition.
 */
static const double NLAT_BOUNDARY[58] = {
    10.47047130, 14.82817437, 18.18626357, 21.02939493,
    23.54504487, 25.82924707, 27.93898710, 29.91135686,
    31.77209708, 33.53993436, 35.22899598, 36.85025108,
    38.41241892, 39.92256684, 41.38651832, 42.80914012,
    44.19454951, 45.54626723, 46.86733252, 48.16039128,
    49.42776439, 50.67150166, 51.89342469, 53.09516153,
    54.27817472, 55.44378444, 56.59318756, 57.72747354,
    58.84763776, 59.95459277, 61.04917774, 62.13216659,
    63.20427479, 64.26616523, 65.31845310, 66.36171008,
    67.39646774, 68.42322022, 69.44242631, 70.45451075,
    71.45986473, 72.45884545, 73.45177442, 74.43893416,
    75.42056257, 76.39684391, 77.36789461, 78.33374083,
    79.29428225, 80.24923213, 81.19801349, 82.13956981,
    83.07199445, 83.99173563, 84.89166191, 85.75541621,
    86.53536998, 87.00000000,
};

int cpr_nl(double lat)
{
    // CPR is symmetric about the equator; fold to the northern hemisphere so a
    // single ascending boundary table covers both.
    if (lat < 0.0) {
        lat = -lat;
    }

    // Above the last boundary there is exactly one longitude zone: the poles.
    if (lat >= 87.0) {
        return 1;
    }

    // NL starts at 59 (equator) and decrements once for every boundary the
    // latitude has climbed past. A linear scan over 58 entries is plenty fast
    // for the handful of positions we decode per second.
    int nl = 59;
    for (int i = 0; i < 58; ++i) {
        if (lat < NLAT_BOUNDARY[i]) {
            break;          // still inside the NL == (59 - i) band
        }
        --nl;
    }
    return nl;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Small math helpers kept local so the decoders read cleanly.
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Positive-result modulo, the "mod()" used throughout the CPR spec.
 *
 * C's fmod() keeps the sign of the dividend; CPR needs a strictly non-negative
 * result so a southern/western zone index does not go negative.
 */
static inline double cpr_mod(double a, double b)
{
    double r = fmod(a, b);
    return (r < 0.0) ? r + b : r;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  cpr_global_decode — absolute fix from a matched even+odd pair.
 *
 *  Algorithm (DO-260B §2.2.3.2.3, airborne):
 *    1. Recover the latitude zone index j from the two encoded latitudes.
 *    2. Reconstruct both candidate latitudes (even uses dLat=6°, odd ≈6.1°).
 *    3. Require both latitudes to fall in the SAME NL zone, otherwise the pair
 *       is inconsistent and must be rejected.
 *    4. Recover the longitude using the NL of the *most recent* frame.
 * ─────────────────────────────────────────────────────────────────────────── */
int cpr_global_decode(const modes_cpr_frame_t *even, const modes_cpr_frame_t *odd,
                      bool latest_is_odd, double *out_lat, double *out_lon)
{
    // Defensive: pure function, but a NULL here means the cache fed us garbage.
    if (even == NULL || odd == NULL || out_lat == NULL || out_lon == NULL) {
        return -1;
    }

    // ── Normalise the raw 17-bit fields into [0,1) fractions. ────────────────
    const double lat_even = (double)even->lat_cpr / CPR_MAX;
    const double lat_odd  = (double)odd->lat_cpr  / CPR_MAX;
    const double lon_even = (double)even->lon_cpr / CPR_MAX;
    const double lon_odd  = (double)odd->lon_cpr  / CPR_MAX;

    // ── Step 1: latitude zone index j (common to both frames). ───────────────
    // j = floor( 59*latEven - 60*latOdd + 0.5 ). The 59/60 come from the number
    // of even (60) vs odd (59) zones over the hemisphere.
    const double j = floor(((59.0 * lat_even) - (60.0 * lat_odd)) + 0.5);

    // ── Step 2: candidate latitudes for each format. ─────────────────────────
    // rlat = dLat * ( mod(j, n_zones) + latFraction ). We then map the northern
    // [0,90) result into the proper hemisphere by subtracting 360 above 270°.
    double rlat_even = CPR_DLAT_EVEN * (cpr_mod(j, 60.0) + lat_even);
    double rlat_odd  = CPR_DLAT_ODD  * (cpr_mod(j, 59.0) + lat_odd);

    if (rlat_even >= 270.0) rlat_even -= 360.0;  // southern-hemisphere wrap
    if (rlat_odd  >= 270.0) rlat_odd  -= 360.0;

    // ── Step 3: both latitudes must agree on their longitude-zone count. ─────
    // A mismatched NL means the two frames came from different latitude bands
    // (different aircraft or a stale/garbled pair) and the result is invalid.
    if (cpr_nl(rlat_even) != cpr_nl(rlat_odd)) {
        return -1;
    }

    // ── Step 4: pick the latitude of the most recent frame, then longitude. ──
    double rlat = latest_is_odd ? rlat_odd : rlat_even;

    // A sanity clamp: real latitudes never exceed the poles. Anything outside
    // this came from corrupted bits that slipped through CRC by chance.
    if (rlat < -90.0 || rlat > 90.0) {
        return -1;
    }

    // Longitude reconstruction uses the NL of the latitude we just anchored on.
    const int nl = cpr_nl(rlat);

    // Longitude zone index m is shared by both formats; it is derived from the
    // two encoded longitudes weighted by NL and NL-1.
    const double m = floor((lon_even * (double)(nl - 1) - lon_odd * (double)nl) + 0.5);

    // Number of longitude zones for the anchoring frame: NL for even, NL-1 for
    // odd, clamped to ≥1 so a polar latitude (NL==1) never divides by zero.
    int ni_i = nl - (latest_is_odd ? 1 : 0);
    if (ni_i < 1) ni_i = 1;
    const double ni = (double)ni_i;

    // Longitude bin width and the fraction belonging to the most recent frame.
    const double dlon = 360.0 / ni;
    const double lon_fraction = latest_is_odd ? lon_odd : lon_even;

    // Absolute longitude in [0,360); folded to [-180,180] just below.
    double rlon = dlon * (cpr_mod(m, ni) + lon_fraction);

    // Fold the result into the conventional [-180, 180] range.
    if (rlon >= 180.0) {
        rlon -= 360.0;
    }

    *out_lat = rlat;
    *out_lon = rlon;
    return 0;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  cpr_local_decode — single-frame fix relative to a known reference.
 *
 *  Algorithm (DO-260B §2.2.3.2.4): the reference position resolves the zone
 *  ambiguity directly, so no pairing is needed. Valid only when the true
 *  position is within ~½ zone (≈180 NM) of the reference, which ownship always
 *  is for traffic we care about.
 * ─────────────────────────────────────────────────────────────────────────── */
int cpr_local_decode(const modes_cpr_frame_t *frame, double ref_lat, double ref_lon,
                     double *out_lat, double *out_lon)
{
    if (frame == NULL || out_lat == NULL || out_lon == NULL) {
        return -1;
    }

    const double lat_cpr = (double)frame->lat_cpr / CPR_MAX;
    const double lon_cpr = (double)frame->lon_cpr / CPR_MAX;

    // dLat depends only on the frame's format bit.
    const double dlat = frame->odd ? CPR_DLAT_ODD : CPR_DLAT_EVEN;

    // ── Latitude: choose the zone nearest the reference. ─────────────────────
    // j picks the multiple of dLat that lands closest to ref_lat.
    const double j = floor(ref_lat / dlat) +
                     floor(0.5 + cpr_mod(ref_lat, dlat) / dlat - lat_cpr);
    double rlat = dlat * (j + lat_cpr);

    // ── Longitude: use the NL of the just-computed latitude. ─────────────────
    const int nl = cpr_nl(rlat);

    // Even frames span NL zones, odd frames span NL-1 (≥1 near the poles).
    int ni = frame->odd ? (nl - 1) : nl;
    if (ni < 1) ni = 1;
    const double dlon = 360.0 / (double)ni;

    const double m = floor(ref_lon / dlon) +
                     floor(0.5 + cpr_mod(ref_lon, dlon) / dlon - lon_cpr);
    double rlon = dlon * (m + lon_cpr);

    // Range sanity: a local fix should not have drifted off the planet.
    if (rlat < -90.0 || rlat > 90.0) {
        return -1;
    }
    if (rlon > 180.0)  rlon -= 360.0;
    if (rlon < -180.0) rlon += 360.0;

    *out_lat = rlat;
    *out_lon = rlon;
    return 0;
}
