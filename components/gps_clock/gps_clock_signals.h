/**
 * @file    gps_clock_signals.h
 * @brief   INTERNAL raw-signal structs shared producers → supervisor (not public).
 *
 * @details
 *   The architectural rule of this component is that there is exactly ONE writer
 *   of clock quality: the supervisor. Layer 2 (NMEA parser) and Layer 3 (PPS PI
 *   filter) are pure SIGNAL PRODUCERS — they emit raw facts into these structs and
 *   NEVER promote/demote or touch the published ::gps_clock_t. The supervisor owns
 *   every debounce counter and the entire transition table, which is what makes
 *   the ladder flap-free by construction (no two machines can disagree).
 *
 *   These structs are plain-old-data, filled and read on the SAME Core-1 task, so
 *   they need no locking among themselves — the supervisor pulls fresh signals,
 *   runs the ladder, then publishes the one coherent ::gps_clock_t via the seqlock.
 *
 *   This header is component-private (it lives next to the .c files, not under
 *   include/), so nothing outside gps_clock can take a dependency on the raw seam.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  Layer 2 (NMEA) raw output — one merged fix per UTC second.
 *
 *  The parser fills this from RMC (+ GGA for the same second) and bumps
 *  @c byte_count for every byte it consumes (the supervisor uses byte flow as the
 *  module-presence and hard-unplug signal). It NEVER decides quality.
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct {
    /* Presence / liveness. */
    uint32_t byte_count;        /**< Monotonic count of UART bytes consumed.        */

    /* Fix validity (the supervisor promotes to NMEA_FIX on a run of these). */
    bool     fix_valid;         /**< RMC status 'A' AND GGA quality ≥ 1, same sec.  */
    int64_t  fix_seq;           /**< Bumped once per accepted same-second fix.      */

    /* UTC label for this fix. */
    int64_t  utc_us;            /**< UTC of the fix, µs since Unix epoch.           */
    int64_t  utc_anchor_now_us; /**< adsbin_now_us() captured at the LF of the fix. */
    int64_t  rmc_second;        /**< Integer UTC second-of-epoch (PPS second label).*/
    uint32_t rmc_frac_ns;       /**< Sub-second part of the RMC time, ns.           */

    /* Position / velocity for ownship_update(). */
    double   lat_deg;           /**< WGS-84 latitude.                               */
    double   lon_deg;           /**< WGS-84 longitude.                              */
    float    altitude_m;        /**< MSL altitude (GGA), metres, or NAN.            */
    uint16_t ground_speed_kt;   /**< Ground speed, knots (RMC).                     */
    uint16_t track_deg;         /**< True ground track, 0..359 (RMC).               */
    bool     has_velocity;      /**< true => speed/track above are valid.           */
} gps_nmea_signals_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Layer 3 (PPS) raw output — one update per supervisor tick.
 *
 *  The PI filter fills this from the hardware-captured 1PPS edge. It reports
 *  whether an edge arrived, how many edges the PCNT saw (to catch double/missing
 *  pulses), the residual phase error, and the coherent anchor/drift triple the
 *  seqlock UTC-map is built from. It NEVER decides quality — @c converged is a
 *  convergence HINT the supervisor consumes for the NMEA_FIX→DISCIPLINED gate.
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct {
    bool     present;           /**< A usable (single, in-tolerance) edge this tick.*/
    int32_t  edge_count_delta;  /**< PCNT edges since last tick (accept iff == 1).  */
    int64_t  phase_err_ns;      /**< Residual |predicted − true| at the edge, ns.   */
    bool     converged;         /**< L3 lock hint: drift settled + S corroborated.  */

    /* Coherent forward-map triple (published atomically by the seqlock). */
    int64_t  last_edge_now_us;  /**< adsbin_now_us() of the last good edge.         */
    int64_t  last_edge_utc_ns;  /**< True UTC (ns) pinned to that edge.             */
    int32_t  drift_ppb;         /**< Estimated fractional frequency error, ppb.     */
} gps_pps_signals_t;

#ifdef __cplusplus
}
#endif
