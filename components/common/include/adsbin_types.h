/**
 * @file    adsbin_types.h
 * @brief   ADSBin shared cross-cutting types — THE frozen inter-component ABI.
 *
 * @details
 *   This header is the single source of truth for every type that travels
 *   between tasks or across the two cores (IMPLEMENTATION_PLAN.md S2). It is the
 *   contract that lets the eight firmware components be implemented in parallel:
 *   each one #includes this file and NEVER redefines a type that lives here.
 *
 *   Hard rules for everything in this file (do not violate during Stage 2):
 *     1. Plain-old-data only. No pointers-to-owned-state that outlive a call,
 *        no locks, no allocation. These structs are memcpy'd through queues and
 *        touched from ISRs/Core-0 hot paths.
 *     2. Fixed-width <stdint.h> types throughout, so C and C++ TUs agree on
 *        layout (main may be compiled as C++).
 *     3. ONE time base: ::adsbin_now_us() (microseconds, int64). Never mix in
 *        xTaskGetTickCount() for any *_us field.
 *     4. ICAO is the universal aircraft key: 24-bit, stored in a uint32 with the
 *        top 8 bits zero.
 *
 *   Types OWNED ELSEWHERE (do not duplicate here):
 *     - ::ownship_ref_t        -> components/ownship/include/ownship.h
 *     - ::adsbin_config_t etc. -> components/config/include/adsbin_config.h
 *     - status_* types         -> components/status/include/status.h
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  RF / sampling constants (S0, S4.1, S5.1)
 * ─────────────────────────────────────────────────────────────────────────── */
#define ADSB_SAMPLE_RATE_HZ   2400000u      /**< 2.4 Msps, 8-bit I/Q.           */
#define ADSB_CENTER_FREQ_HZ   1090000000u   /**< 1090ES centre frequency.       */

/* ───────────────────────────────────────────────────────────────────────────
 *  Mode-S frame geometry. A downlink frame is 56 bits (short) or 112 bits
 *  (long) => 7 or 14 bytes.
 * ─────────────────────────────────────────────────────────────────────────── */
#define MODES_SHORT_BITS   56
#define MODES_LONG_BITS    112
#define MODES_SHORT_BYTES  7
#define MODES_LONG_BYTES   14
#define ADSB_CALLSIGN_LEN  9    /**< 8-char flight id + NUL terminator.         */

/* ───────────────────────────────────────────────────────────────────────────
 *  Core pinning — the single source of truth for S2 affinity. Plain ints so a
 *  Core-0 component can reference its own core without depending on `main`.
 * ─────────────────────────────────────────────────────────────────────────── */
#define ADSBIN_CORE_DSP     0   /**< PRO_CPU: usb_rtlsdr RX + demod1090 (hard RT). */
#define ADSBIN_CORE_DECODE  1   /**< APP_CPU: modes_decode, traffic, sinks, etc.   */

/* ───────────────────────────────────────────────────────────────────────────
 *  Single time base. Every *_us field in this file is stamped with this and
 *  nothing else, so aging / CPR-pairing / correlation all share one clock.
 * ─────────────────────────────────────────────────────────────────────────── */
/** @brief Monotonic microsecond timestamp (wraps esp_timer_get_time()). */
static inline int64_t adsbin_now_us(void) { return esp_timer_get_time(); }

/* ═══════════════════════════════════════════════════════════════════════════
 *  STAGE 1: usb_rtlsdr (Core 0)  ->  demod1090 (Core 0)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief One block of raw IQ samples, carried by reference through the ring.
 *
 * @details
 *   Produced by `usb_rtlsdr` from RTL2832U bulk-IN transfers, consumed by
 *   `demod1090`. The @c samples buffer is BORROWED: it is owned by usb_rtlsdr's
 *   ring allocation and is valid only until the consumer returns the ring item.
 *   demod1090 MUST finish reading a block before releasing it and MUST NOT store
 *   the pointer (contract invariant - prevents use-after-recycle).
 *
 *   Samples are interleaved 8-bit unsigned I,Q,I,Q... in RTL2832U native format
 *   (offset binary, ~127.4 == 0). One IQ pair is two bytes.
 */
typedef struct {
    const uint8_t *samples;       /**< Interleaved 8-bit unsigned I/Q (borrowed). */
    uint32_t       n_bytes;       /**< Length of @c samples in bytes (2 * pairs). */
    uint32_t       seq;           /**< Monotonic block counter; gaps => overflow. */
    int64_t        t_capture_us;  /**< adsbin_now_us() at the block's first sample.*/
} iq_block_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  STAGE 2: demod1090 (Core 0)  ->  modes_decode (Core 1)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief A raw candidate Mode-S frame: preamble-gated and bit-sliced, CRC not
 *        yet checked. Passed BY VALUE through a queue (small + POD => safe
 *        cross-core with no refcounting).
 *
 * @details
 *   demod1090 fills @c data MSB-first; short (56-bit) frames use bytes [0..6].
 *   @c df is precomputed (data[0] >> 3) so the decoder can drop non-ADS-B frames
 *   without re-reading bits. CRC validation and DF17/18 parsing are
 *   modes_decode's job, not demod1090's.
 */
typedef struct {
    uint8_t  data[MODES_LONG_BYTES]; /**< Raw bytes, MSB-first.                 */
    uint8_t  len_bytes;              /**< 7 (short) or 14 (long).               */
    uint8_t  df;                     /**< Downlink format = data[0] >> 3.       */
    uint8_t  preamble_score;         /**< Preamble correlation quality 0..255.  */
    uint16_t signal_level;           /**< Relative burst magnitude (proxy RSSI).*/
    int64_t  rx_time_us;             /**< Capture time, carried from iq_block.   */
} modes_frame_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  STAGE 3: modes_decode (Core 1)  ->  traffic (Core 1)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief ADS-B emitter category (TC 1-4 "CA" field). Produced by modes_decode,
 *        mapped to GDL90 codes by the sinks.
 */
typedef enum {
    ADSB_CAT_NO_INFO        = 0,
    ADSB_CAT_LIGHT          = 1,   /**< < 15 500 lb.                            */
    ADSB_CAT_SMALL          = 2,   /**< 15 500 - 75 000 lb.                     */
    ADSB_CAT_LARGE          = 3,
    ADSB_CAT_HIGH_VORTEX    = 4,
    ADSB_CAT_HEAVY          = 5,
    ADSB_CAT_HIGH_PERF      = 6,
    ADSB_CAT_ROTORCRAFT     = 7,
    ADSB_CAT_GLIDER         = 8,
    ADSB_CAT_LIGHTER_AIR    = 9,
    ADSB_CAT_PARACHUTE      = 10,
    ADSB_CAT_ULTRALIGHT     = 11,
    ADSB_CAT_UAV            = 12,
    ADSB_CAT_SPACE          = 13,
    ADSB_CAT_SURFACE_EMERG  = 14,
    ADSB_CAT_SURFACE_SVC    = 15,
} adsb_emitter_category_t;

/**
 * @brief One decoded ADS-B observation for one aircraft at one instant.
 *
 * @details
 *   Produced by `modes_decode`, consumed by `traffic`. A single DF17/18 frame
 *   only carries a subset of fields, so each optional group has a @c has_*
 *   guard - read a field only when its guard is true.
 *
 *   POSITION IS RESOLVED HERE. The reconciled design (IMPLEMENTATION_PLAN.md
 *   S4.3 + the parallel-build plan's canonical fork choice) puts CPR even/odd
 *   pairing and local decode INSIDE modes_decode, which owns the per-ICAO
 *   pairing cache. Therefore when @c has_position is set, @c lat_deg / @c lon_deg
 *   are an ABSOLUTE WGS-84 fix - traffic never sees raw CPR.
 */
typedef struct {
    uint32_t icao;            /**< 24-bit ICAO address (top byte zero).         */
    int64_t  rx_time_us;      /**< adsbin_now_us() of the source frame.         */
    uint8_t  downlink_format; /**< 17 (ADS-B) or 18 (TIS-B), for sink tagging.  */
    uint8_t  type_code;       /**< ADS-B type code (ME[0] >> 3).                */
    int16_t  signal_level;    /**< Relative burst level; -1 if unknown.         */

    bool     has_callsign;    /**< true => @c callsign valid.                   */
    char     callsign[ADSB_CALLSIGN_LEN];

    bool     has_category;    /**< true => @c emitter_category valid.           */
    adsb_emitter_category_t emitter_category;

    bool     has_position;    /**< true => @c lat_deg / @c lon_deg are absolute. */
    double   lat_deg;         /**< WGS-84 latitude, degrees, +N.                */
    double   lon_deg;         /**< WGS-84 longitude, degrees, +E.               */
    bool     on_ground;       /**< Surface vs airborne position.                */

    bool     has_altitude;    /**< true => @c altitude_ft valid.                */
    int32_t  altitude_ft;     /**< Barometric pressure altitude, feet.          */
    bool     altitude_is_geometric; /**< true => GNSS/geometric, not baro.      */

    bool     has_velocity;    /**< true => @c ground_speed_kt + @c track_deg.   */
    uint16_t ground_speed_kt; /**< Ground speed, knots.                         */
    uint16_t track_deg;       /**< True ground track, 0..359 degrees.           */
    bool     has_vertical_rate; /**< true => @c vertical_rate_fpm valid.        */
    int16_t  vertical_rate_fpm; /**< Vertical rate, ft/min, + = climb.          */
} adsb_msg_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  STAGE 4: traffic (Core 1)  ->  sinks (Core 1)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief The merged, CPR-resolved per-aircraft record. OWNED & stored by
 *        `traffic`; READ (via a snapshot) by the sinks.
 *
 * @details
 *   This is the canonical "what do we currently know about this aircraft" view
 *   that a sink renders or encodes. Optional groups carry @c has_* guards; the
 *   NIC/NACp integrity fields exist because the GDL90 Traffic Report needs them.
 *   Relative geometry (@c range_nm / @c bearing_deg) is filled only when an
 *   ownship reference is set AND the target has a valid position.
 */
typedef struct {
    uint32_t icao;            /**< 24-bit ICAO address (table key).             */

    bool     has_callsign;
    char     callsign[ADSB_CALLSIGN_LEN];
    bool     has_category;
    adsb_emitter_category_t emitter_category;

    bool     position_valid;  /**< lat/lon currently a valid resolved fix.      */
    double   lat_deg;         /**< WGS-84 degrees, +N.                          */
    double   lon_deg;         /**< WGS-84 degrees, +E.                          */
    int64_t  position_us;     /**< adsbin_now_us() of the last position fix.    */
    bool     on_ground;

    bool     has_altitude;
    int32_t  altitude_ft;     /**< Barometric pressure altitude, feet.          */
    bool     altitude_is_geometric;

    bool     has_velocity;
    uint16_t ground_speed_kt;
    uint16_t track_deg;       /**< True ground track, 0..359.                   */
    bool     has_vertical_rate;
    int16_t  vertical_rate_fpm;

    uint8_t  nic;             /**< Navigation Integrity Category 0..11 (GDL90). */
    uint8_t  nacp;            /**< Nav Accuracy Category - Position 0..11.       */
    int16_t  signal_level;    /**< Last burst level; -1 if unknown.             */

    bool     has_relative;    /**< true => @c range_nm / @c bearing_deg valid.  */
    float    range_nm;        /**< Great-circle distance to ownship, NM.        */
    float    bearing_deg;     /**< True bearing ownship -> target, degrees.     */

    uint16_t msg_count;       /**< Messages merged into this target (saturates).*/
    int64_t  first_seen_us;   /**< adsbin_now_us() first heard.                 */
    int64_t  last_seen_us;    /**< adsbin_now_us() last heard (drives aging).   */
} traffic_target_t;

/**
 * @brief A consistent point-in-time copy of the live targets, handed to sinks
 *        so they iterate without holding the traffic table's lock.
 *
 * @details
 *   The @c targets array is owned by the traffic component for the lifetime of
 *   the snapshot; sinks treat it as read-only and must not retain it past the
 *   publish cycle.
 */
typedef struct {
    const traffic_target_t *targets;  /**< Contiguous live targets.             */
    size_t                  count;    /**< Number of valid entries.             */
    int64_t                 taken_us; /**< adsbin_now_us() when captured.       */
} traffic_snapshot_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Small shared helpers (header-inline; no .c needed)
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Extract the 24-bit ICAO address (masks off the high byte). */
static inline uint32_t adsb_icao_get(const adsb_msg_t *m)
{
    return m->icao & 0x00FFFFFFu;
}

#ifdef __cplusplus
}
#endif
