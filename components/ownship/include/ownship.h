/**
 * @file    ownship.h
 * @brief   ADSBin "ownship" reference-position service (public contract).
 *
 * @details
 *   The ownship component holds the receiver's *own* geographic reference
 *   position. For the MVP this is an OPTIONAL, manually configured lat/lon
 *   loaded from NVS (see the `config` component). Two downstream consumers
 *   read it:
 *
 *     - `modes_decode`  : when a valid reference exists it may use *local* CPR
 *                         decoding (single-message, faster) instead of, or in
 *                         addition to, *global* even/odd CPR pairing.
 *     - `traffic`       : early range/altitude culling of distant targets to
 *                         save CPU before expensive merges (see plan §5.2).
 *
 *   If NO valid reference is set the box still works fully: decode falls back
 *   to global CPR (absolute positions, no ownship needed — plan §5.4) and the
 *   range filter is simply disabled.
 *
 *   Later phases (post-MVP) replace the manual reference with a live source
 *   (on-board GPS or NMEA-in from a panel). The API is source-agnostic on
 *   purpose: producers call ownship_update(); consumers call ownship_get().
 *
 * @par Core affinity (plan §2)
 *   Lives on CORE 1 (config / housekeeping side). HOWEVER its getter is read
 *   from BOTH the Core-1 decode/traffic path and potentially the Core-0 DSP
 *   path, so all accessors are internally synchronized and safe to call from
 *   any task / core. Getters are non-blocking and never allocate.
 *
 * @note  This header is the single source of truth for ::ownship_ref_t, which
 *        is a CROSS-CUTTING type also consumed by `modes_decode` and
 *        `traffic`. They include this header; they do not redefine it.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  Types OWNED & exposed by this component
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Provenance of the current ownship reference fix.
 *
 * Lets consumers (and the debug sink) reason about how trustworthy / fresh the
 * reference is. Manual references never go "stale"; live sources can.
 */
typedef enum {
    OWNSHIP_SOURCE_NONE    = 0,  /**< No valid reference; global CPR only.       */
    OWNSHIP_SOURCE_MANUAL  = 1,  /**< Operator-entered lat/lon from NVS config.  */
    OWNSHIP_SOURCE_GPS     = 2,  /**< (Future) on-board GPS module fix.          */
    OWNSHIP_SOURCE_NMEA    = 3,  /**< (Future) NMEA-in from a panel / GPS feed.  */
} ownship_source_t;

/**
 * @brief The ownship reference position (CROSS-CUTTING shared type).
 *
 * @details
 *   Geodetic WGS-84. `valid == false` means "no usable reference" and the
 *   lat/lon fields are undefined — consumers MUST check `valid` first.
 *   `altitude_m` is geometric (MSL) altitude in metres; it is OPTIONAL and
 *   may be NAN/ignored for a manual lat/lon-only entry (range filtering only
 *   needs horizontal distance for MVP).
 *
 * @warning Treated as a value snapshot. ownship_get() copies a coherent
 *          instance; never hold a pointer into the component's internal state.
 */
typedef struct {
    bool             valid;        /**< True if lat/lon are usable.              */
    double           lat_deg;      /**< Reference latitude,  WGS-84 degrees.     */
    double           lon_deg;      /**< Reference longitude, WGS-84 degrees.     */
    float            altitude_m;   /**< Geometric MSL altitude, metres (or NAN). */
    ownship_source_t source;       /**< How this reference was obtained.         */
    int64_t          updated_us;   /**< esp_timer_get_time() at last update.     */
} ownship_ref_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialize the ownship service and seed it from persisted config.
 *
 * @details
 *   Creates internal synchronization, then loads the manual reference position
 *   (if any) from the `config` component and installs it as the initial fix.
 *   Idempotent: a second call returns ESP_OK without re-seeding.
 *
 * @return ESP_OK on success, or an esp_err_t from the underlying primitives.
 */
esp_err_t ownship_init(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Producer API  (manual config, future GPS/NMEA)
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Install/replace the current reference from a manual lat/lon.
 *
 * Sets source = ::OWNSHIP_SOURCE_MANUAL, marks the fix valid, stamps the time,
 * and (when @p persist is true) writes it back through `config` to NVS.
 *
 * @param lat_deg  WGS-84 latitude  [-90, 90].
 * @param lon_deg  WGS-84 longitude [-180, 180].
 * @param persist  If true, also persist via config_set_* / config_commit().
 * @return ESP_OK; ESP_ERR_INVALID_ARG if lat/lon are out of range.
 */
esp_err_t ownship_set_manual(double lat_deg, double lon_deg, bool persist);

/**
 * @brief Push a fresh fix from a live source (future GPS / NMEA producers).
 *
 * Source-agnostic update path. Copies @p ref into the component (re-stamping
 * `updated_us` if zero). Used post-MVP by GPS/NMEA tasks; never persisted.
 *
 * @param ref  Non-NULL reference snapshot to install.
 * @return ESP_OK; ESP_ERR_INVALID_ARG on NULL or invalid coordinates.
 */
esp_err_t ownship_update(const ownship_ref_t *ref);

/**
 * @brief Invalidate the current reference (revert to global-CPR-only mode).
 *
 * Sets source = ::OWNSHIP_SOURCE_NONE and valid = false. When @p persist is
 * true the stored manual position is also cleared from NVS.
 *
 * @param persist  If true, clear the persisted manual reference too.
 * @return ESP_OK.
 */
esp_err_t ownship_clear(bool persist);

/**
 * @brief Conditionally invalidate the reference ONLY if it came from @p only_if.
 *
 * @details
 *   A race-free "clear my own live fix" primitive for a live producer (e.g. the
 *   GPS clock supervisor) that must retract its reference when its signal drops —
 *   WITHOUT ever stomping a reference some OTHER source installed in the meantime.
 *
 *   The compare ("is the current source @p only_if ?") and the clear happen inside
 *   ONE critical section, so this is immune to the read-then-clear race that a
 *   caller-side `if (ownship_get().source == X) ownship_clear()` would suffer:
 *   between the get and the clear, a manual operator could install a MANUAL fix
 *   that the naive sequence would then wrongly wipe. It also sidesteps the fact
 *   that ownship_update(valid=false) normalises @c source to NONE — which would
 *   make a subsequent source check fail to recognise its own prior push.
 *
 *   Never persists (live-source contract). If the current source is NOT @p only_if
 *   (or the reference is already invalid) this is a no-op and still returns ESP_OK,
 *   so callers may invoke it idempotently every cycle.
 *
 * @param only_if  Clear the reference only when its current source equals this.
 * @return ESP_OK always (a non-matching source is a successful no-op).
 */
esp_err_t ownship_clear_if_source(ownship_source_t only_if);

/* ───────────────────────────────────────────────────────────────────────────
 *  Consumer API  (modes_decode, traffic)  — non-blocking, any core
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Copy a coherent snapshot of the current reference position.
 *
 * @param out_ref  Non-NULL destination; populated even when invalid (so the
 *                 caller can read `valid`/`source`). Always safe to call.
 * @return ESP_OK; ESP_ERR_INVALID_ARG if @p out_ref is NULL.
 */
esp_err_t ownship_get(ownship_ref_t *out_ref);

/**
 * @brief Fast predicate: is a usable reference currently available?
 *
 * Convenience for hot paths that only need to choose local-vs-global CPR or
 * enable/disable range filtering without copying the whole struct.
 *
 * @return true if a valid reference is installed.
 */
bool ownship_has_fix(void);

/**
 * @brief Great-circle distance from the reference to a target, in metres.
 *
 * Shared geometry helper so `traffic` (range culling) and the debug sink do
 * not each reinvent haversine. Returns a negative value if no valid reference
 * exists (caller treats <0 as "unknown / do not filter").
 *
 * @param tgt_lat_deg  Target latitude,  WGS-84 degrees.
 * @param tgt_lon_deg  Target longitude, WGS-84 degrees.
 * @return Distance in metres, or a negative sentinel if no reference.
 */
double ownship_distance_m(double tgt_lat_deg, double tgt_lon_deg);

#ifdef __cplusplus
}
#endif
