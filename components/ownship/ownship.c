/**
 * @file    ownship.c
 * @brief   ADSBin "ownship" reference-position service — implementation.
 *
 * @details
 *   Holds the receiver's own geodetic reference position behind a tiny,
 *   any-core-safe accessor surface. The position is a single value snapshot
 *   (::ownship_ref_t); every public entry point either copies it out or swaps
 *   it in wholesale, so consumers never observe a torn struct.
 *
 *   == Concurrency model ==
 *     The reference is read from BOTH cores: the Core-1 decode/traffic path and,
 *     potentially, the Core-0 DSP path. A FreeRTOS mutex would be wrong here —
 *     it can block, and a Core-0 hard-real-time reader must never block on a
 *     Core-1 writer. We instead guard the struct with a portMUX spinlock and
 *     keep every critical section down to a handful of plain field assignments
 *     (a struct copy). That makes getters wait-free in practice: the lock is
 *     held only for the few cycles it takes to memcpy a small POD, so the spin
 *     a contending core might see is bounded and microscopic — exactly the
 *     "non-blocking, never allocates" contract the header promises.
 *
 *   == Persistence ==
 *     The manual reference lives in the `config` component (NVS-backed). We seed
 *     from it at init and, when asked to persist, write straight back through
 *     config_set_ref_position() + config_commit(). Flash I/O happens only on the
 *     explicit persist paths (set_manual / clear), which the header documents as
 *     Core-1 housekeeping calls — never from the DSP hot path.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 *
 * @par Provenance
 *   Original ADSBin code. The haversine constants and great-circle formula are
 *   standard textbook geodesy; no third-party source was copied.
 */

#include "ownship.h"

#include <math.h>

#include "freertos/FreeRTOS.h"   // portMUX_TYPE, taskENTER/EXIT_CRITICAL
#include "freertos/task.h"       // task-level critical-section helpers
#include "esp_log.h"             // diagnostic logging

#include "adsbin_types.h"        // adsbin_now_us() — the ONE time base
#include "adsbin_config.h"       // persisted manual reference (seed + persist)

/* ───────────────────────────────────────────────────────────────────────────
 *  Local constants
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Log tag for this component. */
static const char *TAG = "ownship";

/**
 * @brief Mean Earth radius (WGS-84 authalic sphere), in metres.
 *
 * Haversine assumes a sphere; 6 371 008.8 m is the IUGG mean radius, which keeps
 * great-circle error well under a percent over the ranges ADS-B traffic culling
 * cares about. Plenty accurate for "is this target inside the range filter".
 */
#define OWNSHIP_EARTH_RADIUS_M   (6371008.8)

/**
 * @brief Sentinel returned by ownship_distance_m() when no reference exists.
 *
 * The header specifies "a negative value"; callers treat anything < 0 as
 * "unknown / do not filter". -1 m is unambiguous and cheap to test.
 */
#define OWNSHIP_DISTANCE_NONE    (-1.0)

/*
 * M_PI is a POSIX/BSD extension, not ISO C. newlib (ESP-IDF's libc) normally
 * exposes it, but a strict-ANSI build can hide it — define a fallback so this
 * TU never depends on the toolchain's feature-test macros being set just so.
 */
#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

/** @brief Degrees → radians. */
#define OWNSHIP_DEG2RAD          (M_PI / 180.0)

/* ───────────────────────────────────────────────────────────────────────────
 *  Component state (all access serialized by s_lock)
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Spinlock guarding ::s_ref. portMUX, not a mutex, so Core-0 readers
 *        never block (see the file-header concurrency note).
 */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief The one live reference snapshot. Initialised invalid so the box runs in
 *        global-CPR-only mode until something installs a fix.
 */
static ownship_ref_t s_ref = {
    .valid      = false,
    .lat_deg    = 0.0,
    .lon_deg    = 0.0,
    .altitude_m = NAN,
    .source     = OWNSHIP_SOURCE_NONE,
    .updated_us = 0,
};

/** @brief Guards against re-seeding on a second ownship_init() (idempotent). */
static bool s_initialized = false;

/* ───────────────────────────────────────────────────────────────────────────
 *  Internal helpers
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Validate a WGS-84 latitude/longitude pair.
 *
 * Rejects NaN/Inf as well as out-of-band magnitudes, so a corrupt GPS/NMEA
 * producer can't poison the reference. Latitude is clamped to the poles and
 * longitude to the ±180° meridian, matching the header's documented ranges.
 *
 * @param lat_deg  Candidate latitude  in degrees.
 * @param lon_deg  Candidate longitude in degrees.
 * @return true if both coordinates are finite and in range.
 */
static inline bool ownship_coords_valid(double lat_deg, double lon_deg)
{
    // isfinite() catches NaN and ±Inf in one shot — those would otherwise sail
    // through the range comparisons below (NaN compares false to everything).
    if (!isfinite(lat_deg) || !isfinite(lon_deg)) {
        return false;
    }

    // Geodetic bounds. Inclusive of the limits so exactly-±90 / ±180 is legal.
    if (lat_deg < -90.0 || lat_deg > 90.0) {
        return false;
    }
    if (lon_deg < -180.0 || lon_deg > 180.0) {
        return false;
    }

    return true;
}

/**
 * @brief Atomically install a new reference snapshot under the spinlock.
 *
 * Centralises the one critical section that mutates ::s_ref so every producer
 * (manual / update / clear) writes through the exact same coherent swap. The
 * caller has already fully populated @p new_ref; we just publish it.
 *
 * @param new_ref  Fully-formed snapshot to make live (copied by value).
 */
static inline void ownship_publish_locked(const ownship_ref_t *new_ref)
{
    // Short, bounded critical section: a single small-POD assignment. No loops,
    // no calls that could block — safe to spin against from the other core.
    taskENTER_CRITICAL(&s_lock);
    s_ref = *new_ref;
    taskEXIT_CRITICAL(&s_lock);
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

esp_err_t ownship_init(void)
{
    // Idempotent guard. The flag itself is only ever touched from init, which
    // the system calls once at boot on Core 1; reading it under the lock keeps
    // it coherent with the publish path even if a second caller races in.
    taskENTER_CRITICAL(&s_lock);
    bool already = s_initialized;
    s_initialized = true;
    taskEXIT_CRITICAL(&s_lock);

    // Second (or later) call: nothing to re-seed, just succeed.
    if (already) {
        return ESP_OK;
    }

    // Pull the persisted manual reference from `config`. config_init() is the
    // system's responsibility to have run first; if no manual position is
    // stored we simply stay in the invalid (global-CPR-only) state.
    double lat = 0.0;
    double lon = 0.0;
    bool   have = config_get_ref_position(&lat, &lon);

    // Defensive: even a "stored" position is re-validated before we trust it, in
    // case NVS held a stale/corrupt value from an older firmware revision.
    if (have && ownship_coords_valid(lat, lon)) {
        // Build the seed snapshot. Manual lat/lon carries no altitude, so MSL
        // stays NAN — range culling only needs the horizontal fix for MVP.
        ownship_ref_t seed = {
            .valid      = true,
            .lat_deg    = lat,
            .lon_deg    = lon,
            .altitude_m = NAN,
            .source     = OWNSHIP_SOURCE_MANUAL,
            .updated_us = adsbin_now_us(),
        };
        ownship_publish_locked(&seed);

        ESP_LOGI(TAG, "Seeded manual reference from config: %.6f, %.6f",
                 lat, lon);
    } else {
        // No usable reference — explicit, so the log makes the chosen mode clear.
        ESP_LOGI(TAG, "No stored reference; running global-CPR-only");
    }

    return ESP_OK;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Producer API
 * ─────────────────────────────────────────────────────────────────────────── */

esp_err_t ownship_set_manual(double lat_deg, double lon_deg, bool persist)
{
    // Reject anything that isn't a sane WGS-84 pair before touching state.
    if (!ownship_coords_valid(lat_deg, lon_deg)) {
        ESP_LOGW(TAG, "set_manual rejected out-of-range coords: %.6f, %.6f",
                 lat_deg, lon_deg);
        return ESP_ERR_INVALID_ARG;
    }

    // Compose the new live snapshot: manual source, freshly time-stamped, no
    // altitude (lat/lon-only entry — see header note on altitude_m == NAN).
    ownship_ref_t fix = {
        .valid      = true,
        .lat_deg    = lat_deg,
        .lon_deg    = lon_deg,
        .altitude_m = NAN,
        .source     = OWNSHIP_SOURCE_MANUAL,
        .updated_us = adsbin_now_us(),
    };
    ownship_publish_locked(&fix);

    // Optional write-through to NVS. We persist FIRST through config's RAM setter
    // then commit; a commit failure is surfaced to the caller but the live fix
    // is already in effect (RAM and flash may briefly disagree, which is fine —
    // the next boot re-seeds from whatever actually made it to flash).
    if (persist) {
        esp_err_t err = config_set_ref_position(lat_deg, lon_deg, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "config_set_ref_position failed: 0x%x", err);
            return err;
        }
        err = config_commit();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "config_commit failed: 0x%x", err);
            return err;
        }
        ESP_LOGI(TAG, "Persisted manual reference: %.6f, %.6f", lat_deg, lon_deg);
    }

    return ESP_OK;
}

esp_err_t ownship_update(const ownship_ref_t *ref)
{
    // Source-agnostic ingest for live producers (future GPS / NMEA tasks).
    if (ref == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // A snapshot flagged valid must carry coordinates we can actually use; an
    // explicitly-invalid push is allowed (it just clears the fix, like clear()).
    if (ref->valid && !ownship_coords_valid(ref->lat_deg, ref->lon_deg)) {
        ESP_LOGW(TAG, "update rejected: valid fix with bad coords");
        return ESP_ERR_INVALID_ARG;
    }

    // Copy by value so we own every field and can re-stamp the clock. We never
    // persist live-source fixes (header contract) — they age and get replaced.
    ownship_ref_t fix = *ref;

    // Re-stamp updated_us if the producer left it zero, so staleness checks
    // downstream always have a real time base ( the ONE clock, adsbin_now_us ).
    if (fix.updated_us == 0) {
        fix.updated_us = adsbin_now_us();
    }

    // Normalise the source field for an invalid push so consumers reading
    // `source` see NONE rather than a stale GPS/NMEA tag on a no-fix snapshot.
    if (!fix.valid) {
        fix.source = OWNSHIP_SOURCE_NONE;
    }

    ownship_publish_locked(&fix);
    return ESP_OK;
}

esp_err_t ownship_clear(bool persist)
{
    // Revert to global-CPR-only: invalid fix, no source. Coordinates are left
    // zeroed (they are "undefined" while !valid, per the header) and altitude
    // returns to NAN so nothing downstream mistakes a leftover value for a fix.
    ownship_ref_t cleared = {
        .valid      = false,
        .lat_deg    = 0.0,
        .lon_deg    = 0.0,
        .altitude_m = NAN,
        .source     = OWNSHIP_SOURCE_NONE,
        .updated_us = adsbin_now_us(),
    };
    ownship_publish_locked(&cleared);

    // Optionally wipe the persisted manual reference so a reboot doesn't bring
    // it back. config_set_ref_position(..., valid=false) ignores lat/lon.
    if (persist) {
        esp_err_t err = config_set_ref_position(0.0, 0.0, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "config_set_ref_position(clear) failed: 0x%x", err);
            return err;
        }
        err = config_commit();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "config_commit(clear) failed: 0x%x", err);
            return err;
        }
        ESP_LOGI(TAG, "Cleared persisted manual reference");
    }

    return ESP_OK;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Consumer API  (non-blocking, any core)
 * ─────────────────────────────────────────────────────────────────────────── */

esp_err_t ownship_get(ownship_ref_t *out_ref)
{
    if (out_ref == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Coherent copy-out under the spinlock. Even an invalid reference is copied
    // so the caller can inspect `valid` / `source` without a second call.
    taskENTER_CRITICAL(&s_lock);
    *out_ref = s_ref;
    taskEXIT_CRITICAL(&s_lock);

    return ESP_OK;
}

bool ownship_has_fix(void)
{
    // Single-field fast path for hot callers choosing local-vs-global CPR or
    // toggling the range filter. Still taken under the lock so the read can't
    // straddle a concurrent publish on a platform without atomic bool loads.
    taskENTER_CRITICAL(&s_lock);
    bool valid = s_ref.valid;
    taskEXIT_CRITICAL(&s_lock);

    return valid;
}

double ownship_distance_m(double tgt_lat_deg, double tgt_lon_deg)
{
    // Snapshot the reference once, locally, so the whole calculation runs on a
    // single coherent fix and we never hold the lock across the trig math.
    taskENTER_CRITICAL(&s_lock);
    bool   have    = s_ref.valid;
    double ref_lat = s_ref.lat_deg;
    double ref_lon = s_ref.lon_deg;
    taskEXIT_CRITICAL(&s_lock);

    // No reference, or a nonsensical target → "unknown", the negative sentinel.
    // (A bad target coordinate is treated as unknown rather than filtered out,
    //  so culling never silently drops a target due to one corrupt position.)
    if (!have || !ownship_coords_valid(tgt_lat_deg, tgt_lon_deg)) {
        return OWNSHIP_DISTANCE_NONE;
    }

    // ── Haversine great-circle distance ──────────────────────────────────────
    //   a = sin²(Δφ/2) + cos φ₁ · cos φ₂ · sin²(Δλ/2)
    //   d = 2R · atan2(√a, √(1−a))
    // atan2 form is numerically stable for both tiny and antipodal separations,
    // unlike the asin variant which loses precision near d → πR.
    double lat1 = ref_lat * OWNSHIP_DEG2RAD;
    double lat2 = tgt_lat_deg * OWNSHIP_DEG2RAD;
    double dlat = (tgt_lat_deg - ref_lat) * OWNSHIP_DEG2RAD;
    double dlon = (tgt_lon_deg - ref_lon) * OWNSHIP_DEG2RAD;

    // Half-angle sines reused below; computing them once keeps the hot path lean.
    double sin_dlat_2 = sin(dlat * 0.5);
    double sin_dlon_2 = sin(dlon * 0.5);

    double a = (sin_dlat_2 * sin_dlat_2)
             + cos(lat1) * cos(lat2) * (sin_dlon_2 * sin_dlon_2);

    // Clamp `a` into [0,1] before the sqrt: rounding can nudge it a hair past 1
    // for near-antipodal points, which would make sqrt(1-a) NaN otherwise.
    if (a < 0.0) {
        a = 0.0;
    } else if (a > 1.0) {
        a = 1.0;
    }

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return OWNSHIP_EARTH_RADIUS_M * c;
}
