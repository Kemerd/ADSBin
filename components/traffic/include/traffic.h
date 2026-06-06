/**
 * @file    traffic.h
 * @brief   Traffic table manager — public contract (plan S4.4).
 *
 * @details
 *   Maintains the set of currently-tracked aircraft, keyed by 24-bit ICAO. It
 *   merges decoded ::adsb_msg_t updates into per-target ::traffic_target_t
 *   records, ages out stale targets, applies range/altitude/sanity culling, and
 *   serves consistent snapshots to the output sinks.
 *
 *   Positions arrive ALREADY RESOLVED (modes_decode owns CPR), so traffic stores
 *   absolute lat/lon and never holds even/odd CPR state.
 *
 * @par Time base
 *   Microseconds (::adsbin_now_us()) everywhere, matching adsb_msg_t.rx_time_us
 *   and the *_us fields of traffic_target_t. No millisecond conversions cross
 *   this boundary.
 *
 * @par Core affinity / threading
 *   Core 1 (::ADSBIN_CORE_DECODE). traffic_ingest is the single writer (the
 *   decode task); sinks/status read via traffic_snapshot (preferred) or
 *   traffic_iterate (callback under the lock). All are serialized by a per-
 *   instance mutex; the lock is NEVER taken from Core 0 and critical sections
 *   stay short (no logging / I/O under the lock).
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "adsbin_types.h"   /* adsb_msg_t, traffic_target_t, traffic_snapshot_t */
#include "ownship.h"        /* ownship_ref_t */

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque traffic-manager instance. */
typedef struct traffic_mgr_s *traffic_handle_t;

/** @brief Configuration (traffic_init). Use traffic_config_default() first. */
typedef struct {
    uint16_t max_targets;          /**< Table capacity. Default 256; oldest
                                        last_seen evicted when full.            */
    uint32_t expiry_ms;            /**< Drop a target unheard for this long.
                                        Default 60000.                          */
    uint32_t position_stale_ms;    /**< Clear position (keep target) if the fix
                                        is older than this. Default 30000.      */
    bool     enable_range_filter;  /**< Requires a valid ownship ref. Default off.*/
    float    max_range_nm;         /**< Cull beyond this when range filter on.   */
    bool     enable_altitude_filter;
    int32_t  max_altitude_ft;      /**< Cull above this. Default 60000.          */
    bool     enable_sanity_filter; /**< Reject impossible lat/lon. Default on.   */
} traffic_config_t;

/** @brief Outcome of one ingest (reported via out_result). */
typedef enum {
    TRAFFIC_INGEST_NEW = 0,          /**< Created a new target.                  */
    TRAFFIC_INGEST_UPDATED,          /**< Merged into an existing target.        */
    TRAFFIC_INGEST_POSITION_FIX,     /**< Merge produced a newly-valid position. */
    TRAFFIC_INGEST_FILTERED_RANGE,   /**< Dropped by range filter.               */
    TRAFFIC_INGEST_FILTERED_ALT,     /**< Dropped by altitude filter.            */
    TRAFFIC_INGEST_FILTERED_SANITY,  /**< Dropped by sanity check.               */
    TRAFFIC_INGEST_FULL_EVICTED,     /**< Table full; an old target was evicted. */
} traffic_ingest_result_t;

/** @brief Cumulative diagnostic counters (traffic_get_stats). */
typedef struct {
    uint64_t total_ingested;
    uint64_t total_new;
    uint64_t total_updated;
    uint64_t total_position_fixes;
    uint64_t filtered_range;
    uint64_t filtered_altitude;
    uint64_t filtered_sanity;
    uint64_t total_expired;
    uint16_t peak_live_count;
    uint16_t current_live_count;
} traffic_stats_t;

/**
 * @brief Iterator callback for traffic_iterate. Return false to stop early.
 * @warning Invoked UNDER the table lock — must not call any traffic_* function
 *          and must not block.
 */
typedef bool (*traffic_visit_fn)(const traffic_target_t *target, void *user_ctx);

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Fill @p out_cfg with documented defaults (call before overriding). */
void traffic_config_default(traffic_config_t *out_cfg);

/** @brief Allocate + initialize an instance. @param cfg NULL => defaults. */
esp_err_t traffic_init(const traffic_config_t *cfg, traffic_handle_t *out_handle);

/** @brief Destroy an instance, free its table + mutex, invalidate the handle. */
esp_err_t traffic_deinit(traffic_handle_t handle);

/* ───────────────────────────────────────────────────────────────────────────
 *  Write path (single writer: the decode task)
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Merge one decoded message into the target keyed by msg->icao.
 * @param out_result  May be NULL; reports what happened.
 */
esp_err_t traffic_ingest(traffic_handle_t handle, const adsb_msg_t *msg,
                         traffic_ingest_result_t *out_result);

/** @brief Set the ownship reference for range cull + relative geometry.
 *  @param ref NULL or !valid => disable range filtering (global-CPR fallback). */
esp_err_t traffic_set_ownship(traffic_handle_t handle, const ownship_ref_t *ref);

/**
 * @brief Age the table: drop targets unheard past expiry, demote stale fixes.
 * @param now_us            Current time (adsbin_now_us()).
 * @param out_expired_count May be NULL; number of targets removed.
 * @note  Call periodically (e.g. a 1 Hz Core-1 timer); the component starts no
 *        timer of its own.
 */
esp_err_t traffic_age(traffic_handle_t handle, int64_t now_us, uint32_t *out_expired_count);

/* ───────────────────────────────────────────────────────────────────────────
 *  Read path (sinks / status)
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Copy one target by ICAO. @return ESP_ERR_NOT_FOUND if not live. */
esp_err_t traffic_get(traffic_handle_t handle, uint32_t icao, traffic_target_t *out_target);

/**
 * @brief Copy all live targets into @p out_array (up to @p capacity).
 * @param out_count  Number copied. Preferred read path for sinks (no lock held
 *                   while iterating the copy). Size @p out_array via traffic_count().
 */
esp_err_t traffic_snapshot(traffic_handle_t handle, traffic_target_t *out_array,
                           size_t capacity, size_t *out_count);

/** @brief Invoke @p visit once per live target under the lock. */
esp_err_t traffic_iterate(traffic_handle_t handle, traffic_visit_fn visit,
                          void *user_ctx, size_t *out_visited);

/** @brief Current number of live targets (cheap; for status LED / sizing). */
size_t traffic_count(traffic_handle_t handle);

/** @brief Number of live targets that currently hold a valid position. */
size_t traffic_count_with_position(traffic_handle_t handle);

/* ───────────────────────────────────────────────────────────────────────────
 *  Diagnostics / maintenance
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Copy cumulative counters. */
esp_err_t traffic_get_stats(traffic_handle_t handle, traffic_stats_t *out_stats);

/** @brief Remove all targets (cumulative stats preserved); for test/band reset. */
esp_err_t traffic_clear(traffic_handle_t handle);

#ifdef __cplusplus
}
#endif
