/**
 * @file    traffic.c
 * @brief   ICAO-keyed traffic table manager — implementation (plan S4.4).
 *
 * @details
 *   This file owns the live "what aircraft do we currently know about" table.
 *   It is fed one ::adsb_msg_t at a time by the single decode-task writer and
 *   read back, point-in-time, by the output sinks. Everything is guarded by a
 *   single per-instance FreeRTOS mutex so the (Core-1) writer and the (Core-1)
 *   readers never tear a ::traffic_target_t.
 *
 *   Design intent / invariants honoured throughout:
 *     - Positions are ALREADY RESOLVED by modes_decode, so we store absolute
 *       WGS-84 lat/lon and never touch CPR.
 *     - ONE time base: every *_us field is microseconds from ::adsbin_now_us(),
 *       and aging compares against the @c now_us the caller passes in.
 *     - Critical sections are SHORT: no logging, no I/O, no allocation while the
 *       lock is held. (We do compute relative geometry under the lock, but that
 *       is a handful of transcendental ops, not blocking work.)
 *     - The lock is NEVER taken from Core 0 — only the decode/sink tasks on
 *       Core 1 ever enter here, so a plain (non-ISR) mutex is correct.
 *
 *   The table itself is a flat, pre-allocated array of slots. With the default
 *   256-target capacity a linear scan for "find by ICAO" / "find oldest" is a
 *   few microseconds and avoids the failure modes (rehash storms, pointer churn)
 *   of a fancier structure on a memory-constrained MCU. We keep a dense
 *   @c live_count so the cheap status queries (count / count_with_position) and
 *   sizing helpers stay O(1) where the contract implies they should be cheap.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "traffic.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

/* M_PI is a POSIX/BSD extension, not ISO C — most newlib/picolibc builds expose
 * it via math.h, but a strict-ANSI translation unit can hide it. Define a local
 * fallback so the great-circle math compiles regardless of the libc dialect. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  Tunable defaults — surfaced through traffic_config_default(). Kept here as
 *  named constants so the doc-comment in the header and the code never drift.
 * ─────────────────────────────────────────────────────────────────────────── */
#define TRAFFIC_DEFAULT_MAX_TARGETS        256u
#define TRAFFIC_DEFAULT_EXPIRY_MS          60000u
#define TRAFFIC_DEFAULT_POSITION_STALE_MS  30000u
#define TRAFFIC_DEFAULT_MAX_ALTITUDE_FT    60000

/* Microseconds-per-millisecond — the only unit conversion in the file, used to
 * promote the human-friendly *_ms config knobs into the int64 µs time base that
 * every timestamp comparison actually runs in. */
#define TRAFFIC_US_PER_MS  1000LL

/* msg_count is a uint16 in the shared ABI; clamp merges so a long-lived target
 * saturates instead of silently wrapping back to a tiny value. */
#define TRAFFIC_MSG_COUNT_MAX  UINT16_MAX

/* Earth mean radius in nautical miles, used by the great-circle helpers. One NM
 * is one minute of arc, so R = (180 * 60) / pi NM ≈ 3437.74677. */
#define TRAFFIC_EARTH_RADIUS_NM  3437.7467707849392

/* Sanity bounds for a "plausible" geographic fix. Anything outside these is a
 * decode artefact, not an aircraft. We also reject the exact null-island origin
 * (0,0) which is the classic "uninitialised CPR" giveaway. */
#define TRAFFIC_LAT_ABS_MAX  90.0
#define TRAFFIC_LON_ABS_MAX  180.0

/* ───────────────────────────────────────────────────────────────────────────
 *  One table slot. We embed the public record verbatim and add only the private
 *  bookkeeping the manager itself needs (occupancy). The dense @c live_count on
 *  the manager lets us avoid scanning to answer "how many are live".
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct {
    bool             occupied;   /**< Slot currently holds a live target.        */
    traffic_target_t target;     /**< The public, snapshot-able record.          */
} traffic_slot_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  The opaque instance. Allocated once by traffic_init(); the @c slots array is
 *  a separate allocation sized to cfg.max_targets so the control block stays a
 *  fixed, cache-friendly size regardless of capacity.
 * ─────────────────────────────────────────────────────────────────────────── */
struct traffic_mgr_s {
    traffic_config_t  cfg;          /**< Resolved configuration (post-defaults). */

    SemaphoreHandle_t lock;         /**< Guards every field below. Core-1 only.  */

    traffic_slot_t   *slots;        /**< Pre-allocated slot array [max_targets].  */
    uint16_t          capacity;     /**< == cfg.max_targets; cached for clarity.  */
    uint16_t          live_count;   /**< Number of currently-occupied slots.      */

    ownship_ref_t     ownship;      /**< Last reference installed; .valid gates it.*/

    traffic_stats_t   stats;        /**< Cumulative counters (survive clear()).   */
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  Small internal helpers — all assume the caller already holds @c m->lock
 *  unless explicitly noted. None of them log, allocate, or block.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Convert degrees to radians. Pulled out so the geometry math below reads
 *        like the textbook formula rather than a wall of `* M_PI / 180.0`.
 */
static inline double traffic_deg2rad(double deg)
{
    return deg * (M_PI / 180.0);
}

/**
 * @brief Reject geographically impossible fixes (the sanity filter).
 *
 * @details
 *   A resolved position must sit inside the WGS-84 domain and must not be NaN
 *   (which slips through naive range checks). We additionally treat the exact
 *   (0,0) origin as bogus: a real aircraft is essentially never there, whereas
 *   an un-initialised / mis-paired CPR decode lands on null island constantly.
 *
 * @return true if the lat/lon pair is plausible enough to store.
 */
static bool traffic_position_is_sane(double lat_deg, double lon_deg)
{
    // NaN never compares true, so an explicit isfinite() guard is required before
    // the magnitude tests can be trusted.
    if (!isfinite(lat_deg) || !isfinite(lon_deg)) {
        return false;
    }

    // Hard WGS-84 envelope.
    if (lat_deg < -TRAFFIC_LAT_ABS_MAX || lat_deg > TRAFFIC_LAT_ABS_MAX) {
        return false;
    }
    if (lon_deg < -TRAFFIC_LON_ABS_MAX || lon_deg > TRAFFIC_LON_ABS_MAX) {
        return false;
    }

    // Null-island rejection: exact zero/zero is the tell-tale of a bad decode.
    if (lat_deg == 0.0 && lon_deg == 0.0) {
        return false;
    }

    return true;
}

/**
 * @brief Great-circle distance ownship→target in nautical miles (haversine).
 *
 * @details
 *   Haversine is numerically stable for the short-to-medium ranges an ADS-B
 *   receiver actually sees and is cheap enough to run under the lock. We use the
 *   instance's stored ownship reference, NOT the global ownship service, because
 *   traffic owns its own reference snapshot (see traffic_set_ownship) and must
 *   never reach across to a singleton from inside its own critical section.
 */
static float traffic_range_nm(const ownship_ref_t *ref,
                              double tgt_lat_deg, double tgt_lon_deg)
{
    // Promote everything to radians once; the formula is symmetric in the two
    // points so naming is "1" = ownship, "2" = target.
    const double lat1 = traffic_deg2rad(ref->lat_deg);
    const double lat2 = traffic_deg2rad(tgt_lat_deg);
    const double dlat = lat2 - lat1;
    const double dlon = traffic_deg2rad(tgt_lon_deg - ref->lon_deg);

    // Haversine core: a = sin²(Δφ/2) + cosφ1·cosφ2·sin²(Δλ/2).
    const double sin_dlat = sin(dlat * 0.5);
    const double sin_dlon = sin(dlon * 0.5);
    const double a = (sin_dlat * sin_dlat) +
                     (cos(lat1) * cos(lat2) * sin_dlon * sin_dlon);

    // c = 2·atan2(√a, √(1−a)); clamp the radicand defensively against tiny
    // negative round-off so sqrt() never produces NaN at antipodal extremes.
    const double a_clamped = (a < 0.0) ? 0.0 : (a > 1.0 ? 1.0 : a);
    const double c = 2.0 * atan2(sqrt(a_clamped), sqrt(1.0 - a_clamped));

    return (float)(TRAFFIC_EARTH_RADIUS_NM * c);
}

/**
 * @brief Initial true bearing ownship→target, degrees in [0,360).
 *
 * @details
 *   Standard forward-azimuth of a great circle. Normalised to a compass heading
 *   so the sinks can feed it straight into a GDL90 traffic report or a heads-up
 *   bearing without further wrapping.
 */
static float traffic_bearing_deg(const ownship_ref_t *ref,
                                 double tgt_lat_deg, double tgt_lon_deg)
{
    const double lat1 = traffic_deg2rad(ref->lat_deg);
    const double lat2 = traffic_deg2rad(tgt_lat_deg);
    const double dlon = traffic_deg2rad(tgt_lon_deg - ref->lon_deg);

    // Forward azimuth components.
    const double y = sin(dlon) * cos(lat2);
    const double x = (cos(lat1) * sin(lat2)) -
                     (sin(lat1) * cos(lat2) * cos(dlon));

    // atan2 yields (-pi, pi]; shift into [0, 360) by adding a full turn and
    // taking the modulus, which also collapses the -0 edge case cleanly.
    double brg = atan2(y, x) * (180.0 / M_PI);
    brg = fmod(brg + 360.0, 360.0);

    return (float)brg;
}

/**
 * @brief (Re)compute a target's ownship-relative geometry in place.
 *
 * @details
 *   Called after any mutation that could change either endpoint — a position
 *   merge or a new ownship reference. If we have a valid reference AND the
 *   target currently holds a valid position we fill range/bearing and set the
 *   @c has_relative guard; otherwise we clear the guard so stale geometry can
 *   never leak into a snapshot.
 */
static void traffic_update_relative(const traffic_mgr_s *m, traffic_target_t *t)
{
    // Both endpoints must be known, else relative geometry is undefined.
    if (m->ownship.valid && t->position_valid) {
        t->range_nm     = traffic_range_nm(&m->ownship, t->lat_deg, t->lon_deg);
        t->bearing_deg  = traffic_bearing_deg(&m->ownship, t->lat_deg, t->lon_deg);
        t->has_relative = true;
    } else {
        // Drop the guard (and zero the values, so a stale snapshot can't show a
        // phantom range) whenever either endpoint is missing.
        t->has_relative = false;
        t->range_nm     = 0.0f;
        t->bearing_deg  = 0.0f;
    }
}

/**
 * @brief Locate the slot holding @p icao, or NULL if not live. Linear scan.
 *
 * @details
 *   O(capacity) but capacity is small (256 by default) and this runs on Core 1
 *   only. Keeping a flat scan rather than a hash side-table removes a whole
 *   class of consistency bugs around eviction and is plenty fast for the load.
 */
static traffic_slot_t *traffic_find_slot(traffic_mgr_s *m, uint32_t icao)
{
    for (uint16_t i = 0; i < m->capacity; ++i) {
        if (m->slots[i].occupied && m->slots[i].target.icao == icao) {
            return &m->slots[i];
        }
    }
    return NULL;
}

/**
 * @brief Find the first free slot, or NULL if the table is full. Linear scan.
 */
static traffic_slot_t *traffic_find_free(traffic_mgr_s *m)
{
    for (uint16_t i = 0; i < m->capacity; ++i) {
        if (!m->slots[i].occupied) {
            return &m->slots[i];
        }
    }
    return NULL;
}

/**
 * @brief Find the live slot with the OLDEST last_seen_us (eviction victim).
 *
 * @details
 *   Used only when the table is full and a brand-new ICAO arrives. Returns NULL
 *   only for an empty table, which the caller never reaches in that path.
 */
static traffic_slot_t *traffic_find_oldest(traffic_mgr_s *m)
{
    traffic_slot_t *oldest = NULL;

    for (uint16_t i = 0; i < m->capacity; ++i) {
        if (!m->slots[i].occupied) {
            continue;
        }
        // First occupied slot seeds the comparison; thereafter keep the minimum
        // last_seen_us, i.e. the target we've heard from least recently.
        if (oldest == NULL ||
            m->slots[i].target.last_seen_us < oldest->target.last_seen_us) {
            oldest = &m->slots[i];
        }
    }
    return oldest;
}

/**
 * @brief Recompute and latch peak_live_count from the current live_count.
 *
 * Kept in one place so every mutation path (ingest-new, age, clear) reports a
 * consistent high-water mark.
 */
static void traffic_track_peak(traffic_mgr_s *m)
{
    m->stats.current_live_count = m->live_count;
    if (m->live_count > m->stats.peak_live_count) {
        m->stats.peak_live_count = (uint16_t)m->live_count;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Merge logic — fold one decoded message into a target record. Assumes the
 *  caller holds the lock and that @p msg has already cleared the cull filters.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Merge @p msg into @p t, updating only the groups the message carries.
 *
 * @details
 *   A single DF17/18 frame is a partial observation — it might carry a position
 *   but no callsign, or velocity but no altitude. We therefore merge field-group
 *   by field-group under each `has_*` guard, leaving previously-learned fields
 *   intact when a frame is silent about them. last_seen_us and msg_count always
 *   advance; first_seen_us is only set by the caller on creation.
 *
 * @param[out] produced_fix  Set true iff this merge turned an unfixed/refreshed
 *                           position into a freshly-valid one (drives the
 *                           POSITION_FIX result code).
 */
static void traffic_merge_msg(traffic_mgr_s *m, traffic_target_t *t,
                              const adsb_msg_t *msg, bool *produced_fix)
{
    *produced_fix = false;

    // ── Identity / liveness: always advance, regardless of frame contents. ──
    t->last_seen_us = msg->rx_time_us;
    if (t->msg_count < TRAFFIC_MSG_COUNT_MAX) {
        t->msg_count++;
    }

    // Signal level: only overwrite when the frame actually reports one (-1 means
    // "unknown" in the ABI), so a quiet frame doesn't erase a good RSSI reading.
    if (msg->signal_level >= 0) {
        t->signal_level = msg->signal_level;
    }

    // ── Callsign (flight id). Copy as a whole, NUL-terminated, length-bounded.─
    if (msg->has_callsign) {
        memcpy(t->callsign, msg->callsign, ADSB_CALLSIGN_LEN);
        t->callsign[ADSB_CALLSIGN_LEN - 1] = '\0';   // belt-and-braces terminate
        t->has_callsign = true;
    }

    // ── Emitter category. ──
    if (msg->has_category) {
        t->emitter_category = msg->emitter_category;
        t->has_category     = true;
    }

    // ── Position. The freshest absolute fix wins; note whether THIS merge is
    //    the one that took the target from "no fix" to "has fix" for the result
    //    code, but also count a refreshed fix on an already-positioned target.─
    if (msg->has_position) {
        const bool was_valid = t->position_valid;

        t->lat_deg        = msg->lat_deg;
        t->lon_deg        = msg->lon_deg;
        t->position_us    = msg->rx_time_us;
        t->on_ground      = msg->on_ground;
        t->position_valid = true;

        // A "position fix" event for the caller means we now hold a valid fix
        // that we did not hold before this merge.
        if (!was_valid) {
            *produced_fix = true;
        }
    }

    // ── Altitude (barometric or geometric per the flag). ──
    if (msg->has_altitude) {
        t->altitude_ft           = msg->altitude_ft;
        t->altitude_is_geometric = msg->altitude_is_geometric;
        t->has_altitude          = true;
    }

    // ── Velocity (ground speed + track travel together in the ABI). ──
    if (msg->has_velocity) {
        t->ground_speed_kt = msg->ground_speed_kt;
        t->track_deg       = msg->track_deg;
        t->has_velocity    = true;
    }

    // ── Vertical rate. ──
    if (msg->has_vertical_rate) {
        t->vertical_rate_fpm = msg->vertical_rate_fpm;
        t->has_vertical_rate = true;
    }

    // Any merge can move the target or arrive after an ownship change, so refresh
    // relative geometry once at the end rather than inside the position branch
    // (cheap, and keeps "has position + has ownship ⇒ has relative" airtight).
    traffic_update_relative(m, t);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Lifecycle
 * ═══════════════════════════════════════════════════════════════════════════ */

void traffic_config_default(traffic_config_t *out_cfg)
{
    // Defensive: a NULL out pointer is a caller bug but must not crash here.
    if (out_cfg == NULL) {
        return;
    }

    // Documented defaults, mirrored from the header's field comments.
    out_cfg->max_targets             = TRAFFIC_DEFAULT_MAX_TARGETS;
    out_cfg->expiry_ms               = TRAFFIC_DEFAULT_EXPIRY_MS;
    out_cfg->position_stale_ms       = TRAFFIC_DEFAULT_POSITION_STALE_MS;
    out_cfg->enable_range_filter     = false;   // needs an ownship ref → off
    out_cfg->max_range_nm            = 0.0f;     // unused while filter is off
    out_cfg->enable_altitude_filter  = false;
    out_cfg->max_altitude_ft         = TRAFFIC_DEFAULT_MAX_ALTITUDE_FT;
    out_cfg->enable_sanity_filter    = true;     // on by default
}

esp_err_t traffic_init(const traffic_config_t *cfg, traffic_handle_t *out_handle)
{
    // The handle out-param is the only hard requirement; cfg may be NULL.
    if (out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_handle = NULL;

    // Resolve configuration: caller cfg or documented defaults.
    traffic_config_t resolved;
    if (cfg != NULL) {
        resolved = *cfg;
    } else {
        traffic_config_default(&resolved);
    }

    // A zero capacity would leave us with a table that can never hold a target;
    // clamp up to at least one so the rest of the code has a sane invariant.
    if (resolved.max_targets == 0) {
        resolved.max_targets = 1;
    }

    // ── Allocate the control block. Internal RAM (not PSRAM) because the table
    //    is touched on the decode hot path and we want predictable latency.   ─
    traffic_mgr_s *m = heap_caps_calloc(1, sizeof(*m), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (m == NULL) {
        return ESP_ERR_NO_MEM;
    }

    m->cfg      = resolved;
    m->capacity = resolved.max_targets;

    // ── Allocate the slot array (calloc → every slot starts unoccupied). ──
    m->slots = heap_caps_calloc(m->capacity, sizeof(traffic_slot_t),
                                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (m->slots == NULL) {
        heap_caps_free(m);
        return ESP_ERR_NO_MEM;
    }

    // ── The one synchronization primitive. A plain (non-recursive) mutex: every
    //    public entry point takes it exactly once and never re-enters.        ─
    m->lock = xSemaphoreCreateMutex();
    if (m->lock == NULL) {
        heap_caps_free(m->slots);
        heap_caps_free(m);
        return ESP_ERR_NO_MEM;
    }

    // ownship starts invalid (calloc already zeroed it → .valid == false), so
    // range filtering and relative geometry stay disabled until set explicitly.

    *out_handle = m;
    return ESP_OK;
}

esp_err_t traffic_deinit(traffic_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take the lock once to fence any in-flight call, then tear down. After this
    // the handle is invalid; callers must not use it again (documented contract).
    if (handle->lock != NULL) {
        xSemaphoreTake(handle->lock, portMAX_DELAY);
        xSemaphoreGive(handle->lock);
        vSemaphoreDelete(handle->lock);
        handle->lock = NULL;
    }

    // Free the slot array, then the control block.
    if (handle->slots != NULL) {
        heap_caps_free(handle->slots);
        handle->slots = NULL;
    }
    heap_caps_free(handle);

    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Write path
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t traffic_ingest(traffic_handle_t handle, const adsb_msg_t *msg,
                         traffic_ingest_result_t *out_result)
{
    if (handle == NULL || msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Mask to the canonical 24-bit key up front so every comparison uses the
    // same normalised address (the top byte must be zero per the ABI).
    const uint32_t icao = adsb_icao_get(msg);

    // ── Pre-lock cull evaluation that does NOT need table state. The sanity and
    //    altitude filters depend only on the message; the range filter needs the
    //    ownship ref which lives under the lock, so it is evaluated there.     ─
    //
    //  We still take the lock to bump the cumulative counters atomically even on
    //  a filtered message, because traffic_get_stats reads them under the lock.

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    // Count the ingest attempt regardless of outcome.
    handle->stats.total_ingested++;

    // ── SANITY filter: reject impossible positions. Only meaningful when the
    //    message actually carries a position; a position-less frame (e.g. a
    //    pure callsign or velocity report) is always allowed through.         ─
    if (handle->cfg.enable_sanity_filter && msg->has_position &&
        !traffic_position_is_sane(msg->lat_deg, msg->lon_deg)) {
        handle->stats.filtered_sanity++;
        xSemaphoreGive(handle->lock);
        if (out_result) { *out_result = TRAFFIC_INGEST_FILTERED_SANITY; }
        return ESP_OK;
    }

    // ── ALTITUDE filter: cull targets reporting above the ceiling. Again, only
    //    applies to frames that carry an altitude.                            ─
    if (handle->cfg.enable_altitude_filter && msg->has_altitude &&
        msg->altitude_ft > handle->cfg.max_altitude_ft) {
        handle->stats.filtered_altitude++;
        xSemaphoreGive(handle->lock);
        if (out_result) { *out_result = TRAFFIC_INGEST_FILTERED_ALT; }
        return ESP_OK;
    }

    // ── RANGE filter: cull targets beyond max_range_nm. Requires a valid
    //    ownship reference AND a position in this frame; with no reference the
    //    filter is a no-op (global-CPR fallback, per the header contract).    ─
    if (handle->cfg.enable_range_filter && handle->ownship.valid &&
        msg->has_position) {
        const float range = traffic_range_nm(&handle->ownship,
                                             msg->lat_deg, msg->lon_deg);
        if (range > handle->cfg.max_range_nm) {
            handle->stats.filtered_range++;
            xSemaphoreGive(handle->lock);
            if (out_result) { *out_result = TRAFFIC_INGEST_FILTERED_RANGE; }
            return ESP_OK;
        }
    }

    // ── Locate or create the target. ──
    traffic_ingest_result_t result;
    bool                    produced_fix = false;

    traffic_slot_t *slot = traffic_find_slot(handle, icao);

    if (slot != NULL) {
        // Existing target → straight merge.
        traffic_merge_msg(handle, &slot->target, msg, &produced_fix);
        result = produced_fix ? TRAFFIC_INGEST_POSITION_FIX
                              : TRAFFIC_INGEST_UPDATED;
        handle->stats.total_updated++;
    } else {
        // New target. Find a free slot; if the table is full, evict the oldest.
        bool evicted = false;
        slot = traffic_find_free(handle);

        if (slot == NULL) {
            // Full: reclaim the least-recently-heard target's slot.
            slot = traffic_find_oldest(handle);
            // traffic_find_oldest only returns NULL on an empty table; the table
            // is full here so this is guaranteed non-NULL, but guard anyway.
            if (slot == NULL) {
                xSemaphoreGive(handle->lock);
                if (out_result) { *out_result = TRAFFIC_INGEST_UPDATED; }
                return ESP_ERR_NO_MEM;
            }
            evicted = true;
            // The evicted slot is being recycled, not aged out, so it is NOT an
            // "expired" event — but live_count is about to be re-added below, so
            // decrement first to keep the running tally exact.
            handle->live_count--;
        }

        // Zero the recycled/blank slot so no stale field survives into the new
        // target, then seed the immutable identity / first-seen fields.
        memset(&slot->target, 0, sizeof(slot->target));
        slot->occupied            = true;
        slot->target.icao         = icao;
        slot->target.first_seen_us = msg->rx_time_us;
        slot->target.signal_level  = -1;   // "unknown" until a frame reports one

        handle->live_count++;

        // Merge the message contents into the fresh record.
        traffic_merge_msg(handle, &slot->target, msg, &produced_fix);

        // Account: a fresh target is always "new"; we also surface eviction and
        // a same-frame position fix where they apply.
        handle->stats.total_new++;
        if (evicted) {
            result = TRAFFIC_INGEST_FULL_EVICTED;
        } else if (produced_fix) {
            result = TRAFFIC_INGEST_POSITION_FIX;
        } else {
            result = TRAFFIC_INGEST_NEW;
        }
    }

    // A position fix is counted whenever a merge produced one, on new or
    // existing targets alike.
    if (produced_fix) {
        handle->stats.total_position_fixes++;
    }

    // Refresh the live high-water mark now that live_count is settled.
    traffic_track_peak(handle);

    xSemaphoreGive(handle->lock);

    if (out_result) { *out_result = result; }
    return ESP_OK;
}

esp_err_t traffic_set_ownship(traffic_handle_t handle, const ownship_ref_t *ref)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    // A NULL or !valid reference disables range filtering and relative geometry
    // (header: "NULL or !valid => disable"). We store an explicitly-invalid
    // reference in that case so downstream checks uniformly test .valid.
    if (ref == NULL || !ref->valid) {
        memset(&handle->ownship, 0, sizeof(handle->ownship));
        handle->ownship.valid  = false;
        handle->ownship.source = OWNSHIP_SOURCE_NONE;
    } else {
        handle->ownship = *ref;   // value snapshot; we never hold the pointer
    }

    // The reference moved, so every positioned target's relative geometry must
    // be recomputed. This is the one O(N) pass under the lock, but it only runs
    // on an explicit ownship change (rare), not on the per-message hot path.
    for (uint16_t i = 0; i < handle->capacity; ++i) {
        if (handle->slots[i].occupied) {
            traffic_update_relative(handle, &handle->slots[i].target);
        }
    }

    xSemaphoreGive(handle->lock);
    return ESP_OK;
}

esp_err_t traffic_age(traffic_handle_t handle, int64_t now_us,
                      uint32_t *out_expired_count)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Promote the millisecond knobs into the µs time base used by the records.
    const int64_t expiry_us       = (int64_t)handle->cfg.expiry_ms * TRAFFIC_US_PER_MS;
    const int64_t position_max_us = (int64_t)handle->cfg.position_stale_ms * TRAFFIC_US_PER_MS;

    uint32_t expired = 0;

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    for (uint16_t i = 0; i < handle->capacity; ++i) {
        traffic_slot_t *slot = &handle->slots[i];
        if (!slot->occupied) {
            continue;
        }

        // ── Full expiry: unheard longer than expiry_us → drop the target. ──
        const int64_t age_us = now_us - slot->target.last_seen_us;
        if (age_us >= expiry_us) {
            slot->occupied = false;
            handle->live_count--;
            handle->stats.total_expired++;
            expired++;
            continue;   // slot is dead; nothing more to demote on it
        }

        // ── Position demotion: target is still live (recent heartbeat) but its
        //    last position fix is older than the stale window → drop the fix,
        //    keep the target. Relative geometry must follow the fix down.     ─
        if (slot->target.position_valid) {
            const int64_t pos_age_us = now_us - slot->target.position_us;
            if (pos_age_us >= position_max_us) {
                slot->target.position_valid = false;
                // Without a valid position there can be no relative geometry.
                traffic_update_relative(handle, &slot->target);
            }
        }
    }

    // live_count changed; refresh the current/peak counters.
    traffic_track_peak(handle);

    xSemaphoreGive(handle->lock);

    if (out_expired_count) { *out_expired_count = expired; }
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Read path
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t traffic_get(traffic_handle_t handle, uint32_t icao,
                      traffic_target_t *out_target)
{
    if (handle == NULL || out_target == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Normalise the key so a caller passing a 32-bit value with junk in the top
    // byte still matches the stored 24-bit address.
    const uint32_t key = icao & 0x00FFFFFFu;

    esp_err_t ret = ESP_ERR_NOT_FOUND;

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    const traffic_slot_t *slot = traffic_find_slot(handle, key);
    if (slot != NULL) {
        *out_target = slot->target;   // single struct copy under the lock
        ret = ESP_OK;
    }

    xSemaphoreGive(handle->lock);
    return ret;
}

esp_err_t traffic_snapshot(traffic_handle_t handle, traffic_target_t *out_array,
                           size_t capacity, size_t *out_count)
{
    if (handle == NULL || out_array == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t copied = 0;

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    // Copy out up to @p capacity live targets. We stop early if the caller's
    // buffer is smaller than the live set (sized via traffic_count); the count
    // we report reflects what was actually copied, not the live total.
    for (uint16_t i = 0; i < handle->capacity && copied < capacity; ++i) {
        if (handle->slots[i].occupied) {
            out_array[copied] = handle->slots[i].target;
            copied++;
        }
    }

    xSemaphoreGive(handle->lock);

    if (out_count) { *out_count = copied; }
    return ESP_OK;
}

esp_err_t traffic_iterate(traffic_handle_t handle, traffic_visit_fn visit,
                          void *user_ctx, size_t *out_visited)
{
    if (handle == NULL || visit == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t visited = 0;

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    // Invoke the callback once per live target, UNDER the lock. The contract
    // forbids the callback from re-entering traffic_* or blocking, which is what
    // keeps this critical section short despite the user code running inside it.
    for (uint16_t i = 0; i < handle->capacity; ++i) {
        if (!handle->slots[i].occupied) {
            continue;
        }
        visited++;
        // A false return means "stop early" — honour it immediately.
        if (!visit(&handle->slots[i].target, user_ctx)) {
            break;
        }
    }

    xSemaphoreGive(handle->lock);

    if (out_visited) { *out_visited = visited; }
    return ESP_OK;
}

size_t traffic_count(traffic_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    // Cheap O(1) read of the maintained live tally; still taken under the lock so
    // it can't tear against a concurrent ingest/age updating live_count.
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    const size_t n = handle->live_count;
    xSemaphoreGive(handle->lock);
    return n;
}

size_t traffic_count_with_position(traffic_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    size_t n = 0;

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    // No dedicated counter for this (positions come and go with aging), so a
    // short scan is the honest answer. Capacity is small and this is a status
    // query, not a hot-path call.
    for (uint16_t i = 0; i < handle->capacity; ++i) {
        if (handle->slots[i].occupied && handle->slots[i].target.position_valid) {
            n++;
        }
    }

    xSemaphoreGive(handle->lock);
    return n;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Diagnostics / maintenance
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t traffic_get_stats(traffic_handle_t handle, traffic_stats_t *out_stats)
{
    if (handle == NULL || out_stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    *out_stats = handle->stats;   // whole-struct copy keeps the counters coherent
    xSemaphoreGive(handle->lock);

    return ESP_OK;
}

esp_err_t traffic_clear(traffic_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    // Drop every live target. Cumulative stats are intentionally preserved (the
    // header documents this as a test/band reset, not a counter reset) — but the
    // current live tally must fall to zero.
    for (uint16_t i = 0; i < handle->capacity; ++i) {
        handle->slots[i].occupied = false;
    }
    handle->live_count = 0;

    // current_live_count tracks the live tally; peak stays as the high-water mark.
    handle->stats.current_live_count = 0;

    xSemaphoreGive(handle->lock);
    return ESP_OK;
}
