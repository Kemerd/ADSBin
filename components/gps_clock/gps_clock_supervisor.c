/**
 * @file    gps_clock_supervisor.c
 * @brief   The SOLE owner of the GPS clock-quality ladder + its publication seam.
 *
 * @details
 *   One Core-1 task, one state machine, one writer. This file:
 *     - installs and owns the GPS UART (guarded against the console UART),
 *     - drives the ~5 Hz supervisor loop that pumps the NMEA parser (Layer 2) and
 *       the PPS PI filter (Layer 3) for their RAW SIGNALS,
 *     - runs the full auto-degrading transition table over those signals,
 *     - publishes a coherent ::gps_clock_t through a wait-free seqlock, and
 *     - performs the ownship side effects (push at NMEA_FIX+, clear below).
 *
 *   Layers 2 and 3 are pure producers (see gps_clock_signals.h). They never decide
 *   quality; this file is the only place a rung is chosen, which makes the ladder
 *   flap-free by construction.
 *
 *   == Publication: a seqlock, not a mutex ==
 *     ::gps_clock_get() may be called from any task or core and must never block.
 *     A FreeRTOS mutex could block a reader behind the writer; a portMUX would
 *     serialise readers against each other. We instead use a classic seqlock: the
 *     sole writer brackets its store between an odd→even sequence bump with release
 *     ordering; readers snapshot under acquire ordering and retry on a torn or
 *     in-progress write, falling back to the last-good copy after a bounded spin so
 *     a preempted writer can never livelock them.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "gps_clock.h"
#include "gps_clock_signals.h"

#include <string.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "adsbin_types.h"   // adsbin_now_us(), ADSBIN_CORE_DECODE
#include "ownship.h"        // ownship_update / ownship_clear_if_source / OWNSHIP_SOURCE_GPS
#include "gps_ubx.h"        // gps_ubx_configure() — optional boot config over TX

/* The task priority lives in main's wiring header; mirror the value here without
 * taking a build dependency on `main`. The GPS supervisor is the LOWEST Core-1
 * housekeeping task — it MUST sit below the interactive console/inject reader
 * (ADSBIN_PRIO_INJECT) so a UART-RX-bound GPS task can never starve operator I/O.
 * Keep this in sync with ADSBIN_PRIO_STATUS in main/include/adsbin_app.h (=1). */
#ifndef ADSBIN_PRIO_STATUS
#define ADSBIN_PRIO_STATUS   1
#endif

static const char *TAG = "gps_clock";

/* ───────────────────────────────────────────────────────────────────────────
 *  Internal hooks implemented by the Layer-2 (NMEA) and Layer-3 (PPS) files.
 *  Declared here (not in a public header) so the producers stay component-private.
 *  Both are no-op-safe: gps_pps_* simply report "no edge" when PPS is unwired.
 * ─────────────────────────────────────────────────────────────────────────── */

/* Layer 2 — NMEA parser (gps_clock_nmea.c). */
void gps_nmea_reset(gps_nmea_signals_t *sig);
void gps_nmea_feed(gps_nmea_signals_t *sig, const uint8_t *bytes, size_t n);

/* Layer 3 — PPS PI filter (gps_clock_pps.c). pps_gpio < 0 ⇒ stays inert. */
esp_err_t gps_pps_init(int pps_gpio);
void      gps_pps_tick(gps_pps_signals_t *sig, const gps_nmea_signals_t *nmea);

/* ───────────────────────────────────────────────────────────────────────────
 *  Ladder tuning constants. Asymmetric (promote N ≠ demote M) so every rung has a
 *  dead-band and no signal riding a threshold can flap. Times are in supervisor
 *  ticks unless suffixed _US/_S; the loop runs at GPS_TICK_HZ.
 * ─────────────────────────────────────────────────────────────────────────── */
#define GPS_TICK_HZ                5      /**< Supervisor loop rate (UART drain + ladder). */
#define GPS_TICK_PERIOD_MS         (1000 / GPS_TICK_HZ)

#define GPS_PRESENCE_MIN_BYTES     10     /**< Bytes that prove a module is attached.      */
#define GPS_PRESENCE_WINDOW_TICKS  (5 * GPS_TICK_HZ)   /**< 5 s cold-start presence window. */
#define GPS_BYTES_LOST_TICKS       (10 * GPS_TICK_HZ)  /**< Zero bytes this long ⇒ unplug.  */

#define GPS_FIX_GOOD_TICKS         3      /**< Consecutive good fixes ⇒ promote NMEA_FIX.  */
#define GPS_FIX_MISS_TICKS         5      /**< Consecutive missed fixes ⇒ demote.          */

/* DISCIPLINED/HOLDOVER (Layer 3). The PPS layer reports present/converged; the
 * ladder decides the rung. Holdover decays once its uncertainty exceeds a ceiling. */
#define GPS_PPS_MISS_TICKS         2      /**< Missed edges ⇒ DISCIPLINED→HOLDOVER.        */
#define GPS_HOLDOVER_CEIL_NS       50000u /**< Holdover uncertainty ceiling ⇒ →NMEA_FIX.   */
#define GPS_HOLDOVER_GROWTH_NS     2000u  /**< Uncertainty added per holdover tick (model).*/
#define GPS_DISCIPLINED_UNC_NS     100u   /**< Reported 1σ when freshly disciplined.       */

#define GPS_UART_RX_BUF            512    /**< UART RX ring (NMEA bursts ≤ ~80 B/sentence). */
#define GPS_UART_TX_BUF            256    /**< UART TX ring. MUST be non-zero: with a 0-size
                                          *   TX buffer uart_write_bytes() writes straight to
                                          *   the FIFO and BLOCKS until it drains — and during
                                          *   the boot UBX-CFG burst that block was observed to
                                          *   never return (hanging gps_clock_start()). A real
                                          *   TX ring makes the write copy-and-return, so the
                                          *   config burst can never wedge bring-up. 256 B holds
                                          *   the whole VALSET frame with room to spare. */
#define GPS_DRAIN_CHUNK            128    /**< Bytes pulled per tick from the UART driver.  */

/* ───────────────────────────────────────────────────────────────────────────
 *  Component state (all mutated ONLY on the supervisor task, except the seqlock
 *  publication which is the documented cross-core hand-off).
 * ─────────────────────────────────────────────────────────────────────────── */

static gps_clock_cfg_t  s_cfg;            /**< Wiring/tuning copied at init.        */
static bool             s_inited;         /**< gps_clock_init() ran.                */
static bool             s_disabled;       /**< RX GPIO < 0 ⇒ permanently NONE.      */
static bool             s_uart_installed; /**< We own the GPS UART driver.          */
static TaskHandle_t     s_task;           /**< The one supervisor task.             */

/* Live ladder + signal state (supervisor-task-private). */
static clock_quality_t  s_quality = CLOCK_QUALITY_NONE;
static gps_nmea_signals_t s_nmea;         /**< Layer-2 raw signals.                 */
static gps_pps_signals_t  s_pps;          /**< Layer-3 raw signals.                 */

/* Debounce counters (reset on the opposite-direction event so credit never banks). */
static int      s_presence_ticks;   /**< Ticks elapsed in the cold presence window. */
static int      s_no_byte_ticks;    /**< Consecutive ticks with zero UART bytes.    */
static uint32_t s_last_byte_count;  /**< Byte count at the previous tick.           */
static int      s_fix_good;         /**< Consecutive good-fix ticks.                */
static int      s_fix_miss;         /**< Consecutive missed-fix ticks.              */
static int64_t  s_last_fix_seq;     /**< Last NMEA fix_seq we acted on.             */
static int      s_pps_miss;         /**< Consecutive PPS-missed ticks (→holdover).  */
static uint32_t s_holdover_unc_ns;  /**< Current holdover uncertainty estimate, ns. */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Seqlock publication. One writer (the supervisor), many wait-free readers.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define GPS_SEQLOCK_RETRY_MAX  8   /**< Reader spins before using last-good copy.  */

static _Atomic uint32_t s_seq;       /**< Even = stable, odd = write in progress.  */
static gps_clock_t      s_published;  /**< Guarded by s_seq.                        */
static gps_clock_t      s_last_good;  /**< Reader fallback after RETRY_MAX spins.   */

/**
 * @brief Publish a coherent clock snapshot (SOLE writer; supervisor task only).
 *
 * Bracket the store between an odd→even sequence bump. The release fence pairs
 * with the reader's acquire fence so the struct write is never reordered outside
 * the odd window; the matching even bump signals "stable" to readers.
 */
static void gps_clock_publish(const gps_clock_t *snap)
{
    uint32_t seq = atomic_load_explicit(&s_seq, memory_order_relaxed);
    atomic_store_explicit(&s_seq, seq + 1, memory_order_relaxed);   // -> odd: writing
    atomic_thread_fence(memory_order_release);

    s_published = *snap;

    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&s_seq, seq + 2, memory_order_relaxed);   // -> even: stable

    // Keep a reader-fallback copy current (written only here, on the writer side).
    s_last_good = *snap;
}

esp_err_t gps_clock_get(gps_clock_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Bounded-retry seqlock read: copy, then confirm the sequence didn't move and
    // wasn't odd (mid-write). After RETRY_MAX failed attempts a writer is clearly
    // being starved/preempted, so fall back to the last-good copy rather than spin.
    for (int attempt = 0; attempt < GPS_SEQLOCK_RETRY_MAX; ++attempt) {
        uint32_t before = atomic_load_explicit(&s_seq, memory_order_acquire);
        if (before & 1u) {
            continue;   // a write is in progress; retry
        }
        gps_clock_t tmp = s_published;
        atomic_thread_fence(memory_order_acquire);
        uint32_t after = atomic_load_explicit(&s_seq, memory_order_relaxed);
        if (before == after) {
            *out = tmp;     // clean read
            return ESP_OK;
        }
    }

    *out = s_last_good;     // writer starving us — last coherent snapshot
    return ESP_OK;
}

bool gps_clock_has_fix(void)
{
    gps_clock_t snap;
    gps_clock_get(&snap);
    return snap.quality >= CLOCK_QUALITY_NMEA_FIX;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Ownship side effects. Position flows through the EXISTING ownship producer
 *  API; we are just one more source. Only NMEA_FIX-or-better publishes a valid
 *  fix; below that we idempotently retract OUR fix without touching a MANUAL one.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Push the current NMEA position into ownship as an OWNSHIP_SOURCE_GPS fix.
 */
static void gps_push_ownship(void)
{
    ownship_ref_t ref = {
        .valid      = true,
        .lat_deg    = s_nmea.lat_deg,
        .lon_deg    = s_nmea.lon_deg,
        .altitude_m = s_nmea.altitude_m,   // NAN-safe; ownship tolerates NAN
        .source     = OWNSHIP_SOURCE_GPS,
        .updated_us = adsbin_now_us(),
    };
    // ownship_update validates coords and never persists a live source. A reject
    // (e.g. a momentarily bad coord) just leaves the previous fix in place.
    (void)ownship_update(&ref);
}

/**
 * @brief Retract our GPS fix iff we own it — never disturbs a MANUAL operator ref.
 */
static void gps_clear_ownship(void)
{
    (void)ownship_clear_if_source(OWNSHIP_SOURCE_GPS);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Quality transitions. One helper logs + records each rung change so the console
 *  shows the ladder walking. Ownship/seqlock side effects are sequenced by the
 *  caller: on a DEMOTE we publish the lower quality FIRST then clear ownship; on a
 *  PROMOTE we push ownship FIRST then publish — so a reader never sees a valid
 *  ownship paired with a sub-NMEA_FIX quality (or vice-versa).
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char *quality_str(clock_quality_t q)
{
    switch (q) {
        case CLOCK_QUALITY_NONE:        return "NONE";
        case CLOCK_QUALITY_FREE_RUNNING:return "FREE_RUNNING";
        case CLOCK_QUALITY_NMEA_FIX:    return "NMEA_FIX";
        case CLOCK_QUALITY_HOLDOVER:    return "HOLDOVER";
        case CLOCK_QUALITY_DISCIPLINED: return "DISCIPLINED";
        default:                        return "?";
    }
}

/**
 * @brief Assemble + publish the ::gps_clock_t for the current rung and signals.
 *
 * Centralises the per-rung field policy (what utc_estimate/uncertainty/has_fix
 * mean at each quality) so the transition code only has to set s_quality.
 */
static void gps_clock_republish(void)
{
    gps_clock_t snap = {0};
    snap.quality = s_quality;

    switch (s_quality) {
        case CLOCK_QUALITY_NONE:
        case CLOCK_QUALITY_FREE_RUNNING:
            // No UTC knowledge: hand back the monotonic clock, flagged not-UTC.
            snap.utc_estimate_us = adsbin_now_us();
            snap.uncertainty_ns  = UINT32_MAX;
            snap.has_ownship_fix = false;
            break;

        case CLOCK_QUALITY_NMEA_FIX:
            // Coarse UTC anchored at the last sentence's LF (tens of ms transport).
            snap.utc_estimate_us = s_nmea.utc_us;
            snap.uncertainty_ns  = 30000000u;   // ~30 ms placeholder until L3 refines
            snap.has_ownship_fix = true;
            break;

        case CLOCK_QUALITY_DISCIPLINED:
        case CLOCK_QUALITY_HOLDOVER: {
            // PPS-grade UTC. Project the disciplined map forward to "now": from the
            // last good edge's (now_us, utc_ns) anchor plus the estimated drift.
            // This is the precise estimate; it degrades gracefully through holdover
            // because the anchor/drift are frozen at the last lock and only the
            // reported uncertainty grows.
            int64_t now_us = adsbin_now_us();
            int64_t dt_ns  = (now_us - s_pps.last_edge_now_us) * 1000LL;
            // 64-bit-safe: dt_ns·drift_ppb stays well under int64 max (see pps tick).
            int64_t drift  = (dt_ns * (int64_t)s_pps.drift_ppb) / 1000000000LL;
            int64_t utc_ns = s_pps.last_edge_utc_ns + dt_ns + drift;
            snap.utc_estimate_us = (s_pps.last_edge_utc_ns != 0)
                                       ? (utc_ns / 1000LL)   // disciplined map
                                       : s_nmea.utc_us;      // fall back to NMEA UTC
            snap.uncertainty_ns  = (s_quality == CLOCK_QUALITY_DISCIPLINED)
                                       ? GPS_DISCIPLINED_UNC_NS
                                       : s_holdover_unc_ns;
            snap.has_ownship_fix = true;
            break;
        }
    }

    gps_clock_publish(&snap);
}

/**
 * @brief Move to a new rung, sequencing ownship vs publish to stay consistent.
 */
static void gps_set_quality(clock_quality_t to)
{
    if (to == s_quality) {
        return;
    }

    const clock_quality_t from = s_quality;
    const bool was_fix  = (from >= CLOCK_QUALITY_NMEA_FIX);
    const bool now_fix  = (to   >= CLOCK_QUALITY_NMEA_FIX);

    ESP_LOGI(TAG, "%s -> %s", quality_str(from), quality_str(to));

    if (now_fix && !was_fix) {
        // PROMOTE into a fix-bearing rung: install ownship FIRST, then publish, so
        // no reader sees quality>=NMEA_FIX with a stale/absent ownship.
        gps_push_ownship();
        s_quality = to;
        gps_clock_republish();
    } else if (!now_fix && was_fix) {
        // DEMOTE out of a fix-bearing rung: publish the lower quality FIRST, then
        // retract ownship, so no reader sees a valid ownship with quality<NMEA_FIX.
        s_quality = to;
        gps_clock_republish();
        gps_clear_ownship();
    } else {
        // Within-fix or within-no-fix move: order is immaterial.
        s_quality = to;
        gps_clock_republish();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  The ladder. Evaluated once per supervisor tick AFTER the producers have run.
 *  Task #3 implements NONE/FREE_RUNNING + the hard-unplug short-circuit and the
 *  NMEA_FIX promote/demote skeleton; Task #7 layers DISCIPLINED/HOLDOVER on top.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief One step of the auto-degrading state machine.
 *
 * @param byte_delta  UART bytes consumed since the previous tick (presence signal).
 */
static void gps_ladder_step(uint32_t byte_delta)
{
    /* ── Hard-unplug short-circuit (checked in EVERY state) ──────────────────
     * Zero bytes for T_BYTES_LOST means the module is physically gone. This must
     * bypass every holdover/dwell timer and crash straight to NONE, or we'd keep
     * advertising trusted time for ~10 s after a cable pull. */
    if (byte_delta == 0) {
        if (s_no_byte_ticks < GPS_BYTES_LOST_TICKS) {
            s_no_byte_ticks++;
        }
    } else {
        s_no_byte_ticks = 0;
    }
    const bool hard_lost = (s_no_byte_ticks >= GPS_BYTES_LOST_TICKS);

    if (hard_lost) {
        // Reset the cold-start machinery so a re-attach re-arms the presence window.
        s_presence_ticks = 0;
        s_fix_good = 0;
        s_fix_miss = 0;
        gps_set_quality(CLOCK_QUALITY_NONE);
        return;
    }

    /* ── NONE → FREE_RUNNING : presence ─────────────────────────────────────
     * Confirm a module is really there (>= MIN_BYTES) before leaving NONE, so a
     * single noise byte on a floating pin cannot fake a GPS into existence. */
    if (s_quality == CLOCK_QUALITY_NONE) {
        if (s_presence_ticks < GPS_PRESENCE_WINDOW_TICKS) {
            s_presence_ticks++;
        }
        if (s_nmea.byte_count >= GPS_PRESENCE_MIN_BYTES) {
            gps_set_quality(CLOCK_QUALITY_FREE_RUNNING);
        }
        return;
    }

    /* ── Fix promote/demote debounce ────────────────────────────────────────
     * A NEW accepted same-second fix bumps fix_seq; we count consecutive ticks
     * that did/didn't see one. Asymmetric 3-up / 5-down dead-band. */
    const bool fresh_fix = (s_nmea.fix_valid && s_nmea.fix_seq != s_last_fix_seq);
    if (fresh_fix) {
        s_last_fix_seq = s_nmea.fix_seq;
        s_fix_good = (s_fix_good < GPS_FIX_GOOD_TICKS) ? s_fix_good + 1 : s_fix_good;
        s_fix_miss = 0;
    } else {
        s_fix_miss = (s_fix_miss < GPS_FIX_MISS_TICKS) ? s_fix_miss + 1 : s_fix_miss;
        if (s_fix_miss >= GPS_FIX_MISS_TICKS) {
            s_fix_good = 0;
        }
    }
    const bool have_fix = (s_fix_good >= GPS_FIX_GOOD_TICKS);
    const bool lost_fix = (s_fix_miss >= GPS_FIX_MISS_TICKS);

    /* ── PPS edge tracking (drives the DISCIPLINED/HOLDOVER rungs) ───────────
     * The Layer-3 producer reports present (a usable edge this tick) and converged
     * (drift settled + second-label corroborated). We debounce a run of misses. */
    if (s_pps.present) {
        s_pps_miss = 0;
    } else if (s_pps_miss < GPS_PPS_MISS_TICKS) {
        s_pps_miss++;
    }
    const bool pps_lost = (s_pps_miss >= GPS_PPS_MISS_TICKS);

    /* ── Rung selection. The ladder above NMEA_FIX is:
     *      NMEA_FIX ──(PPS converged)──► DISCIPLINED
     *      DISCIPLINED ──(PPS lost)────► HOLDOVER ──(unc>ceiling)──► NMEA_FIX
     *      HOLDOVER ──(PPS reconverged)► DISCIPLINED
     *    Below that, the FREE_RUNNING⇄NMEA_FIX behaviour from Task #3. */
    if (s_quality >= CLOCK_QUALITY_NMEA_FIX) {
        // Sustained NMEA loss collapses the whole upper ladder to FREE_RUNNING
        // (no UTC, no ownship) regardless of PPS — PPS time is meaningless without
        // a fix to name the second.
        if (lost_fix) {
            s_pps_miss = GPS_PPS_MISS_TICKS;
            s_holdover_unc_ns = 0;
            gps_set_quality(CLOCK_QUALITY_FREE_RUNNING);
            return;
        }

        // Refresh position from the newest fix on every cycle we hold a fix.
        gps_push_ownship();

        switch (s_quality) {
            case CLOCK_QUALITY_NMEA_FIX:
                // Promote to DISCIPLINED once the PPS layer reports convergence.
                if (s_pps.converged) {
                    s_holdover_unc_ns = GPS_DISCIPLINED_UNC_NS;
                    gps_set_quality(CLOCK_QUALITY_DISCIPLINED);
                } else {
                    gps_clock_republish();
                }
                break;

            case CLOCK_QUALITY_DISCIPLINED:
                // Demote to HOLDOVER on a debounced run of missed edges; seed the
                // holdover uncertainty from the last disciplined value.
                if (pps_lost) {
                    s_holdover_unc_ns = GPS_DISCIPLINED_UNC_NS;
                    gps_set_quality(CLOCK_QUALITY_HOLDOVER);
                } else {
                    gps_clock_republish();   // still locked — refresh UTC estimate
                }
                break;

            case CLOCK_QUALITY_HOLDOVER:
                if (s_pps.converged) {
                    // PPS returned and re-locked → back to DISCIPLINED.
                    s_holdover_unc_ns = GPS_DISCIPLINED_UNC_NS;
                    gps_set_quality(CLOCK_QUALITY_DISCIPLINED);
                } else {
                    // Grow the uncertainty each tick; once it exceeds the ceiling the
                    // extrapolated time is no better than serial UTC → drop to NMEA_FIX.
                    if (s_holdover_unc_ns < UINT32_MAX - GPS_HOLDOVER_GROWTH_NS) {
                        s_holdover_unc_ns += GPS_HOLDOVER_GROWTH_NS;
                    }
                    if (s_holdover_unc_ns >= GPS_HOLDOVER_CEIL_NS) {
                        gps_set_quality(CLOCK_QUALITY_NMEA_FIX);
                    } else {
                        gps_clock_republish();
                    }
                }
                break;

            default:
                gps_clock_republish();
                break;
        }
    } else {
        // FREE_RUNNING: promote the moment we have a debounced run of good fixes.
        if (have_fix) {
            s_pps_miss = 0;
            gps_set_quality(CLOCK_QUALITY_NMEA_FIX);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  The supervisor task: drain the UART, run the producers, step the ladder.
 * ═══════════════════════════════════════════════════════════════════════════ */

static void gps_supervisor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "gps_clock supervisor up on core %d", xPortGetCoreID());

    uint8_t buf[GPS_DRAIN_CHUNK];

    for (;;) {
        /* 1. Drain whatever NMEA bytes the UART driver buffered since last tick and
         *    feed them to the Layer-2 parser. A short timeout keeps the loop at the
         *    tick rate even when the GPS is silent (it blocks only THIS task). */
        for (;;) {
            int n = uart_read_bytes((uart_port_t)s_cfg.uart_num, buf, sizeof(buf),
                                    pdMS_TO_TICKS(GPS_TICK_PERIOD_MS));
            if (n <= 0) {
                break;   // nothing (more) waiting; tick budget spent on the wait
            }
            gps_nmea_feed(&s_nmea, buf, (size_t)n);
            if (n < (int)sizeof(buf)) {
                break;   // partial read => drained for now
            }
        }

        /* 2. Run the PPS layer (no-op when unwired) for this tick's edge signals. */
        gps_pps_tick(&s_pps, &s_nmea);

        /* 3. Compute the byte delta (presence) and step the ladder once. */
        uint32_t now_bytes  = s_nmea.byte_count;
        uint32_t byte_delta = now_bytes - s_last_byte_count;   // wraps cleanly (uint)
        s_last_byte_count   = now_bytes;

        gps_ladder_step(byte_delta);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Lifecycle
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t gps_clock_init(const gps_clock_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_inited) {
        return ESP_OK;   // idempotent
    }

    s_cfg = *cfg;
    gps_nmea_reset(&s_nmea);
    memset(&s_pps, 0, sizeof(s_pps));

    // "Not installed => not sent": a negative RX pin pins the ladder at NONE and
    // makes start() a no-op, so the firmware is identical to a board with no GPS.
    s_disabled = (s_cfg.uart_rx_gpio < 0);

    s_quality = CLOCK_QUALITY_NONE;
    gps_clock_republish();   // publish the initial inert snapshot

    if (s_disabled) {
        ESP_LOGI(TAG, "GPS disabled (RX GPIO < 0) - feature inert");
    } else {
        ESP_LOGI(TAG, "GPS configured: UART%d rx=%d tx=%d pps=%d @ %u baud",
                 s_cfg.uart_num, s_cfg.uart_rx_gpio, s_cfg.uart_tx_gpio,
                 s_cfg.pps_gpio, (unsigned)s_cfg.baud);
    }

    s_inited = true;
    return ESP_OK;
}

esp_err_t gps_clock_start(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_disabled) {
        return ESP_OK;   // nothing to start; we stay at NONE forever
    }

    // Guard against stealing the console UART (it owns UART0 for GDL90/debug + the
    // +INJECT reader). Refuse rather than silently corrupt the bench link.
    if (s_cfg.uart_num == CONFIG_ESP_CONSOLE_UART_NUM) {
        ESP_LOGE(TAG, "GPS UART%d collides with the console UART - refusing",
                 s_cfg.uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    if (uart_is_driver_installed((uart_port_t)s_cfg.uart_num)) {
        ESP_LOGE(TAG, "UART%d already has a driver installed - refusing", s_cfg.uart_num);
        return ESP_ERR_INVALID_STATE;
    }

    // Install the GPS UART: 8-N-1 at the configured baud, RX-only data path plus
    // the optional TX wire for the UBX config burst. No HW flow control.
    const uart_config_t ucfg = {
        .baud_rate  = (int)s_cfg.baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config((uart_port_t)s_cfg.uart_num, &ucfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }
    // TX pin only matters when the config wire is present (>=0); UART_PIN_NO_CHANGE
    // leaves the TX/CTS/RTS lines unrouted otherwise.
    const int tx_pin = (s_cfg.uart_tx_gpio >= 0) ? s_cfg.uart_tx_gpio : UART_PIN_NO_CHANGE;
    err = uart_set_pin((uart_port_t)s_cfg.uart_num, tx_pin, s_cfg.uart_rx_gpio,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_driver_install((uart_port_t)s_cfg.uart_num, GPS_UART_RX_BUF,
                              GPS_UART_TX_BUF, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    s_uart_installed = true;

    // Optional boot config over the TX wire: put the M10 into 1 Hz + timepulse +
    // GGA/RMC-only NMEA. Best-effort — if the wire is absent or the write fails the
    // module keeps its factory defaults and the NMEA path still works.
    if (s_cfg.uart_tx_gpio >= 0) {
        (void)gps_ubx_configure(s_cfg.uart_num);
    }

    // Bring up the PPS capture layer if wired. Failure is non-fatal — the ladder
    // simply never reaches DISCIPLINED, falling back to NMEA timing.
    if (s_cfg.pps_gpio >= 0) {
        esp_err_t perr = gps_pps_init(s_cfg.pps_gpio);
        if (perr != ESP_OK) {
            ESP_LOGW(TAG, "gps_pps_init failed (%s) - PPS discipline disabled",
                     esp_err_to_name(perr));
        }
    }

    // Spawn the single supervisor task on Core 1, below the sinks/traffic/decode
    // band, far below the Core-0 DSP path — it can never preempt real-time work.
    BaseType_t ok = xTaskCreatePinnedToCore(gps_supervisor_task, "gps_clock",
                                            4096, NULL, ADSBIN_PRIO_STATUS,
                                            &s_task, ADSBIN_CORE_DECODE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create gps_clock task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "gps_clock started");
    return ESP_OK;
}
