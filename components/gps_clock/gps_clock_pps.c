/**
 * @file    gps_clock_pps.c
 * @brief   Layer 3 — 1PPS hardware capture + PI clock discipline (signal producer).
 *
 * @details
 *   Disciplines the firmware's free-running clock (adsbin_now_us(), an esp_timer/
 *   SYSTIMER counter) against the u-blox MAX-M10S 1PPS TIMEPULSE edge. The edge is
 *   timestamped ENTIRELY IN HARDWARE — there is no ISR on any core:
 *
 *     PPS GPIO ──(rising edge)──► GPIO ETM event ──► ETM channel ──► GPTimer
 *                                                       CAPTURE task (latches count)
 *
 *   A dedicated GPTimer runs at 80 MHz (12.5 ns/tick) so capture quantization is
 *   far below the ~10 ns u-blox sawtooth. Because the GPTimer capture has NO
 *   completion event, the supervisor's ~1 Hz tick READS the latched count
 *   (gptimer_get_captured_count); "count changed" => a new edge, "unchanged" => a
 *   missed pulse. A PCNT unit on the SAME pin counts edges between ticks so a
 *   double/extra pulse (which would silently overwrite the single capture register)
 *   is detectable: we accept an interval only when EXACTLY ONE edge occurred.
 *
 *   == What it estimates ==
 *     - OFFSET: each accepted edge pins true_utc_ns (= the RMC integer second × 1e9)
 *       to the captured adsbin_now_us() of that edge — an exact phase pin.
 *     - DRIFT (ppb): an integral term tracks the crystal's fractional frequency
 *       error so the forward UTC map stays accurate between edges and through
 *       holdover. The integrator has full anti-windup (frozen while unlocked or
 *       while the residual is large) so a provisional first second cannot rail it.
 *
 *   == Rigor guards (all from the verified design) ==
 *     single-edge PCNT gate · interval tolerance (acquire vs locked) · re-anchor on
 *     every reject so dt never desyncs · K_REJECT rejects ⇒ holdover · integrator
 *     anti-windup + output clamp · do-not-feed before RMC 'A' + GNSS-locked, re-seed
 *     on the first locked edge · second-label corroboration against ≥2 RMC seconds.
 *
 *   Like the NMEA parser this is a PURE SIGNAL PRODUCER: it fills
 *   ::gps_pps_signals_t and NEVER decides clock quality — the supervisor owns the
 *   ladder. If PPS is unwired or any peripheral fails to allocate, the module stays
 *   inert (present=false every tick) and the ladder simply tops out at NMEA_FIX.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "gps_clock_signals.h"

#include <string.h>
#include <stdlib.h>

#include "driver/gptimer.h"
#include "driver/gptimer_etm.h"
#include "driver/gpio_etm.h"
#include "driver/pulse_cnt.h"
#include "esp_etm.h"
#include "esp_log.h"

#include "adsbin_types.h"   // adsbin_now_us()

static const char *TAG = "gps_pps";

/* ───────────────────────────────────────────────────────────────────────────
 *  Capture + discipline tuning
 * ─────────────────────────────────────────────────────────────────────────── */

#define PPS_TIMER_RES_HZ      80000000ULL   /**< 80 MHz => 12.5 ns/tick.          */

/* Interval acceptance window around 1.000000 s, in capture ticks. Wider while
 * acquiring (the clock is still coarse), tight once locked. */
#define PPS_TICKS_PER_SEC     (PPS_TIMER_RES_HZ)            /* 80,000,000 ticks/s   */
#define PPS_TOL_ACQUIRE_TICKS (PPS_TICKS_PER_SEC / 500)    /* ±2 ms while acquiring */
#define PPS_TOL_LOCKED_TICKS  (PPS_TICKS_PER_SEC / 5000)   /* ±200 µs once locked  */

#define PPS_LOCK_EDGES        8     /**< Clean single edges before "converged".    */
#define PPS_RELOCK_EDGES      5     /**< Clean edges to re-converge from holdover.  */
#define PPS_K_REJECT          3     /**< Consecutive rejects ⇒ force holdover.      */
#define PPS_SECOND_CORROB     2     /**< RMC seconds that must agree before lock.   */

/* PI integrator. tau ~ 200 s => Ki = 1/tau per second. We work in ppb. */
#define PPS_KI_NUM            1
#define PPS_KI_TAU_S          200
#define PPS_DRIFT_CLAMP_PPB   100000   /**< ±100 ppm hard output clamp.            */
#define PPS_WINDUP_FREEZE_NS  5000     /**< Freeze integration while |err|>5 µs.   */

/* ───────────────────────────────────────────────────────────────────────────
 *  Module state (all touched only on the Core-1 supervisor task)
 * ─────────────────────────────────────────────────────────────────────────── */

static bool                     s_active;        /**< Hardware brought up OK.       */
static gptimer_handle_t         s_timer;         /**< 80 MHz capture timer.         */
static esp_etm_event_handle_t   s_gpio_event;    /**< PPS rising-edge ETM event.    */
static esp_etm_task_handle_t    s_cap_task;      /**< GPTimer CAPTURE task.         */
static esp_etm_channel_handle_t s_etm_chan;      /**< Wires event -> capture task.  */
static pcnt_unit_handle_t       s_pcnt;          /**< Edge counter on the same pin. */
static pcnt_channel_handle_t    s_pcnt_chan;

/* Capture/edge tracking. */
static uint64_t s_last_cap;        /**< Previous tick's captured count.            */
static bool     s_have_cap;        /**< We have at least one prior capture.        */
static int      s_last_pcnt;       /**< Previous tick's PCNT total.                */

/* Discipline estimator. */
static bool     s_locked;          /**< L3 considers itself phase-locked.          */
static bool     s_seeded;          /**< Offset has been pinned at least once.      */
static int      s_good_edges;      /**< Consecutive clean single edges.            */
static int      s_reject_run;      /**< Consecutive rejected intervals.            */
static int      s_second_agrees;   /**< RMC seconds corroborating our label.       */
static int64_t  s_anchor_now_us;   /**< adsbin_now_us() of the last good edge.     */
static int64_t  s_anchor_utc_ns;   /**< True UTC (ns) pinned to that edge.         */
static int32_t  s_drift_ppb;       /**< Estimated fractional frequency error.      */
static int64_t  s_prev_second;     /**< Integer UTC second of the previous edge.   */

/* Forward decl: reset helper is defined below but used during bring-up. */
static void pps_estimator_reset(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Hardware bring-up
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Allocate the GPTimer capture + ETM event/channel + PCNT for @p pps_gpio.
 *
 * Each step's return code is checked; on ANY failure we tear down what we built
 * and leave s_active = false, so the ladder degrades cleanly to NMEA timing.
 */
esp_err_t gps_pps_init(int pps_gpio)
{
    esp_err_t err;

    // Start from a clean estimator so a re-init (or a prior failed attempt) can't
    // carry stale lock state into a fresh bring-up.
    pps_estimator_reset();
    s_have_cap = false;
    s_last_pcnt = 0;

    // 1) An 80 MHz up-counting GPTimer dedicated to PPS capture.
    const gptimer_config_t tcfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = (uint32_t)PPS_TIMER_RES_HZ,
    };
    err = gptimer_new_timer(&tcfg, &s_timer);
    if (err != ESP_OK) { ESP_LOGW(TAG, "gptimer_new_timer: %s", esp_err_to_name(err)); goto fail; }

    // 2) The GPTimer's CAPTURE task — fired by ETM, latches the count in hardware.
    const gptimer_etm_task_config_t task_cfg = { .task_type = GPTIMER_ETM_TASK_CAPTURE };
    err = gptimer_new_etm_task(s_timer, &task_cfg, &s_cap_task);
    if (err != ESP_OK) { ESP_LOGW(TAG, "gptimer_new_etm_task: %s", esp_err_to_name(err)); goto fail; }

    // 3) A rising-edge ETM event bound to the PPS GPIO.
    gpio_etm_event_config_t ecfg = {0};
    ecfg.edges[0] = GPIO_ETM_EVENT_EDGE_POS;
    err = gpio_new_etm_event(&ecfg, &s_gpio_event);
    if (err != ESP_OK) { ESP_LOGW(TAG, "gpio_new_etm_event: %s", esp_err_to_name(err)); goto fail; }
    err = gpio_etm_event_bind_gpio(s_gpio_event, pps_gpio);
    if (err != ESP_OK) { ESP_LOGW(TAG, "gpio_etm_event_bind_gpio: %s", esp_err_to_name(err)); goto fail; }

    // 4) An ETM channel wiring the edge event to the capture task — pure silicon.
    const esp_etm_channel_config_t ccfg = {0};
    err = esp_etm_new_channel(&ccfg, &s_etm_chan);
    if (err != ESP_OK) { ESP_LOGW(TAG, "esp_etm_new_channel: %s", esp_err_to_name(err)); goto fail; }
    err = esp_etm_channel_connect(s_etm_chan, s_gpio_event, s_cap_task);
    if (err != ESP_OK) { ESP_LOGW(TAG, "esp_etm_channel_connect: %s", esp_err_to_name(err)); goto fail; }

    // 5) Enable + start the timer and the ETM channel.
    err = gptimer_enable(s_timer);
    if (err != ESP_OK) { ESP_LOGW(TAG, "gptimer_enable: %s", esp_err_to_name(err)); goto fail; }
    err = gptimer_start(s_timer);
    if (err != ESP_OK) { ESP_LOGW(TAG, "gptimer_start: %s", esp_err_to_name(err)); goto fail; }
    err = esp_etm_channel_enable(s_etm_chan);
    if (err != ESP_OK) { ESP_LOGW(TAG, "esp_etm_channel_enable: %s", esp_err_to_name(err)); goto fail; }

    // 6) PCNT on the same pin: count rising edges so a double/extra pulse between
    //    ticks is visible (the single capture register alone cannot reveal it).
    const pcnt_unit_config_t ucfg = {
        .low_limit  = -1,           // we only ever increase; a tiny range is fine
        .high_limit = 32767,        // wraps far beyond one tick's worth of edges
    };
    err = pcnt_new_unit(&ucfg, &s_pcnt);
    if (err != ESP_OK) { ESP_LOGW(TAG, "pcnt_new_unit: %s", esp_err_to_name(err)); goto fail; }
    const pcnt_chan_config_t pchan = { .edge_gpio_num = pps_gpio, .level_gpio_num = -1 };
    err = pcnt_new_channel(s_pcnt, &pchan, &s_pcnt_chan);
    if (err != ESP_OK) { ESP_LOGW(TAG, "pcnt_new_channel: %s", esp_err_to_name(err)); goto fail; }
    // Count up on the rising edge, ignore the falling edge; level input unused.
    pcnt_channel_set_edge_action(s_pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(s_pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    err = pcnt_unit_enable(s_pcnt);
    if (err != ESP_OK) { ESP_LOGW(TAG, "pcnt_unit_enable: %s", esp_err_to_name(err)); goto fail; }
    err = pcnt_unit_start(s_pcnt);
    if (err != ESP_OK) { ESP_LOGW(TAG, "pcnt_unit_start: %s", esp_err_to_name(err)); goto fail; }

    s_active = true;
    ESP_LOGI(TAG, "PPS capture engaged on GPIO%d (80 MHz, ETM, PCNT-gated)", pps_gpio);
    return ESP_OK;

fail:
    // Best-effort teardown so a partial bring-up doesn't leak peripherals.
    if (s_etm_chan)   { esp_etm_channel_disable(s_etm_chan); esp_etm_del_channel(s_etm_chan); s_etm_chan = NULL; }
    if (s_gpio_event) { esp_etm_del_event(s_gpio_event); s_gpio_event = NULL; }
    if (s_cap_task)   { esp_etm_del_task(s_cap_task); s_cap_task = NULL; }
    if (s_timer)      { gptimer_disable(s_timer); gptimer_del_timer(s_timer); s_timer = NULL; }
    if (s_pcnt_chan)  { pcnt_del_channel(s_pcnt_chan); s_pcnt_chan = NULL; }
    if (s_pcnt)       { pcnt_unit_disable(s_pcnt); pcnt_del_unit(s_pcnt); s_pcnt = NULL; }
    s_active = false;
    return err;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Estimator helpers
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Reset the discipline estimator to the cold (unseeded) state. */
static void pps_estimator_reset(void)
{
    s_locked        = false;
    s_seeded        = false;
    s_good_edges    = 0;
    s_reject_run    = 0;
    s_second_agrees = 0;
    s_drift_ppb     = 0;
    s_prev_second   = INT64_MIN;
}

/**
 * @brief Pin the phase: anchor this edge's adsbin_now_us() to a true UTC second.
 *
 * This is the exact phase correction applied on every accepted edge — it never
 * winds up because it is a direct assignment, not an integral.
 */
static void pps_pin_phase(int64_t now_us, int64_t utc_second)
{
    s_anchor_now_us = now_us;
    s_anchor_utc_ns = utc_second * 1000000000LL;
    s_seeded = true;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Per-tick processing (called by the supervisor once per loop)
 * ─────────────────────────────────────────────────────────────────────────── */

void gps_pps_tick(gps_pps_signals_t *sig, const gps_nmea_signals_t *nmea)
{
    if (sig == NULL) {
        return;
    }
    memset(sig, 0, sizeof(*sig));

    // Inert when PPS isn't wired/failed: report no edge so the ladder caps at
    // NMEA_FIX. (Cheap and the common case on a board without the PPS wire.)
    if (!s_active) {
        return;
    }

    // Read the latched capture + the edge counter for THIS interval.
    uint64_t cap = 0;
    int      pcnt_now = 0;
    if (gptimer_get_captured_count(s_timer, &cap) != ESP_OK) {
        return;
    }
    pcnt_unit_get_count(s_pcnt, &pcnt_now);

    int edge_delta = pcnt_now - s_last_pcnt;
    s_last_pcnt = pcnt_now;

    // First call just primes the references; no interval to judge yet.
    if (!s_have_cap) {
        s_last_cap = cap;
        s_have_cap = true;
        return;
    }

    // No new capture this tick => a MISSED pulse (count unchanged). Report it; the
    // supervisor treats a miss as the DISCIPLINED->HOLDOVER trigger.
    if (cap == s_last_cap || edge_delta == 0) {
        sig->present = false;
        sig->edge_count_delta = edge_delta;
        sig->last_edge_now_us = s_anchor_now_us;
        sig->last_edge_utc_ns = s_anchor_utc_ns;
        sig->drift_ppb        = s_drift_ppb;
        return;
    }

    // ── Do-not-feed-pre-fix: the M10 runs PPS off its LOCAL clock until it has
    //    GNSS time. Only ingest once RMC is 'A' (the supervisor passes the live
    //    NMEA signals). Until then, drop edges and stay unseeded. ──────────────
    bool gnss_time = (nmea != NULL) && nmea->fix_valid;
    if (!gnss_time) {
        s_last_cap = cap;
        s_good_edges = 0;
        s_locked = false;
        sig->present = false;
        return;
    }

    // The integer UTC second this edge marks the TOP of. RMC names the second; PPS
    // is the precise instant it begins. (rmc_second is seconds-since-epoch.)
    int64_t edge_second = nmea->rmc_second;
    int64_t now_us      = adsbin_now_us();

    // ── Single-edge + interval gate ─────────────────────────────────────────
    // Ticks elapsed since the last capture, and how far that is from 1.000000 s.
    uint64_t dticks = cap - s_last_cap;
    uint64_t tol    = s_locked ? PPS_TOL_LOCKED_TICKS : PPS_TOL_ACQUIRE_TICKS;
    int64_t  off    = (int64_t)dticks - (int64_t)PPS_TICKS_PER_SEC;
    bool single     = (edge_delta == 1);
    bool in_band    = (off <= (int64_t)tol && off >= -(int64_t)tol);

    if (!single || !in_band) {
        // REJECT: re-anchor the dt reference to this capture so one early/late or
        // double pulse can't desync dt for multiple seconds. Hold drift; count the
        // reject run — K_REJECT in a row forces holdover via present=false + unlock.
        s_last_cap = cap;
        s_reject_run++;
        if (s_reject_run >= PPS_K_REJECT) {
            s_locked = false;
            s_good_edges = 0;
        }
        sig->present = false;
        sig->edge_count_delta = edge_delta;
        sig->last_edge_now_us = s_anchor_now_us;
        sig->last_edge_utc_ns = s_anchor_utc_ns;
        sig->drift_ppb        = s_drift_ppb;
        return;
    }
    s_reject_run = 0;

    // ── Second-label corroboration ──────────────────────────────────────────
    // Require the RMC-named second to advance by exactly one per good edge before
    // we trust the integer-second label enough to advertise DISCIPLINED. A jump
    // (gap or glitch) resets corroboration and re-seeds the phase.
    bool sequential = (s_prev_second != INT64_MIN) && (edge_second == s_prev_second + 1);
    if (sequential) {
        if (s_second_agrees < PPS_SECOND_CORROB) {
            s_second_agrees++;
        }
    } else {
        s_second_agrees = 0;     // discontinuity — must re-corroborate
        s_locked = false;
        s_good_edges = 0;
    }
    s_prev_second = edge_second;

    // ── Estimator update ────────────────────────────────────────────────────
    if (!s_seeded || !sequential) {
        // First good edge (or after a gap): pin phase exactly, reset drift int.
        pps_pin_phase(now_us, edge_second);
        // Do NOT integrate drift during (re)acquisition — phase-pin only.
    } else {
        // Predict where UTC SHOULD be at this edge from the prior anchor + drift,
        // and form the residual. dt in ns from the anchor, scaled by (1+drift).
        int64_t dt_ns       = (now_us - s_anchor_now_us) * 1000LL;
        // dt_ns·drift_ppb ≤ ~3e11·1e5 = 3e16, safely inside int64 (≈9.2e18 max),
        // so no 128-bit intermediate is needed (and the RISC-V32 libc lacks one).
        int64_t drift_corr  = (dt_ns * (int64_t)s_drift_ppb) / 1000000000LL;
        int64_t predicted   = s_anchor_utc_ns + dt_ns + drift_corr;
        int64_t true_utc_ns = edge_second * 1000000000LL;
        int64_t err_ns      = true_utc_ns - predicted;

        // Integrator with anti-windup: only integrate when the residual is small
        // (we're genuinely tracking) AND we're past the acquisition phase. Ki=1/tau.
        if (s_locked && err_ns < PPS_WINDUP_FREEZE_NS && err_ns > -PPS_WINDUP_FREEZE_NS) {
            int64_t dd = (PPS_KI_NUM * err_ns) / PPS_KI_TAU_S;   // ppb step ≈ err/tau
            int64_t nd = (int64_t)s_drift_ppb + dd;
            if (nd >  PPS_DRIFT_CLAMP_PPB) nd =  PPS_DRIFT_CLAMP_PPB;
            if (nd < -PPS_DRIFT_CLAMP_PPB) nd = -PPS_DRIFT_CLAMP_PPB;
            s_drift_ppb = (int32_t)nd;
        }

        // Always re-pin the phase exactly to this edge (kills accumulated phase
        // error; drift carries the between-edge prediction). Report the residual.
        pps_pin_phase(now_us, edge_second);
        sig->phase_err_ns = (err_ns < 0) ? -err_ns : err_ns;
    }

    // ── Lock bookkeeping ────────────────────────────────────────────────────
    if (s_good_edges < (s_locked ? PPS_RELOCK_EDGES : PPS_LOCK_EDGES)) {
        s_good_edges++;
    }
    int need = s_locked ? PPS_RELOCK_EDGES : PPS_LOCK_EDGES;
    bool converged = (s_good_edges >= need) && (s_second_agrees >= PPS_SECOND_CORROB);
    if (converged) {
        s_locked = true;
    }

    // Advance references for the next interval.
    s_last_cap = cap;

    // ── Emit raw signals ────────────────────────────────────────────────────
    sig->present          = true;
    sig->edge_count_delta = edge_delta;
    sig->converged        = converged;
    sig->last_edge_now_us = s_anchor_now_us;
    sig->last_edge_utc_ns = s_anchor_utc_ns;
    sig->drift_ppb        = s_drift_ppb;
}
