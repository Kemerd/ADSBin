/**
 * @file    gps_clock.h
 * @brief   ADSBin GPS clock-source ladder (u-blox MAX-M10S) — public contract.
 *
 * @details
 *   This component turns an OPTIONAL u-blox MAX-M10S GPS module into two things:
 *
 *     1. A live ownship POSITION, pushed into the existing `ownship` service via
 *        ownship_update(source = ::OWNSHIP_SOURCE_GPS). Consumers (modes_decode
 *        local CPR, traffic range cull, the GDL90 Ownship Report) need no changes
 *        — they already read ownship fresh each cycle.
 *
 *     2. A disciplined notion of TIME, exposed through the single accessor
 *        ::gps_clock_get(). The module's serial NMEA gives coarse UTC; its 1PPS
 *        timepulse, hardware-captured and PI-filtered, sharpens it to ~tens of ns.
 *
 *   == The auto-degrading ladder ==
 *     A single supervisor task owns one state machine that walks five quality
 *     rungs and automatically demotes on signal loss / promotes (with hysteresis)
 *     on signal return — NO operator action required:
 *
 *       DISCIPLINED (PPS PI-locked UTC, ~tens of ns)
 *         ↓ PPS lost / glitch-storm
 *       HOLDOVER    (PPS gone, UTC extrapolated from cached drift)
 *         ↓ holdover decays past its uncertainty ceiling
 *       NMEA_FIX    (UTC from RMC/GGA, ~tens of ms)        ← ownship goes VALID here
 *         ↓ no fix for a few seconds
 *       FREE_RUNNING(module present, no usable fix — bare monotonic clock)
 *         ↓ no serial bytes for T_BYTES_LOST  (hard unplug)
 *       NONE        (module absent / not wired — feature fully inert)
 *
 *     "If it's not installed, we do not send it": a valid GPS ownship is published
 *     ONLY at rung ≥ ::CLOCK_QUALITY_NMEA_FIX. At NONE/FREE_RUNNING the supervisor
 *     idempotently clears any GPS-sourced ownship (never stomping a MANUAL ref), so
 *     the GDL90 Ownship Report and heartbeat GPS-valid bit stay silent exactly as
 *     they do on a board with no GPS at all.
 *
 *   == Numeric trust order ==
 *     The enum is numbered so a higher value means MORE trustworthy time. A naive
 *     `q >= CLOCK_QUALITY_NMEA_FIX` is therefore a correct "do we have UTC?" test;
 *     ::clock_quality_rank() is retained as a defensive shim for the same purpose.
 *
 * @par Core affinity
 *   Lives entirely on CORE 1 (::ADSBIN_CORE_DECODE). The PPS edge timestamp is
 *   latched in silicon (GPIO→ETM→GPTimer) with ZERO interrupt on ANY core, so the
 *   hard-real-time Core-0 DSP path (usb_rtlsdr RX + demod1090) is never perturbed.
 *   ::gps_clock_get() and the UTC-map inlines are wait-free and safe from any core.
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
 *  Clock quality — the five rungs of the auto-degrading ladder.
 *
 *  NUMBERED so that numeric order == trust order (NONE lowest, DISCIPLINED
 *  highest). HOLDOVER sits BELOW DISCIPLINED but ABOVE NMEA_FIX: extrapolated
 *  PPS time is still better than serial-anchored UTC, until its growing
 *  uncertainty says otherwise (the supervisor decays it down to NMEA_FIX then).
 * ─────────────────────────────────────────────────────────────────────────── */
typedef enum {
    CLOCK_QUALITY_NONE        = 0,  /**< Module absent / not wired. Inert; nothing published. */
    CLOCK_QUALITY_FREE_RUNNING = 1, /**< Present, no fix yet. Time is bare monotonic, NOT UTC. */
    CLOCK_QUALITY_NMEA_FIX    = 2,  /**< UTC from RMC/GGA (±tens of ms). Ownship is valid.     */
    CLOCK_QUALITY_HOLDOVER    = 3,  /**< PPS lost; UTC extrapolated from cached drift.         */
    CLOCK_QUALITY_DISCIPLINED = 4,  /**< PPS PI-locked UTC (±tens of ns). Best.                */
} clock_quality_t;

/**
 * @brief A coherent snapshot of the GPS clock state (POD; copied out by value).
 *
 * @details
 *   Returned by ::gps_clock_get(). @c utc_estimate_us is only true UTC when
 *   @c quality ≥ ::CLOCK_QUALITY_NMEA_FIX; at NONE/FREE_RUNNING it carries the
 *   bare monotonic clock and @c uncertainty_ns is UINT32_MAX to flag "not UTC".
 *   Gate on @c uncertainty_ns (not just the enum) when you care about precision.
 */
typedef struct {
    clock_quality_t quality;          /**< Current ladder rung.                          */
    int64_t         utc_estimate_us;  /**< Best UTC estimate, µs since the Unix epoch
                                       *   (only UTC when quality ≥ NMEA_FIX).            */
    uint32_t        uncertainty_ns;   /**< 1σ time uncertainty, ns (UINT32_MAX = none).  */
    bool            has_ownship_fix;  /**< true => a valid GPS position is live.         */
} gps_clock_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Configuration (built once by `main` from Kconfig, mirroring status_config_t).
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Wiring + tuning for the GPS clock, assembled by main from Kconfig.
 *
 * A negative @c uart_rx_gpio means "GPS not wired" — ::gps_clock_init() then sets
 * up the inert NONE state and installs no UART. A negative @c uart_tx_gpio skips
 * the UBX boot-config burst; a negative @c pps_gpio disables the PPS layer.
 */
typedef struct {
    int      uart_num;     /**< P4 UART controller (must differ from the console). */
    int      uart_rx_gpio; /**< P4 RX ← module TX (NMEA). < 0 ⇒ GPS disabled.      */
    int      uart_tx_gpio; /**< P4 TX → module RX (UBX). < 0 ⇒ no config wire.     */
    int      pps_gpio;     /**< P4 ← module PPS. < 0 ⇒ no PPS discipline.          */
    uint32_t baud;         /**< Initial NMEA baud (MAX-M10S default 9600).         */
} gps_clock_cfg_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialise the GPS clock service (does NOT start the task).
 *
 * @details
 *   Publishes the initial ::CLOCK_QUALITY_NONE snapshot. If @p cfg->uart_rx_gpio
 *   is negative, the feature is marked permanently absent and ::gps_clock_start()
 *   becomes a no-op — the firmware then behaves exactly as a board with no GPS.
 *   Must be called AFTER ownship_init() (it pushes/clears ownship). Idempotent.
 *
 * @param cfg  Non-NULL wiring/tuning config (copied internally).
 * @return ESP_OK; ESP_ERR_INVALID_ARG on NULL @p cfg.
 */
esp_err_t gps_clock_init(const gps_clock_cfg_t *cfg);

/**
 * @brief Start the supervisor task (UART RX + NMEA parse + ladder + PPS).
 *
 * @details
 *   No-op (returns ESP_OK) when GPS is disabled (uart_rx_gpio < 0). Otherwise
 *   installs the GPS UART (guarded against the console UART), brings up the PPS
 *   capture if wired, and spawns ONE Core-1 task at ::ADSBIN_PRIO_STATUS. Failures
 *   are surfaced to the caller, which treats GPS as best-effort (log and continue).
 *
 * @return ESP_OK on success or when GPS is disabled; an esp_err_t on bring-up
 *         failure (UART install, task create).
 */
esp_err_t gps_clock_start(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Consumer API — wait-free, any core
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Copy a coherent snapshot of the current GPS clock state.
 *
 * @details
 *   Wait-free seqlock read: never blocks, never allocates, safe from any task or
 *   core. After a bounded number of retries against a concurrent writer it returns
 *   the last-known-good snapshot rather than spinning, so a preempted writer can
 *   never livelock a reader.
 *
 * @param out  Non-NULL destination snapshot.
 * @return ESP_OK; ESP_ERR_INVALID_ARG if @p out is NULL.
 */
esp_err_t gps_clock_get(gps_clock_t *out);

/**
 * @brief Fast predicate: is a valid GPS ownship fix currently live?
 * @return true iff the current rung is ≥ ::CLOCK_QUALITY_NMEA_FIX.
 */
bool gps_clock_has_fix(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Header-inline helpers
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Defensive trust-rank shim. The enum is already ordered by trust, so this
 *        is the identity on the enum value — kept so callers can express intent
 *        ("rank") without depending on the numeric encoding.
 */
static inline int clock_quality_rank(clock_quality_t q)
{
    return (int)q;
}

#ifdef __cplusplus
}
#endif
