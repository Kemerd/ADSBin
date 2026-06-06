/**
 * @file    sink_gdl90.c
 * @brief   GDL90 output sink (plan S4.5): Heartbeat + Traffic (+ optional Ownship).
 *
 * @details
 *   Each publish cycle this sink emits, in order:
 *     1. one GDL90 Heartbeat (msg 0x00) — the ~1 Hz "I'm alive / clock" beacon,
 *     2. an optional Ownship Report (msg 0x0A) when an ownship fix is set,
 *     3. one Traffic Report (msg 0x14) per live target (optionally capped).
 *
 *   The actual byte framing (CRC-16, 0x7E flags, 0x7D/0x20 stuffing, field
 *   packing) lives in the pure, host-testable gdl90_encoder.c. This file only
 *   maps records, drives the encoder, and pushes the framed bytes through a
 *   ::sink_transport_t (USB-CDC in the MVP; the same encoder later feeds UDP).
 *
 * @par Non-blocking
 *   Runs on the Core-1 publisher task. Every transport write is bounded and
 *   drops on back-pressure, so a stalled host can never starve the Core-0 DSP.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 *
 * GDL90 framing is CLEAN-ROOM from the public Garmin GDL90 spec + ForeFlight
 * extension; no GPL dump1090 fork was consulted.
 */

#include "sink_gdl90.h"
#include "sink_internal.h"
#include "gdl90_encoder.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "adsbin_types.h"
#include "adsbin_err.h"

/* ───────────────────────────────────────────────────────────────────────────
 *  A framed GDL90 message is at most the 28-byte Traffic body, plus a worst-case
 *  doubling from byte-stuffing, plus two flags and a 2-byte CRC. Round up.
 * ─────────────────────────────────────────────────────────────────────────── */
#define GDL90_FRAME_MAX   80u    /**< Safe upper bound for one framed message.   */

/* Seconds in a UTC day — the Heartbeat timestamp field wraps on this. */
#define SECONDS_PER_DAY   86400u

static const char *TAG = "sink_gdl90";

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private per-sink state (behind sink_vtable_t.ctx).
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    sink_transport_t  transport;            /**< Output byte stream.             */
    bool              emit_ownship_report;  /**< Emit 0x0A when ownship valid.   */
    uint8_t           max_targets_per_cycle;/**< 0 => no cap.                     */

    SemaphoreHandle_t own_lock;             /**< Guards @c ownship below.         */
    ownship_ref_t     ownship;              /**< Optional ownship for 0x0A.       */

    uint8_t           frame[GDL90_FRAME_MAX];/**< Per-message framing scratch.    */
} sink_gdl90_ctx_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Small helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Seconds since UTC midnight for the Heartbeat timestamp field.
 *
 * @details
 *   Uses the real system clock. Before any SNTP/RTC sync this is uptime-relative
 *   wall time, which is still a genuine monotonic-ish value GDL90 receivers
 *   tolerate (it is NOT fabricated). Once the clock is set it becomes true UTC.
 */
static uint32_t gdl90_seconds_since_midnight(void)
{
    // POSIX time in seconds; modulo a day gives the GDL90 field directly.
    time_t now = time(NULL);
    if (now < 0) {
        now = 0;
    }
    return (uint32_t)((uint64_t)now % SECONDS_PER_DAY);
}

/**
 * @brief Encode + frame + write one Heartbeat for this cycle.
 */
static void gdl90_emit_heartbeat(sink_gdl90_ctx_t *ctx, bool gps_valid)
{
    // Build the heartbeat fields. Message counts are left at zero for the MVP —
    // we do not relay uplink/long counts (receive-only, no traffic relay).
    gdl90_heartbeat_t hb = {
        .gps_pos_valid       = gps_valid,
        .maint_required      = false,
        .timestamp_s         = gdl90_seconds_since_midnight(),
        .msg_count_uplink    = 0,
        .msg_count_basic_long = 0,
    };

    // Frame into scratch; a negative return is an overflow / arg error.
    int n = gdl90_frame_heartbeat(ctx->frame, sizeof(ctx->frame), &hb);
    if (n > 0) {
        sink_transport_write(ctx->transport, ctx->frame, (size_t)n);
    }
}

/**
 * @brief Encode + frame + write one Traffic (0x14) or Ownship (0x0A) report.
 *
 * @param ctx     Sink state (provides framing scratch + transport).
 * @param tr      The wire fields to encode.
 * @param ownship true => Ownship Report (0x0A); false => Traffic Report (0x14).
 */
static void gdl90_emit_report(sink_gdl90_ctx_t *ctx, const gdl90_traffic_t *tr, bool ownship)
{
    int n = ownship
                ? gdl90_frame_ownship_report(ctx->frame, sizeof(ctx->frame), tr)
                : gdl90_frame_traffic_report(ctx->frame, sizeof(ctx->frame), tr);
    if (n > 0) {
        sink_transport_write(ctx->transport, ctx->frame, (size_t)n);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  vtable: publish
 * ═══════════════════════════════════════════════════════════════════════════ */

static esp_err_t sink_gdl90_publish(void *vctx, const traffic_snapshot_t *snap,
                                    const ownship_ref_t *own)
{
    sink_gdl90_ctx_t *ctx = (sink_gdl90_ctx_t *)vctx;
    if (ctx == NULL || ctx->transport == NULL || snap == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Resolve the ownship to use: prefer the per-sink ownship set via
    // sink_gdl90_set_ownship(), else fall back to the publisher-supplied one.
    ownship_ref_t use_own;
    xSemaphoreTake(ctx->own_lock, portMAX_DELAY);
    if (ctx->ownship.valid) {
        use_own = ctx->ownship;
    } else if (own != NULL) {
        use_own = *own;
    } else {
        memset(&use_own, 0, sizeof(use_own));
        use_own.valid = false;
    }
    xSemaphoreGive(ctx->own_lock);

    // 1) Heartbeat. GPS-valid bit mirrors whether we have an ownship fix.
    gdl90_emit_heartbeat(ctx, use_own.valid);

    // 2) Optional Ownship Report (0x0A) when enabled AND we have a valid fix.
    if (ctx->emit_ownship_report && use_own.valid) {
        // Map the ownship reference into the shared traffic wire layout. We reuse
        // gdl90_traffic_t directly because Ownship and Traffic share the format;
        // only the position + a couple of fields are meaningful for ownship.
        gdl90_traffic_t otr;
        memset(&otr, 0, sizeof(otr));
        otr.alert_status  = 0;
        otr.addr_type     = 0;
        otr.icao          = 0;                 /* own ICAO unknown on a receiver */
        otr.lat_deg       = use_own.lat_deg;
        otr.lon_deg       = use_own.lon_deg;
        otr.alt_press_ft  = INT32_MIN;         /* no baro alt for a manual ref   */
        otr.airborne      = false;             /* a ground reference position     */
        otr.nic           = 0;
        otr.nacp          = 0;
        otr.h_velocity_kt = 0xFFFu;            /* horizontal velocity unavailable */
        otr.v_velocity_fpm = INT16_MIN;        /* vertical rate unavailable        */
        otr.track_heading = 0;
        otr.emitter_cat   = 0;
        otr.emergency_code = 0;
        otr.callsign[0]   = '\0';
        gdl90_emit_report(ctx, &otr, /*ownship=*/true);
    }

    // 3) Traffic Reports, one per live target, optionally capped per cycle so a
    //    busy sky can't blow the per-cycle USB budget. The cap throttles count,
    //    not which targets — snapshot order is preserved.
    size_t limit = snap->count;
    if (ctx->max_targets_per_cycle != 0 && limit > ctx->max_targets_per_cycle) {
        limit = ctx->max_targets_per_cycle;
    }

    for (size_t i = 0; i < limit; ++i) {
        // Map the canonical record to GDL90 wire fields, then frame + write.
        gdl90_traffic_t tr;
        gdl90_traffic_from_target(&tr, &snap->targets[i]);
        gdl90_emit_report(ctx, &tr, /*ownship=*/false);
    }

    // One flush at end of cycle keeps the framed messages moving promptly.
    sink_transport_flush(ctx->transport);
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  vtable: destroy
 * ═══════════════════════════════════════════════════════════════════════════ */

static void sink_gdl90_destroy_ctx(void *vctx)
{
    sink_gdl90_ctx_t *ctx = (sink_gdl90_ctx_t *)vctx;
    if (ctx == NULL) {
        return;
    }

    // The transport is owned by the caller (possibly shared) — never freed here.
    if (ctx->own_lock != NULL) {
        vSemaphoreDelete(ctx->own_lock);
    }
    free(ctx);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Construction / public API
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t sink_gdl90_create(const sink_gdl90_cfg_t *cfg, sink_handle_t *out_sink)
{
    if (cfg == NULL || out_sink == NULL || cfg->transport == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_sink = NULL;

    // Allocate handle shell + private context.
    struct sink_s *sink = calloc(1, sizeof(struct sink_s));
    sink_gdl90_ctx_t *ctx = calloc(1, sizeof(sink_gdl90_ctx_t));
    if (sink == NULL || ctx == NULL) {
        free(sink);
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    // Mutex guards the optional ownship that sink_gdl90_set_ownship() updates
    // from another task while publish() reads it on the publisher task.
    ctx->own_lock = xSemaphoreCreateMutex();
    if (ctx->own_lock == NULL) {
        free(sink);
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    // Capture config.
    ctx->transport             = cfg->transport;
    ctx->emit_ownship_report   = cfg->emit_ownship_report;
    ctx->max_targets_per_cycle = cfg->max_targets_per_cycle;
    ctx->ownship.valid         = false;

    // Wire up the vtable. GDL90 has no per-message output, so feed_msg is NULL —
    // the registry simply skips it for feeds.
    sink->vt.kind     = SINK_KIND_GDL90;
    sink->vt.name     = "gdl90";
    sink->vt.publish  = sink_gdl90_publish;
    sink->vt.feed_msg = NULL;
    sink->vt.destroy  = sink_gdl90_destroy_ctx;
    sink->vt.ctx      = ctx;
    sink->registered  = false;

    ESP_LOGI(TAG, "created (ownship_report=%d cap=%u)",
             (int)ctx->emit_ownship_report, (unsigned)ctx->max_targets_per_cycle);
    *out_sink = sink;
    return ESP_OK;
}

void sink_gdl90_destroy(sink_handle_t sink)
{
    if (sink == NULL) {
        return;
    }

    // Free private state via the destroy hook, then the handle shell. Caller
    // must sinks_unregister() first (documented in the header).
    if (sink->vt.destroy != NULL) {
        sink->vt.destroy(sink->vt.ctx);
    }
    free(sink);
}

esp_err_t sink_gdl90_set_ownship(sink_handle_t sink, const ownship_ref_t *ownship)
{
    if (sink == NULL || sink->vt.ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sink_gdl90_ctx_t *ctx = (sink_gdl90_ctx_t *)sink->vt.ctx;

    xSemaphoreTake(ctx->own_lock, portMAX_DELAY);
    if (ownship == NULL || !ownship->valid) {
        // NULL or invalid => suppress the ownship report next cycle.
        ctx->ownship.valid = false;
    } else {
        // Copy a coherent snapshot of the reference for the publisher to read.
        ctx->ownship = *ownship;
    }
    xSemaphoreGive(ctx->own_lock);

    return ESP_OK;
}
