/**
 * @file    sink_debug.c
 * @brief   Human-readable ASCII traffic-table sink (plan S4.5).
 *
 * @details
 *   Renders the live traffic snapshot as the frozen token table defined in
 *   tools/bench/WIRE_CONTRACT.md so the Python bench harness parses it
 *   deterministically. The exact per-target line is:
 *
 *     ICAO=<6hex> [CS=..] [LAT=..] [LON=..] [ALT=..] [GS=..] [TRK=..] [VR=..]
 *                 [CAT=..] [NIC=..] [NACp=..] [RNG=..] [BRG=..] MSGS=<n> SEEN=<ms>
 *
 *   Optional tokens are OMITTED ENTIRELY when their field is invalid; ICAO=,
 *   MSGS= and SEEN= are always present. The block is wrapped in
 *   "=== ADSBIN TRAFFIC <count> @ <now_us> ===" / "=== END ===" context lines
 *   (which the parser may ignore — it keys on lines starting with "ICAO=").
 *
 *   All bytes go out through a ::sink_transport_t (USB-CDC in the MVP); the sink
 *   never touches USB directly. Lines are UTF-8, '\n'-terminated, never CRLF, so
 *   they stay byte-clean alongside the GDL90 binary stream on the same link.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "sink_debug.h"
#include "sink_internal.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "adsbin_types.h"
#include "adsbin_err.h"
#include "demod1090.h"      /* live DSP counters for the RF diagnostic line */

/* ───────────────────────────────────────────────────────────────────────────
 *  Sizing. One target line tops out well under this; we render into a per-sink
 *  scratch buffer and write it in one transport call to keep lines atomic on
 *  the wire.
 * ─────────────────────────────────────────────────────────────────────────── */
#define DEBUG_LINE_MAX   256u   /**< Max bytes for one rendered target/MSG line. */

static const char *TAG = "sink_debug";

/* ANSI clear-screen + cursor-home, emitted before each block when configured so
 * a terminal shows a live, in-place updating table. */
static const char ANSI_CLEAR_HOME[] = "\x1b[2J\x1b[H";

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private per-sink state (lives behind sink_vtable_t.ctx).
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    sink_transport_t transport;     /**< Output byte stream (USB-CDC).           */
    bool             verbose;       /**< Also dump per-message MSG lines.        */
    bool             clear_screen;  /**< Emit ANSI clear+home before each block. */
    char             line[DEBUG_LINE_MAX]; /**< Render scratch (publisher task).  */

    /*
     * Previous-cycle DSP counter snapshot, kept so the RF diagnostic line can
     * report PER-SECOND RATES (deltas) rather than ever-growing cumulative
     * totals. The publisher ticks at a fixed ~1 Hz cadence, so each delta is
     * effectively "events in the last second" — the gauge a field tester reads
     * to see the antenna catching RF the instant a 1090 signal is in range.
     */
    uint64_t prev_preambles;        /**< preambles_detected at last publish.     */
    uint64_t prev_frames;           /**< frames_emitted at last publish.         */
    bool     have_prev;             /**< False until the first snapshot is taken. */
} sink_debug_ctx_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Rendering helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Append a NUL-terminated fragment to a bounded line buffer.
 *
 * @details
 *   A tiny snprintf-cursor wrapper so each token append is one call. We track
 *   the write position and clamp at the buffer end; on overflow we stop
 *   appending (the line is truncated rather than overrunning, and still ends
 *   with a newline added by the caller).
 *
 * @param buf  Destination buffer.
 * @param cap  Capacity of @p buf.
 * @param pos  In/out cursor (current string length).
 * @param frag NUL-terminated fragment to append.
 */
static void line_append(char *buf, size_t cap, size_t *pos, const char *frag)
{
    // Nothing sensible to do once we're at (or past) the end of the buffer.
    if (*pos >= cap - 1) {
        return;
    }

    // Copy until either the fragment ends or the buffer is full.
    size_t i = 0;
    while (frag[i] != '\0' && *pos < cap - 1) {
        buf[(*pos)++] = frag[i++];
    }
    buf[*pos] = '\0';
}

/**
 * @brief Render one target into @c ctx->line per the frozen token order.
 *
 * @param ctx   Debug sink state (provides the scratch line buffer).
 * @param t     The target to render.
 * @param now_us Cycle timestamp, used to compute the SEEN age in ms.
 * @return Length of the rendered line (excluding the trailing NUL).
 */
static size_t render_target_line(sink_debug_ctx_t *ctx, const traffic_target_t *t, int64_t now_us)
{
    char *buf = ctx->line;
    size_t pos = 0;
    char frag[64];   /* per-token formatting scratch */

    // ── ICAO= : always present, 6 uppercase zero-padded hex digits. ──────────
    snprintf(frag, sizeof(frag), "ICAO=%06X", (unsigned)(t->icao & 0x00FFFFFFu));
    line_append(buf, DEBUG_LINE_MAX, &pos, frag);

    // ── CS= : only when a callsign is known; emitted with no embedded spaces. ─
    if (t->has_callsign && t->callsign[0] != '\0') {
        // Copy out a space-free, length-bounded callsign (defensive — upstream
        // already strips spaces, but the wire token must never contain one).
        char cs[ADSB_CALLSIGN_LEN];
        size_t k = 0;
        for (size_t i = 0; i < ADSB_CALLSIGN_LEN - 1 && t->callsign[i] != '\0'; ++i) {
            if (t->callsign[i] != ' ') {
                cs[k++] = t->callsign[i];
            }
        }
        cs[k] = '\0';
        if (k > 0) {
            snprintf(frag, sizeof(frag), " CS=%s", cs);
            line_append(buf, DEBUG_LINE_MAX, &pos, frag);
        }
    }

    // ── LAT=/LON= : signed decimal degrees, >= 4 fractional digits. Emitted ──
    //    only when the target currently holds a valid resolved fix.
    if (t->position_valid) {
        snprintf(frag, sizeof(frag), " LAT=%.5f", t->lat_deg);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
        snprintf(frag, sizeof(frag), " LON=%.5f", t->lon_deg);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
    }

    // ── ALT= : signed integer feet (pressure altitude). ──────────────────────
    if (t->has_altitude) {
        snprintf(frag, sizeof(frag), " ALT=%ld", (long)t->altitude_ft);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
    }

    // ── GS=/TRK= : unsigned integer knots / degrees, only with a velocity fix.
    if (t->has_velocity) {
        snprintf(frag, sizeof(frag), " GS=%u", (unsigned)t->ground_speed_kt);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
        snprintf(frag, sizeof(frag), " TRK=%u", (unsigned)t->track_deg);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
    }

    // ── VR= : signed integer feet-per-minute, only when known. ───────────────
    if (t->has_vertical_rate) {
        snprintf(frag, sizeof(frag), " VR=%d", (int)t->vertical_rate_fpm);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
    }

    // ── CAT= : emitter category integer, only when classified. ───────────────
    if (t->has_category) {
        snprintf(frag, sizeof(frag), " CAT=%u", (unsigned)t->emitter_category);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
    }

    // ── NIC=/NACp= : integrity integers 0..11. We emit them whenever a position
    //    exists (they describe that position's integrity/accuracy). ───────────
    if (t->position_valid) {
        snprintf(frag, sizeof(frag), " NIC=%u", (unsigned)t->nic);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
        snprintf(frag, sizeof(frag), " NACp=%u", (unsigned)t->nacp);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
    }

    // ── RNG=/BRG= : relative geometry, present only when an ownship reference is
    //    set AND the target has a position (traffic sets has_relative for that). ─
    if (t->has_relative) {
        snprintf(frag, sizeof(frag), " RNG=%.1f", (double)t->range_nm);
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
        snprintf(frag, sizeof(frag), " BRG=%u",
                 (unsigned)((unsigned)(t->bearing_deg + 0.5f) % 360u));
        line_append(buf, DEBUG_LINE_MAX, &pos, frag);
    }

    // ── MSGS= : always present, merged message count. ────────────────────────
    snprintf(frag, sizeof(frag), " MSGS=%u", (unsigned)t->msg_count);
    line_append(buf, DEBUG_LINE_MAX, &pos, frag);

    // ── SEEN= : always present, age in ms since last heard. The single time ──
    //    base is microseconds; convert the delta to ms here for the contract.
    int64_t age_us = now_us - t->last_seen_us;
    if (age_us < 0) age_us = 0;   /* guard against a slightly-future stamp */
    snprintf(frag, sizeof(frag), " SEEN=%lld\n", (long long)(age_us / 1000));
    line_append(buf, DEBUG_LINE_MAX, &pos, frag);

    return pos;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  vtable: publish
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Emit one full traffic block: header, one line per target, footer.
 */
static esp_err_t sink_debug_publish(void *vctx, const traffic_snapshot_t *snap,
                                    const ownship_ref_t *own)
{
    (void)own;   /* range/bearing already resolved into the target by traffic */

    sink_debug_ctx_t *ctx = (sink_debug_ctx_t *)vctx;
    if (ctx == NULL || ctx->transport == NULL || snap == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Optional ANSI clear+home so a live terminal redraws in place.
    if (ctx->clear_screen) {
        sink_transport_write(ctx->transport,
                             (const uint8_t *)ANSI_CLEAR_HOME,
                             sizeof(ANSI_CLEAR_HOME) - 1);
    }

    // Header line: "=== ADSBIN TRAFFIC <count> @ <now_us> ===".
    int hlen = snprintf(ctx->line, DEBUG_LINE_MAX,
                        "=== ADSBIN TRAFFIC %u @ %lld ===\n",
                        (unsigned)snap->count, (long long)snap->taken_us);
    if (hlen > 0) {
        sink_transport_write(ctx->transport, (const uint8_t *)ctx->line, (size_t)hlen);
    }

    /* ── Live RF diagnostic line ─────────────────────────────────────────────
     * The single most useful field-test gauge: it tells you whether the antenna
     * is actually catching 1090 RF, BEFORE any aircraft fully decodes. We snap
     * the cumulative DSP counters and report PER-SECOND DELTAS:
     *
     *   RF PRE=<n>/s FRM=<n>/s SIG=<0..65535>
     *
     *   PRE  — preambles detected this second. >0 means the antenna IS hearing
     *          Mode-S energy (the 8 µs sync). PRE=0 forever => no RF reaching the
     *          tuner: check the antenna/connector, not the software.
     *   FRM  — candidate frames that passed bit-slicing this second.
     *   SIG  — magnitude of the most recent candidate's signal level (relative).
     *
     * Deltas (not totals) because the publisher ticks ~1 Hz, so each value reads
     * as "events in the last second" — a live needle, not an odometer.
     */
    demod1090_stats_t st;
    demod1090_get_stats(&st);

    uint64_t d_pre = 0, d_frm = 0;
    if (ctx->have_prev) {
        // Plain unsigned subtraction; counters are monotonic so this never wraps
        // negative under normal operation (a counter reset just yields one large
        // delta, which is harmless for a diagnostic line).
        d_pre = st.preambles_detected - ctx->prev_preambles;
        d_frm = st.frames_emitted     - ctx->prev_frames;
    }
    ctx->prev_preambles = st.preambles_detected;
    ctx->prev_frames    = st.frames_emitted;
    ctx->have_prev      = true;

    int rflen = snprintf(ctx->line, DEBUG_LINE_MAX,
                         "RF PRE=%llu/s FRM=%llu/s SIG=%u\n",
                         (unsigned long long)d_pre,
                         (unsigned long long)d_frm,
                         (unsigned)st.last_signal_level);
    if (rflen > 0) {
        sink_transport_write(ctx->transport, (const uint8_t *)ctx->line, (size_t)rflen);
    }

    // One line per live target, in snapshot order.
    for (size_t i = 0; i < snap->count; ++i) {
        size_t len = render_target_line(ctx, &snap->targets[i], snap->taken_us);
        if (len > 0) {
            sink_transport_write(ctx->transport, (const uint8_t *)ctx->line, len);
        }
    }

    // Footer line.
    static const char FOOTER[] = "=== END ===\n";
    sink_transport_write(ctx->transport, (const uint8_t *)FOOTER, sizeof(FOOTER) - 1);

    // Push the block out promptly so the host sees a complete, fresh table.
    sink_transport_flush(ctx->transport);

    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  vtable: feed_msg (verbose per-message dump)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Emit the verbose per-message diagnostic line when verbose is on.
 *
 * @details
 *   WIRE_CONTRACT.md documents this line as:
 *       MSG ICAO=<6hex> DF=<n> TC=<n> RAW=<14hex|28hex>
 *   The frozen feed_msg() contract delivers a DECODED ::adsb_msg_t, which does
 *   not carry the original raw frame bytes (those live in modes_frame_t upstream
 *   and are not part of this ABI). We therefore emit every token we can derive
 *   from adsb_msg_t — ICAO/DF/TC — and OMIT RAW= because fabricating it would be
 *   fake data. See the integrator note returned with this component.
 */
static esp_err_t sink_debug_feed_msg(void *vctx, const adsb_msg_t *msg)
{
    sink_debug_ctx_t *ctx = (sink_debug_ctx_t *)vctx;
    if (ctx == NULL || msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Quietly do nothing unless the operator turned verbose on.
    if (!ctx->verbose || ctx->transport == NULL) {
        return ESP_OK;
    }

    // Render "MSG ICAO=.. DF=.. TC=..". RAW= is intentionally absent (see above).
    int len = snprintf(ctx->line, DEBUG_LINE_MAX,
                       "MSG ICAO=%06X DF=%u TC=%u\n",
                       (unsigned)(msg->icao & 0x00FFFFFFu),
                       (unsigned)msg->downlink_format,
                       (unsigned)msg->type_code);
    if (len > 0) {
        sink_transport_write(ctx->transport, (const uint8_t *)ctx->line, (size_t)len);
    }

    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  vtable: destroy
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Free the per-sink context. The transport is owned by the caller and is
 *        NOT destroyed here (it may be shared with other sinks on the link).
 */
static void sink_debug_destroy_ctx(void *vctx)
{
    sink_debug_ctx_t *ctx = (sink_debug_ctx_t *)vctx;
    if (ctx != NULL) {
        free(ctx);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Construction / public API
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t sink_debug_create(const sink_debug_cfg_t *cfg, sink_handle_t *out_sink)
{
    if (cfg == NULL || out_sink == NULL || cfg->transport == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_sink = NULL;

    // Allocate the handle (registry view) and the private context together so a
    // single create has a single failure point and a single free per object.
    struct sink_s *sink = calloc(1, sizeof(struct sink_s));
    sink_debug_ctx_t *ctx = calloc(1, sizeof(sink_debug_ctx_t));
    if (sink == NULL || ctx == NULL) {
        free(sink);
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    // Capture configuration into the private context.
    ctx->transport    = cfg->transport;
    ctx->verbose      = cfg->verbose;
    ctx->clear_screen = cfg->clear_screen;

    // Wire up the vtable. feed_msg is provided so verbose dumps work; the
    // registry only calls it for sinks that expose it (this one does).
    sink->vt.kind     = SINK_KIND_DEBUG;
    sink->vt.name     = "debug";
    sink->vt.publish  = sink_debug_publish;
    sink->vt.feed_msg = sink_debug_feed_msg;
    sink->vt.destroy  = sink_debug_destroy_ctx;
    sink->vt.ctx      = ctx;
    sink->registered  = false;

    ESP_LOGI(TAG, "created (verbose=%d clear=%d)", (int)ctx->verbose, (int)ctx->clear_screen);
    *out_sink = sink;
    return ESP_OK;
}

void sink_debug_destroy(sink_handle_t sink)
{
    if (sink == NULL) {
        return;
    }

    // Defer the private-state free to the vtable destroy hook, then free the
    // handle shell itself. Caller must sinks_unregister() first (documented).
    if (sink->vt.destroy != NULL) {
        sink->vt.destroy(sink->vt.ctx);
    }
    free(sink);
}

esp_err_t sink_debug_set_verbose(sink_handle_t sink, bool verbose)
{
    if (sink == NULL || sink->vt.ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // The publisher reads this flag from another task; a plain store is fine
    // because a bool toggle is atomic on Xtensa/RISC-V and a one-cycle-late
    // visibility is harmless for a debug toggle.
    sink_debug_ctx_t *ctx = (sink_debug_ctx_t *)sink->vt.ctx;
    ctx->verbose = verbose;
    return ESP_OK;
}
