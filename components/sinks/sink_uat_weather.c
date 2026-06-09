/**
 * @file    sink_uat_weather.c
 * @brief   UAT FIS-B weather sink — frames uplinks as GDL90 Uplink (0x07).
 *
 * @details
 *   Standalone sink (not in the publisher vtable). It is fed FEC-corrected UAT
 *   uplink frames by a Core-1 glue task, frames each one as a GDL90 Uplink Data
 *   message (id 0x07) using the pure, host-tested gdl90_frame_uplink(), and writes
 *   the framed bytes to both transports — the wired USB-CDC link (always) and the
 *   WiFi/UDP :4000 broadcast (when up). The FIS-B payload is relayed VERBATIM;
 *   ADSBin never re-encodes weather products — the EFB parses them.
 *
 * @par Non-blocking
 *   Every transport write is bounded and drops on back-pressure, the same
 *   discipline as sink_gdl90, so a stalled host never starves Core 0.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 *
 * GDL90 framing is CLEAN-ROOM from the public Garmin GDL90 spec; no GPL source.
 */

#include "sink_uat_weather.h"
#include "gdl90_encoder.h"

#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "adsbin_err.h"

static const char *TAG = "sink_uat_wx";

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private per-sink state.
 *
 *  The framing scratch is sized for the worst-case stuffed 0x07 message
 *  (::GDL90_UPLINK_FRAME_MAX ~896 bytes) — far larger than the 80-byte traffic
 *  scratch, which is exactly why the weather sink is separate from sink_gdl90.
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct sink_uat_weather_s {
    sink_transport_t cdc;                       /**< Wired USB-CDC (required).     */
    sink_transport_t udp;                       /**< UDP broadcast or NULL.        */
    uint8_t          frame[GDL90_UPLINK_FRAME_MAX]; /**< 0x07 framing scratch.     */
} sink_uat_weather_impl_t;

esp_err_t sink_uat_weather_create(const sink_uat_weather_cfg_t *cfg,
                                  sink_uat_weather_t *out_sink)
{
    if (!cfg || !out_sink || !cfg->cdc) {
        return ESP_ERR_INVALID_ARG;
    }

    /* One heap allocation for the whole sink (it owns no other resources; the
     * transports are shared and owned by main). */
    sink_uat_weather_impl_t *s = (sink_uat_weather_impl_t *)calloc(1, sizeof(*s));
    if (!s) {
        return ESP_ERR_NO_MEM;
    }
    s->cdc = cfg->cdc;
    s->udp = cfg->udp;   /* may be NULL — relay over CDC only in that case. */

    *out_sink = (sink_uat_weather_t)s;
    ESP_LOGI(TAG, "weather sink up (udp=%s)", cfg->udp ? "yes" : "no");
    return ESP_OK;
}

esp_err_t sink_uat_weather_feed(sink_uat_weather_t sink, const uat_uplink_t *up)
{
    if (!sink || !up || !up->payload) {
        return ESP_ERR_INVALID_ARG;
    }
    sink_uat_weather_impl_t *s = (sink_uat_weather_impl_t *)sink;

    /* Frame the FIS-B payload as a GDL90 Uplink Data (0x07) message ONCE. The
     * Time of Reception is left 0 ("unknown") — ADSBin has no synchronized UAT
     * frame clock, and ForeFlight tolerates a zero ToR (Stratux ships it this
     * way). The payload is relayed verbatim. */
    int n = gdl90_frame_uplink(s->frame, sizeof(s->frame),
                               0u, up->payload, up->payload_len);
    if (n <= 0) {
        ESP_LOGW(TAG, "uplink frame failed (%d)", n);
        return ESP_FAIL;
    }

    /* Write to the wired link always. Drop-on-backpressure: a transport that
     * can't take it returns non-OK and we simply move on (never block Core 1). */
    esp_err_t cdc_err = sink_transport_write(s->cdc, s->frame, (size_t)n);
    if (cdc_err == ESP_OK) {
        sink_transport_flush(s->cdc);
    }

    /* Mirror to UDP/ForeFlight when the AP is up. A NULL udp transport simply
     * means no WiFi this run — the wired path already carried it. */
    if (s->udp) {
        if (sink_transport_write(s->udp, s->frame, (size_t)n) == ESP_OK) {
            sink_transport_flush(s->udp);
        }
    }

    return cdc_err;
}

void sink_uat_weather_destroy(sink_uat_weather_t sink)
{
    if (sink) {
        /* The transports are shared (owned by main) — do NOT destroy them here. */
        free(sink);
    }
}
