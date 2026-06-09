/**
 * @file    sink_uat_weather.h
 * @brief   UAT FIS-B weather sink — relays uplink frames as GDL90 Uplink (0x07).
 *
 * @details
 *   A STANDALONE sink (NOT a publisher-loop vtable sink). UAT weather uplinks are
 *   event-driven and large (432 bytes), so the 1 Hz traffic-snapshot vtable that
 *   sink_gdl90 uses is the wrong shape. Instead a small Core-1 glue task drains
 *   the uplink ring and feeds each frame here; this sink frames it once as a GDL90
 *   Uplink Data message (id 0x07) and writes it to BOTH transports (the wired
 *   USB-CDC link and, if up, the WiFi/UDP :4000 broadcast ForeFlight listens on).
 *
 *   It holds its transports directly and does NOT touch the traffic table, so it
 *   shares the existing sink_transport + gdl90_encoder seams with ZERO changes to
 *   either. Writes are non-blocking and drop-on-backpressure, the same discipline
 *   as sink_gdl90.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include "esp_err.h"
#include "adsbin_types.h"      /* uat_uplink_t */
#include "sink_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque weather-sink instance. */
typedef struct sink_uat_weather_s *sink_uat_weather_t;

/**
 * @brief Weather sink config: the two transports to relay onto.
 *
 * @c cdc is the wired USB-CDC link (always present); @c udp is the WiFi/UDP
 * broadcast transport and may be NULL when the C6 SoftAP is not up — the sink
 * then relays over CDC only.
 */
typedef struct {
    sink_transport_t cdc;   /**< Wired USB-CDC transport (required).             */
    sink_transport_t udp;   /**< UDP-broadcast transport, or NULL if no WiFi.    */
} sink_uat_weather_cfg_t;

/** @brief Create a weather sink. @return ESP_OK or an esp_err_t. */
esp_err_t sink_uat_weather_create(const sink_uat_weather_cfg_t *cfg,
                                  sink_uat_weather_t *out_sink);

/**
 * @brief Frame one FIS-B uplink as GDL90 0x07 and write it to both transports.
 *
 * Non-blocking; a busy/absent transport drops the frame (counted) rather than
 * stalling. Safe to call from the Core-1 uplink glue task.
 *
 * @param sink   The weather sink.
 * @param up     The FEC-corrected uplink frame (payload borrowed; copied here).
 * @return ESP_OK if at least the CDC write was attempted; an esp_err_t otherwise.
 */
esp_err_t sink_uat_weather_feed(sink_uat_weather_t sink, const uat_uplink_t *up);

/** @brief Destroy a weather sink (does NOT destroy the shared transports). */
void sink_uat_weather_destroy(sink_uat_weather_t sink);

#ifdef __cplusplus
}
#endif
