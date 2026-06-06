/**
 * @file    sink_gdl90.h
 * @brief   GDL90 output sink (plan S4.5 sink_gdl90).
 *
 * @details
 *   Encodes GDL90 Heartbeat + Traffic Report (and optional Ownship Report)
 *   messages and writes them through a transport. MVP transport is USB-CDC; the
 *   identical encoder later drives WiFi/UDP (only the transport changes). The
 *   pure framing lives in gdl90_encoder.h so it is unit-testable on a host.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sinks.h"
#include "sink_transport.h"
#include "ownship.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief GDL90-sink configuration. */
typedef struct {
    sink_transport_t transport;          /**< USB-CDC (MVP) or future UDP.       */
    bool             emit_ownship_report;/**< Emit msg 0x0A when ownship valid.  */
    uint8_t          max_targets_per_cycle; /**< 0 => no cap; else throttle.      */
} sink_gdl90_cfg_t;

/** @brief Construct a GDL90 sink. Register it with sinks_register() afterwards. */
esp_err_t sink_gdl90_create(const sink_gdl90_cfg_t *cfg, sink_handle_t *out_sink);

/** @brief Destroy a GDL90 sink (sinks_unregister() it first). */
void sink_gdl90_destroy(sink_handle_t sink);

/** @brief Update the ownship used for the optional Ownship Report.
 *  @param ownship NULL or !valid => suppress the ownship report. */
esp_err_t sink_gdl90_set_ownship(sink_handle_t sink, const ownship_ref_t *ownship);

#ifdef __cplusplus
}
#endif
