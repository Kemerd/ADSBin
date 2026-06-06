/**
 * @file    sink_debug.h
 * @brief   Human-readable traffic-table sink (plan S4.5 sink_debug).
 *
 * @details
 *   Renders the live traffic table as an ASCII table over a transport (USB-CDC
 *   in the MVP) for bring-up and field debugging. The exact token format it
 *   emits is frozen in tools/bench/WIRE_CONTRACT.md so the Python bench harness
 *   can parse it deterministically (ICAO=/CS=/LAT=/...). Optionally dumps raw
 *   per-message frames when verbose.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "sinks.h"
#include "sink_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Debug-sink configuration. */
typedef struct {
    sink_transport_t transport;    /**< Where the table is written (USB-CDC).    */
    bool             verbose;      /**< Also dump per-message raw frames.        */
    bool             clear_screen; /**< Emit ANSI clear+home for a live table.   */
} sink_debug_cfg_t;

/** @brief Construct a debug sink. Register it with sinks_register() afterwards. */
esp_err_t sink_debug_create(const sink_debug_cfg_t *cfg, sink_handle_t *out_sink);

/** @brief Destroy a debug sink (sinks_unregister() it first). */
void sink_debug_destroy(sink_handle_t sink);

/** @brief Toggle verbose per-message dumping at runtime. */
esp_err_t sink_debug_set_verbose(sink_handle_t sink, bool verbose);

#ifdef __cplusplus
}
#endif
