/**
 * @file    sinks.h
 * @brief   Output sink registry + shared sink vtable (plan S4.5).
 *
 * @details
 *   The sink layer fans the traffic table out to one or more outputs. A registry
 *   model lets sinks "light up" based on detected hardware / config (the build-
 *   superset-sell-subsets goal, S0) with no #ifdefs in the sink code:
 *
 *     main builds transport(s) -> creates sink(s) -> sinks_register() each ->
 *     sinks_start(). A periodic Core-1 publisher task snapshots traffic + ownship
 *     and calls every registered sink's publish() through the shared vtable.
 *
 *   Each concrete sink (sink_debug, sink_gdl90, future sink_tis) implements
 *   ::sink_vtable_t, so they can be authored in separate files against one ABI.
 *
 * @par Core affinity
 *   Core 1 (::ADSBIN_CORE_DECODE). The publisher runs at a modest priority so it
 *   never preempts the Core-0 DSP path. Sinks do no work on Core 0.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "adsbin_types.h"   /* traffic_snapshot_t, adsb_msg_t */
#include "ownship.h"        /* ownship_ref_t */
#include "traffic.h"        /* traffic_handle_t (the snapshot source) */

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque per-sink instance. */
typedef struct sink_s *sink_handle_t;

/** @brief Which concrete sink a handle is. */
typedef enum {
    SINK_KIND_DEBUG = 0,
    SINK_KIND_GDL90 = 1,
    SINK_KIND_TIS   = 2,   /**< Future: Garmin TIS-A / RS-232 (plan S11).      */
} sink_kind_t;

/** @brief Publisher-loop configuration (sinks_start). */
typedef struct {
    uint32_t    publish_interval_ms; /**< Snapshot cadence (GDL90 Heartbeat ~1 Hz).*/
    uint32_t    task_stack_size;     /**< Publisher task stack bytes (0 => 4096). */
    UBaseType_t task_priority;       /**< Keep low so Core-0 DSP is never starved.*/
    BaseType_t  task_core_id;        /**< Pin to ADSBIN_CORE_DECODE.             */
} sinks_loop_cfg_t;

/**
 * @brief The vtable every concrete sink implements.
 *
 * @c publish is called once per cycle with a coherent traffic snapshot and the
 * current ownship. @c feed_msg (optional, may be NULL) receives freshly decoded
 * raw messages for per-message outputs (e.g. verbose debug).
 */
typedef struct {
    sink_kind_t kind;
    const char *name;
    esp_err_t (*publish)(void *ctx, const traffic_snapshot_t *snap, const ownship_ref_t *own);
    esp_err_t (*feed_msg)(void *ctx, const adsb_msg_t *msg);
    void      (*destroy)(void *ctx);
    void      *ctx;                  /**< Sink-private state.                    */
} sink_vtable_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Registry + publisher lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Init the registry and bind the traffic source it snapshots from. */
esp_err_t sinks_init(traffic_handle_t traffic);

/** @brief Register a constructed sink so it joins the publish loop. */
esp_err_t sinks_register(sink_handle_t sink);

/** @brief Remove a sink from the registry (does not destroy it). */
esp_err_t sinks_unregister(sink_handle_t sink);

/** @brief Number of registered sinks (feeds status / auto-detect). */
size_t sinks_count(void);

/** @brief Spawn the Core-1 publisher task. */
esp_err_t sinks_start(const sinks_loop_cfg_t *cfg);

/** @brief Stop + join the publisher task; sinks stay registered. */
esp_err_t sinks_stop(void);

/** @brief Force one synchronous publish cycle now (bench / on-demand dump). */
esp_err_t sinks_publish_now(void);

/** @brief Fan one freshly decoded message to sinks that opted into feed_msg. */
esp_err_t sink_feed_msg(const adsb_msg_t *msg);

#ifdef __cplusplus
}
#endif
