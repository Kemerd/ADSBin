/**
 * @file    adsbin_app.h
 * @brief   ADSBin application wiring contract: pipeline plumbing + lifecycle.
 *
 * @details
 *   `main` is the top of the dependency graph (IMPLEMENTATION_PLAN.md S2). It is
 *   the ONLY translation unit that:
 *     - constructs the inter-task IPC objects (the IQ ring + frame/msg queues),
 *     - creates every FreeRTOS task and pins it to its design core, and
 *     - calls each component's `*_start()` entry point.
 *
 *   Components never set their own core affinity or create the shared queues;
 *   they receive the handles as parameters. This header is the single source of
 *   truth for that plumbing (::adsbin_pipeline_t) and the task tuning knobs.
 *
 *   Core-id macros (::ADSBIN_CORE_DSP / ::ADSBIN_CORE_DECODE) live in
 *   `common/adsbin_types.h`, NOT here, so a Core-0 component can self-document
 *   its affinity without taking a dependency on `main`.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_err.h"
#include "adsbin_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  Task priorities. The hard-real-time DSP chain sits above everything; the
 *  Core-1 consumers fan out below it so they can never starve Core 0 (S5.2).
 * ─────────────────────────────────────────────────────────────────────────── */
#define ADSBIN_PRIO_USB      (configMAX_PRIORITIES - 2)  /**< usb_rtlsdr RX (Core0). */
#define ADSBIN_PRIO_DEMOD    (configMAX_PRIORITIES - 3)  /**< demod1090 DSP (Core0). */
#define ADSBIN_PRIO_DECODE   5                           /**< modes_decode (Core1).  */
#define ADSBIN_PRIO_TRAFFIC  4                           /**< traffic table (Core1). */
#define ADSBIN_PRIO_SINKS    3                           /**< output sinks  (Core1). */
#define ADSBIN_PRIO_STATUS   2                           /**< LEDs / temp   (Core1). */

/* ───────────────────────────────────────────────────────────────────────────
 *  IPC sizing. Producers and consumers must agree on these, so they are fixed
 *  here once. Tuned in Phase 3 once real frame rates are measured.
 * ─────────────────────────────────────────────────────────────────────────── */
#define ADSBIN_IQ_RING_BLOCKS   8     /**< Depth of the IQ ring (>= 8 blocks).   */
#define ADSBIN_FRAME_QUEUE_LEN  256   /**< modes_frame_t slots (Core0 -> Core1). */
#define ADSBIN_MSG_QUEUE_LEN    128   /**< adsb_msg_t slots (decode -> traffic). */

/**
 * @brief Shared inter-task plumbing built once by `main` and handed to tasks.
 *
 * @details
 *   Holds live FreeRTOS handles (NOT plain-old-data), which is why it lives in
 *   `main` rather than `common`. Each component receives only the handle(s) it
 *   needs through its `*_start()` call; nobody reaches into globals.
 *
 *     iq_ring     : usb_rtlsdr (producer, Core 0)  -> demod1090 (consumer, Core 0)
 *     frame_queue : demod1090  (producer, Core 0)  -> modes_decode (consumer, Core 1)
 *     msg_queue   : modes_decode(producer, Core 1) -> traffic (consumer, Core 1)
 */
typedef struct {
    RingbufHandle_t iq_ring;      /**< IQ blocks, no-split ring (owned by usb_rtlsdr). */
    QueueHandle_t   frame_queue;  /**< Candidate ::modes_frame_t, by value.            */
    QueueHandle_t   msg_queue;    /**< Decoded ::adsb_msg_t, by value.                 */
} adsbin_pipeline_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle (implemented in main/adsbin_app.c)
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief One-shot bring-up: NVS/config, then any shared IPC construction.
 *
 * Called from app_main() before adsbin_app_start(). In the Phase-0 scaffold this
 * only prepares logging/state; pipeline (ring + queues) construction is added as
 * the producing/consuming components come online (Stage 3 integration).
 *
 * @return ESP_OK on success, or an esp_err_t from an init step.
 */
esp_err_t adsbin_app_init(void);

/**
 * @brief Create and core-pin every task in the S2 graph, then return.
 *
 * Core 0: usb_rtlsdr RX + demod1090. Core 1: modes_decode, traffic, sinks,
 * status, config. In the Phase-0 scaffold this launches the dual-core liveness
 * tasks that the pipeline tasks later replace/augment.
 *
 * @return ESP_OK on success, or an esp_err_t from task creation.
 */
esp_err_t adsbin_app_start(void);

/**
 * @brief Accessor for the singleton pipeline plumbing (handles for the tasks).
 * @return Pointer to the process-wide ::adsbin_pipeline_t (never NULL after init).
 */
const adsbin_pipeline_t *adsbin_app_pipeline(void);

#ifdef __cplusplus
}
#endif
