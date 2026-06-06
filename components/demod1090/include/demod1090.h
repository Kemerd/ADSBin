/**
 * @file    demod1090.h
 * @brief   1090ES demodulator — public contract (plan S4.2).
 *
 * @details
 *   The Core-0 DSP front end. It computes magnitude from 8-bit I/Q (precomputed
 *   LUT), correlates the 8 us Mode-S preamble, PPM bit-slices candidates into
 *   56/112-bit frames, and hands them off. It does NO CRC, DF parse, CPR, or
 *   adsb_msg work — those belong to modes_decode (plan S4.3).
 *
 *   DATA FLOW (reconciled, plan S2):
 *     usb_rtlsdr IQ ring  --(RingbufHandle_t)-->  demod1090  --(QueueHandle_t of
 *     modes_frame_t, by value)-->  modes_decode.
 *   `main` owns the queue and passes both handles to demod1090_start(); the
 *   driver never creates IPC or pins itself (invariant: only main sets affinity).
 *
 * @par Core affinity
 *   Core 0 (::ADSBIN_CORE_DSP). Hard real-time. The demod task MUST NOT block:
 *   on output backpressure it drops-with-counter rather than stalling Core 0.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "adsbin_types.h"   /* iq_block_t, modes_frame_t */

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Demodulator configuration (demod1090_init). */
typedef struct {
    uint32_t sample_rate_hz;     /**< Expected IQ rate (default ADSB_SAMPLE_RATE_HZ). */
    uint8_t  task_core_id;       /**< Pin core; MUST be ADSBIN_CORE_DSP in production. */
    uint8_t  task_priority;      /**< Demod task priority (high, below USB).          */
    uint32_t task_stack_size;    /**< Demod task stack bytes (0 => default).          */
    uint8_t  preamble_threshold; /**< 0..255 min correlation score to accept.         */
} demod1090_config_t;

/** @brief Running DSP counters (demod1090_get_stats). Atomically snapshotted. */
typedef struct {
    uint64_t samples_processed;
    uint64_t preambles_detected;
    uint64_t frames_emitted;
    uint64_t frames_56bit;
    uint64_t frames_112bit;
    uint64_t iq_blocks_consumed;
    uint64_t iq_blocks_dropped;  /**< IQ blocks skipped (could not keep up).      */
    uint64_t queue_overflows;    /**< Candidate frames dropped (out queue full).  */
    uint16_t last_signal_level;
} demod1090_stats_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Build the magnitude LUT + internal state and validate @p cfg.
 *  Does not start the task. @param cfg NULL => defaults. */
esp_err_t demod1090_init(const demod1090_config_t *cfg);

/**
 * @brief Create + start the Core-0 demod task.
 * @param iq_ring          Source IQ ring (from usb_rtlsdr_get_iq_ring()).
 * @param out_frame_queue  Destination queue of ::modes_frame_t (owned by main).
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if not initialized / already running.
 */
esp_err_t demod1090_start(RingbufHandle_t iq_ring, QueueHandle_t out_frame_queue);

/** @brief Signal the demod task to drain and exit, then join it. */
esp_err_t demod1090_stop(void);

/** @brief Free the LUT and internal state (stops the task first if running). */
void demod1090_deinit(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Diagnostics (safe from any core)
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Copy a snapshot of the DSP counters into @p out. */
void demod1090_get_stats(demod1090_stats_t *out);

/** @brief Zero all counters (e.g. at the start of a bench-test run). */
void demod1090_reset_stats(void);

#ifdef __cplusplus
}
#endif
