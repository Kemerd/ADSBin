/**
 * @file    demod978.h
 * @brief   978 MHz UAT demodulator — public contract (plan §9 / weather phase).
 *
 * @details
 *   The Core-0 DSP front end for the SECOND band. It pulls the same 8-bit I/Q
 *   ::iq_block_t blocks usb_rtlsdr produces (from the dongle in the 978 role),
 *   runs an FM/FSK discriminator (UAT is binary CPFSK, not the PPM that 1090
 *   uses), recovers the bit stream, correlates the 36-bit UAT sync words, and on
 *   a hit hands the captured coded bytes to the Reed-Solomon layer (uat_fec). It
 *   emits two kinds of FEC-corrected output:
 *
 *     - UAT ADS-B frames  -> a queue of ::uat_frame_t (by value), consumed by
 *       uat_decode which turns them into ::adsb_msg_t for the shared traffic table.
 *     - UAT uplink frames -> a no-split ring of ::uat_uplink_t (by reference), the
 *       FIS-B weather carrier, relayed by the weather sink as GDL90 Uplink (0x07).
 *
 *   It does NO message parsing — that belongs to uat_decode — exactly mirroring
 *   how demod1090 stops at candidate frames and leaves decode to modes_decode.
 *
 * @par Core affinity
 *   Core 0 (::ADSBIN_CORE_DSP), same hard-real-time discipline as demod1090: the
 *   task MUST NOT block; on output backpressure it drops-with-counter. The FSK
 *   discriminator is cheaper per sample than the 1090 magnitude LUT (no 128 KiB
 *   table), so two demods can timeshare Core 0; if the budget is ever tight the
 *   978 task can move to Core 1 (UAT's FEC tolerates the added jitter).
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "adsbin_types.h"   /* iq_block_t, uat_frame_t, uat_uplink_t */

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Demodulator configuration (demod978_init). Zero fields take defaults. */
typedef struct {
    uint32_t sample_rate_hz;    /**< Expected IQ rate (default UAT_SAMPLE_RATE_HZ). */
    uint8_t  task_core_id;      /**< Pin core (default ADSBIN_CORE_DSP).            */
    uint8_t  task_priority;     /**< Task priority (high, below USB RX).            */
    uint32_t task_stack_size;   /**< Task stack bytes (0 => default).               */
    uint8_t  sync_max_errors;   /**< Max bit errors tolerated in the 36-bit sync.   */
} demod978_config_t;

/** @brief Running DSP counters (demod978_get_stats). */
typedef struct {
    uint64_t iq_blocks_consumed;   /**< IQ blocks pulled from the ring.            */
    uint64_t iq_blocks_dropped;    /**< IQ blocks skipped (could not keep up).     */
    uint64_t adsb_sync_hits;       /**< ADS-B sync-word matches.                   */
    uint64_t uplink_sync_hits;     /**< Uplink sync-word matches.                  */
    uint64_t frames_emitted;       /**< UAT ADS-B frames that decoded + enqueued.  */
    uint64_t uplinks_emitted;      /**< Uplink frames that decoded + enqueued.     */
    uint64_t rs_uncorrectable;     /**< Sync hits whose FEC was uncorrectable.     */
    uint64_t queue_overflows;      /**< Outputs dropped (queue/ring full).         */
    uint16_t last_signal_level;    /**< Most recent burst magnitude proxy.         */
} demod978_stats_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle (mirrors demod1090)
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Build internal state and validate @p cfg. Does not start the task.
 *  @param cfg NULL => defaults. */
esp_err_t demod978_init(const demod978_config_t *cfg);

/**
 * @brief Create + start the Core-0 demod task.
 * @param iq_ring          Source IQ ring (usb_rtlsdr 978-role ring).
 * @param out_frame_queue  Destination queue of ::uat_frame_t (owned by main).
 * @param out_uplink_ring  Destination no-split ring of ::uat_uplink_t (owned by main).
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if not initialized / already running.
 */
esp_err_t demod978_start(RingbufHandle_t iq_ring, QueueHandle_t out_frame_queue,
                         RingbufHandle_t out_uplink_ring);

/** @brief Signal the demod task to drain and exit, then join it. */
esp_err_t demod978_stop(void);

/** @brief Free internal state (stops the task first if running). */
void demod978_deinit(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Diagnostics (safe from any core)
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Copy a snapshot of the DSP counters into @p out. */
void demod978_get_stats(demod978_stats_t *out);

/** @brief Zero all counters. */
void demod978_reset_stats(void);

#ifdef __cplusplus
}
#endif
