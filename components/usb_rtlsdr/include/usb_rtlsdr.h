/**
 * @file    usb_rtlsdr.h
 * @brief   RTL-SDR USB-HS host driver — public contract (plan S4.1).
 *
 * @details
 *   Enumerates an RTL2832U + R820T2 dongle on the ESP32-P4's USB 2.0 High-Speed
 *   host port, configures the tuner for 1090ES (2.4 Msps, fixed max gain), and
 *   streams continuous bulk-IN IQ into an internally-owned no-split ring buffer.
 *
 *   HANDOFF MODEL (reconciled, plan S2): this driver OWNS the IQ ring. The USB
 *   completion path (Core 0) writes ::iq_block_t-described chunks into it;
 *   `demod1090` pulls from the same ring (also Core 0). `main` fetches the ring
 *   via usb_rtlsdr_get_iq_ring() and passes it to demod1090_start(). There is no
 *   per-block copy callback on the hot path — the ring IS the seam.
 *
 *   Each received ring item is an ::iq_block_t whose @c samples points at the
 *   bytes that immediately follow it in the same ring allocation; the consumer
 *   must release the item (vRingbufferReturnItem) after processing and must not
 *   retain @c samples (see the borrow rule in adsbin_types.h).
 *
 * @par Core affinity
 *   Core 0 (::ADSBIN_CORE_DSP). The USB host task + bulk-IN completions run
 *   pinned here so nothing competes with the ~4.8 MB/s ingest. set_* / get_* are
 *   internally serialized and safe to call from Core 1 while streaming.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "adsbin_types.h"   /* iq_block_t, ADSB_SAMPLE_RATE_HZ, ADSB_CENTER_FREQ_HZ */

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  Owned enums / types
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Tuner gain mode. Default is fixed/manual with AGC off (plan S5.3). */
typedef enum {
    USB_RTLSDR_GAIN_MANUAL_FIXED = 0, /**< Fixed gain, tuner+RTL AGC off (ADS-B). */
    USB_RTLSDR_GAIN_HW_AGC       = 1, /**< Hardware AGC on (noisy installs only). */
} usb_rtlsdr_gain_mode_t;

/** @brief Tuner silicon reported by the dongle. */
typedef enum {
    USB_RTLSDR_TUNER_UNKNOWN = 0,
    USB_RTLSDR_TUNER_R820T   = 1,
    USB_RTLSDR_TUNER_R820T2  = 2,   /**< Expected for the NESDR Nano 3.          */
    USB_RTLSDR_TUNER_FC0012  = 3,
    USB_RTLSDR_TUNER_FC0013  = 4,
    USB_RTLSDR_TUNER_E4000   = 5,
} usb_rtlsdr_tuner_type_t;

/** @brief Driver lifecycle / liveness state. */
typedef enum {
    USB_RTLSDR_STATE_UNINIT = 0,
    USB_RTLSDR_STATE_NO_DEVICE,   /**< Host up, no dongle enumerated.            */
    USB_RTLSDR_STATE_OPEN_IDLE,   /**< Device open, not streaming.               */
    USB_RTLSDR_STATE_STREAMING,   /**< Bulk-IN flowing.                          */
    USB_RTLSDR_STATE_RECOVERING,  /**< Stall/error; re-enumerating (plan S10).   */
    USB_RTLSDR_STATE_FAULT,       /**< Unrecoverable.                            */
} usb_rtlsdr_state_t;

/** @brief Async lifecycle events delivered to the event callback. */
typedef enum {
    USB_RTLSDR_EVENT_CONNECTED = 0,
    USB_RTLSDR_EVENT_DISCONNECTED,
    USB_RTLSDR_EVENT_STREAM_STARTED,
    USB_RTLSDR_EVENT_STREAM_STOPPED,
    USB_RTLSDR_EVENT_OVERFLOW,     /**< Ring full; block(s) dropped.             */
    USB_RTLSDR_EVENT_USB_STALL,
    USB_RTLSDR_EVENT_RECOVERED,
    USB_RTLSDR_EVENT_FAULT,
} usb_rtlsdr_event_id_t;

/**
 * @brief Event callback. Runs in the driver housekeeping task (NOT the hot
 *        path): may post to queues / do light work. Pass NULL to clear.
 *
 * @param event         Which lifecycle event fired.
 * @param device_index  The slot (0 or 1) the event pertains to. Slot 0 is the
 *                      1090 device, slot 1 the 978 device (by adoption order).
 * @param user_ctx      The opaque pointer registered alongside the callback.
 */
typedef void (*usb_rtlsdr_event_cb_t)(usb_rtlsdr_event_id_t event, int device_index, void *user_ctx);

/** @brief One-time install config (usb_rtlsdr_init). Zeroed fields take defaults. */
typedef struct {
    size_t ring_capacity_blocks;  /**< IQ ring depth in blocks (0 => default 16). */
    size_t block_size_iq_pairs;   /**< IQ pairs per delivered block (0 => default).*/
    int    usb_task_priority;     /**< USB host task priority (0 => default).     */
    int    usb_task_core_id;      /**< Core to pin to (default ADSBIN_CORE_DSP).  */
    bool   auto_recover;          /**< Re-enumerate on stall/disconnect (S10).    */
} usb_rtlsdr_config_t;

/** @brief Per-stream tuner config (usb_rtlsdr_start). Defaults target 1090ES. */
typedef struct {
    uint32_t               center_freq_hz;      /**< Default ADSB_CENTER_FREQ_HZ. */
    uint32_t               sample_rate_sps;     /**< Default ADSB_SAMPLE_RATE_HZ. */
    usb_rtlsdr_gain_mode_t gain_mode;           /**< Default MANUAL_FIXED.        */
    int                    gain_tenth_db;       /**< Default 496 (49.6 dB).       */
    int                    freq_correction_ppm; /**< Default 0 (plan S5.5).       */
    bool                   bias_tee_enable;     /**< Default false (S8 LNA opt-in).*/
    int                    device_index;        /**< LEGACY/IGNORED: the band is
                                                 *   now driven by the device's
                                                 *   role (1090/978), assigned by
                                                 *   stable adoption order, not by
                                                 *   this selector. Kept for ABI.  */
} usb_rtlsdr_stream_config_t;

/** @brief Opened-device identity (usb_rtlsdr_get_device_info). */
typedef struct {
    uint16_t                vid;
    uint16_t                pid;
    usb_rtlsdr_tuner_type_t tuner;
    char                    serial[64];        /**< EEPROM serial, NUL-term.      */
    char                    product_name[64];  /**< EEPROM product, NUL-term.     */
    bool                    has_bias_tee;      /**< Capability for set_bias_tee.  */
} usb_rtlsdr_device_info_t;

/** @brief Liveness snapshot (usb_rtlsdr_get_status). */
typedef struct {
    usb_rtlsdr_state_t state;
    bool               device_present;
    bool               streaming;
    esp_err_t          last_error;     /**< Last non-fatal error, ESP_OK if none. */
    int64_t            last_block_us;  /**< adsbin_now_us() of latest block (0=n/a).*/
} usb_rtlsdr_status_t;

/** @brief Throughput / health counters (usb_rtlsdr_get_stats). */
typedef struct {
    uint64_t total_bytes;     /**< Cumulative IQ bytes delivered.                */
    uint64_t total_blocks;    /**< Cumulative blocks delivered.                  */
    uint64_t overflow_drops;  /**< Blocks dropped (ring full).                   */
    uint32_t usb_stall_count; /**< Bulk endpoint stalls.                         */
    uint32_t reset_count;     /**< Re-enumeration/reset events.                  */
    uint32_t measured_sps;    /**< Most recent measured effective sample rate.   */
} usb_rtlsdr_stats_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Install the USB-HS host stack + driver and allocate the IQ ring.
 *  Does not start streaming. @param cfg NULL => all defaults. */
esp_err_t usb_rtlsdr_init(const usb_rtlsdr_config_t *cfg);

/** @brief Stop streaming, release transfers, close device, uninstall host stack. */
esp_err_t usb_rtlsdr_deinit(void);

/** @brief Open the dongle, apply @p stream_cfg, begin continuous bulk-IN.
 *  Idempotent if already streaming the same config. @param stream_cfg NULL => defaults.
 *
 *  Legacy single-dongle shim: latches a GLOBAL streaming intent (so a dongle that
 *  enumerates LATER still auto-starts) AND applies @p stream_cfg to device 0 (the
 *  1090 slot), exactly as the single-dongle build did. The stream_cfg.device_index
 *  field is ignored — the band follows the device's role. */
esp_err_t usb_rtlsdr_start(const usb_rtlsdr_stream_config_t *stream_cfg);

/** @brief Halt streaming but keep the device open + host stack installed.
 *  Legacy shim: clears the global intent and stops device 0. */
esp_err_t usb_rtlsdr_stop(void);

/** @brief Get the IQ ring this driver produces into; main hands it to demod1090.
 *  Returns DEVICE 0's ring (the 1090 slot); valid right after init.
 *  @return Ring handle (valid after init), or NULL if uninitialized. */
RingbufHandle_t usb_rtlsdr_get_iq_ring(void);

/** @brief Get the IQ ring of whichever in-use device currently holds @p role.
 *  Each slot owns a ring from init, but this only returns it when a device is
 *  actually adopted AND assigned that role (so the caller never feeds a demod
 *  from a slot with no dongle). @return The matching ring, or NULL if no in-use
 *  device holds @p role. */
RingbufHandle_t usb_rtlsdr_get_iq_ring_for_role(adsbin_rf_role_t role);

/** @brief Number of dongles currently adopted (in_use), 0..RTLSDR_MAX_DEVICES.
 *  Unlike usb_rtlsdr_count() (which scans the bus) this reflects ADOPTED slots. */
int usb_rtlsdr_active_count(void);

/** @brief The role (1090 / 978 / NONE) currently assigned to slot @p idx. */
adsbin_rf_role_t usb_rtlsdr_role_of(int idx);

/** @brief Pin the role of the FIRST adopted dongle (call before/after init).
 *  When set to anything other than ::ADSBIN_ROLE_NONE, the very first dongle the
 *  driver adopts is forced into this role instead of the default 1090 — so a lone
 *  stick can be commanded to serve 978 UAT. Subsequent dongles still take the
 *  next free role by adoption order. This is a SETTER, not a config-component
 *  dependency: the app calls it; the driver never reads the config component. */
void usb_rtlsdr_set_role_override(adsbin_rf_role_t role);

/* ---- Indexed variants of the start/status/identity calls (multi-device) ---- */

/** @brief Like usb_rtlsdr_start() but targets a specific slot @p idx. Stores
 *  @p cfg into that device and latches ITS streaming intent. */
esp_err_t usb_rtlsdr_start_index(int idx, const usb_rtlsdr_stream_config_t *cfg);

/** @brief Like usb_rtlsdr_get_status() but for a specific slot @p idx. */
esp_err_t usb_rtlsdr_get_status_index(int idx, usb_rtlsdr_status_t *out);

/** @brief Like usb_rtlsdr_get_device_info() but for a specific slot @p idx.
 *  @return ESP_ERR_NOT_FOUND if that slot is not in use. */
esp_err_t usb_rtlsdr_get_device_info_index(int idx, usb_rtlsdr_device_info_t *out);

/* ───────────────────────────────────────────────────────────────────────────
 *  Discovery / identity
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Number of supported dongles enumerated (feeds band auto-detect S0/S4.4).
 *  Scans the BUS (not adopted slots); see usb_rtlsdr_active_count() for adopted.
 *  @return 0/1/...; -1 on error before init. */
int usb_rtlsdr_count(void);

/** @brief Fill @p out_info with device 0's (the 1090 slot's) identity.
 *  @return ESP_ERR_NOT_FOUND if device 0 is not open. */
esp_err_t usb_rtlsdr_get_device_info(usb_rtlsdr_device_info_t *out_info);

/* ───────────────────────────────────────────────────────────────────────────
 *  Runtime tuning (safe from Core 1 while streaming; internally serialized)
 * ─────────────────────────────────────────────────────────────────────────── */

esp_err_t usb_rtlsdr_set_center_freq(uint32_t freq_hz);                 /**< Retune (e.g. 978).    */
esp_err_t usb_rtlsdr_set_sample_rate(uint32_t sample_rate_sps);         /**< Set sample rate.      */
esp_err_t usb_rtlsdr_set_tuner_gain(usb_rtlsdr_gain_mode_t mode, int gain_tenth_db); /**< Gain.    */
esp_err_t usb_rtlsdr_set_freq_correction(int ppm);                     /**< PPM trim (S5.5).      */
esp_err_t usb_rtlsdr_set_bias_tee(bool enable);                        /**< 4.5 V LNA feed (S8).  */

/* ───────────────────────────────────────────────────────────────────────────
 *  Health / stats / events
 * ─────────────────────────────────────────────────────────────────────────── */

esp_err_t usb_rtlsdr_get_status(usb_rtlsdr_status_t *out_status);      /**< Liveness snapshot.    */
esp_err_t usb_rtlsdr_get_stats(usb_rtlsdr_stats_t *out_stats);        /**< Throughput counters.  */
void      usb_rtlsdr_reset_stats(void);                               /**< Zero counters.        */

/** @brief Register the async lifecycle-event callback (NULL clears). */
esp_err_t usb_rtlsdr_register_event_callback(usb_rtlsdr_event_cb_t cb, void *user_ctx);

#ifdef __cplusplus
}
#endif
