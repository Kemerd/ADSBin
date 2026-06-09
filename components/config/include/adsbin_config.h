/**
 * @file    adsbin_config.h
 * @brief   ADSBin persistent settings store (NVS-backed) — public contract.
 *
 * @details
 *   Single source of truth for all operator-tunable settings, persisted in
 *   NVS (plan §4.7). Mirrors LidarAGL's NVS philosophy: load defaults at boot,
 *   allow override via serial command / hold-at-boot, commit back to flash.
 *
 *   Settings covered: tuner gain, manual reference position, active output
 *   sink(s), range/altitude cull filters, and the band map (which RF bands are
 *   enabled — feeds the "build superset, sell subsets" auto-tier goal, §0).
 *
 *   Access model:
 *     - At boot, call config_init() once. It opens NVS and loads the live
 *       config (falling back to compiled defaults for any missing key).
 *     - Readers call config_get() for a coherent snapshot, or the typed
 *       convenience getters for single fields. Reads are cheap (RAM copy).
 *     - Writers mutate via the typed setters (RAM only), then call
 *       config_commit() to flush dirty keys to NVS atomically.
 *
 * @par Core affinity (plan §2)
 *   CORE 1 (config / housekeeping). Accessors are internally synchronized and
 *   safe from any task; commits perform flash I/O and must NOT be called from
 *   the Core-0 hard-real-time DSP path.
 *
 * @note  ::adsbin_config_t and its sub-types are OWNED here. The ownship
 *        component consumes ::ownship_ref_t (defined by `ownship`), not this
 *        struct; this header only stores the scalar lat/lon it persists.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  Compile-time defaults (overridable via menuconfig later; concrete here so
 *  implementers and the bench harness agree on out-of-box behaviour)
 * ─────────────────────────────────────────────────────────────────────────── */

#define ADSBIN_CFG_NVS_NAMESPACE        "adsbin"     /**< NVS namespace.        */
#define ADSBIN_CFG_GAIN_AUTO            (-1)         /**< Sentinel: AGC/auto.   */
#define ADSBIN_CFG_DEFAULT_GAIN_TENTHDB (496)        /**< 49.6 dB, AGC off §5.3 */
#define ADSBIN_CFG_RANGE_DISABLED       (0.0f)       /**< 0 ⇒ no range cull.    */
#define ADSBIN_CFG_ALT_DISABLED         (0)          /**< 0 ⇒ no altitude cull. */

/* ───────────────────────────────────────────────────────────────────────────
 *  Types OWNED & exposed by this component
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Bitmask of RF bands the firmware should service.
 *
 * Auto-tier hook: at boot the band map is intersected with detected hardware
 * (dongle count from usb_rtlsdr) to decide which decode paths to spin up.
 */
typedef enum {
    ADSBIN_BAND_NONE = 0,
    ADSBIN_BAND_1090 = (1u << 0),   /**< 1090 MHz / 1090ES (MVP).             */
    ADSBIN_BAND_978  = (1u << 1),   /**< 978 MHz UAT (future, plan §9).       */
} adsbin_band_t;

/**
 * @brief Bitmask of output sinks to enable (plan §4.5).
 *
 * Multiple sinks may run concurrently (e.g. debug + gdl90 over USB-CDC).
 */
typedef enum {
    ADSBIN_SINK_NONE  = 0,
    ADSBIN_SINK_DEBUG = (1u << 0),  /**< Human-readable table → USB-CDC.      */
    ADSBIN_SINK_GDL90 = (1u << 1),  /**< GDL90 frames → USB-CDC (MVP).        */
    ADSBIN_SINK_WIFI  = (1u << 2),  /**< (Future) GDL90 over WiFi/UDP, §10.   */
    ADSBIN_SINK_TIS   = (1u << 3),  /**< (Future) Garmin TIS-A / RS-232, §11. */
} adsbin_sink_t;

/**
 * @brief The complete persisted configuration snapshot (OWNED here).
 *
 * @details
 *   Plain-old-data so it can be memcpy'd to/from callers and (de)serialized to
 *   NVS field-by-field. Gain is in tenths-of-dB to stay integer-exact with the
 *   R820T2 gain table; ::ADSBIN_CFG_GAIN_AUTO selects tuner AGC.
 */
typedef struct {
    /* ── Tuner / RF (consumed by usb_rtlsdr) ─────────────────────────────── */
    int32_t  tuner_gain_tenth_db;   /**< Fixed gain ×10 dB, or GAIN_AUTO.     */
    uint32_t band_map;              /**< ::adsbin_band_t bitmask of bands.    */

    /* ── Single-dongle role override (consumed by usb_rtlsdr at adopt) ────── */
    /* Forces the FIRST-adopted dongle's RF role so a LONE dongle can be tested
     * as 978 weather before a second stick exists (staged bring-up). Values are
     * ::adsbin_rf_role_t (0 = auto/count-based, 1 = force 1090, 2 = force 978).
     * Ignored once two dongles are present (count-based assignment wins). */
    uint8_t  role_override;         /**< 0=auto, 1=1090, 2=978 (adsbin_rf_role_t).*/

    /* ── Ownship manual reference (seeds the `ownship` component) ─────────── */
    bool     ref_valid;             /**< True if a manual lat/lon is stored.  */
    double   ref_lat_deg;           /**< Manual reference latitude  (WGS-84). */
    double   ref_lon_deg;           /**< Manual reference longitude (WGS-84). */

    /* ── Traffic culling filters (consumed by traffic) ───────────────────── */
    float    range_filter_m;        /**< Max horizontal range; 0 ⇒ disabled.  */
    int32_t  alt_filter_ft;         /**< Max altitude (ft MSL); 0 ⇒ disabled. */
    uint32_t target_expiry_s;       /**< Drop targets unheard for N seconds.  */
    uint32_t max_targets;           /**< Hard cap on traffic table size.      */

    /* ── Output (consumed by sinks) ──────────────────────────────────────── */
    uint32_t sink_map;              /**< ::adsbin_sink_t bitmask of sinks.    */
} adsbin_config_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialize NVS and load the live configuration.
 *
 * Initializes the NVS subsystem if needed, opens the ADSBin namespace, and
 * loads every key into the in-RAM config — substituting compiled defaults for
 * any key not yet present in flash. Idempotent.
 *
 * @return ESP_OK on success; an esp_err_t from the NVS layer otherwise.
 */
esp_err_t config_init(void);

/**
 * @brief Reset the in-RAM config to compiled defaults (does NOT auto-commit).
 *
 * Call config_commit() afterwards to persist the factory reset.
 *
 * @return ESP_OK.
 */
esp_err_t config_reset_defaults(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Bulk access
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Copy a coherent snapshot of the entire live configuration.
 * @param out_cfg  Non-NULL destination struct.
 * @return ESP_OK; ESP_ERR_INVALID_ARG if @p out_cfg is NULL.
 */
esp_err_t config_get(adsbin_config_t *out_cfg);

/**
 * @brief Replace the entire live configuration (RAM only; commit to persist).
 * @param cfg  Non-NULL, validated source struct.
 * @return ESP_OK; ESP_ERR_INVALID_ARG on NULL or out-of-range fields.
 */
esp_err_t config_set(const adsbin_config_t *cfg);

/**
 * @brief Flush all dirty keys to NVS atomically.
 *
 * Performs flash I/O — Core-1 only; never call from the DSP path.
 *
 * @return ESP_OK; an esp_err_t from nvs_commit() on failure.
 */
esp_err_t config_commit(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Typed convenience getters/setters (RAM only; pair with config_commit)
 * ─────────────────────────────────────────────────────────────────────────── */

int32_t  config_get_tuner_gain(void);                       /**< ×10 dB or AUTO.   */
esp_err_t config_set_tuner_gain(int32_t gain_tenth_db);     /**< Set tuner gain.   */

uint32_t config_get_band_map(void);                         /**< Enabled bands.    */
esp_err_t config_set_band_map(uint32_t band_map);           /**< Set band bitmask. */

uint8_t  config_get_role_override(void);                    /**< 0=auto/1=1090/2=978.*/
esp_err_t config_set_role_override(uint8_t role);           /**< Single-dongle role. */

uint32_t config_get_sink_map(void);                         /**< Enabled sinks.    */
esp_err_t config_set_sink_map(uint32_t sink_map);           /**< Set sink bitmask. */

/**
 * @brief Read the stored manual reference position.
 * @param out_lat   Non-NULL latitude out  (untouched if !valid).
 * @param out_lon   Non-NULL longitude out (untouched if !valid).
 * @return true if a valid manual reference is stored.
 */
bool config_get_ref_position(double *out_lat, double *out_lon);

/**
 * @brief Store (or clear) the manual reference position in RAM.
 * @param lat    WGS-84 latitude  [-90, 90].
 * @param lon    WGS-84 longitude [-180, 180].
 * @param valid  False clears the stored reference (lat/lon ignored).
 * @return ESP_OK; ESP_ERR_INVALID_ARG if valid && coords out of range.
 */
esp_err_t config_set_ref_position(double lat, double lon, bool valid);

float    config_get_range_filter_m(void);                   /**< 0 ⇒ disabled.     */
esp_err_t config_set_range_filter_m(float range_m);         /**< Set range cull.   */

int32_t  config_get_alt_filter_ft(void);                    /**< 0 ⇒ disabled.     */
esp_err_t config_set_alt_filter_ft(int32_t alt_ft);         /**< Set altitude cull.*/

#ifdef __cplusplus
}
#endif
