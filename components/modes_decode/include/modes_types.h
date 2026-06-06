/**
 * @file    modes_types.h
 * @brief   modes_decode owned types: config, results, message classification.
 *
 * @details
 *   These types are OWNED by `modes_decode` (not shared) — they describe the
 *   decoder's own configuration and the status of a decode attempt. The
 *   cross-cutting product of decoding (::adsb_msg_t) lives in `common`.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Decoder configuration (modes_decode_init). */
typedef struct {
    uint32_t max_cpr_pair_age_us;  /**< Max age of the opposite-parity CPR frame
                                        for a global pair (default 10 s per
                                        dump1090, in microseconds).             */
    bool     enable_local_cpr;     /**< Allow single-message local decode when a
                                        valid ownship reference exists.         */
    uint16_t cpr_cache_slots;      /**< Per-ICAO pairing-cache size (default 256).*/
} modes_decode_cfg_t;

/** @brief Result of a decode attempt (also used by the granular helpers). */
typedef enum {
    MODES_OK = 0,
    MODES_ERR_BAD_LEN,         /**< Frame length not 7 or 14.                   */
    MODES_ERR_BAD_CRC,         /**< 24-bit parity check failed.                 */
    MODES_ERR_UNSUPPORTED_DF,  /**< Not DF17/DF18.                              */
    MODES_ERR_UNSUPPORTED_TC,  /**< Type code we do not decode yet.             */
    MODES_ERR_NO_POSITION,     /**< Position needs its CPR partner (incomplete).*/
    MODES_ERR_CPR_REJECT,      /**< CPR math rejected (NL mismatch / range).    */
    MODES_ERR_STALE_PAIR,      /**< Opposite-parity frame too old to pair.      */
    MODES_ERR_NULL_ARG,
} modes_result_t;

/** @brief CRC check outcome (modes_decode_check_crc). */
typedef enum {
    MODES_CRC_OK = 0,
    MODES_CRC_FAIL,
    MODES_CRC_BAD_LEN,
} modes_crc_t;

/** @brief Downlink-format classification (modes_decode_df). */
typedef enum {
    MODES_DF_ADSB        = 17,  /**< 1090ES extended squitter.                  */
    MODES_DF_TISB        = 18,  /**< TIS-B / non-direct ADS-B.                  */
    MODES_DF_UNSUPPORTED = -1,
} modes_df_t;

/** @brief High-level message kind from the ADS-B type code. */
typedef enum {
    MODES_MSG_UNKNOWN = 0,
    MODES_MSG_IDENT,         /**< TC 1-4 identification + category.            */
    MODES_MSG_SURFACE_POS,   /**< TC 5-8 surface position.                     */
    MODES_MSG_AIRBORNE_POS,  /**< TC 9-18 airborne position (CPR).             */
    MODES_MSG_AIRBORNE_VEL,  /**< TC 19 airborne velocity.                     */
    MODES_MSG_OPSTATUS,      /**< TC 31 operational status (NIC/NACp source).  */
} modes_msg_kind_t;

/** @brief Intermediate velocity result (folded into adsb_msg_t by the caller). */
typedef struct {
    uint16_t ground_speed_kt;
    uint16_t track_deg;        /**< 0..359 true.                               */
    int16_t  vertical_rate_fpm;/**< + = climb.                                 */
    bool     gnss_baro_alt;    /**< Vertical rate referenced to GNSS vs baro.  */
} adsb_velocity_t;

/** @brief Cumulative decode counters (modes_decode_get_stats). */
typedef struct {
    uint32_t frames_in;
    uint32_t crc_ok;
    uint32_t crc_fail;
    uint32_t df_dropped;        /**< Dropped at the DF gate.                    */
    uint32_t positions_global;  /**< Resolved via global even/odd pairing.      */
    uint32_t positions_local;   /**< Resolved via local (reference) decode.     */
    uint32_t cpr_rejects;
} modes_decode_stats_t;

#ifdef __cplusplus
}
#endif
