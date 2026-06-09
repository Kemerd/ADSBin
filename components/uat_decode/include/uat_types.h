/**
 * @file    uat_types.h
 * @brief   UAT-decode-internal enums + the FIS-B uplink summary (component-local).
 *
 * @details
 *   These types are PUBLIC to consumers of uat_decode but are NOT part of the
 *   frozen cross-component ABI in adsbin_types.h — they describe UAT-specific
 *   details (decode result codes, a parsed uplink summary used only for logging /
 *   sink_debug). The decoded aircraft observation itself is emitted as the SHARED
 *   ::adsb_msg_t so UAT traffic merges into the same traffic table as 1090.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Result of a UAT decode call. */
typedef enum {
    UAT_OK = 0,                 /**< Decoded; output populated.                  */
    UAT_ERR_BAD_LEN,            /**< Payload length not a valid UAT frame size.  */
    UAT_ERR_NO_POSITION,        /**< Frame carried no usable state vector.       */
    UAT_ERR_BAD_UPLINK,         /**< Uplink header failed validation.            */
    UAT_ERR_NULL,               /**< NULL argument.                              */
} uat_result_t;

/**
 * @brief A light parsed summary of a FIS-B uplink frame (for logging / debug).
 *
 * @details
 *   ADSBin relays the raw uplink payload to the EFB as GDL90 Uplink Data (0x07)
 *   and does NOT re-encode FIS-B products, so this summary is informational only:
 *   it confirms the uplink header is well-formed and reports how much application
 *   data it carries. The weather sink uses the validation result (not this struct)
 *   to decide whether to relay; sink_debug can print the struct.
 */
typedef struct {
    bool     position_valid;    /**< Ground station position present + valid.    */
    double   station_lat_deg;   /**< Ground station latitude (if valid).         */
    double   station_lon_deg;   /**< Ground station longitude (if valid).        */
    uint16_t app_data_bytes;    /**< Bytes of FIS-B application data present.     */
    uint8_t  num_info_frames;   /**< Count of parsed FIS-B information frames.    */
} uat_uplink_summary_t;

#ifdef __cplusplus
}
#endif
