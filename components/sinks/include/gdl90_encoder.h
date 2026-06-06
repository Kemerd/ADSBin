/**
 * @file    gdl90_encoder.h
 * @brief   Pure GDL90 message framing — no I/O, host-unit-testable (plan S3, S4.5).
 *
 * @details
 *   Encodes GDL90 messages per the Garmin "GDL 90 Data Interface Specification"
 *   and the ForeFlight GDL90 Extended Spec. These functions are PURE (buffer in,
 *   buffer out) so the Python bench harness and host tests can validate them
 *   byte-for-byte without firmware.
 *
 *   FRAMING CONTRACT (must match tools/bench exactly):
 *     - Each frame is: 0x7E flag | message-id + payload | CRC-16 (LE) | 0x7E flag.
 *     - Byte-stuffing escapes 0x7E and 0x7D as 0x7D, (byte XOR 0x20) over
 *       id+payload+CRC, NEVER over the flags.
 *     - gdl90_crc16() is computed over the UNstuffed id+payload (the caller
 *       passes the pre-stuff buffer).
 *     - frame_* return total framed length on success, or a NEGATIVE esp_err
 *       (e.g. -ESP_ERR_INVALID_SIZE) on out_cap overflow.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "adsbin_types.h"   /* traffic_target_t */

#ifdef __cplusplus
extern "C" {
#endif

/** @brief GDL90 Heartbeat (msg id 0x00) fields. */
typedef struct {
    bool     gps_pos_valid;       /**< Status byte 1, bit 7.                     */
    bool     maint_required;      /**< Status byte 2.                           */
    uint32_t timestamp_s;         /**< Seconds since UTC midnight (0..86399).    */
    uint16_t msg_count_uplink;
    uint16_t msg_count_basic_long;
} gdl90_heartbeat_t;

/**
 * @brief GDL90 Traffic/Ownship Report payload (msg id 0x14 / 0x0A share layout).
 *
 * Fields are in the encoder's input units; the encoder performs the GDL90 bit-
 * packing (24-bit semicircle lat/lon, 12-bit pressure altitude, etc.).
 */
typedef struct {
    uint8_t  alert_status;   /**< 0 = no alert, 1 = alert.                       */
    uint8_t  addr_type;      /**< 0 = ADS-B with ICAO address, ...              */
    uint32_t icao;           /**< 24-bit participant address.                   */
    double   lat_deg;        /**< WGS-84 latitude  (encoder packs to semicircle).*/
    double   lon_deg;        /**< WGS-84 longitude (encoder packs to semicircle).*/
    int32_t  alt_press_ft;   /**< Pressure altitude ft; INT32_MIN = invalid.    */
    bool     airborne;       /**< Airborne (vs on-ground) misc bit.             */
    uint8_t  nic;            /**< Navigation Integrity Category.                */
    uint8_t  nacp;           /**< Nav Accuracy Category - Position.             */
    uint16_t h_velocity_kt;  /**< Horizontal velocity, knots (12-bit).          */
    int16_t  v_velocity_fpm; /**< Vertical velocity, ft/min (encoder -> 64 fpm). */
    uint16_t track_heading;  /**< Track/heading, degrees (encoder -> 360/256).  */
    uint8_t  emitter_cat;    /**< GDL90 emitter category.                       */
    char     callsign[ADSB_CALLSIGN_LEN]; /**< 8 chars + NUL.                   */
    uint8_t  emergency_code; /**< Priority / emergency status.                  */
} gdl90_traffic_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Encoders. Each returns framed length, or a negative esp_err on overflow.
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Encode a Heartbeat (0x00) into @p out (framed, CRC, stuffed). */
int gdl90_frame_heartbeat(uint8_t *out, size_t out_cap, const gdl90_heartbeat_t *hb);

/** @brief Encode a Traffic Report (0x14) into @p out. */
int gdl90_frame_traffic_report(uint8_t *out, size_t out_cap, const gdl90_traffic_t *tr);

/** @brief Encode an Ownship Report (0x0A), sharing the Traffic payload layout. */
int gdl90_frame_ownship_report(uint8_t *out, size_t out_cap, const gdl90_traffic_t *own);

/* ───────────────────────────────────────────────────────────────────────────
 *  Helpers (also exposed for the bench harness + unit tests)
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Map a shared ::traffic_target_t into GDL90 wire fields. */
void gdl90_traffic_from_target(gdl90_traffic_t *out, const traffic_target_t *tgt);

/** @brief GDL90 CRC-16 (CCITT table variant) over an UNframed id+payload. */
uint16_t gdl90_crc16(const uint8_t *payload, size_t len);

#ifdef __cplusplus
}
#endif
