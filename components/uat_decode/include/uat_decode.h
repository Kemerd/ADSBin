/**
 * @file    uat_decode.h
 * @brief   UAT (978 MHz) message decoder — public contract.
 *
 * @details
 *   The 978-band analog of modes_decode. It takes a FEC-CORRECTED UAT payload
 *   (the bytes demod978 + uat_fec produced) and parses it:
 *
 *     - UAT ADS-B frames -> a shared ::adsb_msg_t (ICAO key, ABSOLUTE lat/lon with
 *       NO CPR pairing — UAT carries the position directly — plus altitude,
 *       velocity, callsign, emitter category). That ::adsb_msg_t then flows onto
 *       the SAME msg_queue -> traffic_ingest path as a 1090 message, so UAT
 *       traffic merges into the one traffic table with ZERO changes to traffic /
 *       sinks / the GDL90 traffic encoder.
 *
 *     - UAT uplink (FIS-B) frames -> validated, with an optional parsed summary.
 *       The raw payload is relayed verbatim by the weather sink as GDL90 0x07;
 *       uat_decode only confirms the header is well-formed so we never relay junk.
 *
 *   Clean-room from the public DO-282B UAT message format. The decoded
 *   ::adsb_msg_t marks its origin with downlink_format = 0 (UAT), which is a
 *   sink-tagging label only — the GDL90 traffic encoder ignores it.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "adsbin_types.h"   /* adsb_msg_t */
#include "uat_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Decoder configuration (uat_decode_init). Reserved for future tuning. */
typedef struct {
    uint8_t reserved;   /**< Placeholder; pass NULL for defaults.                */
} uat_decode_cfg_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle (uat_decode is stateless beyond init validation; mirrors the
 *  modes_decode shape so main wires it identically).
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Initialise the decoder. @param cfg NULL => defaults. */
esp_err_t uat_decode_init(const uat_decode_cfg_t *cfg);

/** @brief Tear down the decoder. */
void uat_decode_deinit(void);

/* ───────────────────────────────────────────────────────────────────────────
 *  Decode
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Parse a FEC-corrected UAT ADS-B payload into a shared ::adsb_msg_t.
 *
 * @param payload   18 (basic) or 34 (long) corrected payload bytes.
 * @param len       18 or 34.
 * @param rx_time_us  Capture time (carried into adsb_msg_t.rx_time_us).
 * @param out_msg   Receives the decoded observation on UAT_OK.
 * @return UAT_OK, or a uat_result_t error.
 */
uat_result_t uat_decode_adsb(const uint8_t *payload, size_t len,
                             int64_t rx_time_us, adsb_msg_t *out_msg);

/**
 * @brief Validate a FEC-corrected UAT uplink (FIS-B) payload for relaying.
 *
 * @param payload      432 corrected uplink payload bytes.
 * @param len          432.
 * @param rx_time_us   Capture time (unused today; kept for symmetry/future ToR).
 * @param out_summary  Optional; receives a light parsed summary on UAT_OK.
 * @return UAT_OK if the header validates (safe to relay), else a uat_result_t.
 */
uat_result_t uat_decode_uplink(const uint8_t *payload, size_t len,
                               int64_t rx_time_us,
                               uat_uplink_summary_t *out_summary);

/* ───────────────────────────────────────────────────────────────────────────
 *  Granular helpers — exposed for host unit tests (and reuse). Each operates on
 *  the full corrected payload and fills the relevant part of @p out_msg.
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Decode the HDR: ICAO address + address qualifier into @p out_msg. */
uat_result_t uat_decode_address(const uint8_t *payload, adsb_msg_t *out_msg);

/** @brief Decode the STATE VECTOR: position, altitude, velocity into @p out_msg. */
uat_result_t uat_decode_state_vector(const uint8_t *payload, adsb_msg_t *out_msg);

/** @brief Decode the MODE STATUS: callsign + emitter category into @p out_msg. */
uat_result_t uat_decode_mode_status(const uint8_t *payload, size_t len,
                                    adsb_msg_t *out_msg);

#ifdef __cplusplus
}
#endif
