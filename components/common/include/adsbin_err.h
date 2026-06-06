/**
 * @file    adsbin_err.h
 * @brief   ADSBin error codes — an esp_err_t extension shared by all components.
 *
 * @details
 *   ADSBin uses ESP-IDF's ::esp_err_t everywhere so any IDF call's result flows
 *   straight through our APIs. This header adds an ADSBin-private range for the
 *   handful of domain errors that have no good IDF equivalent (no dongle, ring
 *   overflow, bad CRC, ...). The range is offset well clear of the IDF code
 *   space to avoid collisions.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ADSBin error type — an alias of ::esp_err_t (ESP_OK == 0). */
typedef esp_err_t adsbin_err_t;

/**
 * @brief Base of the ADSBin-private error range.
 *
 * Chosen above the standard component error spaces so our codes never alias an
 * ESP-IDF one. Keep ADSBin codes contiguous from here.
 */
#define ADSBIN_ERR_BASE  0x7000

/** @brief ADSBin domain-specific error codes. */
enum {
    ADSBIN_ERR_NO_DONGLE      = ADSBIN_ERR_BASE, /**< No RTL-SDR enumerated.     */
    ADSBIN_ERR_USB_STALL,                        /**< Bulk endpoint stalled.     */
    ADSBIN_ERR_RING_OVERFLOW,                    /**< IQ ring full; block dropped.*/
    ADSBIN_ERR_BAD_CRC,                          /**< Mode-S parity check failed. */
    ADSBIN_ERR_CPR_INCOMPLETE,                   /**< Need the even/odd partner.  */
    ADSBIN_ERR_NO_OWNSHIP,                        /**< Op needs a reference fix.  */
    ADSBIN_ERR_TABLE_FULL,                       /**< Traffic table at capacity.  */
    ADSBIN_ERR_SINK_FAIL,                        /**< Output sink write failed.   */
};

/**
 * @brief Map an ADSBin/ESP error code to a stable human-readable string.
 *
 * For ADSBin-private codes returns our own label; otherwise falls through to
 * esp_err_to_name(). Safe to call from any context; the returned pointer is to
 * static storage and must not be freed.
 *
 * @param e  An ::adsbin_err_t (or any ::esp_err_t).
 * @return Static, NUL-terminated description.
 */
const char *adsbin_err_to_str(adsbin_err_t e);

#ifdef __cplusplus
}
#endif
