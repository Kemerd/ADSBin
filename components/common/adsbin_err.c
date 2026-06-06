/**
 * @file    adsbin_err.c
 * @brief   Implementation of adsbin_err_to_str() — the one .c in `common`.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "adsbin_err.h"

const char *adsbin_err_to_str(adsbin_err_t e)
{
    // ADSBin-private range first; anything else defers to the IDF table so a
    // wrapped IDF error (e.g. ESP_ERR_TIMEOUT) still prints meaningfully.
    switch (e) {
        case ADSBIN_ERR_NO_DONGLE:     return "ADSBIN_ERR_NO_DONGLE";
        case ADSBIN_ERR_USB_STALL:     return "ADSBIN_ERR_USB_STALL";
        case ADSBIN_ERR_RING_OVERFLOW: return "ADSBIN_ERR_RING_OVERFLOW";
        case ADSBIN_ERR_BAD_CRC:       return "ADSBIN_ERR_BAD_CRC";
        case ADSBIN_ERR_CPR_INCOMPLETE:return "ADSBIN_ERR_CPR_INCOMPLETE";
        case ADSBIN_ERR_NO_OWNSHIP:    return "ADSBIN_ERR_NO_OWNSHIP";
        case ADSBIN_ERR_TABLE_FULL:    return "ADSBIN_ERR_TABLE_FULL";
        case ADSBIN_ERR_SINK_FAIL:     return "ADSBIN_ERR_SINK_FAIL";
        default:                       return esp_err_to_name(e);
    }
}
