/**
 * @file    gps_ubx.c
 * @brief   One-shot UBX-CFG-VALSET boot configuration over the optional TX wire.
 *
 * @details
 *   When the config wire (P4 TX → module RX) is present, ADSBin sends a single
 *   UBX-CFG-VALSET burst at boot to put the MAX-M10S into a known-good mode:
 *   enable the 1PPS timepulse, set a 1 Hz nav rate, and trim the NMEA output to
 *   just GGA + RMC (the two sentences the parser needs). The module's
 *   configuration-database (VALSET) interface is the M10 way — legacy CFG-* poll
 *   messages are not used.
 *
 *   NOTE: This file currently provides the no-op seam so the component links and
 *   the default (no TX wire) path works. The actual UBX-CFG-VALSET frames are
 *   filled in by Task #6; the seam (gps_ubx_configure) does not change.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "gps_ubx";

/**
 * @brief Send the boot-time UBX-CFG-VALSET burst on @p uart_num (no-op stub).
 *
 * @param uart_num  The installed GPS UART controller.
 * @return ESP_OK.
 */
esp_err_t gps_ubx_configure(int uart_num)
{
    (void)uart_num;
    // Task #6: emit UBX-CFG-VALSET (CFG-TP-*, CFG-RATE-MEAS, CFG-MSGOUT-NMEA-*).
    ESP_LOGI(TAG, "UBX boot config not yet enabled; using module NMEA defaults");
    return ESP_OK;
}
