/**
 * @file    gps_ubx.h
 * @brief   INTERNAL UBX-CFG-VALSET boot-config seam (component-private).
 *
 * @details
 *   Declares the one entry point the supervisor calls after installing the GPS
 *   UART, when the optional TX (config) wire is present. Kept private to the
 *   gps_clock component (next to the .c files, not under include/).
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send the boot-time UBX-CFG-VALSET burst on @p uart_num.
 *
 * @details
 *   Configures a MAX-M10S over its configuration-database (VALSET) interface:
 *   enable the 1PPS timepulse (locked to GNSS time), set a 1 Hz measurement/nav
 *   rate, and enable GGA + RMC NMEA on the UART while disabling the sentences the
 *   parser ignores (GLL/GSA/GSV/VTG). Applied to RAM only (not battery-backed
 *   flash), so a power-cycle returns the module to its factory defaults — the
 *   firmware re-applies on every boot. Best-effort: a write error is returned but
 *   the caller treats it as non-fatal (the module still streams default NMEA).
 *
 * @param uart_num  The already-installed GPS UART controller.
 * @return ESP_OK on success; an esp_err_t from the UART write path otherwise.
 */
esp_err_t gps_ubx_configure(int uart_num);

#ifdef __cplusplus
}
#endif
