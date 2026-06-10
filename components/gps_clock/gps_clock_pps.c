/**
 * @file    gps_clock_pps.c
 * @brief   Layer 3 — 1PPS hardware capture + PI clock discipline (signal producer).
 *
 * @details
 *   Hardware-captures the MAX-M10S TIMEPULSE rising edge with GPTimer + ETM (zero
 *   ISR on any core) and a PCNT edge counter, then runs a phase-pin + integral
 *   drift PI loop to discipline UTC against the edge. Like the NMEA parser, this
 *   is a PURE SIGNAL PRODUCER: it fills ::gps_pps_signals_t and never decides
 *   clock quality — the supervisor owns the ladder.
 *
 *   NOTE: This file currently provides the inert (no-PPS) behaviour so the
 *   component links and the NMEA-fix ladder works end-to-end. The full GPTimer/
 *   ETM/PCNT capture + PI filter + rigor guards are layered in by Task #7; the
 *   public producer seam (gps_pps_init / gps_pps_tick) does not change.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "gps_clock_signals.h"

#include <string.h>
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "gps_pps";

/** @brief Whether a PPS pin was successfully brought up (false ⇒ inert). */
static bool s_pps_active;

esp_err_t gps_pps_init(int pps_gpio)
{
    // Task #7 brings up the GPTimer@80MHz + ETM capture + PCNT here. Until then we
    // record the request but stay inert, so the ladder tops out at NMEA_FIX.
    (void)pps_gpio;
    s_pps_active = false;
    ESP_LOGI(TAG, "PPS layer present but not yet engaged (NMEA-fix timing)");
    return ESP_OK;
}

void gps_pps_tick(gps_pps_signals_t *sig, const gps_nmea_signals_t *nmea)
{
    (void)nmea;
    // Inert: report "no usable edge" every tick. The supervisor reads present=false
    // and simply never promotes above NMEA_FIX. Zeroing keeps the seqlock UTC map
    // coherent (the supervisor only consults these once L3 is engaged).
    if (sig != NULL) {
        memset(sig, 0, sizeof(*sig));
    }
    (void)s_pps_active;
}
