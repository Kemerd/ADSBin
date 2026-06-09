/**
 * @file    wifi_link.h
 * @brief   Bring up the on-board ESP32-C6 as an OPEN Wi-Fi SoftAP (no password).
 *
 * @details
 *   ADSBin streams GDL90 to ForeFlight over Wi-Fi (plan S10). On the Waveshare
 *   ESP32-P4-WIFI6 the P4 has no radio of its own - the Wi-Fi PHY lives on the
 *   companion ESP32-C6, reached over SDIO. With the esp_wifi_remote "hosted"
 *   backend enabled in sdkconfig, the ordinary esp_wifi.h calls used here are
 *   transparently RPC'd to the C6, so this file is plain, board-agnostic
 *   esp_wifi usage: it neither knows nor cares that the radio is remote.
 *
 *   Stands up a single open access point (::WIFI_AUTH_OPEN, no PSK) so a tablet
 *   running ForeFlight can join and receive the UDP GDL90 broadcast (the UDP
 *   transport is a separate unit). The AP carries no traffic of its own - it is
 *   purely the L2 fabric for the broadcast.
 *
 *   Ownership: this unit owns the Wi-Fi/netif/event bring-up and tolerates a
 *   system where NVS / the default event loop were already started elsewhere
 *   (config_init() brings up NVS at boot), so it can be called after the rest of
 *   the firmware is alive without double-init faults.
 *
 * @par Core affinity (plan S2)
 *   CORE 1 (housekeeping / connectivity). One-shot at start; the Wi-Fi driver
 *   runs its own tasks. Nothing here sits on the Core-0 DSP path.
 *
 * @note  SSID / channel / client-cap defaults come from Kconfig
 *        (CONFIG_ADSBIN_AP_SSID, _CHANNEL, _MAX_CLIENTS), but the API takes an
 *        explicit ::wifi_link_ap_cfg_t so main remains the single point that
 *        decides the live values.
 *
 * @note  wifi_link_start_ap() is a boot singleton: call it exactly once. A
 *        second call is a safe no-op - an internal guard returns ESP_OK without
 *        re-initialising the Wi-Fi driver.
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

/**
 * @brief Open SoftAP parameters.
 *
 * @details
 *   Plain-old-data, copied by value into the driver's wifi_config_t. @p ssid is
 *   borrowed only for the duration of wifi_link_start_ap() (its bytes are copied
 *   into the driver config), so the caller may pass a Kconfig string literal.
 */
typedef struct {
    const char *ssid;          /**< AP network name. NULL/empty => Kconfig default. */
    uint8_t     channel;       /**< 2.4 GHz channel 1..13; 0 => driver auto-select. */
    uint8_t     max_clients;   /**< Max associated stations (driver hard caps this).*/
} wifi_link_ap_cfg_t;

/**
 * @brief Start the open Wi-Fi SoftAP on the C6 and return once it is running.
 *
 * @details
 *   Boot singleton: call exactly once. Tolerates NVS and the default event loop
 *   already being initialized (config_init() does NVS at boot). A second call
 *   returns ESP_OK without re-initialising the driver. On success the AP is
 *   beaconing and accepting associations; ::wifi_link_is_up() returns true.
 *
 * @param cfg  Non-NULL SoftAP parameters. A NULL/empty @p cfg->ssid falls back
 *             to CONFIG_ADSBIN_AP_SSID; a zero @p cfg->channel lets the driver
 *             auto-pick; a zero @p cfg->max_clients falls back to the Kconfig cap.
 *
 * @return ESP_OK once the AP is up; ESP_ERR_INVALID_ARG if @p cfg is NULL; an
 *         esp_err_t from the esp_netif / esp_wifi layer otherwise.
 */
esp_err_t wifi_link_start_ap(const wifi_link_ap_cfg_t *cfg);

/**
 * @brief Report whether the SoftAP is currently up (esp_wifi_start() succeeded).
 *
 * @return true after a successful wifi_link_start_ap(); false otherwise.
 */
bool wifi_link_is_up(void);

#ifdef __cplusplus
}
#endif
