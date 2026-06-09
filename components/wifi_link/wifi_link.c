/**
 * @file    wifi_link.c
 * @brief   Open Wi-Fi SoftAP bring-up on the companion ESP32-C6 (plan S10).
 *
 * @details
 *   The textbook ESP-IDF SoftAP sequence - NVS, netif, event loop,
 *   esp_wifi_init/set_mode/set_config/start - with two concessions to the fact
 *   that ADSBin has already booted much of the firmware before connectivity:
 *     1. NVS may already be up (config_init() does it at boot). We run the
 *        standard erase-on-bad-version retry but also swallow
 *        ESP_ERR_INVALID_STATE so we never fault on it.
 *     2. The default event loop may already exist;
 *        esp_event_loop_create_default() returns ESP_ERR_INVALID_STATE there,
 *        which we treat as success.
 *
 *   The radio lives on the C6 over SDIO via the esp_wifi_remote "hosted" backend
 *   (a sdkconfig concern). That backend RPC-forwards the esp_wifi.h surface, so
 *   every call below is ordinary esp_wifi usage - no C6-specific code. On the
 *   hosted build the esp_wifi_* symbols and WIFI_INIT_CONFIG_DEFAULT() come from
 *   the esp_wifi_remote shim, not the in-core netif stubs; the surface is
 *   identical by design.
 *
 * @par Core affinity (plan S2)
 *   CORE 1. One-shot bring-up from the housekeeping path; the Wi-Fi stack then
 *   runs on its own driver tasks. Nothing here touches the Core-0 DSP loop.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "wifi_link.h"

#include <string.h>

#include "sdkconfig.h"        /* CONFIG_ADSBIN_AP_* defaults.                     */
#include "esp_log.h"          /* Diagnostic logging (Core-1, non-hot-path).      */
#include "esp_err.h"
#include "esp_mac.h"          /* MACSTR / MAC2STR for the join/leave diagnostics. */
#include "esp_event.h"        /* Default event loop + WIFI_EVENT handler.        */
#include "esp_netif.h"        /* esp_netif_init / create_default_wifi_ap.        */
#include "esp_wifi.h"         /* esp_wifi_init / set_mode / set_config / start.  */
#include "nvs_flash.h"        /* nvs_flash_init / erase recovery (matches config).*/

/* Kconfig-backed fallbacks. The API always wins; these only fill blanks so a
 * caller can pass a partly-zeroed cfg and still get a sane AP. */
#ifndef CONFIG_ADSBIN_AP_SSID
#define CONFIG_ADSBIN_AP_SSID         "ADSBin"
#endif
#ifndef CONFIG_ADSBIN_AP_CHANNEL
#define CONFIG_ADSBIN_AP_CHANNEL      6
#endif
#ifndef CONFIG_ADSBIN_AP_MAX_CLIENTS
#define CONFIG_ADSBIN_AP_MAX_CLIENTS  4
#endif

static const char *TAG = "wifi_link";

/* Module state. s_ap_up is written from the bring-up task AND the Wi-Fi
 * event-loop task and read by wifi_link_is_up() from a third context, so it is
 * volatile (the bool store/load is naturally atomic on this target). s_started
 * is the one-shot guard: wifi_link_start_ap() is a boot singleton. */
static volatile bool s_ap_up   = false;
static volatile bool s_started = false;

/* Diagnostics: log station join/leave so a field tester can confirm the tablet
 * actually associated. Also assert the up flag on the first association. */
static void wifi_ap_event_handler(void *arg, esp_event_base_t base,
                                  int32_t id, void *data)
{
    (void)arg;

    if (base != WIFI_EVENT) {
        return;
    }

    switch (id) {
    case WIFI_EVENT_AP_STACONNECTED: {
        const wifi_event_ap_staconnected_t *e = (const wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "station joined: " MACSTR " (aid %u)", MAC2STR(e->mac), e->aid);
        s_ap_up = true;   /* Association proves the AP is live. */
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
        const wifi_event_ap_stadisconnected_t *e = (const wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "station left: " MACSTR " (aid %u)", MAC2STR(e->mac), e->aid);
        break;
    }
    default:
        break;
    }
}

/* NVS bring-up - mirrors components/config/adsbin_config.c, plus tolerance for
 * "already initialized" since config_init() runs first at boot. */
static esp_err_t ensure_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_INVALID_STATE) {
        return ESP_OK;   /* Already initialized by another component. */
    }

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS unusable (%s) - erasing and retrying", esp_err_to_name(err));
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
        if (err == ESP_ERR_INVALID_STATE) {
            err = ESP_OK;
        }
    }
    return err;
}

esp_err_t wifi_link_start_ap(const wifi_link_ap_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Boot-singleton guard: a second call is a safe no-op rather than an
     * ESP_ERR_WIFI_INIT_STATE fault from re-initialising the driver. */
    if (s_started) {
        return ESP_OK;
    }

    /* 1. NVS - the Wi-Fi driver persists calibration / PHY data here. */
    esp_err_t err = ensure_nvs();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 2. TCP/IP stack + default AP netif. */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* The default event loop may already exist - INVALID_STATE means already there. */
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop create failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Wires up the AP interface (DHCP server, default IP, lwIP glue). This call
     * asserts internally on its documented failure paths rather than returning
     * NULL, so the guard is defensive only - another reason the one-shot
     * contract matters (a second call would re-attach handlers). */
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (ap_netif == NULL) {
        ESP_LOGE(TAG, "esp_netif_create_default_wifi_ap returned NULL");
        return ESP_FAIL;
    }

    /* 3. Wi-Fi driver init with library defaults. On the hosted backend this
     * transparently initializes the remote stack on the C6 over SDIO. */
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Register the diagnostics handler for ALL WIFI_EVENT ids before start. */
    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_ap_event_handler,
                                              NULL,
                                              NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not register WIFI_EVENT handler: %s", esp_err_to_name(err));
    }

    /* 4. AP mode + open-AP config. */
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(AP) failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Resolve the live values: explicit cfg wins, Kconfig fills any blank. */
    const char *ssid = (cfg->ssid && cfg->ssid[0]) ? cfg->ssid : CONFIG_ADSBIN_AP_SSID;
    uint8_t channel  = (cfg->channel != 0) ? cfg->channel : (uint8_t)CONFIG_ADSBIN_AP_CHANNEL;
    uint8_t maxconn  = (cfg->max_clients != 0) ? cfg->max_clients
                                               : (uint8_t)CONFIG_ADSBIN_AP_MAX_CLIENTS;

    /* Length-bounded copy of the SSID into the driver's 32-byte field. We set
     * ssid_len explicitly so an SSID of exactly 32 bytes is still carried. */
    wifi_config_t wc = { 0 };
    size_t ssid_len = strnlen(ssid, sizeof(wc.ap.ssid));
    memcpy(wc.ap.ssid, ssid, ssid_len);
    wc.ap.ssid_len       = (uint8_t)ssid_len;
    wc.ap.channel        = channel;
    wc.ap.authmode       = WIFI_AUTH_OPEN;   /* OPEN AP - no password, by design. */
    wc.ap.max_connection = maxconn;

    /* esp_wifi_set_config takes a non-const wifi_config_t* in IDF v6. */
    err = esp_wifi_set_config(WIFI_IF_AP, &wc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 5. Go on air. */
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return err;
    }

    s_ap_up   = true;
    s_started = true;
    ESP_LOGI(TAG, "open SoftAP up: ssid=\"%s\" ch=%u max_clients=%u (no password)",
             ssid, channel, maxconn);
    return ESP_OK;
}

bool wifi_link_is_up(void)
{
    return s_ap_up;
}
