# ------------------------------------------------------------------------------
#  components/wifi_link - open Wi-Fi SoftAP on the companion ESP32-C6 (plan S10)
#
#  Stands up the AP that carries the GDL90 UDP broadcast to ForeFlight. The radio
#  is remote (on the C6 over SDIO); the esp_wifi_remote hosted backend makes the
#  ordinary esp_wifi.h calls work transparently. Deps are public (REQUIRES) since
#  the bring-up is the component's whole job.
#
#  esp_wifi_remote is a MANAGED component; it is fetched into managed_components/
#  via main/idf_component.yml. Naming it in REQUIRES only resolves AFTER that
#  manifest has been processed by `idf.py reconfigure`, so wifi_link is built as
#  part of the same change set, never standalone.
# ------------------------------------------------------------------------------
idf_component_register(
    SRCS         "wifi_link.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        esp_wifi_remote
)
