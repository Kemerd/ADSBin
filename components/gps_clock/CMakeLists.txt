# ─────────────────────────────────────────────────────────────────────────────
#  components/gps_clock — OPTIONAL u-blox MAX-M10S GPS: live ownship position +
#  the auto-degrading clock-source ladder (NMEA fix → PPS-disciplined UTC).
#
#  Core 1 only. Pushes position through the existing `ownship` producer API and
#  publishes time/quality behind its own wait-free seqlock accessor. The whole
#  feature is inert when CONFIG_ADSBIN_GPS_UART_RX_GPIO < 0 (un-wired board).
#
#  IDF 6.x split the old monolithic `driver` into per-peripheral esp_driver_*
#  components, so each peripheral is requested explicitly:
#    REQUIRES      ownship           -> ownship_update / ownship_clear_if_source
#                  common            -> adsbin_types.h (adsbin_now_us, core ids)
#                  esp_driver_uart   -> driver/uart.h (NMEA serial in)
#                  esp_driver_gptimer-> driver/gptimer.h + gptimer_etm.h (PPS capture)
#                  esp_driver_gpio   -> driver/gpio_etm.h (PPS edge event)
#                  esp_driver_pcnt   -> driver/pulse_cnt.h (PPS edge counter)
#                  esp_timer         -> backs adsbin_now_us()
#    PRIV_REQUIRES esp_hw_support    -> esp_etm.h (the ETM channel API lives here)
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "gps_clock_supervisor.c"
                  "gps_clock_nmea.c"
                  "gps_clock_pps.c"
                  "gps_ubx.c"
    INCLUDE_DIRS  "include"
    REQUIRES      ownship common esp_driver_uart esp_driver_gptimer
                  esp_driver_gpio esp_driver_pcnt esp_timer
    PRIV_REQUIRES esp_hw_support
)
