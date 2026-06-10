# ─────────────────────────────────────────────────────────────────────────────
#  components/gps_clock — OPTIONAL u-blox MAX-M10S GPS: live ownship position +
#  the auto-degrading clock-source ladder (NMEA fix → PPS-disciplined UTC).
#
#  Core 1 only. Pushes position through the existing `ownship` producer API and
#  publishes time/quality behind its own wait-free seqlock accessor. The whole
#  feature is inert when CONFIG_ADSBIN_GPS_UART_RX_GPIO < 0 (un-wired board).
#
#    REQUIRES      ownship  -> ownship_update / ownship_clear_if_source
#                  common   -> adsbin_types.h (adsbin_now_us, core ids)
#                  driver   -> uart / gptimer / gpio_etm / pcnt (PPS capture)
#                  esp_timer-> backs adsbin_now_us()
#    PRIV_REQUIRES esp_etm  -> ETM channel wiring for the PPS hardware capture
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "gps_clock_supervisor.c"
                  "gps_clock_nmea.c"
                  "gps_clock_pps.c"
                  "gps_ubx.c"
    INCLUDE_DIRS  "include"
    REQUIRES      ownship common driver esp_timer
    PRIV_REQUIRES esp_etm
)
