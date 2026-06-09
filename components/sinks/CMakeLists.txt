# ─────────────────────────────────────────────────────────────────────────────
#  components/sinks — pluggable output layer (plan S4.5): sink_debug + sink_gdl90.
#
#  Public headers reference traffic_target_t / traffic_snapshot_t (common),
#  ownship_ref_t (ownship), and traffic_handle_t (traffic) -> all public REQUIRES.
#  The console-link transport and the publisher task are implementation details
#  (PRIV). The transport binds to whichever peripheral the console uses, so both
#  driver components are required; sink_transport.c #ifdef-selects one at compile
#  time from the CONFIG_ESP_CONSOLE_* choice.
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "sinks.c" "sink_debug.c" "sink_gdl90.c" "sink_uat_weather.c" "sink_transport.c" "gdl90_encoder.c"
    INCLUDE_DIRS  "include"
    REQUIRES      common ownship traffic
    PRIV_REQUIRES esp_driver_usb_serial_jtag esp_driver_uart esp_timer freertos lwip
)
