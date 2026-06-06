# ─────────────────────────────────────────────────────────────────────────────
#  components/sinks — pluggable output layer (plan S4.5): sink_debug + sink_gdl90.
#
#  Public headers reference traffic_target_t / traffic_snapshot_t (common),
#  ownship_ref_t (ownship), and traffic_handle_t (traffic) -> all public REQUIRES.
#  The USB-CDC transport (USB Serial/JTAG) and the publisher task are
#  implementation details (PRIV).
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "sinks.c" "sink_debug.c" "sink_gdl90.c" "sink_transport.c" "gdl90_encoder.c"
    INCLUDE_DIRS  "include"
    REQUIRES      common ownship traffic
    PRIV_REQUIRES esp_driver_usb_serial_jtag esp_timer freertos
)
