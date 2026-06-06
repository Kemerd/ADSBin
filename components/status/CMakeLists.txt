# ─────────────────────────────────────────────────────────────────────────────
#  components/status — LEDs + temperature watchdog (plan §4.8, §7)
#
#  Owns the LED/health/temperature API. Driven by main, usb_rtlsdr (dongle
#  presence), and traffic (heartbeat). No dependency on other ADSBin components.
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS        "status.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_timer   # GPIO + internal temp sensor + render timer
)
