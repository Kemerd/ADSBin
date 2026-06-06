# ─────────────────────────────────────────────────────────────────────────────
#  components/status — LEDs + temperature watchdog (plan §4.8, §7)
#
#  Owns the LED/health/temperature API. Driven by main, usb_rtlsdr (dongle
#  presence), and traffic (heartbeat). No dependency on other ADSBin components.
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS        "status.c"
    INCLUDE_DIRS "include"
    # All requires are PRIVATE: the public header (status.h) pulls in only
    # esp_err.h, which IDF supplies transitively, so nothing leaks into our ABI.
    #   driver           -> GPIO output control (driver/gpio.h)
    #   esp_driver_tsens -> ESP32-P4 internal die-temperature sensor (split out of
    #                       the legacy `driver` umbrella in recent IDF; required
    #                       explicitly so this builds cleanly across IDF versions)
    #   esp_timer        -> backs adsbin_now_us() pulled in via adsbin_types.h
    #   common           -> adsbin_types.h (ADSBIN_CORE_*, adsbin_now_us), .c only
    #   freertos         -> render/watchdog task + portMUX temperature spinlock
    #   esp_log          -> diagnostics
    PRIV_REQUIRES driver esp_driver_tsens esp_timer common freertos esp_log
)
