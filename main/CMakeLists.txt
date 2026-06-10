# ============================================================================
#  ADSBin - application (main) component.
#  Entry point + task-wiring hub (core pinning, pipeline orchestration).
#
#  Stage 3 integration: main is the top of the dependency graph, so it pulls
#  EVERY firmware component into the build closure and calls each one's *_init() /
#  *_start() entry point. The full S2 pipeline is wired in adsbin_app.c.
# ============================================================================
idf_component_register(
    SRCS        "adsbin_app.c"
    INCLUDE_DIRS "include"
    REQUIRES
        # ── ADSBin firmware components (every public header main calls) ───────
        common          # shared ABI: adsbin_types.h, adsbin_err.h, core-id macros
        config          # config_init / config_get (NVS-backed settings)
        ownship         # ownship_init / ownship_get (reference position)
        gps_clock       # OPTIONAL MAX-M10S GPS: ownship position + clock ladder
        usb_rtlsdr      # RTL-SDR USB-HS host driver; owns the IQ ring(s)
        demod1090       # 1090ES demodulator (Core 0)
        demod978        # 978 UAT demodulator (Core 0, weather path)
        modes_decode    # Mode-S / ADS-B decoder (Core 1)
        uat_decode      # UAT message decoder (Core 1, weather path)
        traffic         # ICAO-keyed traffic table (Core 1)
        sinks           # output sink registry + USB-CDC/UDP transport + sinks (incl. weather)
        wifi_link       # open SoftAP on the on-board C6 (esp-hosted, S10)
        status          # LEDs + temperature watchdog (Core 1)
        # ── ESP-IDF components main itself touches ───────────────────────────
        esp_driver_gpio                # status LED pin (board-default convention)
        esp_timer                      # backs adsbin_now_us()
        esp_ringbuf                    # RingbufHandle_t in the pipeline struct
        freertos                       # tasks + queues (the glue layer)
)
