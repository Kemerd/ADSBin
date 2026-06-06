# ─────────────────────────────────────────────────────────────────────────────
#  components/config — NVS-backed persistent settings (plan §4.7)
#
#  Owns adsbin_config_t. Foundational: depended on by ownship, traffic,
#  usb_rtlsdr, and sinks. Keep its own REQUIRES minimal to avoid cycles.
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS        "adsbin_config.c"
    INCLUDE_DIRS "include"
    # ── Private deps only — config is a true leaf and must add NO dependency on
    #    any consumer component, or it would form a cycle (ownship/traffic/sinks
    #    all REQUIRE config). ────────────────────────────────────────────────
    PRIV_REQUIRES
        nvs_flash    # persistent storage backend (NVS)
        common       # shared ABI: adsbin_err.h (adsbin_err_to_str) used in .c
)
