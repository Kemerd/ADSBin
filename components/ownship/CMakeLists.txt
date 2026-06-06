# ─────────────────────────────────────────────────────────────────────────────
#  components/ownship — reference-position service (plan §4.6)
#
#  Owns ownship_ref_t (cross-cutting). Seeds itself from `config` (NVS) at init.
#  Consumed by modes_decode (local CPR) and traffic (range cull).
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS        "ownship.c"
    INCLUDE_DIRS "include"
    REQUIRES     config        # reads/writes the persisted manual reference
    # common  -> adsbin_types.h (adsbin_now_us, the one time base), only used in .c
    # esp_timer -> backs adsbin_now_us(); freertos -> portMUX spinlock; esp_log -> diags
    PRIV_REQUIRES esp_timer common freertos esp_log
)
