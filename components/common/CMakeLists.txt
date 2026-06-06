# ─────────────────────────────────────────────────────────────────────────────
#  components/common — the shared cross-cutting ABI (IMPLEMENTATION_PLAN.md S2).
#
#  This component is the single home of every type that crosses a task/core
#  boundary (iq_block_t, modes_frame_t, adsb_msg_t, traffic_target_t, ...).
#  EVERY other component REQUIRES it. It carries exactly one .c (the error-string
#  table); everything else is header-only and ISR-safe.
#
#  REQUIRES esp_timer is PUBLIC because adsbin_now_us() is an inline in the
#  public header (esp_timer_get_time()).
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS         "adsbin_err.c"
    INCLUDE_DIRS "include"
    REQUIRES     esp_timer
)
