# ─────────────────────────────────────────────────────────────────────────────
#  components/traffic — ICAO-keyed traffic table manager, Core 1 (plan S4.4).
#
#  Public header uses adsb_msg_t / traffic_target_t / traffic_snapshot_t (common)
#  and ownship_ref_t (ownship), so both are public REQUIRES. It consumes already-
#  RESOLVED positions from modes_decode via the shared adsb_msg_t, so it needs no
#  compile dependency on modes_decode.
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "traffic.c"
    INCLUDE_DIRS  "include"
    REQUIRES      common ownship
    PRIV_REQUIRES esp_timer
)
