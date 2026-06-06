# ─────────────────────────────────────────────────────────────────────────────
#  components/modes_decode — Mode-S / ADS-B parser, Core 1 (plan S4.3).
#
#  RESOLVES CPR internally (owns the per-ICAO even/odd pairing cache) and emits
#  adsb_msg_t with ABSOLUTE lat/lon. Public headers use adsb_msg_t (common) and
#  ownship_ref_t (ownship) for local-CPR, hence REQUIRES common ownship. It does
#  NOT depend on demod1090 or traffic at compile time (decoupled via the shared
#  byte frame + adsb_msg_t) - that is what lets the three build in parallel.
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "modes_decode.c" "cpr.c"
    INCLUDE_DIRS  "include"
    REQUIRES      common ownship
    PRIV_REQUIRES esp_timer
)
