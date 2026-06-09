# ─────────────────────────────────────────────────────────────────────────────
#  components/uat_decode — UAT (978 MHz) message decoder, Core 1 (weather phase).
#
#  The 978-band analog of modes_decode. Takes a FEC-corrected UAT payload (from
#  demod978) and parses it into the SHARED adsb_msg_t (so UAT traffic merges into
#  the same traffic table as 1090), or validates a FIS-B uplink for relaying.
#
#  REQUIRES common only — it consumes adsb_msg_t and the UAT_* sizes from the
#  frozen ABI and is otherwise self-contained (no CPR cache, no allocation, no
#  libm). It does NOT depend on demod978 (decoupled via the shared uat_frame_t /
#  raw payload), which is what lets the two build in parallel.
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "uat_decode.c"
    INCLUDE_DIRS  "include"
    REQUIRES      common
)
