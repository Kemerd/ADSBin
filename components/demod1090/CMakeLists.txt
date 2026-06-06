# ─────────────────────────────────────────────────────────────────────────────
#  components/demod1090 — 1090ES demodulator, Core 0 (IMPLEMENTATION_PLAN.md S4.2).
#
#  Pulls IQ from usb_rtlsdr's ring (RingbufHandle_t) and pushes candidate
#  modes_frame_t (by value) to a queue main owns. The public header takes both
#  handle types, so REQUIRES esp_ringbuf (ring) + freertos (queue) + common
#  (iq_block_t / modes_frame_t). It does NOT depend on modes_decode (decoupled
#  via the shared frame type) - that is what lets the two build in parallel.
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "demod1090.c"
    INCLUDE_DIRS  "include"
    REQUIRES      common esp_ringbuf freertos
    PRIV_REQUIRES esp_timer
)
