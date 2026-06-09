# ─────────────────────────────────────────────────────────────────────────────
#  components/demod978 — 978 MHz UAT demodulator, Core 0 (weather/FIS-B phase).
#
#  Mirrors demod1090: pulls IQ from the 978-role usb_rtlsdr ring (RingbufHandle_t)
#  and pushes FEC-corrected output — UAT ADS-B frames (by value) to a uat_frame_t
#  queue, and UAT uplink frames (by reference) to a uat_uplink_t ring, both owned
#  by main. The public header takes the handle types, so REQUIRES esp_ringbuf
#  (rings) + freertos (queue) + common (iq_block_t / uat_frame_t / uat_uplink_t).
#  It does NOT depend on uat_decode (decoupled via the shared frame types).
#
#  uat_fec.c is the clean-room Reed-Solomon layer; it links into this component
#  AND into the host unit tests under test_host/ (which are NOT part of the
#  firmware build — they are compiled standalone with gcc, see test_host/README).
#
#  PRIV_REQUIRES: esp_timer (adsbin_now_us backing clock), heap (the discriminator
#  scratch + realloc), log (diagnostics). Private — never leak through the header.
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "demod978.c" "uat_fec.c"
    INCLUDE_DIRS  "include"
    REQUIRES      common esp_ringbuf freertos
    PRIV_REQUIRES esp_timer heap
)
