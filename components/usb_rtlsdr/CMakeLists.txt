# ─────────────────────────────────────────────────────────────────────────────
#  components/usb_rtlsdr — RTL-SDR USB-HS host driver (IMPLEMENTATION_PLAN.md S4.1).
#
#  Core 0. Owns the IQ ring buffer it exposes via usb_rtlsdr_get_iq_ring(); the
#  public header therefore needs RingbufHandle_t (esp_ringbuf) and iq_block_t
#  (common). The USB Host stack itself is an implementation detail (PRIV).
# ─────────────────────────────────────────────────────────────────────────────
idf_component_register(
    SRCS          "usb_rtlsdr.c"
    INCLUDE_DIRS  "include"
    REQUIRES      common esp_ringbuf
    PRIV_REQUIRES usb esp_timer
)
