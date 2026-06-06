# ============================================================================
#  ADSBin - application (main) component.
#  Entry point + task-wiring hub (core pinning, pipeline orchestration).
#
#  Phase 0 scaffold: depends only on `common` (shared types / core-id macros),
#  GPIO (status LED), and esp_timer. As each component lands in Stage 2/3 it is
#  added to REQUIRES here so the linker pulls it into the build closure and main
#  can call its *_start() entry points.
# ============================================================================
idf_component_register(
    SRCS        "adsbin_app.c"
    INCLUDE_DIRS "include"
    REQUIRES     common esp_driver_gpio esp_timer esp_ringbuf
)
