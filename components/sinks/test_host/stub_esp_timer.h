/* Host-test stub: satisfies adsbin_types.h's #include "esp_timer.h" so the pure
 * gdl90_encoder.c can be compiled and unit-tested on a development host without
 * the ESP-IDF. NOT part of the firmware build. */
#pragma once
#include <stdint.h>
static inline int64_t esp_timer_get_time(void) { return 0; }
