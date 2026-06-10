/* Host-only stub for esp_timer.h — satisfies adsbin_types.h's include in a host
 * unit-test compile of the pure NMEA parser. NOT part of the firmware build. */
#pragma once
#include <stdint.h>
static inline int64_t esp_timer_get_time(void) { return 0; }
