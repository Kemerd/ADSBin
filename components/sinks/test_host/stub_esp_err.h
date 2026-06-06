/* Host-test stub for esp_err.h. Mirrors the handful of ESP-IDF error codes the
 * pure encoder references so it links on a host. NOT part of the firmware build. */
#pragma once
typedef int esp_err_t;
#define ESP_OK                0
#define ESP_FAIL             -1
#define ESP_ERR_INVALID_ARG   0x102
#define ESP_ERR_INVALID_SIZE  0x104
static inline const char *esp_err_to_name(esp_err_t e) { (void)e; return "ERR"; }
