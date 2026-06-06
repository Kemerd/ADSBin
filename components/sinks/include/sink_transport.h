/**
 * @file    sink_transport.h
 * @brief   Byte-stream transport abstraction for the output sinks (plan S4.5).
 *
 * @details
 *   The seam that lets the SAME encoder target USB-CDC now and WiFi/UDP later
 *   (S4.5: "Later transport: WiFi UDP broadcast ... same encoder"). A sink holds
 *   a ::sink_transport_t and writes bytes through it; it never references USB-CDC
 *   directly. A future sink_transport_udp_create() drops in with zero changes to
 *   the sinks or the GDL90 encoder.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque transport instance. */
typedef struct sink_transport_s *sink_transport_t;

/** @brief USB-CDC (USB Serial/JTAG) transport config. No params needed for MVP. */
typedef struct {
    int placeholder_reserved;  /**< Reserved; keeps the ABI stable for additions.*/
} sink_transport_usb_cdc_cfg_t;

/** @brief Construct the MVP USB-CDC transport. */
esp_err_t sink_transport_usb_cdc_create(const sink_transport_usb_cdc_cfg_t *cfg,
                                        sink_transport_t *out_transport);

/** @brief Write @p len bytes; should be friendly to a Core-1 caller. */
esp_err_t sink_transport_write(sink_transport_t transport, const uint8_t *buf, size_t len);

/** @brief Flush any buffered bytes. */
esp_err_t sink_transport_flush(sink_transport_t transport);

/** @brief Destroy a transport and release its underlying device/socket. */
void sink_transport_destroy(sink_transport_t transport);

#ifdef __cplusplus
}
#endif
