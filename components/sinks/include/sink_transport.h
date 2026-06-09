/**
 * @file    sink_transport.h
 * @brief   Byte-stream transport abstraction for the output sinks (plan S4.5).
 *
 * @details
 *   The seam that lets the SAME encoder target USB-CDC now and WiFi/UDP later
 *   (S4.5: "Later transport: WiFi UDP broadcast ... same encoder"). A sink holds
 *   a ::sink_transport_t and writes bytes through it; it never references the
 *   underlying link directly. Two backends now sit behind the one opaque handle:
 *   ::sink_transport_usb_cdc_create() (the console link) and
 *   ::sink_transport_udp_create() (limited-broadcast UDP for ForeFlight over the
 *   C6 SoftAP). Adding the UDP backend required ZERO changes to the sinks or the
 *   GDL90 encoder - they still just call ::sink_transport_write().
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

/**
 * @brief UDP limited-broadcast transport config.
 *
 * One datagram is emitted per ::sink_transport_write call (one GDL90 frame per
 * packet), sent to 255.255.255.255:@p port. ForeFlight listens for GDL90 on
 * broadcast UDP :4000, so @p port is normally 4000.
 */
typedef struct {
    uint16_t port;  /**< UDP destination port (e.g. 4000 for ForeFlight).        */
} sink_transport_udp_cfg_t;

/**
 * @brief Construct a non-blocking limited-broadcast UDP transport.
 *
 * Opens a datagram socket with SO_BROADCAST and O_NONBLOCK pointed at
 * 255.255.255.255:@p cfg->port. Each ::sink_transport_write becomes exactly one
 * sendto(); a missing/!-associated AP drops the frame (counted) instead of
 * stalling the caller. Requires the WiFi/lwip stack to be up.
 *
 * @param cfg           Destination port (must be non-NULL).
 * @param out_transport Receives the new handle on success.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_NO_MEM, or ADSBIN_ERR_SINK_FAIL.
 */
esp_err_t sink_transport_udp_create(const sink_transport_udp_cfg_t *cfg,
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
