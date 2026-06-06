/**
 * @file    sink_transport.c
 * @brief   USB-CDC (USB Serial/JTAG) implementation of ::sink_transport_t.
 *
 * @details
 *   The transport seam (sink_transport.h) lets every sink write raw bytes
 *   without knowing where they go. This file provides the MVP backend: the P4's
 *   USB Serial/JTAG peripheral — the same USB-C link that carries the ESP-IDF
 *   console. The GDL90 binary stream and the sink_debug text share that link
 *   (the host bench deframes GDL90 on its 0x7E flags and reads text as UTF-8),
 *   so this transport must be binary-clean: it never translates newlines and
 *   never line-buffers.
 *
 *   Coexistence with the console: the IDF console may already have installed the
 *   USB Serial/JTAG driver. We therefore install it ONLY if it is not present,
 *   and we remember whether *we* installed it so destroy() doesn't yank a driver
 *   the console still depends on. Either way, byte output goes through
 *   usb_serial_jtag_write_bytes(), which is the binary-safe path.
 *
 * @par Core affinity
 *   Written from the Core-1 publisher task. We bound every write/flush with a
 *   short RTOS timeout so a stalled or disconnected host (no reader draining the
 *   FIFO) drops bytes instead of blocking the publisher — Core-0 DSP must never
 *   be starved by back-pressure on the USB link.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "sink_transport.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "adsbin_err.h"   /* ADSBIN_ERR_SINK_FAIL */

/* ───────────────────────────────────────────────────────────────────────────
 *  Tunables. The TX ring must comfortably hold one publish cycle's worth of
 *  GDL90 + debug text so a momentarily-busy host doesn't immediately overflow.
 * ─────────────────────────────────────────────────────────────────────────── */
#define SINK_USB_TX_BUF_BYTES   4096u   /**< Driver TX ring size.                */
#define SINK_USB_RX_BUF_BYTES   256u    /**< Driver RX ring (console may use it).*/

/* Write/flush timeout. Short enough that a dead host link never stalls Core 1,
 * long enough that a healthy host keeps up under normal publish cadence.       */
#define SINK_USB_WRITE_TIMEOUT_MS  20u

static const char *TAG = "sink_transport";

/* ───────────────────────────────────────────────────────────────────────────
 *  Concrete transport instance. The opaque ::sink_transport_t in the header is
 *  a pointer to this struct.
 * ─────────────────────────────────────────────────────────────────────────── */
struct sink_transport_s {
    bool              we_installed_driver; /**< true => destroy() uninstalls it.  */
    SemaphoreHandle_t tx_lock;             /**< Serializes concurrent writers.    */
    uint64_t          bytes_written;       /**< Diagnostic: total bytes accepted. */
    uint64_t          bytes_dropped;       /**< Diagnostic: bytes lost to back-pressure. */
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  Construction
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t sink_transport_usb_cdc_create(const sink_transport_usb_cdc_cfg_t *cfg,
                                        sink_transport_t *out_transport)
{
    // The cfg currently carries only a reserved field; accept NULL gracefully so
    // callers that have nothing to configure can pass NULL.
    (void)cfg;

    if (out_transport == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_transport = NULL;

    // Allocate the instance. calloc zeroes the diagnostic counters for us.
    struct sink_transport_s *t = calloc(1, sizeof(*t));
    if (t == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // A small mutex guards the write path: multiple sinks (debug + GDL90) can be
    // driven from the one publisher task, but keeping the lock makes the
    // transport safe even if a future caller writes from another context.
    t->tx_lock = xSemaphoreCreateMutex();
    if (t->tx_lock == NULL) {
        free(t);
        return ESP_ERR_NO_MEM;
    }

    // Install the USB Serial/JTAG driver only if nobody (e.g. the console) has.
    // This is the key to sharing the single USB-C link without double-install.
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t dcfg = {
            .tx_buffer_size = SINK_USB_TX_BUF_BYTES,
            .rx_buffer_size = SINK_USB_RX_BUF_BYTES,
        };
        esp_err_t err = usb_serial_jtag_driver_install(&dcfg);
        if (err != ESP_OK) {
            // Couldn't bring up the link ourselves — clean up and report.
            ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s",
                     esp_err_to_name(err));
            vSemaphoreDelete(t->tx_lock);
            free(t);
            return err;
        }
        t->we_installed_driver = true;
    } else {
        // Console already owns the driver; we just borrow its TX path.
        t->we_installed_driver = false;
    }

    *out_transport = t;
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Byte output
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t sink_transport_write(sink_transport_t transport, const uint8_t *buf, size_t len)
{
    // Validate. A zero-length write is a successful no-op, not an error.
    if (transport == NULL || buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len == 0) {
        return ESP_OK;
    }

    // Take the write lock with a bounded wait. If we can't even acquire it
    // promptly something is wrong with another writer — drop rather than hang.
    const TickType_t wait = pdMS_TO_TICKS(SINK_USB_WRITE_TIMEOUT_MS);
    if (xSemaphoreTake(transport->tx_lock, wait) != pdTRUE) {
        transport->bytes_dropped += len;
        return ADSBIN_ERR_SINK_FAIL;
    }

    // Push into the driver's TX ring. With a non-zero tx_buffer_size the call
    // copies into the ring and returns; the ISR drains it to the host. A short
    // timeout means a full ring (host not reading) drops bytes deterministically
    // instead of blocking the Core-1 publisher.
    int written = usb_serial_jtag_write_bytes(buf, len, wait);

    xSemaphoreGive(transport->tx_lock);

    if (written < 0) {
        // Driver-level failure (e.g. not installed) — surface as a sink fault.
        transport->bytes_dropped += len;
        return ESP_FAIL;
    }

    // Account for accepted vs dropped bytes for diagnostics / status.
    transport->bytes_written += (uint64_t)written;
    if ((size_t)written < len) {
        // Partial accept => the remainder was dropped on back-pressure. This is
        // the documented drop-with-counter behavior, not an error to the caller.
        transport->bytes_dropped += (len - (size_t)written);
    }

    return ESP_OK;
}

esp_err_t sink_transport_flush(sink_transport_t transport)
{
    if (transport == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Bounded wait so a disconnected host can't wedge the publisher here. A
    // timeout is benign: the bytes are already queued in the driver ring and
    // will drain when the host resumes reading.
    esp_err_t err = usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(SINK_USB_WRITE_TIMEOUT_MS));
    if (err == ESP_ERR_TIMEOUT) {
        // Not fatal — treat as "queued, host slow". Caller continues publishing.
        return ESP_OK;
    }
    return err;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Teardown
 * ═══════════════════════════════════════════════════════════════════════════ */

void sink_transport_destroy(sink_transport_t transport)
{
    if (transport == NULL) {
        return;
    }

    // Only uninstall the driver if WE installed it — otherwise the console (or
    // another owner) still needs it on this shared USB-C link.
    if (transport->we_installed_driver) {
        usb_serial_jtag_driver_uninstall();
    }

    if (transport->tx_lock != NULL) {
        vSemaphoreDelete(transport->tx_lock);
    }

    free(transport);
}
