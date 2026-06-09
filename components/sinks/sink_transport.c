/**
 * @file    sink_transport.c
 * @brief   Console-link (UART0 or USB Serial/JTAG) implementation of
 *          ::sink_transport_t.
 *
 * @details
 *   The transport seam (sink_transport.h) lets every sink write raw bytes
 *   without knowing where they go. This file provides the MVP backend: the same
 *   USB-C link that carries the ESP-IDF console. WHICH peripheral that is depends
 *   on the board and sdkconfig — boards whose USB-C jack routes through an
 *   on-board USB-UART bridge (e.g. a CH343) run the console on UART0, while
 *   boards that break out the native controller use USB Serial/JTAG. We bind to
 *   whichever the console is configured for (CONFIG_ESP_CONSOLE_*), so the GDL90
 *   stream and sink_debug text always land on the link the host is listening to.
 *
 *   The GDL90 binary stream and the sink_debug text share that link (the host
 *   bench deframes GDL90 on its 0x7E flags and reads text as UTF-8), so this
 *   transport MUST be binary-clean: it talks to the peripheral driver directly
 *   (never stdio), because the newlib console layer applies LF->CRLF translation
 *   that would corrupt any 0x0A byte inside a GDL90 frame.
 *
 *   Coexistence with the console: the IDF console may already have installed the
 *   relevant driver. We therefore install it ONLY if it is not present, and we
 *   remember whether *we* installed it so destroy() doesn't yank a driver the
 *   console still depends on.
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
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "adsbin_err.h"   /* ADSBIN_ERR_SINK_FAIL */

/* ───────────────────────────────────────────────────────────────────────────
 *  Console-link backend selection.
 *
 *  The byte output must go to the SAME peripheral the ESP-IDF console uses, so
 *  the host bench (which talks to the console port) actually receives it. We pick
 *  the driver at compile time from the console sdkconfig choice and wrap each
 *  driver in a tiny uniform API (install / is-installed / write / wait-tx /
 *  uninstall) so the logic below is transport-agnostic. We deliberately bypass
 *  stdio: newlib's console adds LF->CRLF translation that would corrupt binary
 *  GDL90 frames.
 * ─────────────────────────────────────────────────────────────────────────── */
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
  /* Console is on a UART (e.g. an on-board CH343 USB-UART bridge). */
  #include "driver/uart.h"
  #include "driver/uart_vfs.h"   /* uart_vfs_dev_use_driver(): route stdio here too */
  #define SINK_CON_UART_PORT          (CONFIG_ESP_CONSOLE_UART_NUM)
  #define SINK_CON_IS_INSTALLED()     uart_is_driver_installed(SINK_CON_UART_PORT)
  #define SINK_CON_WRITE(buf, len, t) uart_write_bytes(SINK_CON_UART_PORT, (buf), (len))
  #define SINK_CON_WAIT_TX(ticks)     uart_wait_tx_done(SINK_CON_UART_PORT, (ticks))
  #define SINK_CON_BACKEND_NAME       "UART"
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
  /* Console is on the native USB Serial/JTAG controller (USB-C broken out). */
  #include "driver/usb_serial_jtag.h"
  #define SINK_CON_IS_INSTALLED()     usb_serial_jtag_is_driver_installed()
  #define SINK_CON_WRITE(buf, len, t) usb_serial_jtag_write_bytes((buf), (len), (t))
  #define SINK_CON_WAIT_TX(ticks)     usb_serial_jtag_wait_tx_done(ticks)
  #define SINK_CON_BACKEND_NAME       "USB-Serial/JTAG"
#else
  #error "sink_transport: no supported ESP console backend selected (need UART or USB-Serial/JTAG)."
#endif

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

    // Install the console-link driver only if nobody (e.g. the console) has.
    // This is the key to sharing the single console link without double-install.
    // The two backends have different install signatures, so branch here; the
    // rest of the file uses the uniform SINK_CON_* wrappers.
    if (!SINK_CON_IS_INSTALLED()) {
        esp_err_t err;
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
        // UART backend: install an interrupt-driven TX ring on the console UART.
        // The console already configured the port's pins/baud, so we only add the
        // buffered driver (TX ring for our bytes; a small RX ring the inject
        // console can share). No event queue — we never consume UART events here.
        err = uart_driver_install(SINK_CON_UART_PORT,
                                  SINK_USB_RX_BUF_BYTES,   // RX ring
                                  SINK_USB_TX_BUF_BYTES,   // TX ring
                                  0, NULL, 0);
        // Route stdio (the +INJECT reply, ESP_LOGx) through the SAME installed
        // driver so console text and our binary GDL90 writes serialize on one
        // interrupt-driven path instead of racing the rom polling path byte-for-
        // byte (which could splice a log line into the middle of a GDL90 frame).
        if (err == ESP_OK) {
            uart_vfs_dev_use_driver(SINK_CON_UART_PORT);
        }
#else
        // USB Serial/JTAG backend: install with TX/RX rings sized for one cycle.
        usb_serial_jtag_driver_config_t dcfg = {
            .tx_buffer_size = SINK_USB_TX_BUF_BYTES,
            .rx_buffer_size = SINK_USB_RX_BUF_BYTES,
        };
        err = usb_serial_jtag_driver_install(&dcfg);
#endif
        if (err != ESP_OK) {
            // Couldn't bring up the link ourselves — clean up and report.
            ESP_LOGE(TAG, "%s driver install failed: %s",
                     SINK_CON_BACKEND_NAME, esp_err_to_name(err));
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

    // Push into the driver's TX ring. The ISR drains it to the host. The write
    // MUST be all-or-nothing per call: a sink_debug line and a GDL90 frame are
    // each one write, and a partial write would let the other sink's bytes splice
    // into the middle of a line/frame on the shared link (corrupting the host's
    // text grammar and GDL90 framing alike). Both backends therefore queue the
    // WHOLE buffer:
    //   * USB-JTAG: usb_serial_jtag_write_bytes() copies up to `len` with a short
    //     timeout; the TX ring is sized to hold a full publish cycle.
    //   * UART: uart_write_bytes() copies the entire buffer into the TX ring
    //     (blocking only if the ring is momentarily full — which, at 4 KB, it is
    //     not under our 1 Hz cadence). NOTE: uart_tx_chars() must NOT be used here
    //     — it writes only what currently fits and returns, breaking atomicity.
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    (void)wait;
    int written = uart_write_bytes(SINK_CON_UART_PORT, (const char *)buf, len);
#else
    int written = SINK_CON_WRITE(buf, len, wait);
#endif

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
    // will drain when the host resumes reading. SINK_CON_WAIT_TX maps to the
    // configured backend's tx-done wait.
    esp_err_t err = SINK_CON_WAIT_TX(pdMS_TO_TICKS(SINK_USB_WRITE_TIMEOUT_MS));
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
    // another owner) still needs it on this shared console link.
    if (transport->we_installed_driver) {
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
        uart_driver_delete(SINK_CON_UART_PORT);
#else
        usb_serial_jtag_driver_uninstall();
#endif
    }

    if (transport->tx_lock != NULL) {
        vSemaphoreDelete(transport->tx_lock);
    }

    free(transport);
}
