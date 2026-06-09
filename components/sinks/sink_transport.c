/**
 * @file    sink_transport.c
 * @brief   Two-backend implementation of ::sink_transport_t: CONSOLE (the
 *          UART0 / USB-Serial-JTAG link that also carries the ESP-IDF console,
 *          the MVP) and UDP (a non-blocking limited-broadcast socket that emits
 *          one datagram per write for streaming to ForeFlight over the C6 SoftAP).
 *
 * @details
 *   The seam lets every sink write raw bytes without knowing where they go. Both
 *   backends are reached through the one opaque ::sink_transport_t; write/flush/
 *   destroy branch on a per-instance KIND tag.
 *
 *   CONSOLE: binds to whichever peripheral the console uses (CONFIG_ESP_CONSOLE_*)
 *   and talks to the driver directly (never stdio) so the binary GDL90 stream is
 *   not corrupted by newlib LF->CRLF translation. Installs the driver only if the
 *   console has not, and remembers whether it did so destroy() does not yank a
 *   driver the console still needs.
 *
 *   UDP: a datagram socket pointed at 255.255.255.255:<port>. ForeFlight listens
 *   for GDL90 on broadcast UDP :4000, so one socket reaches every client. The
 *   socket is NON-BLOCKING (O_NONBLOCK) so a missing AP / full TX path / no
 *   associated station can NEVER stall the Core-1 publisher: the write is dropped
 *   and counted, exactly like a back-pressured console write.
 *
 * @par Binary-atomicity invariant (BOTH backends)
 *   One sink_debug line and one GDL90 frame are each exactly ONE write, and a
 *   write is ALL-OR-NOTHING. CONSOLE queues the WHOLE buffer (uart_write_bytes /
 *   usb_serial_jtag_write_bytes; uart_tx_chars() must NOT be used). UDP does
 *   exactly one sendto() per write => one frame per datagram (message-oriented,
 *   so it is enqueued whole or fails wholesale).
 *
 * @par Core affinity
 *   Written from the Core-1 publisher. Every write/flush is bounded so a stalled
 *   host (CONSOLE) or absent AP (UDP) drops bytes instead of blocking.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "sink_transport.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>      /* close() on a socket fd (canonical declaration).       */
#include <fcntl.h>       /* fcntl()/F_GETFL/F_SETFL/O_NONBLOCK (canonical decls).  */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "adsbin_err.h"   /* ADSBIN_ERR_SINK_FAIL */

/* lwip BSD sockets for the UDP backend. In ESP-IDF the upstream BSD-name macro
 * block in lwip/sockets.h is disabled (LWIP_COMPAT_SOCKETS=0,
 * LWIP_POSIX_SOCKETS_IO_NAMES=0): socket()/setsockopt()/sendto() are real lwip
 * symbols and close()/fcntl() route through the lwip VFS layer. Including
 * lwip/sockets.h + lwip/inet.h is sufficient (struct sockaddr_in, AF_INET,
 * SOCK_DGRAM, SO_BROADCAST, INADDR_BROADCAST, htonl/htons, per-task errno). */
#include "lwip/sockets.h"
#include "lwip/inet.h"

/* Console-link backend selection: pick the driver from the console sdkconfig
 * choice and wrap each in a uniform API so the logic below is transport-agnostic. */
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
  #include "driver/uart.h"
  #include "driver/uart_vfs.h"   /* uart_vfs_dev_use_driver(): route stdio here too */
  #define SINK_CON_UART_PORT          (CONFIG_ESP_CONSOLE_UART_NUM)
  #define SINK_CON_IS_INSTALLED()     uart_is_driver_installed(SINK_CON_UART_PORT)
  #define SINK_CON_WRITE(buf, len, t) uart_write_bytes(SINK_CON_UART_PORT, (buf), (len))
  #define SINK_CON_WAIT_TX(ticks)     uart_wait_tx_done(SINK_CON_UART_PORT, (ticks))
  #define SINK_CON_BACKEND_NAME       "UART"
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
  #include "driver/usb_serial_jtag.h"
  #define SINK_CON_IS_INSTALLED()     usb_serial_jtag_is_driver_installed()
  #define SINK_CON_WRITE(buf, len, t) usb_serial_jtag_write_bytes((buf), (len), (t))
  #define SINK_CON_WAIT_TX(ticks)     usb_serial_jtag_wait_tx_done(ticks)
  #define SINK_CON_BACKEND_NAME       "USB-Serial/JTAG"
#else
  #error "sink_transport: no supported ESP console backend selected (need UART or USB-Serial/JTAG)."
#endif

#define SINK_USB_TX_BUF_BYTES   4096u   /**< Driver TX ring size.                */
#define SINK_USB_RX_BUF_BYTES   256u    /**< Driver RX ring (console may use it).*/
#define SINK_USB_WRITE_TIMEOUT_MS  20u  /**< Bounded write/flush; never stall Core 1.*/

static const char *TAG = "sink_transport";

/* Backend kind: the opaque handle can front either link; methods dispatch on this. */
typedef enum {
    SINK_TRANSPORT_CONSOLE = 0, /**< UART0 / USB-Serial-JTAG console link.        */
    SINK_TRANSPORT_UDP,         /**< Non-blocking broadcast UDP datagram socket.  */
} sink_transport_kind_t;

/* Concrete instance: KIND tag + fields common to both backends (write mutex,
 * diagnostic counters) + a union of per-backend state. Only the arm matching
 * `kind` is valid. */
struct sink_transport_s {
    sink_transport_kind_t kind;          /**< Selects the union arm + method path.*/
    SemaphoreHandle_t     tx_lock;       /**< Serializes concurrent writers.      */
    uint64_t              bytes_written; /**< Diagnostic: total bytes accepted.   */
    uint64_t              bytes_dropped; /**< Diagnostic: bytes lost to back-pressure.*/
    union {
        struct { bool we_installed_driver; } console;        /**< CONSOLE arm. */
        struct { int sockfd; struct sockaddr_in dest; } udp; /**< UDP arm.     */
    } u;
};

/* === Construction - CONSOLE backend (behavior unchanged) ==================== */

esp_err_t sink_transport_usb_cdc_create(const sink_transport_usb_cdc_cfg_t *cfg,
                                        sink_transport_t *out_transport)
{
    (void)cfg;  // reserved-field cfg; NULL is fine.

    if (out_transport == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_transport = NULL;

    // calloc zeroes the counters and the union (unused arm is benign).
    struct sink_transport_s *t = calloc(1, sizeof(*t));
    if (t == NULL) {
        return ESP_ERR_NO_MEM;
    }
    t->kind = SINK_TRANSPORT_CONSOLE;

    t->tx_lock = xSemaphoreCreateMutex();
    if (t->tx_lock == NULL) {
        free(t);
        return ESP_ERR_NO_MEM;
    }

    // Install the console driver only if nobody (e.g. the console) has - the key
    // to sharing the single console link without a double-install.
    if (!SINK_CON_IS_INSTALLED()) {
        esp_err_t err;
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
        // Add the buffered driver (TX ring for our bytes; small RX ring the
        // inject console can share). No event queue - we never consume events.
        err = uart_driver_install(SINK_CON_UART_PORT,
                                  SINK_USB_RX_BUF_BYTES, SINK_USB_TX_BUF_BYTES,
                                  0, NULL, 0);
        // Route stdio through the SAME driver so console text and binary GDL90
        // serialize on one interrupt-driven path (no log/frame splicing).
        if (err == ESP_OK) {
            uart_vfs_dev_use_driver(SINK_CON_UART_PORT);
        }
#else
        usb_serial_jtag_driver_config_t dcfg = {
            .tx_buffer_size = SINK_USB_TX_BUF_BYTES,
            .rx_buffer_size = SINK_USB_RX_BUF_BYTES,
        };
        err = usb_serial_jtag_driver_install(&dcfg);
#endif
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s driver install failed: %s",
                     SINK_CON_BACKEND_NAME, esp_err_to_name(err));
            vSemaphoreDelete(t->tx_lock);
            free(t);
            return err;
        }
        t->u.console.we_installed_driver = true;
    } else {
        t->u.console.we_installed_driver = false;
    }

    *out_transport = t;
    return ESP_OK;
}

/* === Construction - UDP backend ============================================ */

esp_err_t sink_transport_udp_create(const sink_transport_udp_cfg_t *cfg,
                                    sink_transport_t *out_transport)
{
    if (out_transport == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_transport = NULL;

    // calloc clears the union; mark the socket invalid until opened so a failure
    // path that calls destroy() never closes fd 0.
    struct sink_transport_s *t = calloc(1, sizeof(*t));
    if (t == NULL) {
        return ESP_ERR_NO_MEM;
    }
    t->kind         = SINK_TRANSPORT_UDP;
    t->u.udp.sockfd = -1;

    t->tx_lock = xSemaphoreCreateMutex();
    if (t->tx_lock == NULL) {
        free(t);
        return ESP_ERR_NO_MEM;
    }

    // Unbound IPv4 UDP socket: sends from an ephemeral local port, which is what
    // a one-way broadcast emitter wants.
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        ESP_LOGE(TAG, "udp socket() failed: errno=%d", errno);
        vSemaphoreDelete(t->tx_lock);
        free(t);
        return ADSBIN_ERR_SINK_FAIL;
    }

    // Permit broadcast (without it lwip refuses sendto() to 255.255.255.255).
    int one = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one)) != 0) {
        ESP_LOGE(TAG, "udp SO_BROADCAST failed: errno=%d", errno);
        close(fd);
        vSemaphoreDelete(t->tx_lock);
        free(t);
        return ADSBIN_ERR_SINK_FAIL;
    }

    // NON-BLOCKING via O_NONBLOCK is the load-bearing guarantee that a missing/
    // !-associated AP can never stall Core 1. If it cannot be set we refuse to
    // build the transport at all, so a blocking socket is never installed.
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGE(TAG, "udp O_NONBLOCK failed: errno=%d", errno);
        close(fd);
        vSemaphoreDelete(t->tx_lock);
        free(t);
        return ADSBIN_ERR_SINK_FAIL;
    }
    // Zero SO_SNDTIMEO: NOTE in lwip a zero send-timeout means the timeout is
    // DISABLED (block forever), NOT "return immediately" - it is NOT a
    // non-blocking fallback. Set only to keep the timeout deterministic; the
    // non-blocking behavior is governed entirely by O_NONBLOCK above.
    struct timeval no_wait = { .tv_sec = 0, .tv_usec = 0 };
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &no_wait, sizeof(no_wait));

    // Pin the fixed broadcast destination once.
    t->u.udp.sockfd               = fd;
    t->u.udp.dest.sin_family      = AF_INET;
    t->u.udp.dest.sin_port        = htons(cfg->port);
    t->u.udp.dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    ESP_LOGI(TAG, "udp transport up: broadcast 255.255.255.255:%u (fd=%d)",
             (unsigned)cfg->port, fd);

    *out_transport = t;
    return ESP_OK;
}

/* === Byte output =========================================================== */

esp_err_t sink_transport_write(sink_transport_t transport, const uint8_t *buf, size_t len)
{
    if (transport == NULL || buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len == 0) {
        return ESP_OK;   // zero-length write is a successful no-op.
    }

    // Bounded lock wait: if we cannot acquire it promptly, drop rather than hang.
    const TickType_t wait = pdMS_TO_TICKS(SINK_USB_WRITE_TIMEOUT_MS);
    if (xSemaphoreTake(transport->tx_lock, wait) != pdTRUE) {
        transport->bytes_dropped += len;
        return ADSBIN_ERR_SINK_FAIL;
    }

    esp_err_t ret = ESP_OK;

    if (transport->kind == SINK_TRANSPORT_UDP) {
        // One sendto() per write => one GDL90 frame per datagram (message-
        // oriented: enqueued whole or fails wholesale). On a non-blocking socket
        // with no path to the destination we DROP and count so the publisher is
        // never stalled by a missing receiver.
        int sent = sendto(transport->u.udp.sockfd, buf, len, 0,
                          (const struct sockaddr *)&transport->u.udp.dest,
                          sizeof(transport->u.udp.dest));
        if (sent < 0) {
            const int e = errno;          // snapshot before any call perturbs it
            transport->bytes_dropped += len;
            // EXPECTED "no AP / no route / TX path full yet" errnos - lwip returns
            // these (not just EWOULDBLOCK) while the SoftAP has no associated
            // station or the netif TX queue is full; keep the publisher running:
            //   EWOULDBLOCK/EAGAIN, EHOSTUNREACH/ENETUNREACH, ENOMEM/ENOBUFS.
            ret = (e == EWOULDBLOCK || e == EAGAIN ||
                   e == EHOSTUNREACH || e == ENETUNREACH ||
                   e == ENOMEM || e == ENOBUFS) ? ESP_OK : ADSBIN_ERR_SINK_FAIL;
        } else {
            transport->bytes_written += (uint64_t)sent;
            if ((size_t)sent < len) {
                transport->bytes_dropped += (len - (size_t)sent);
            }
        }
    } else {
        // CONSOLE (unchanged): queue the WHOLE buffer into the driver TX ring;
        // a partial write would splice another sink's bytes into a frame/line.
        // uart_tx_chars() must NOT be used here.
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
        (void)wait;
        int written = uart_write_bytes(SINK_CON_UART_PORT, (const char *)buf, len);
#else
        int written = SINK_CON_WRITE(buf, len, wait);
#endif
        if (written < 0) {
            transport->bytes_dropped += len;
            ret = ESP_FAIL;
        } else {
            transport->bytes_written += (uint64_t)written;
            if ((size_t)written < len) {
                transport->bytes_dropped += (len - (size_t)written);
            }
        }
    }

    xSemaphoreGive(transport->tx_lock);
    return ret;
}

esp_err_t sink_transport_flush(sink_transport_t transport)
{
    if (transport == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // UDP datagrams leave the moment sendto() accepts them - no buffer to drain.
    if (transport->kind == SINK_TRANSPORT_UDP) {
        return ESP_OK;
    }

    // CONSOLE (unchanged): bounded wait; a timeout is benign (bytes already queued).
    esp_err_t err = SINK_CON_WAIT_TX(pdMS_TO_TICKS(SINK_USB_WRITE_TIMEOUT_MS));
    if (err == ESP_ERR_TIMEOUT) {
        return ESP_OK;
    }
    return err;
}

/* === Teardown ============================================================== */

void sink_transport_destroy(sink_transport_t transport)
{
    if (transport == NULL) {
        return;
    }

    if (transport->kind == SINK_TRANSPORT_UDP) {
        if (transport->u.udp.sockfd >= 0) {
            close(transport->u.udp.sockfd);
        }
    } else {
        // CONSOLE (unchanged): uninstall only if WE installed it.
        if (transport->u.console.we_installed_driver) {
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
            uart_driver_delete(SINK_CON_UART_PORT);
#else
            usb_serial_jtag_driver_uninstall();
#endif
        }
    }

    if (transport->tx_lock != NULL) {
        vSemaphoreDelete(transport->tx_lock);
    }

    free(transport);
}
