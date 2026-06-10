/**
 * @file    gps_ubx.c
 * @brief   One-shot UBX-CFG-VALSET boot configuration over the optional TX wire.
 *
 * @details
 *   When the config wire (P4 TX → module RX) is present, ADSBin sends a single
 *   UBX-CFG-VALSET burst at boot to put the MAX-M10S into a known-good mode:
 *
 *     - enable the 1PPS TIMEPULSE, locked to GNSS time (the precise edge the PPS
 *       clock-discipline layer captures),
 *     - set a 1 Hz measurement + navigation rate (one fix, one PPS edge per sec),
 *     - enable GGA + RMC NMEA on this UART and silence the sentences the parser
 *       ignores (GLL/GSA/GSV/VTG), trimming the serial budget.
 *
 *   The M10 uses the configuration-database (VALSET) interface — a flat key/value
 *   store addressed by 32-bit configuration keys — not the legacy CFG-* messages.
 *   We write to the RAM layer only (not the battery-backed flash), so the change
 *   is non-destructive: a power cycle reverts to factory defaults and the firmware
 *   simply re-applies on the next boot.
 *
 *   == UBX frame format ==
 *     [0xB5 0x62] [class] [id] [len_LSB len_MSB] [payload…] [ck_a ck_b]
 *   where (ck_a, ck_b) is the 8-bit Fletcher checksum over class..end-of-payload.
 *   CFG-VALSET is class 0x06, id 0x8A; its payload is:
 *     [version=0x01] [layers] [reserved0 reserved1] then repeated (key32, value…).
 *
 * @par Provenance
 *   Frame layout and configuration-key IDs are from the PUBLIC u-blox M10 / UBX
 *   protocol interface description. No vendor code was copied.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "gps_ubx.h"

#include <string.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "gps_ubx";

/* ───────────────────────────────────────────────────────────────────────────
 *  UBX constants
 * ─────────────────────────────────────────────────────────────────────────── */

#define UBX_SYNC1          0xB5
#define UBX_SYNC2          0x62
#define UBX_CLASS_CFG      0x06
#define UBX_ID_CFG_VALSET  0x8A

#define UBX_VALSET_VERSION 0x01
#define UBX_LAYER_RAM      0x01   /**< Apply to the volatile RAM layer only.        */

/* ───────────────────────────────────────────────────────────────────────────
 *  Configuration keys (from the public M10 interface description).
 *
 *  Key encoding: bits[31:28] reserved, [27:24] = value size class, the rest is
 *  the group+item id. We only need the literal key constants below; the size of
 *  each value (1/2/4 bytes) is tracked by us when we append it.
 *    - CFG-RATE-MEAS  : measurement period, ms (U2). 1000 = 1 Hz.
 *    - CFG-RATE-NAV   : nav solutions per measurement (U2). 1.
 *    - CFG-TP-* PULSE : enable timepulse + lock to GNSS, 1 Hz.
 *    - CFG-MSGOUT-NMEA_* : per-sentence per-port output rate (U1). UART1 keys.
 * ─────────────────────────────────────────────────────────────────────────── */
#define CFG_RATE_MEAS              0x30210001u   /* U2 ms   */
#define CFG_RATE_NAV               0x30210002u   /* U2      */

#define CFG_TP_PULSE_DEF           0x20050023u   /* E1: 0=period(us),1=freq(Hz) */
#define CFG_TP_TP1_ENA             0x10050007u   /* L  : enable TIMEPULSE 1     */
#define CFG_TP_USE_LOCKED_TP1      0x10050009u   /* L  : use locked params once GNSS-locked */
#define CFG_TP_FREQ_TP1            0x40050024u   /* U4 Hz: unlocked pulse freq  */
#define CFG_TP_FREQ_LOCK_TP1       0x40050025u   /* U4 Hz: locked pulse freq    */

#define CFG_MSGOUT_NMEA_GGA_UART1  0x209100bbu   /* U1 rate */
#define CFG_MSGOUT_NMEA_RMC_UART1  0x209100acu   /* U1 rate */
#define CFG_MSGOUT_NMEA_GLL_UART1  0x209100cau   /* U1 rate (silence) */
#define CFG_MSGOUT_NMEA_GSA_UART1  0x209100c0u   /* U1 rate (silence) */
#define CFG_MSGOUT_NMEA_GSV_UART1  0x209100c5u   /* U1 rate (silence) */
#define CFG_MSGOUT_NMEA_VTG_UART1  0x209100b1u   /* U1 rate (silence) */

/* ───────────────────────────────────────────────────────────────────────────
 *  A tiny builder that appends key/value pairs into a payload buffer, tracking
 *  the running length, then frames + checksums the whole thing.
 * ─────────────────────────────────────────────────────────────────────────── */

#define UBX_MAX_PAYLOAD  192   /**< Plenty for our ~12 key/value pairs.            */

typedef struct {
    uint8_t buf[UBX_MAX_PAYLOAD];
    size_t  len;
    bool    ok;     /**< Cleared if any append would overflow (defensive).        */
} ubx_payload_t;

/** @brief Little-endian append of @p nbytes from @p val (LSB first, UBX order). */
static void ubx_put(ubx_payload_t *p, uint64_t val, size_t nbytes)
{
    if (p->len + nbytes > sizeof(p->buf)) {
        p->ok = false;
        return;
    }
    for (size_t i = 0; i < nbytes; ++i) {
        p->buf[p->len++] = (uint8_t)((val >> (8 * i)) & 0xFFu);
    }
}

/** @brief Append one CFG-VALSET key (U4) followed by its @p vsize-byte value. */
static void ubx_kv(ubx_payload_t *p, uint32_t key, uint64_t value, size_t vsize)
{
    ubx_put(p, key, 4);
    ubx_put(p, value, vsize);
}

/**
 * @brief Frame the VALSET payload and write the complete UBX message to the UART.
 */
static esp_err_t ubx_send(int uart_num, const ubx_payload_t *p)
{
    if (!p->ok) {
        return ESP_ERR_NO_MEM;
    }

    // Header: sync, class, id, length (LE). Then the payload, then the checksum.
    uint8_t hdr[6] = {
        UBX_SYNC1, UBX_SYNC2, UBX_CLASS_CFG, UBX_ID_CFG_VALSET,
        (uint8_t)(p->len & 0xFFu), (uint8_t)((p->len >> 8) & 0xFFu),
    };

    // Fletcher-8 checksum runs over class..last payload byte (i.e. hdr[2..] + buf).
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < sizeof(hdr); ++i) { ck_a += hdr[i]; ck_b += ck_a; }
    for (size_t i = 0; i < p->len; ++i)      { ck_a += p->buf[i]; ck_b += ck_a; }
    uint8_t ck[2] = { ck_a, ck_b };

    // Write the three segments. A short/failed write is surfaced to the caller.
    if (uart_write_bytes((uart_port_t)uart_num, hdr, sizeof(hdr)) != (int)sizeof(hdr)) {
        return ESP_FAIL;
    }
    if (uart_write_bytes((uart_port_t)uart_num, p->buf, p->len) != (int)p->len) {
        return ESP_FAIL;
    }
    if (uart_write_bytes((uart_port_t)uart_num, ck, sizeof(ck)) != (int)sizeof(ck)) {
        return ESP_FAIL;
    }
    // Block briefly until the bytes are on the wire before we move on.
    return uart_wait_tx_done((uart_port_t)uart_num, pdMS_TO_TICKS(100));
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Public seam
 * ─────────────────────────────────────────────────────────────────────────── */

esp_err_t gps_ubx_configure(int uart_num)
{
    ubx_payload_t p = { .len = 0, .ok = true };

    // VALSET header: version, layer = RAM, two reserved bytes.
    ubx_put(&p, UBX_VALSET_VERSION, 1);
    ubx_put(&p, UBX_LAYER_RAM, 1);
    ubx_put(&p, 0, 2);   // reserved0, reserved1

    // 1 Hz solution: one measurement per second, one nav fix per measurement.
    ubx_kv(&p, CFG_RATE_MEAS, 1000u, 2);   // 1000 ms
    ubx_kv(&p, CFG_RATE_NAV,  1u,    2);   // 1 nav/meas

    // Timepulse: 1 Hz, frequency mode, enabled, and use the LOCKED frequency once
    // the receiver has GNSS time (so the captured edge is GNSS-disciplined).
    ubx_kv(&p, CFG_TP_PULSE_DEF,      1u, 1);   // 1 => frequency mode (Hz)
    ubx_kv(&p, CFG_TP_TP1_ENA,        1u, 1);   // enable TIMEPULSE 1
    ubx_kv(&p, CFG_TP_USE_LOCKED_TP1, 1u, 1);   // switch to locked params when locked
    ubx_kv(&p, CFG_TP_FREQ_TP1,       1u, 4);   // 1 Hz before lock
    ubx_kv(&p, CFG_TP_FREQ_LOCK_TP1,  1u, 4);   // 1 Hz after lock

    // NMEA trim on UART1: GGA + RMC on (rate 1), the rest off (rate 0).
    ubx_kv(&p, CFG_MSGOUT_NMEA_GGA_UART1, 1u, 1);
    ubx_kv(&p, CFG_MSGOUT_NMEA_RMC_UART1, 1u, 1);
    ubx_kv(&p, CFG_MSGOUT_NMEA_GLL_UART1, 0u, 1);
    ubx_kv(&p, CFG_MSGOUT_NMEA_GSA_UART1, 0u, 1);
    ubx_kv(&p, CFG_MSGOUT_NMEA_GSV_UART1, 0u, 1);
    ubx_kv(&p, CFG_MSGOUT_NMEA_VTG_UART1, 0u, 1);

    esp_err_t err = ubx_send(uart_num, &p);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "UBX-CFG-VALSET sent: 1Hz + timepulse + GGA/RMC-only NMEA");
    } else {
        ESP_LOGW(TAG, "UBX-CFG-VALSET write failed (%s) - using module defaults",
                 esp_err_to_name(err));
    }
    return err;
}
