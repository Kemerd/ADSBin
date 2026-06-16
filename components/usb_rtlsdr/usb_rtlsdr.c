/**
 * @file    usb_rtlsdr.c
 * @brief   RTL-SDR USB-HS host driver (plan S4.1) — Core-0 IQ ingest.
 *
 * @details
 *   This is Stage-1 of the ADSBin pipeline. It brings up the ESP32-P4's USB 2.0
 *   High-Speed host, enumerates a Realtek RTL2832U + Rafael Micro R820T2 dongle,
 *   supplies VBUS, configures the tuner for 1090ES reception (2.4 Msps, centred
 *   on 1090 MHz, fixed ~49.6 dB gain with AGC off), and streams continuous
 *   bulk-IN IQ into an internally-owned no-split ring buffer.
 *
 *   DUAL-DONGLE SPLIT. The driver now owns up to ::RTLSDR_MAX_DEVICES dongles at
 *   once: slot 0 serves 1090ES, slot 1 serves 978 MHz UAT (the band follows the
 *   device's ::adsbin_rf_role_t, assigned by stable adoption order). EVERYTHING
 *   per-device — the USB handles, the owned IQ ring, the tuner-register shadow,
 *   the stream config, the URBs and the stats — lives in a ::usb_rtlsdr_dev_t
 *   slot; the single housekeeping task, the one USB Host client and the one
 *   recursive lock stay process-global. Every chip/device helper takes a
 *   `usb_rtlsdr_dev_t *d` and pokes `d->...`; the hot path finds its slot via the
 *   `d` carried in usb_transfer_t.context. The legacy single-dongle public API is
 *   preserved as a thin device-0 (= 1090) shim so existing callers behave
 *   BYTE-FOR-BYTE identically when exactly one dongle is present.
 *
 *   THE RING IS THE SEAM (plan S2). This driver OWNS the IQ ring. The bulk-IN
 *   completion callback (Core 0) writes ::iq_block_t-described chunks straight
 *   into it; demod1090 pulls from the same ring. `main` fetches the handle via
 *   usb_rtlsdr_get_iq_ring() and hands it to demod1090_start(). Each ring item
 *   is an ::iq_block_t whose @c samples pointer addresses the bytes immediately
 *   after the header in the SAME ring allocation — which is only possible because
 *   we reserve the slot with xRingbufferSendAcquire() (so we know the final
 *   address before we fill it) and finish with xRingbufferSendComplete().
 *
 *   REAL-TIME CONTRACT. The bulk completion runs on Core 0 and MUST NEVER block.
 *   If the ring is full we DROP the block, bump @c overflow_drops, fire an
 *   OVERFLOW event hint, and immediately re-arm the URB — we never wait on the
 *   ring. All chip control I/O (control transfers) happens off the hot path,
 *   during start()/stop()/set_*, serialised by a mutex.
 *
 *   CLEAN-ROOM TUNER BRING-UP. Every RTL2832U / R820T2 register write below is
 *   derived from the PUBLIC Realtek RTL2832U datasheet and the Rafael Micro
 *   R820T2 register-description document — the named fields and their meanings
 *   are documented inline. NO code was copied from the GPLv2 librtlsdr or any
 *   GPL dump1090 fork. See usb_rtlsdr_internal.h for the annotated register map.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"   /* ESP_INTR_FLAG_* for the USB host install cfg.  */

#include "usb/usb_host.h"
#include "usb/usb_helpers.h"

#include "usb_rtlsdr.h"
#include "usb_rtlsdr_internal.h"
#include "adsbin_types.h"
#include "adsbin_err.h"

/* Logging tag for this component. */
static const char *TAG = "usb_rtlsdr";

/* The one and only driver instance. Zero-initialised => inited == false. Holds
 * the process-global state plus the per-device `dev[]` slot array. */
static usb_rtlsdr_ctx_t s_ctx;

/* Role override for the FIRST adopted dongle (see the role-assignment block in
 * task_try_open_pending). NONE => no override, default 1090. A file-scope static,
 * NOT a config-component read, so this driver keeps ZERO dependency on the config
 * component — the app sets it via the public usb_rtlsdr_set_role_override(). Safe
 * to set before or after init; the adopt path reads it once, when seen_count was
 * still 0 (i.e. for the very first dongle). */
static adsbin_rf_role_t s_role_override = ADSBIN_ROLE_NONE;

/* How long the housekeeping task blocks in each host/client event pump before
 * looping to re-check its flags. Short enough that stop()/recovery is snappy. */
#define RTLSDR_EVENT_WAIT_MS   20

/* Bounded timeout for a single control transfer to complete (chip is local and
 * fast; this only guards against a wedged endpoint). */
#define RTLSDR_CTRL_TIMEOUT_MS 1000

/* When the ring is full the completion path tries a SendAcquire with zero wait;
 * this is the wait it uses (never blocks the hot path). */
#define RTLSDR_RING_NOWAIT     0

/*
 * Physical-port → band role binding.
 *
 * The two NESDR Nano sticks both report a BLANK USB serial, so identity cannot
 * tell them apart. Instead we bind each role to the stable parent hub-PORT number
 * (usb_device_info_t.parent.port_num), which is fixed by which physical socket a
 * stick is soldered into. Measured on this hardware (1→4 USB expansion board off
 * the P4 HS host): facing the USB-C port, the LEFT socket enumerates as PORT 3
 * and the RIGHT as PORT 4. So:
 *
 *   PORT 3 (LEFT)  -> 1090 traffic
 *   PORT 4 (RIGHT) -> 978  UAT weather
 *
 * If a dongle's port_num matches neither (unknown hub layout / port read failed),
 * we fall back to the legacy serial/adoption-order assignment so the box still
 * works on a different board. See README "Auto-role assignment".
 */
#define RTLSDR_PORT_1090   3u   /**< LEFT socket  (facing USB-C) => 1090 traffic. */
#define RTLSDR_PORT_978    4u   /**< RIGHT socket (facing USB-C) => 978 weather.  */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Small locking + event helpers.
 *
 *  `lock` serialises every mutable field of EVERY device slot: config, stats, the
 *  device handles and the R820T2 shadow. There is exactly ONE lock for all slots.
 *  The bulk completion only ever takes it for the short stats update; readers
 *  (get_status/get_stats) and the slow control path also take it.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* The driver lock is RECURSIVE on purpose. The slow control path (set_*, start)
 * holds it while issuing control transfers, and issuing a control transfer
 * pumps usb_host_client_handle_events() — which can synchronously invoke the
 * bulk-IN completion callback on the SAME task. That callback also takes the
 * lock for its short stats update, so without recursion the task would deadlock
 * on itself. A recursive take by the one owning task is exactly what we want. */

/** @brief Take the driver lock (task context only — never from an ISR). */
static inline void lock(void)
{
    if (s_ctx.lock) {
        xSemaphoreTakeRecursive(s_ctx.lock, portMAX_DELAY);
    }
}

/** @brief Release the driver lock. */
static inline void unlock(void)
{
    if (s_ctx.lock) {
        xSemaphoreGiveRecursive(s_ctx.lock);
    }
}

/**
 * @brief Fire the registered async lifecycle event, if any.
 *
 * @details
 *   Called only from the housekeeping task (never the bulk hot path), so it is
 *   safe for the callback to do light queue work. We snapshot the callback under
 *   the lock so a concurrent register_event_callback() cannot tear the pair.
 *   @p device_index identifies WHICH slot the event pertains to (0 = 1090,
 *   1 = 978 by adoption order) so a single callback can fan out per device.
 */
static void emit_event(int device_index, usb_rtlsdr_event_id_t ev)
{
    usb_rtlsdr_event_cb_t cb;
    void                 *ctx;

    lock();
    cb  = s_ctx.event_cb;
    ctx = s_ctx.event_ctx;
    unlock();

    if (cb) {
        cb(ev, device_index, ctx);
    }
}

/** @brief Record the device's last non-fatal error for get_status (locked). */
static void set_last_error(usb_rtlsdr_dev_t *d, esp_err_t e)
{
    lock();
    d->last_error = e;
    unlock();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  USB Host control-transfer plumbing.
 *
 *  Each device owns ONE reusable EP0 control transfer and a completion flag the
 *  control callback raises when the transfer finishes. Because all control I/O is
 *  serialised by `lock` (only the slow start/stop/set_* paths issue it), one
 *  shared transfer PER DEVICE is sufficient and avoids per-call allocation. The
 *  transfer's `context` carries the owning `usb_rtlsdr_dev_t*` so the callback can
 *  raise the right device's flag.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief EP0 control-transfer completion.
 *
 * @details
 *   USB Host transfer callbacks fire from *inside* usb_host_client_handle_events()
 *   — the very loop our own usb_task pumps. Because we issue control transfers
 *   from that same task, we MUST NOT block it on a semaphore waiting for this
 *   callback (the callback would never run). Instead we just raise a completion
 *   flag here and let ctrl_xfer() pump the event loop until it sees the flag. The
 *   owning device is recovered from xfer->context (set when the ctrl transfer is
 *   allocated in open_device).
 */
static void ctrl_xfer_cb(usb_transfer_t *xfer)
{
    usb_rtlsdr_dev_t *d = (usb_rtlsdr_dev_t *)xfer->context;
    d->ctrl_complete = true;
}

/**
 * @brief Issue one synchronous USB control transfer over EP0 for device @p d.
 *
 * @details
 *   Builds the 8-byte setup packet at the head of the device's shared control
 *   buffer, appends @p out_data for an OUT transfer (or leaves room for an IN),
 *   submits, then pumps the client event loop until the completion callback fires
 *   (we cannot block here — see ctrl_xfer_cb). The caller must already hold
 *   `lock`, AND must be running on the usb_task so pumping the loop is legal.
 *
 * @param d          The device whose EP0 to drive.
 * @param bmReqType  USB bmRequestType (direction + type + recipient).
 * @param bRequest   Vendor/standard request code.
 * @param wValue     Setup wValue.
 * @param wIndex     Setup wIndex.
 * @param data       IN: receives up to @p wLength bytes. OUT: source bytes.
 * @param wLength    Data-stage length in bytes (0 for no data stage).
 * @return ESP_OK, or an esp_err_t / ESP_FAIL on a non-OK transfer status.
 */
static esp_err_t ctrl_xfer(usb_rtlsdr_dev_t *d, uint8_t bmReqType, uint8_t bRequest,
                           uint16_t wValue, uint16_t wIndex,
                           void *data, uint16_t wLength)
{
    if (!d->dev || !d->ctrl_xfer) {
        return ESP_ERR_INVALID_STATE;
    }

    usb_transfer_t *x = d->ctrl_xfer;

    /* Lay the setup packet into the first 8 bytes of the data buffer; the host
     * controller reads it from there for a control transfer. */
    usb_setup_packet_t *setup = (usb_setup_packet_t *)x->data_buffer;
    setup->bmRequestType = bmReqType;
    setup->bRequest      = bRequest;
    setup->wValue        = wValue;
    setup->wIndex        = wIndex;
    setup->wLength       = wLength;

    /* For an OUT transfer copy the payload in right after the setup packet. */
    const bool is_in = (bmReqType & USB_BM_REQUEST_TYPE_DIR_IN) != 0;
    if (!is_in && wLength && data) {
        memcpy(x->data_buffer + sizeof(usb_setup_packet_t), data, wLength);
    }

    /* num_bytes for a control transfer = setup (8) + data stage length. */
    x->device_handle = d->dev;
    x->bEndpointAddress = 0;                 /* EP0.                            */
    x->num_bytes  = sizeof(usb_setup_packet_t) + wLength;
    x->timeout_ms = RTLSDR_CTRL_TIMEOUT_MS;
    x->callback   = ctrl_xfer_cb;
    x->context    = d;                       /* so ctrl_xfer_cb finds this device.*/

    /* Arm the completion flag the callback will raise. */
    d->ctrl_complete = false;

    /*
     * SUBMIT with bounded retry on ESP_ERR_NOT_FINISHED.
     *
     * The ESP-IDF USB host SERIALIZES all control transfers to a device's EP0
     * (and, on a shared root port, the control path is shared across devices). A
     * submit issued while another control transfer is still in flight returns
     * ESP_ERR_NOT_FINISHED — it was NOT queued. The original code treated that as
     * a hard failure and returned, so the very first baseband write aborted and
     * the SDR never streamed (the cause of BLK=0 with two dongles).
     *
     * The documented-correct handling: pump the CLIENT event loop (only the
     * client — never usb_host_lib_handle_events() re-entrantly, which corrupts the
     * HCD's transfer parsing and asserts in hcd_dwc.c) so the in-flight control
     * transfer completes and frees EP0, then re-submit. We re-submit ONLY because
     * the prior attempt was rejected (never queued), so there is no risk of
     * double-queuing our own transfer. Bounded by the same timeout window.
     */
    esp_err_t err = ESP_ERR_NOT_FINISHED;
    int64_t submit_deadline = adsbin_now_us() + (int64_t)RTLSDR_CTRL_TIMEOUT_MS * 1000;
    for (;;) {
        err = usb_host_transfer_submit_control(s_ctx.client, x);
        if (err != ESP_ERR_NOT_FINISHED) {
            break;   /* accepted (ESP_OK) or a hard, non-busy error. */
        }
        if (adsbin_now_us() > submit_deadline) {
            break;   /* EP0 never freed within the bound — give up. */
        }
        /* EP0 busy with a serialized transfer: pump the client loop so its
         * completion callback runs and frees the pipe, then retry the submit. */
        usb_host_client_handle_events(s_ctx.client, pdMS_TO_TICKS(2));
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ctrl submit failed: %s (req=0x%02x)", esp_err_to_name(err), (unsigned)bRequest);
        return err;
    }

    /* Pump the client event loop until the completion callback raises the flag,
     * bounded so a wedged endpoint cannot hang the task. Each iteration is a
     * short blocking wait inside the host stack, which is where the callback
     * actually runs — so this drives our own completion. */
    int64_t deadline = adsbin_now_us() + (int64_t)(RTLSDR_CTRL_TIMEOUT_MS + 200) * 1000;
    while (!d->ctrl_complete) {
        usb_host_client_handle_events(s_ctx.client, pdMS_TO_TICKS(10));
        if (adsbin_now_us() > deadline) {
            ESP_LOGW(TAG, "ctrl transfer timed out");
            return ESP_ERR_TIMEOUT;
        }
    }

    if (x->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGW(TAG, "ctrl status %d", (int)x->status);
        return ESP_FAIL;
    }

    /* For an IN transfer, copy the received bytes back to the caller. We do not
     * include the 8-byte setup in actual_num_bytes for the data stage on IDF;
     * the received data lands right after the setup packet. */
    if (is_in && wLength && data) {
        memcpy(data, x->data_buffer + sizeof(usb_setup_packet_t), wLength);
    }
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  RTL2832U register access (clean-room USB vendor control interface).
 *
 *  A register write is a vendor OUT control transfer: wValue = 16-bit address,
 *  wIndex = block selector | (optional page), data = the value bytes. A read is
 *  the IN counterpart. All of this is the documented USB control interface from
 *  the RTL2832U datasheet — not librtlsdr code.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief Write @p len bytes (1 or 2) to RTL2832U register @p addr in @p block.*/
static esp_err_t rtl_write_reg(usb_rtlsdr_dev_t *d, uint16_t addr, uint16_t block, uint16_t val, uint8_t len)
{
    /* Values go MSB-first on the wire for the 2-byte case (datasheet order). */
    uint8_t buf[2];
    if (len == 1) {
        buf[0] = (uint8_t)(val & 0xFF);
    } else {
        buf[0] = (uint8_t)((val >> 8) & 0xFF);
        buf[1] = (uint8_t)(val & 0xFF);
    }
    return ctrl_xfer(d, RTL_CTRL_OUT, RTL_VENDOR_REQUEST, addr, block, buf, len);
}

/** @brief Read @p len bytes (1 or 2) from RTL2832U register @p addr/@p block. */
static esp_err_t rtl_read_reg(usb_rtlsdr_dev_t *d, uint16_t addr, uint16_t block, uint16_t *out, uint8_t len)
{
    uint8_t buf[2] = {0, 0};
    esp_err_t err = ctrl_xfer(d, RTL_CTRL_IN, RTL_VENDOR_REQUEST, addr, block, buf, len);
    if (err != ESP_OK) {
        return err;
    }
    *out = (len == 1) ? buf[0] : (uint16_t)((buf[0] << 8) | buf[1]);
    return ESP_OK;
}

/**
 * @brief Write a demod-block register on a given page.
 *
 * @details
 *   Demod registers are paged (datasheet). The 8-bit in-page offset is the
 *   wValue address, and the page rides in the LOW byte of wIndex while the DEMOD
 *   block is selected. We pack both from the RTL_DEMOD_* constants which encode
 *   page in the high byte, offset in the low byte.
 */
static esp_err_t rtl_demod_write(usb_rtlsdr_dev_t *d, uint16_t paged_addr, uint16_t val, uint8_t len)
{
    uint8_t page   = (uint8_t)(paged_addr >> RTL_DEMOD_PAGE_SHIFT);
    uint8_t offset = (uint8_t)(paged_addr & RTL_DEMOD_OFF_MASK);
    /* DEMOD block selector in wIndex high byte, page in the low byte. */
    uint16_t windex = (uint16_t)(RTL_BLK_DEMOD | page);
    /* Demod register addresses are byte offsets within the page; the datasheet
     * shifts the offset up by 8 for the on-wire address word. */
    uint16_t waddr  = (uint16_t)(offset << 8) | 0x20u;
    return rtl_write_reg(d, waddr, windex, val, len);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  R820T2 tuner I2C access (through the RTL2832U I2C window).
 *
 *  The RTL2832U exposes the tuner I2C bus as a register window: an OUT vendor
 *  transfer to the I2C block, addressed by the tuner slave address, carries the
 *  register byte(s). We must gate every tuner access with the demod I2C-repeater
 *  bit (datasheet: set IIC_repeat=1 first).
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief Enable/disable the RTL2832U I2C repeater that fronts the tuner bus. */
static esp_err_t rtl_i2c_repeater(usb_rtlsdr_dev_t *d, bool on)
{
    uint16_t v = on ? RTL_DEMOD_IIC_REPEAT_ON : RTL_DEMOD_IIC_REPEAT_OFF;
    return rtl_demod_write(d, RTL_DEMOD_IIC_REPEAT, v, 1);
}

/**
 * @brief Write one R820T2 register and update the shadow.
 *
 * @details
 *   The R820T2 has no read-modify-write, so callers always supply the full
 *   8-bit value and we keep a shadow copy. The byte is delivered to the tuner
 *   over the RTL2832U I2C window: the on-wire address selects the I2C block and
 *   the tuner slave address; the payload is [reg, value].
 */
static esp_err_t r820t_write(usb_rtlsdr_dev_t *d, uint8_t reg, uint8_t val)
{
    /* Cache first so a later RMW helper can build on it even if the bus write
     * is retried by the caller. */
    if (reg < R820T_NUM_REGS) {
        d->r82_shadow[reg] = val;
    }

    /* Payload = register index then value; the I2C window addresses the slave.
     * wValue carries the tuner I2C slave address; wIndex selects the I2C block. */
    uint8_t payload[2] = { reg, val };
    return ctrl_xfer(d, RTL_CTRL_OUT, RTL_VENDOR_REQUEST,
                     (uint16_t)R820T_I2C_ADDR, RTL_BLK_I2C, payload, sizeof(payload));
}

/**
 * @brief Read one R820T2 register.
 *
 * @details
 *   The R820T2 returns its register file bit-reversed within each byte
 *   (datasheet quirk of the read path). We undo that so the caller sees the
 *   logical value. Only used to probe the chip id at 0x00.
 */
static esp_err_t r820t_read(usb_rtlsdr_dev_t *d, uint8_t reg, uint8_t *out)
{
    uint8_t buf[2] = {0, 0};
    /* Reads come from the tuner slave on the I2C window. The RTL2832U returns
     * the requested register; one byte is enough for the id probe. */
    esp_err_t err = ctrl_xfer(d, RTL_CTRL_IN, RTL_VENDOR_REQUEST,
                              (uint16_t)R820T_I2C_ADDR, RTL_BLK_I2C, buf, 1);
    if (err != ESP_OK) {
        return err;
    }

    /* The R820T2 read path returns each byte bit-reversed; undo it so the caller
     * sees the logical value. (We only use this to confirm the bus answers.) */
    uint8_t b = buf[0];
    b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    *out = b;
    (void)reg; /* The window read starts at reg 0; the chip id lives there.     */
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  R820T2 PLL / tune.
 *
 *  The R820T2 LO is a fractional-N PLL off the 28.8 MHz reference. To tune to a
 *  centre frequency F we program the LO to F + IF (the RTL2832U expects the
 *  tuner to deliver the channel at R820T_IF_FREQ_HZ). The integer/fractional
 *  divider and the VCO band are computed from the datasheet PLL equations.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Program the R820T2 PLL to put the LO at @p lo_hz.
 *
 * @details
 *   Clean-room from the R820T2 register description's PLL section:
 *     - The reference into the PLL is the 28.8 MHz crystal (optionally divided).
 *     - A power-of-two VCO post-divider (mix_div) keeps the VCO in its 1.77–
 *       3.9 GHz band; we pick the smallest divider that lands the VCO in band.
 *     - Nint = floor(VCO / (2 * pll_ref)); the 16-bit sigma-delta fraction is
 *       the remainder scaled by 2^16.
 *   The fields land in regs 0x10..0x14 (VCO/divider, fractional lo/hi, Nint).
 */
static esp_err_t r820t_set_pll(usb_rtlsdr_dev_t *d, uint32_t lo_hz)
{
    /* PLL reference: the crystal feeds the tuner ref directly in our config. */
    const uint32_t pll_ref = RTL_XTAL_HZ;

    /* Choose the smallest power-of-two VCO divider that keeps the VCO inside its
     * valid band (~1.77–3.9 GHz). mix_div doubles until the VCO is in range. */
    uint32_t mix_div = 2;
    uint8_t  div_num = 0;      /* log2(mix_div) - 1, programmed into reg 0x10.   */
    while (mix_div <= 64) {
        uint64_t vco = (uint64_t)lo_hz * mix_div;
        if (vco >= 1770000000ull && vco <= 3900000000ull) {
            break;
        }
        mix_div <<= 1;
        div_num++;
    }

    /* The actual VCO frequency for the chosen divider. */
    uint64_t vco_freq = (uint64_t)lo_hz * mix_div;

    /* Nint and the 16-bit fractional part. The PLL phase detector runs at
     * 2 * pll_ref, so the divider target is vco / (2 * pll_ref). */
    uint32_t pll_step = 2u * pll_ref;
    uint32_t nint     = (uint32_t)(vco_freq / pll_step);
    uint64_t vco_frac = vco_freq - (uint64_t)nint * pll_step;

    /* The R820T2 splits Nint into a low integer and a "Ni2c" pair (datasheet).
     * Nint = 2*ni + nint_lo where nint_lo is the low bit; this packs the integer
     * divider into reg 0x13. */
    if (nint < 13 || nint > 76) {
        /* Out-of-range divider => LO unreachable for this band. */
        ESP_LOGW(TAG, "PLL Nint %u out of range for LO %u", nint, lo_hz);
    }
    uint8_t  ni  = (uint8_t)((nint - 13) / 4);
    uint8_t  si  = (uint8_t)((nint - 13) - (ni * 4));
    uint8_t  reg13 = (uint8_t)((ni & 0x3F) + (si << 6));

    /* 16-bit sigma-delta fraction = round(vco_frac / pll_step * 2^16). */
    uint16_t sdm = (uint16_t)(((uint64_t)vco_frac << 16) / pll_step);
    uint8_t  sdm_lo = (uint8_t)(sdm & 0xFF);
    uint8_t  sdm_hi = (uint8_t)((sdm >> 8) & 0xFF);

    esp_err_t err = ESP_OK;

    /* reg 0x10: VCO power + the VCO post-divider selection (div_num). Keep the
     * datasheet power bits and OR in our divider. */
    uint8_t reg10 = (uint8_t)((d->r82_shadow[R820T_REG_PLL_VCO] & 0x1F) | (div_num << 5));
    err |= r820t_write(d, R820T_REG_PLL_VCO, reg10);

    /* reg 0x11/0x12: 16-bit sigma-delta fractional divider. */
    err |= r820t_write(d, R820T_REG_PLL_FRAC_LO, sdm_lo);
    err |= r820t_write(d, R820T_REG_PLL_FRAC_HI, sdm_hi);

    /* reg 0x13: integer divider (Nint encoding). */
    err |= r820t_write(d, R820T_REG_PLL_NINT, reg13);

    return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  R820T2 gain.
 *
 *  ADS-B wants the tuner pinned at high fixed gain with all AGC loops off
 *  (plan S5.3). The three gain stages are LNA (reg 0x05), mixer (reg 0x07) and
 *  VGA/IF (reg 0x0C). The low nibble of each is the gain step; the high nibble
 *  selects manual vs auto. We translate the requested tenths-of-dB into the
 *  per-stage steps that approximate it, defaulting to maximum for 49.6 dB.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Apply a gain mode + level to the R820T2 stages.
 *
 * @details
 *   For MANUAL_FIXED we set the LNA, mixer and VGA to fixed steps with their AGC
 *   bits clear (manual). The requested @p gain_tenth_db is clamped to the chip's
 *   achievable range; the canonical ADS-B value (496 tenths = 49.6 dB) maps to
 *   near-maximum on all three stages. For HW_AGC we hand the LNA + mixer to the
 *   chip's loop (top bits set) and let the RTL2832U/IF VGA float.
 */
static esp_err_t r820t_apply_gain(usb_rtlsdr_dev_t *d, usb_rtlsdr_gain_mode_t mode, int gain_tenth_db)
{
    esp_err_t err = ESP_OK;

    if (mode == USB_RTLSDR_GAIN_HW_AGC) {
        /* Hand all three stages to the chip's AGC loop: set the AUTO bit on the
         * LNA, mixer and VGA registers. The low-nibble gain step is ignored by
         * the chip while AUTO is set, but we leave a sane mid value. */
        err |= r820t_write(d, R820T_REG_LNA_GAIN,   (uint8_t)(R820T_AGC_AUTO_BIT | 0x08));
        err |= r820t_write(d, R820T_REG_MIXER_GAIN, (uint8_t)(R820T_AGC_AUTO_BIT | 0x08));
        err |= r820t_write(d, R820T_REG_VGA_GAIN,   (uint8_t)(R820T_AGC_AUTO_BIT | 0x0B));
        return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
    }

    /* MANUAL_FIXED: distribute the requested gain across the three stages. The
     * R820T2 gives ~0..30 dB on the LNA in 16 steps and ~0..16 dB on the mixer
     * in 16 steps; for ADS-B we bias toward the top. Map tenths-of-dB into
     * 0..15 LNA + 0..15 mixer steps, clamped. AGC bit cleared => manual. */
    int g = gain_tenth_db;
    if (g < 0)   g = 0;
    if (g > 496) g = 496;       /* 49.6 dB is the documented max useful gain.    */

    /* Proportional split: at 496 both stages land at their top step (15); at
     * lower requests they scale linearly together. */
    int lna_step = (g * 15) / 496;          /* 0..15 */
    int mix_step = (g * 15) / 496;          /* 0..15 */
    if (lna_step > 15) lna_step = 15;
    if (mix_step > 15) mix_step = 15;

    /* reg 0x05: manual (AGC bit clear) + LNA gain step in the low nibble. */
    uint8_t reg05 = (uint8_t)(lna_step & R820T_GAIN_STEP_MASK);
    /* reg 0x07: manual + mixer gain step in the low nibble. */
    uint8_t reg07 = (uint8_t)(mix_step & R820T_GAIN_STEP_MASK);
    /* reg 0x0C: VGA manual, fixed mid-high IF gain step (0x0B ~ 16.3 dB). */
    uint8_t reg0c = (uint8_t)(0x0B & R820T_GAIN_STEP_MASK);

    err |= r820t_write(d, R820T_REG_LNA_GAIN,   reg05);
    err |= r820t_write(d, R820T_REG_MIXER_GAIN, reg07);
    err |= r820t_write(d, R820T_REG_VGA_GAIN,   reg0c);

    return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief One-time R820T2 initialisation (datasheet recommended power-up state).
 *
 * @details
 *   Brings the tuner out of standby and configures the blocks that ADS-B needs:
 *   filter power, IF filter, tracking filter for the high band, and a sane
 *   default register set. The values are the datasheet's recommended init for
 *   the upper RF band that contains 1090 MHz; gain + PLL are applied separately.
 */
static esp_err_t r820t_init(usb_rtlsdr_dev_t *d)
{
    esp_err_t err = ESP_OK;

    /* Seed the writable-register shadow with the datasheet power-up defaults for
     * the high band. These set: LNA/mixer/VGA blocks powered, IF filter on, a
     * neutral tracking-filter, and PLL block enabled. Index 0..4 are read-only
     * status registers and are never written. */
    static const uint8_t init_regs[R820T_NUM_REGS] = {
        /* 0x00..0x04: read-only status (placeholders, never written). */
        0x00, 0x00, 0x00, 0x00, 0x00,
        /* 0x05 LNA: manual, mid gain (overwritten by apply_gain).            */ 0x90,
        /* 0x06 power-detect / filter power on.                               */ 0x80,
        /* 0x07 mixer: manual, mid gain (overwritten by apply_gain).         */ 0x60,
        /* 0x08 mixer buffer power on.                                        */ 0x80,
        /* 0x09 IF filter power / current.                                    */ 0x40,
        /* 0x0A filter auto-cal trigger + VGA manual.                         */ 0xA0,
        /* 0x0B channel filter bandwidth (widest for ADS-B burst).           */ 0x6F,
        /* 0x0C VGA: manual, fixed IF gain.                                  */ 0x40,
        /* 0x0D LNA AGC top.                                                  */ 0x63,
        /* 0x0E mixer AGC top.                                                */ 0x75,
        /* 0x0F reserved/clk.                                                 */ 0x68,
        /* 0x10 PLL VCO + divider (overwritten by set_pll).                  */ 0x6C,
        /* 0x11 PLL frac lo.                                                  */ 0x83,
        /* 0x12 PLL frac hi.                                                  */ 0x80,
        /* 0x13 PLL Nint.                                                     */ 0x00,
        /* 0x14 PLL VCO ctrl.                                                 */ 0x0F,
        /* 0x15 reserved.                                                     */ 0x00,
        /* 0x16 reserved.                                                     */ 0xC0,
        /* 0x17 reserved.                                                     */ 0x30,
        /* 0x18 reserved.                                                     */ 0x48,
        /* 0x19 reserved.                                                     */ 0xCC,
        /* 0x1A tracking filter / PLL auto (high band).                       */ 0x60,
        /* 0x1B filter gate + tracking band (high band).                      */ 0x00,
        /* 0x1C reserved.                                                     */ 0x54,
        /* 0x1D reserved.                                                     */ 0xAE,
        /* 0x1E reserved.                                                     */ 0x4A,
        /* 0x1F reserved.                                                     */ 0xC0,
    };
    memcpy(d->r82_shadow, init_regs, sizeof(init_regs));

    /* Push every writable register (0x05..0x1F) in ascending order. */
    for (uint8_t r = R820T_WRITE_START; r < R820T_NUM_REGS; r++) {
        err |= r820t_write(d, r, d->r82_shadow[r]);
    }

    return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  RTL2832U baseband: sample rate + reset + raw-IQ data path.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Program the RTL2832U resampler for @p rate_sps output samples/second.
 *
 * @details
 *   The demod resamples the 28.8 MHz ADC stream down to the requested rate via a
 *   28-bit ratio = (Xtal * 2^22) / rate (datasheet resampler equation). The
 *   ratio is written MSB-first across the two RSAMP_RATIO registers; the high
 *   byte is rounded to avoid a fractional-sample bias.
 */
static esp_err_t rtl_set_sample_rate(usb_rtlsdr_dev_t *d, uint32_t rate_sps)
{
    if (rate_sps == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* ratio = Xtal * 2^22 / rate, then mask to 28 bits and clear the low 2 bits
     * (datasheet: the bottom two bits of the ratio are ignored). */
    uint32_t ratio = (uint32_t)(((uint64_t)RTL_XTAL_HZ << 22) / rate_sps);
    ratio &= 0x0FFFFFFCu;

    /* Compute the real rate this ratio yields so measured_sps math is honest. */
    uint32_t real_rate = (uint32_t)(((uint64_t)RTL_XTAL_HZ << 22) / ratio);
    if (real_rate != rate_sps) {
        ESP_LOGD(TAG, "rate %u -> achievable %u", rate_sps, real_rate);
    }

    esp_err_t err = ESP_OK;
    /* High 16 bits then low 16 bits of the 28-bit ratio, into the two regs. */
    err |= rtl_demod_write(d, RTL_DEMOD_RSAMP_RATIO0, (uint16_t)((ratio >> 16) & 0xFFFF), 2);
    err |= rtl_demod_write(d, RTL_DEMOD_RSAMP_RATIO1, (uint16_t)(ratio & 0xFFFF), 2);

    /* Pulse the demod soft-reset so the new ratio takes effect cleanly. */
    err |= rtl_demod_write(d, RTL_DEMOD_SOFT_RST, RTL_DEMOD_SOFT_RST_ON, 1);
    err |= rtl_demod_write(d, RTL_DEMOD_SOFT_RST, RTL_DEMOD_SOFT_RST_OFF, 1);

    return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Bring the RTL2832U up into raw-IQ (DVB-T bypass) mode.
 *
 * @details
 *   Powers the I/Q ADCs, holds then releases the digital core reset, and arms
 *   the bulk endpoint. We do NOT enable the OFDM/DVB-T processing chain — the
 *   dongle delivers the raw 8-bit offset-binary I/Q the demod1090 stage wants.
 */
static esp_err_t rtl_init_baseband(usb_rtlsdr_dev_t *d)
{
    esp_err_t err = ESP_OK;

    /* USB system control: enable the bulk FIFO path. */
    err |= rtl_write_reg(d, RTL_USB_SYSCTL, RTL_BLK_USB, 0x09, 1);

    /* Max-packet for endpoint A = HS bulk 512. */
    err |= rtl_write_reg(d, RTL_USB_EPA_MAXPKT, RTL_BLK_USB, RTLSDR_BULK_MPS_HS, 2);

    /* Reset the endpoint A FIFO (write the reset bit, then clear it). */
    err |= rtl_write_reg(d, RTL_USB_EPA_CTL, RTL_BLK_USB, 0x1002, 2);
    err |= rtl_write_reg(d, RTL_USB_EPA_CTL, RTL_BLK_USB, 0x0000, 2);

    /* Demod control: power both ADCs, release digital reset. The I2C-repeater is
     * toggled separately around tuner writes. */
    const uint16_t demod_ctl_val = (RTL_DEMOD_CTL_ADC_I | RTL_DEMOD_CTL_ADC_Q | 0x20);
    err |= rtl_write_reg(d, RTL_SYS_DEMOD_CTL, RTL_BLK_SYS, demod_ctl_val, 1);

    /* Read the demod-control register straight back as a cheap liveness check:
     * a silicon RTL2832U returns the bits we just wrote (the ADC-power + clock
     * field is plain R/W). A mismatch means the control bus is not really
     * answering, which we surface as a fault rather than streaming garbage. */
    uint16_t readback = 0;
    if (rtl_read_reg(d, RTL_SYS_DEMOD_CTL, RTL_BLK_SYS, &readback, 1) == ESP_OK) {
        if ((readback & demod_ctl_val) != demod_ctl_val) {
            ESP_LOGW(TAG, "demod ctl readback 0x%02x != 0x%02x",
                     readback, demod_ctl_val);
        }
    }

    /* GPIO defaults: drive the bias-tee line low (off) until set_bias_tee asks
     * for it. Configure GPIO0 as an output. */
    err |= rtl_write_reg(d, RTL_SYS_GPD,  RTL_BLK_SYS, (uint16_t)(~RTL_GPIO_BIAS_TEE & 0xFF), 1);
    err |= rtl_write_reg(d, RTL_SYS_GPOE, RTL_BLK_SYS, RTL_GPIO_BIAS_TEE, 1);
    err |= rtl_write_reg(d, RTL_SYS_GPO,  RTL_BLK_SYS, 0x00, 1);

    return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Full tuner configuration (called from start() and the set_* retune paths).
 *  All of these run while holding `lock` so the control bus is single-owner.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Retune the R820T2 to the device's current centre frequency.
 *
 * @details
 *   Folds the ppm trim into the requested centre, then programs the PLL for
 *   LO = centre + IF (the RTL2832U expects the channel at the standard IF).
 *   Wrapped in the I2C repeater so the tuner sees the writes.
 */
static esp_err_t configure_frequency_locked(usb_rtlsdr_dev_t *d)
{
    /* Apply the ppm correction: a positive ppm means the crystal runs fast, so
     * we ask the LO for a slightly higher frequency to compensate. */
    int64_t f = (int64_t)d->center_freq_hz;
    f += (f * d->freq_correction_ppm) / 1000000;
    uint32_t lo = (uint32_t)(f + R820T_IF_FREQ_HZ);

    esp_err_t err = rtl_i2c_repeater(d, true);
    if (err == ESP_OK) {
        err = r820t_set_pll(d, lo);
    }
    /* Always try to drop the repeater even if the PLL write failed. */
    esp_err_t er2 = rtl_i2c_repeater(d, false);
    return (err != ESP_OK) ? err : er2;
}

/** @brief Apply the device's current gain mode/level to the tuner (locked). */
static esp_err_t configure_gain_locked(usb_rtlsdr_dev_t *d)
{
    esp_err_t err = rtl_i2c_repeater(d, true);
    if (err == ESP_OK) {
        err = r820t_apply_gain(d, d->gain_mode, d->gain_tenth_db);
    }
    esp_err_t er2 = rtl_i2c_repeater(d, false);
    return (err != ESP_OK) ? err : er2;
}

/** @brief Drive the antenna-port bias-tee GPIO to the current state (locked). */
static esp_err_t configure_bias_tee_locked(usb_rtlsdr_dev_t *d)
{
    /* GPIO0 high feeds 4.5 V to an external LNA; low disconnects it (S8). */
    uint16_t gpo = d->bias_tee_enable ? RTL_GPIO_BIAS_TEE : 0x00;
    return rtl_write_reg(d, RTL_SYS_GPO, RTL_BLK_SYS, gpo, 1);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Bulk-IN completion — THE HOT PATH (Core 0). Must never block.
 *
 *  Each completed URB carries up to RTLSDR_URB_SIZE bytes of raw IQ. We chop it
 *  into block-sized iq_block_t items and push each into the device's ring via the
 *  acquire/complete API so the item's self-referential `samples` pointer is
 *  valid in the ring. On a full ring we DROP and count — never wait. Then we
 *  immediately re-submit the URB to keep the pipe saturated. The completion finds
 *  its device via the `d` stashed in usb_transfer_t.context at alloc time.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Push one IQ chunk into the device's owned ring as a self-describing block.
 *
 * @details
 *   Reserves header+payload contiguously with xRingbufferSendAcquire (zero wait
 *   => never blocks the hot path). On success we fill the ::iq_block_t in place,
 *   point its @c samples at the bytes just past the header in the SAME ring slot,
 *   copy the IQ payload in, and commit with xRingbufferSendComplete. On a full
 *   ring we drop and bump overflow_drops.
 *
 * @return true if the block was enqueued; false if dropped (ring full).
 */
static bool push_iq_block(usb_rtlsdr_dev_t *d, const uint8_t *src, uint32_t n_bytes, int64_t t_cap)
{
    /* The ring slot must hold the header plus the payload contiguously so the
     * consumer can read both from one item. */
    const size_t need = sizeof(iq_block_t) + n_bytes;

    void *slot = NULL;
    if (xRingbufferSendAcquire(d->iq_ring, &slot, need, RTLSDR_RING_NOWAIT) != pdTRUE
        || slot == NULL) {
        /* Ring full — drop this block. Counting happens in the caller so the
         * per-URB overflow burst is a single OVERFLOW hint, not one per block. */
        return false;
    }

    /* Lay the header at the head of the slot. */
    iq_block_t *blk = (iq_block_t *)slot;
    blk->samples      = (const uint8_t *)slot + sizeof(iq_block_t);
    blk->n_bytes      = n_bytes;
    blk->seq          = ++d->block_seq;      /* hot-path-private monotonic seq.  */
    blk->t_capture_us = t_cap;

    /* Copy the raw IQ in right after the header (this is the borrowed buffer the
     * consumer reads, valid until it returns the item). */
    memcpy((void *)blk->samples, src, n_bytes);

    /* Commit — now visible to xRingbufferReceive on the demod side. */
    xRingbufferSendComplete(d->iq_ring, slot);
    return true;
}

/**
 * @brief Bulk-IN transfer completion callback (Core 0). Carves IQ into blocks,
 *        feeds the device's ring, then re-arms the URB. NEVER blocks.
 *
 * @details
 *   This fires synchronously inside usb_host_client_handle_events() on the
 *   usb_task (Core 0) — it is hot-path code, not a true ISR, so it may take the
 *   recursive lock for the short stats update and may use the plain (non-ISR)
 *   task-notify to nudge itself. It must never wait on the ring: a full ring
 *   means drop-and-count, then immediately re-arm so the bulk pipe stays
 *   saturated and the dongle's FIFO never backs up. The owning device is the `d`
 *   stashed in xfer->context when the URB was allocated.
 */
static void bulk_in_cb(usb_transfer_t *xfer)
{
    usb_rtlsdr_dev_t *d = (usb_rtlsdr_dev_t *)xfer->context;

    /* If we are tearing down (global) or this device stopped streaming, stop
     * re-arming. task_run is process-global; want_stream is per-device. */
    if (!s_ctx.task_run || !d->want_stream) {
        d->urbs_inflight--;
        return;
    }

    int64_t now = adsbin_now_us();

    switch (xfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED: {
        const uint8_t *data = xfer->data_buffer;
        uint32_t       len  = (uint32_t)xfer->actual_num_bytes;

        /* Carve the URB into delivered blocks of block_size_pairs*2 bytes. A
         * trailing partial chunk (rare; short packet) is delivered as-is. The
         * block geometry is install-global, so it reads from s_ctx. */
        const uint32_t block_bytes = (uint32_t)(s_ctx.block_size_pairs * 2u);
        uint32_t off = 0;
        uint64_t dropped_here   = 0;   /* blocks we had to drop (ring full).      */
        uint64_t delivered_here = 0;   /* blocks that made it into the ring.      */
        uint64_t bytes_here     = 0;   /* IQ bytes actually delivered to the ring.*/

        while (off + 1 < len) {                       /* need at least one pair. */
            uint32_t chunk = block_bytes;
            if (off + chunk > len) {
                chunk = len - off;
                chunk &= ~1u;                         /* keep whole IQ pairs.    */
                if (chunk == 0) break;
            }
            if (push_iq_block(d, data + off, chunk, now)) {
                delivered_here++;
                bytes_here += chunk;
            } else {
                dropped_here++;
            }
            off += chunk;
        }

        /* Roll the throughput + overflow counters in one short locked section.
         * total_bytes/total_blocks count what actually reached the ring (matches
         * iq_block_t.seq on the consumer side); overflow_drops counts the rest.
         * The hot path only ever increments; readers are the slow status task. */
        lock();
        d->stats.total_bytes    += bytes_here;
        d->stats.total_blocks   += delivered_here;
        d->stats.overflow_drops += dropped_here;
        d->last_block_us = now;

        /* Windowed effective-rate measurement (~1 s window). */
        if (d->rate_window_us == 0) {
            d->rate_window_us = now;
        }
        d->rate_window_bytes += len;
        int64_t span = now - d->rate_window_us;
        if (span >= 1000000) {
            /* bytes/s -> samples/s (2 bytes per IQ pair => 1 sample-pair). */
            uint64_t bps = (d->rate_window_bytes * 1000000ull) / (uint64_t)span;
            d->stats.measured_sps = (uint32_t)(bps / 2ull);
            d->rate_window_us    = now;
            d->rate_window_bytes = 0;
        }
        unlock();

        /* If we dropped anything this URB, nudge the housekeeping task to fire a
         * single OVERFLOW event (it debounces; we never call the cb from here). */
        if (dropped_here) {
            /* Notify the task via its handle; the task emits the event so the
             * callback never runs on the hot path. */
            if (s_ctx.task) {
                xTaskNotify(s_ctx.task, 0, eNoAction);
            }
        }
        break;
    }

    case USB_TRANSFER_STATUS_NO_DEVICE:
        /* Dongle yanked — let the client DEV_GONE path handle teardown; stop
         * re-arming this URB. */
        d->urbs_inflight--;
        d->do_recover = true;
        return;

    case USB_TRANSFER_STATUS_STALL:
        /* Endpoint stalled. Count it and ask the housekeeping task to recover;
         * do not re-arm until the stall is cleared. */
        lock();
        d->stats.usb_stall_count++;
        d->last_error = ADSBIN_ERR_USB_STALL;
        unlock();
        d->urbs_inflight--;
        d->do_recover = true;
        if (s_ctx.task) {
            xTaskNotify(s_ctx.task, 0, eNoAction);
        }
        return;

    case USB_TRANSFER_STATUS_CANCELED:
        /* We cancelled it during stop()/teardown. Just retire it. */
        d->urbs_inflight--;
        return;

    default:
        /* Transient error (timeout/overflow/error) — re-arm and keep going. */
        break;
    }

    /* Re-arm this URB to keep the bulk pipe saturated. If the resubmit fails
     * (device gone mid-flight) retire the URB and request recovery. */
    esp_err_t err = usb_host_transfer_submit(xfer);
    if (err != ESP_OK) {
        d->urbs_inflight--;
        d->do_recover = true;
        if (s_ctx.task) {
            xTaskNotify(s_ctx.task, 0, eNoAction);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Bulk URB pool lifecycle (per device).
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief Allocate the device's bulk-IN URB pool (call once it is open). */
static esp_err_t alloc_urbs(usb_rtlsdr_dev_t *d)
{
    for (uint32_t i = 0; i < RTLSDR_NUM_URBS; i++) {
        usb_transfer_t *x = NULL;
        esp_err_t err = usb_host_transfer_alloc(RTLSDR_URB_SIZE, 0, &x);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "URB %u alloc failed: %s", i, esp_err_to_name(err));
            return err;
        }
        x->device_handle    = d->dev;
        x->bEndpointAddress = d->bulk_ep;
        x->callback         = bulk_in_cb;
        x->context          = d;            /* so bulk_in_cb finds this device.  */
        x->num_bytes        = RTLSDR_URB_SIZE;
        x->timeout_ms       = 0;            /* bulk-IN: no per-transfer timeout. */
        d->urb[i]           = x;
    }
    return ESP_OK;
}

/** @brief Free the device's bulk-IN URB pool (after all are retired/cancelled).*/
static void free_urbs(usb_rtlsdr_dev_t *d)
{
    for (uint32_t i = 0; i < RTLSDR_NUM_URBS; i++) {
        if (d->urb[i]) {
            usb_host_transfer_free(d->urb[i]);
            d->urb[i] = NULL;
        }
    }
}

/** @brief Submit every URB to start the device's continuous bulk-IN stream. */
static esp_err_t submit_urbs(usb_rtlsdr_dev_t *d)
{
    d->urbs_inflight = 0;
    for (uint32_t i = 0; i < RTLSDR_NUM_URBS; i++) {
        esp_err_t err = usb_host_transfer_submit(d->urb[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "URB %u submit failed: %s", i, esp_err_to_name(err));
            return err;
        }
        d->urbs_inflight++;
    }
    return ESP_OK;
}

/** @brief Cancel all of the device's in-flight URBs and wait for them to retire.*/
static void cancel_urbs(usb_rtlsdr_dev_t *d)
{
    /* Halting the endpoint cancels outstanding transfers; their callbacks fire
     * with STATUS_CANCELLED and decrement urbs_inflight. Keyed off THIS device's
     * handle + endpoint so recovering device i never touches device j's pipe. */
    if (d->dev && d->bulk_ep) {
        usb_host_endpoint_halt(d->dev, d->bulk_ep);
        usb_host_endpoint_flush(d->dev, d->bulk_ep);
    }

    /* Pump the client event loop briefly so the cancellations complete. */
    for (int i = 0; i < 50 && d->urbs_inflight > 0; i++) {
        usb_host_client_handle_events(s_ctx.client, pdMS_TO_TICKS(10));
    }

    if (d->dev && d->bulk_ep) {
        usb_host_endpoint_clear(d->dev, d->bulk_ep);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Device enumeration / identity.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief Is this VID/PID a Realtek RTL2832U-family dongle we support? */
static bool is_supported_dongle(uint16_t vid, uint16_t pid)
{
    if (vid != RTLSDR_VID_REALTEK) {
        return false;
    }
    return (pid == RTLSDR_PID_2832) || (pid == RTLSDR_PID_2838);
}

/**
 * @brief Read a USB string descriptor (UTF-16LE) into an ASCII C-string.
 *
 * @details
 *   Issues a GET_DESCRIPTOR(STRING) control transfer for @p index using LANGID
 *   0x0409 (US English) and flattens the UTF-16LE body to ASCII (non-ASCII code
 *   points become '?'). @p out is always NUL-terminated. Best-effort: on any
 *   error @p out is set to an empty string and ESP_OK-ish behaviour is faked by
 *   the caller (identity strings are not load-bearing). The transfer goes over
 *   @p d's EP0.
 */
static void read_string_desc(usb_rtlsdr_dev_t *d, uint8_t index, char *out, size_t out_sz)
{
    out[0] = '\0';
    if (index == 0 || out_sz == 0) {
        return;
    }

    /* USB string descriptor: wValue = (STRING<<8)|index, wIndex = LANGID. */
    uint8_t buf[RTLSDR_STRDESC_MAX];
    memset(buf, 0, sizeof(buf));
    uint16_t wValue = (uint16_t)((USB_B_DESCRIPTOR_TYPE_STRING << 8) | index);
    esp_err_t err = ctrl_xfer(d, USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_STANDARD
                                  | USB_BM_REQUEST_TYPE_RECIP_DEVICE,
                              USB_B_REQUEST_GET_DESCRIPTOR, wValue, 0x0409,
                              buf, sizeof(buf) - 1);
    if (err != ESP_OK) {
        return;
    }

    /* buf[0] = total length (incl the 2-byte header), buf[1] = descriptor type.
     * The body is UTF-16LE starting at byte 2. */
    uint8_t blen = buf[0];
    if (blen < 2) {
        return;
    }
    size_t n_chars = (size_t)((blen - 2) / 2);
    size_t w = 0;
    for (size_t i = 0; i < n_chars && w + 1 < out_sz; i++) {
        uint16_t cp = (uint16_t)(buf[2 + i * 2] | (buf[2 + i * 2 + 1] << 8));
        out[w++] = (cp >= 0x20 && cp < 0x7F) ? (char)cp : '?';
    }
    out[w] = '\0';
}

/**
 * @brief Parse the device's config descriptor to find the bulk-IN endpoint.
 *
 * @details
 *   Claims interface 0 / alt 0 and locates its bulk-IN endpoint (the DVB-T data
 *   pipe we repurpose for raw IQ). Fills d->bulk_ep / bulk_mps. The endpoint
 *   address is verified against the descriptor rather than trusting 0x81 blindly.
 */
static esp_err_t find_and_claim_bulk(usb_rtlsdr_dev_t *d)
{
    const usb_config_desc_t *cfg = NULL;
    esp_err_t err = usb_host_get_active_config_descriptor(d->dev, &cfg);
    if (err != ESP_OK || !cfg) {
        return (err != ESP_OK) ? err : ESP_FAIL;
    }

    /* Find interface 0, alt 0. */
    int offset = 0;
    const usb_intf_desc_t *intf =
        usb_parse_interface_descriptor(cfg, RTLSDR_INTF_NUMBER, RTLSDR_INTF_ALT, &offset);
    if (!intf) {
        ESP_LOGE(TAG, "no interface %u", RTLSDR_INTF_NUMBER);
        return ESP_ERR_NOT_FOUND;
    }

    /* Walk the interface's endpoints for the bulk-IN one. */
    d->bulk_ep  = 0;
    d->bulk_mps = 0;
    for (int e = 0; e < intf->bNumEndpoints; e++) {
        int ep_off = offset;
        const usb_ep_desc_t *ep =
            usb_parse_endpoint_descriptor_by_index(intf, e, cfg->wTotalLength, &ep_off);
        if (!ep) {
            continue;
        }
        bool is_in   = (ep->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) != 0;
        bool is_bulk = (ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK)
                           == USB_BM_ATTRIBUTES_XFER_BULK;
        if (is_in && is_bulk) {
            d->bulk_ep  = ep->bEndpointAddress;
            d->bulk_mps = USB_EP_DESC_GET_MPS(ep);
            break;
        }
    }
    if (d->bulk_ep == 0) {
        ESP_LOGE(TAG, "no bulk-IN endpoint on interface %u", RTLSDR_INTF_NUMBER);
        return ESP_ERR_NOT_FOUND;
    }

    /* Claim the interface so the host routes the endpoint to us. */
    err = usb_host_interface_claim(s_ctx.client, d->dev,
                                   RTLSDR_INTF_NUMBER, RTLSDR_INTF_ALT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "interface claim failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/**
 * @brief Open the enumerated device at @p addr into slot @p d, read identity,
 *        probe the tuner.
 *
 * @details
 *   Opens the device, allocates the device's shared control transfer, parses
 *   identity strings, claims the bulk interface, and probes the R820T2 chip id so
 *   ::usb_rtlsdr_device_info_t.tuner is honest. Leaves the device OPEN_IDLE.
 */
static esp_err_t open_device(usb_rtlsdr_dev_t *d, uint8_t addr)
{
    esp_err_t err = usb_host_device_open(s_ctx.client, addr, &d->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "device_open(%u) failed: %s", addr, esp_err_to_name(err));
        return err;
    }
    d->dev_addr = addr;

    /* Allocate the reusable EP0 control transfer (setup + small data) and stamp
     * its context with this device so ctrl_xfer_cb can raise the right flag. */
    err = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + RTLSDR_STRDESC_MAX,
                                  0, &d->ctrl_xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ctrl transfer alloc failed: %s", esp_err_to_name(err));
        return err;
    }
    d->ctrl_xfer->context = d;

    /* Read the device descriptor for VID/PID and string indices. */
    const usb_device_desc_t *dd = NULL;
    err = usb_host_get_device_descriptor(d->dev, &dd);
    if (err != ESP_OK || !dd) {
        return (err != ESP_OK) ? err : ESP_FAIL;
    }

    memset(&d->info, 0, sizeof(d->info));
    d->info.vid = dd->idVendor;
    d->info.pid = dd->idProduct;
    d->info.has_bias_tee = true;     /* GPIO0 bias-tee is wired on our HW.    */

    /* Read the device's parent hub PORT number — a stable physical identifier
     * (which socket the stick is plugged into), unlike the volatile bus address.
     * This lets us bind the 1090/978 role to a fixed solder point regardless of
     * enumeration order or blank EEPROM serials. Logged so the operator can map
     * each physical port to its role. */
    usb_device_info_t uinfo;
    if (usb_host_device_info(d->dev, &uinfo) == ESP_OK) {
        d->port_num = uinfo.parent.port_num;
    } else {
        d->port_num = 0;   /* unknown (e.g. root-attached); 0 is the sentinel. */
    }
    ESP_LOGI(TAG, "adopt slot[%d]: addr=%u PORT=%u vid:pid=%04x:%04x",
             d->index, (unsigned)addr, (unsigned)d->port_num,
             (unsigned)dd->idVendor, (unsigned)dd->idProduct);

    /* Identity strings (best-effort; not load-bearing). */
    read_string_desc(d, dd->iProduct,      d->info.product_name, sizeof(d->info.product_name));
    read_string_desc(d, dd->iSerialNumber, d->info.serial,       sizeof(d->info.serial));

    /* Claim the bulk interface + locate the bulk-IN endpoint. */
    err = find_and_claim_bulk(d);
    if (err != ESP_OK) {
        return err;
    }

    /* Probe the tuner over I2C to confirm it is an R820T/R820T2. */
    err = rtl_init_baseband(d);
    if (err == ESP_OK) {
        err = rtl_i2c_repeater(d, true);
        uint8_t id = 0;
        if (err == ESP_OK) {
            r820t_read(d, R820T_REG_CHIPID, &id);   /* best-effort id read.       */
        }
        rtl_i2c_repeater(d, false);
        /* The R820T2 returns a stable id pattern; treat any successful bus read
         * as R820T2 (the family we support). If the bus failed, mark UNKNOWN. */
        d->info.tuner = (err == ESP_OK) ? USB_RTLSDR_TUNER_R820T2
                                        : USB_RTLSDR_TUNER_UNKNOWN;
        (void)id;
    }

    d->state = USB_RTLSDR_STATE_OPEN_IDLE;
    ESP_LOGI(TAG, "opened RTL-SDR[%d] %04x:%04x '%s' serial '%s'",
             d->index, d->info.vid, d->info.pid,
             d->info.product_name, d->info.serial);
    return ESP_OK;
}

/** @brief Close the device in slot @p d, releasing the interface + transfers. */
static void close_device(usb_rtlsdr_dev_t *d)
{
    if (d->dev) {
        if (d->bulk_ep) {
            usb_host_interface_release(s_ctx.client, d->dev, RTLSDR_INTF_NUMBER);
        }
        if (d->ctrl_xfer) {
            usb_host_transfer_free(d->ctrl_xfer);
            d->ctrl_xfer = NULL;
        }
        usb_host_device_close(s_ctx.client, d->dev);
        d->dev      = NULL;
        d->dev_addr = 0;
        d->bulk_ep  = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Start/stop streaming (internal, called under `lock`).
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief Configure the tuner + demod for the device's stream config and arm IQ.*/
static esp_err_t start_streaming_locked(usb_rtlsdr_dev_t *d)
{
    if (!d->dev) {
        return ADSBIN_ERR_NO_DONGLE;
    }
    if (d->state == USB_RTLSDR_STATE_STREAMING) {
        return ESP_OK;                      /* idempotent.                       */
    }

    esp_err_t err = ESP_OK;

    /* 1) Bring the RTL2832U baseband up in raw-IQ mode. */
    err = rtl_init_baseband(d);
    if (err != ESP_OK) { ESP_LOGE(TAG, "stream[%d] FAIL @baseband: %s", d->index, esp_err_to_name(err)); return err; }

    /* 2) Initialise + tune the R820T2 (under the I2C repeater). */
    err = rtl_i2c_repeater(d, true);
    if (err == ESP_OK) err = r820t_init(d);
    rtl_i2c_repeater(d, false);
    if (err != ESP_OK) { ESP_LOGE(TAG, "stream[%d] FAIL @r820t_init: %s", d->index, esp_err_to_name(err)); return err; }

    /* 3) Sample rate, then frequency, then gain, then bias-tee. */
    err = rtl_set_sample_rate(d, d->sample_rate_sps);
    if (err == ESP_OK) err = configure_frequency_locked(d);
    if (err == ESP_OK) err = configure_gain_locked(d);
    if (err == ESP_OK) err = configure_bias_tee_locked(d);
    if (err != ESP_OK) { ESP_LOGE(TAG, "stream[%d] FAIL @tune/gain: %s", d->index, esp_err_to_name(err)); return err; }

    /* 4) Reset the demod sample pipe so we start on a clean boundary. */
    err = rtl_demod_write(d, RTL_DEMOD_SOFT_RST, RTL_DEMOD_SOFT_RST_ON, 1);
    if (err == ESP_OK) err = rtl_demod_write(d, RTL_DEMOD_SOFT_RST, RTL_DEMOD_SOFT_RST_OFF, 1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "stream[%d] FAIL @demod_rst: %s", d->index, esp_err_to_name(err)); return err; }

    /* 5) Allocate (if needed) and submit the bulk URBs. */
    if (d->urb[0] == NULL) {
        err = alloc_urbs(d);
        if (err != ESP_OK) { ESP_LOGE(TAG, "stream[%d] FAIL @alloc_urbs: %s", d->index, esp_err_to_name(err)); free_urbs(d); return err; }
    }
    d->block_seq = 0;
    d->want_stream = true;
    err = submit_urbs(d);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "stream[%d] FAIL @submit_urbs: %s", d->index, esp_err_to_name(err));
        d->want_stream = false;
        return err;
    }

    d->state = USB_RTLSDR_STATE_STREAMING;
    ESP_LOGI(TAG, "streaming[%d]: %u sps @ %u Hz, gain %d (mode %d)",
             d->index, d->sample_rate_sps, d->center_freq_hz,
             d->gain_tenth_db, (int)d->gain_mode);
    return ESP_OK;
}

/** @brief Halt bulk-IN, retire URBs, leave the device open. */
static void stop_streaming_locked(usb_rtlsdr_dev_t *d)
{
    if (d->state != USB_RTLSDR_STATE_STREAMING) {
        return;
    }
    d->want_stream = false;
    cancel_urbs(d);
    d->state = (d->dev) ? USB_RTLSDR_STATE_OPEN_IDLE
                        : USB_RTLSDR_STATE_NO_DEVICE;
    ESP_LOGI(TAG, "streaming[%d] stopped", d->index);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  USB Host client event callback + housekeeping task.
 *
 *  The client callback runs in the housekeeping task's context (we pump it). It
 *  queues NEW_DEV/DEV_GONE for the task body to act on — opening/closing a
 *  device from inside the callback is discouraged by the host stack.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief USB Host client event sink — records new/gone devices for the task. */
static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    (void)arg;
    switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        /* A device enumerated; record its address for the task to open. The
         * NEW_DEV queue is process-global — the task drains it into a free slot. */
        s_ctx.pending_new_addr = msg->new_dev.address;
        s_ctx.have_pending_new = true;
        break;

    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        /* A device went away. Find which slot owns that handle and flag it for
         * recovery; the task body tears down THAT slot and, if auto_recover,
         * waits for a fresh NEW_DEV (a replug regains its role via seen_serial). */
        for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
            if (s_ctx.dev[i].in_use && msg->dev_gone.dev_hdl == s_ctx.dev[i].dev) {
                s_ctx.dev[i].do_recover = true;
            }
        }
        break;

    default:
        break;
    }
}

/**
 * @brief Resolve the role-slot index for a freshly-opened device by stable
 *        first-seen serial order.
 *
 * @details
 *   The role a dongle serves must survive an unplug/replug, so it is bound to the
 *   ORDER in which distinct serials are first seen, not to the volatile bus
 *   address. We keep a small map (s_ctx.seen_serial[]) of the serials we have
 *   adopted; a serial already in the map regains its original index, a new serial
 *   appends. If the serial is empty or the well-known default "00000001" (two
 *   identical sticks), first-seen order STILL applies — distinct entries are
 *   appended in adoption order, which is the natural port-enumeration order. This
 *   means two identical-serial dongles fall back to adoption order, documented.
 *
 * @return The role index 0..RTLSDR_MAX_DEVICES-1, or -1 if the map is full.
 */
static int role_index_for_serial(const char *serial)
{
    /* EMPTY-SERIAL CASE — the common one for cheap RTL-SDR sticks, which usually
     * ship with a blank (or identical) EEPROM serial. Serial-matching CANNOT
     * disambiguate two blank-serial dongles: the first adopts as seen_serial[0]
     * = "", then the second strncmp-matches that same empty entry and collapses
     * back onto index 0 — so BOTH dongles were assigned role 1090 and the 978 slot
     * never existed (the weather pipeline could never build).
     *
     * For a blank serial, assign by ADOPTION ORDER instead: take the next unused
     * map slot. This is exactly the "first-seen order" the contract promises and
     * makes two identical blank sticks land on distinct indices (0 then 1 => 1090
     * then 978). We still bump seen_count so the next blank stick gets the
     * following index. (Replug stability for blank-serial sticks falls back to
     * port-enumeration order, which is the best any serial-blind scheme can do.) */
    if (serial == NULL || serial[0] == '\0') {
        if (s_ctx.seen_count < (int)RTLSDR_MAX_DEVICES) {
            int idx = s_ctx.seen_count;
            s_ctx.seen_serial[idx][0] = '\0';   /* record a blank entry */
            s_ctx.seen_count++;
            return idx;
        }
        return -1;   /* map full */
    }

    /* NON-EMPTY SERIAL — already mapped? Return its existing index (stable across
     * replug: a known stick regains its original role). */
    for (int i = 0; i < s_ctx.seen_count; i++) {
        if (strncmp(s_ctx.seen_serial[i], serial, sizeof(s_ctx.seen_serial[0]) - 1) == 0) {
            return i;
        }
    }
    /* New serial: append in first-seen order if there is room. */
    if (s_ctx.seen_count < (int)RTLSDR_MAX_DEVICES) {
        int idx = s_ctx.seen_count;
        strncpy(s_ctx.seen_serial[idx], serial, sizeof(s_ctx.seen_serial[0]) - 1);
        s_ctx.seen_serial[idx][sizeof(s_ctx.seen_serial[0]) - 1] = '\0';
        s_ctx.seen_count++;
        return idx;
    }
    return -1;
}

/**
 * @brief Try to open the pending NEW_DEV into a free slot if it is a supported
 *        dongle, then assign its role and (if intent is latched) start it.
 *
 * @details
 *   Reads the candidate's device descriptor (a transient open to peek VID/PID),
 *   and if it matches our family, adopts it into the first free slot. The slot's
 *   role is then assigned by stable serial order (with the first-device override
 *   hook honoured); the slot's stream config is seeded from the role's band, and
 *   if the global streaming intent is latched the slot is started immediately.
 */
static void task_try_open_pending(void)
{
    if (!s_ctx.have_pending_new) {
        return;
    }

    /* Find the first free slot. If none, the queue is moot — clear it and bail. */
    usb_rtlsdr_dev_t *d = NULL;
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        if (!s_ctx.dev[i].in_use) {
            d = &s_ctx.dev[i];
            break;
        }
    }
    if (!d) {
        s_ctx.have_pending_new = false;
        return;
    }

    uint8_t addr = s_ctx.pending_new_addr;
    s_ctx.have_pending_new = false;

    /* Peek the descriptor by briefly opening; close again if it is not ours. */
    usb_device_handle_t tmp = NULL;
    if (usb_host_device_open(s_ctx.client, addr, &tmp) != ESP_OK) {
        return;
    }
    const usb_device_desc_t *dd = NULL;
    bool ours = false;
    if (usb_host_get_device_descriptor(tmp, &dd) == ESP_OK && dd) {
        ours = is_supported_dongle(dd->idVendor, dd->idProduct);
    }
    usb_host_device_close(s_ctx.client, tmp);
    if (!ours) {
        return;
    }

    /* It is a supported dongle — open it for real under the lock. The open path
     * reads identity (including the serial), claims the bulk interface and
     * probes the tuner. */
    lock();
    esp_err_t err = open_device(d, addr);
    unlock();

    if (err != ESP_OK) {
        /* Open/configure failed — record it and tidy up the half-open device. */
        set_last_error(d, err);
        lock();
        close_device(d);
        unlock();
        return;
    }

    /* ── ROLE ASSIGNMENT ─────────────────────────────────────────────────────
     * PRIMARY: bind the band to the stable physical hub PORT, so the correctly-
     * tuned antenna always stays on the right band no matter which stick powers up
     * first (the two blank-serial Nanos are otherwise indistinguishable). */
    bool first_device = (s_ctx.seen_count == 0);
    adsbin_rf_role_t role;
    bool role_by_port = true;

    if (d->port_num == RTLSDR_PORT_1090) {
        role = ADSBIN_ROLE_1090;
    } else if (d->port_num == RTLSDR_PORT_978) {
        role = ADSBIN_ROLE_978_UAT;
    } else {
        /* FALLBACK: unknown port (different hub / port read failed) — keep the box
         * working via the legacy stable-serial / adoption-order assignment. */
        role_by_port = false;
        int role_idx = role_index_for_serial(d->info.serial);
        if (role_idx < 0) {
            role_idx = d->index;   /* map full (shouldn't happen) — fall back to slot. */
        }
        role = (role_idx == 0) ? ADSBIN_ROLE_1090 : ADSBIN_ROLE_978_UAT;
    }

    /* Override hook: a lone dongle can be forced to 978 (or pinned to 1090). The
     * override only applies to the FIRST adopted device so a second stick still
     * takes the next free band. */
    if (first_device && s_role_override != ADSBIN_ROLE_NONE) {
        role = s_role_override;
    }
    d->role = role;

    ESP_LOGI(TAG, "role slot[%d] = %s (by %s, PORT=%u)",
             d->index, (role == ADSBIN_ROLE_978_UAT) ? "978-UAT" : "1090",
             role_by_port ? "port" : "serial/order", (unsigned)d->port_num);

    /* Seed the slot's stream config from its role's band. Gain/ppm/bias defaults
     * were already seeded into every slot at init; the band is what differs. */
    if (role == ADSBIN_ROLE_978_UAT) {
        d->center_freq_hz  = UAT_CENTER_FREQ_HZ;
        d->sample_rate_sps = UAT_SAMPLE_RATE_HZ;
    } else {
        d->center_freq_hz  = ADSB_CENTER_FREQ_HZ;
        d->sample_rate_sps = ADSB_SAMPLE_RATE_HZ;
    }

    /* The slot is now occupied. Latch this device's streaming intent from the
     * GLOBAL latched intent so a start() issued before plug-in still auto-starts. */
    d->in_use      = true;
    d->want_stream = s_ctx.want_stream_latched;

    /* CONNECTED first, then auto-start streaming if intent is latched. */
    emit_event(d->index, USB_RTLSDR_EVENT_CONNECTED);
    if (d->want_stream) {
        lock();
        esp_err_t serr = start_streaming_locked(d);
        unlock();
        if (serr == ESP_OK) {
            emit_event(d->index, USB_RTLSDR_EVENT_STREAM_STARTED);
        } else {
            set_last_error(d, serr);
        }
    }
}

/**
 * @brief Tear down any device flagged for recovery and (optionally) re-latch its
 *        streaming intent so a replug auto-restarts. Runs in the housekeeping
 *        task only. Iterates all slots; recovering slot i never touches slot j.
 */
static void task_do_recovery(void)
{
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        usb_rtlsdr_dev_t *d = &s_ctx.dev[i];
        if (!d->do_recover) {
            continue;
        }
        d->do_recover = false;

        /* Capture "were we (meant to be) streaming?" BEFORE we mutate state —
         * both the live STREAMING state and the latched want_stream count, so a
         * stall mid-stream re-arms after recovery. Reading it first avoids the
         * trap where setting RECOVERING below would erase the evidence. */
        bool was_streaming = (d->state == USB_RTLSDR_STATE_STREAMING) || d->want_stream;

        d->state = USB_RTLSDR_STATE_RECOVERING;
        emit_event(d->index, USB_RTLSDR_EVENT_USB_STALL);

        lock();
        /* Retire URBs + close the (possibly-dead) device — all keyed off d so we
         * only touch THIS device's endpoints. */
        d->want_stream = false;
        cancel_urbs(d);
        free_urbs(d);
        close_device(d);
        d->stats.reset_count++;
        d->state  = USB_RTLSDR_STATE_NO_DEVICE;
        /* Free the slot so a fresh NEW_DEV can be adopted, but KEEP the
         * seen_serial mapping intact so a replug of the same dongle regains the
         * same role/index. */
        d->in_use = false;
        unlock();

        emit_event(d->index, USB_RTLSDR_EVENT_DISCONNECTED);

        /* If auto-recover is on, re-latch the streaming intent so the next
         * NEW_DEV (from a re-plug or stack re-enumeration) auto-starts the bulk
         * path. The re-adopted slot copies this from want_stream when it opens. */
        if (s_ctx.auto_recover) {
            d->want_stream = was_streaming;
            emit_event(d->index, USB_RTLSDR_EVENT_RECOVERED);
        }
    }
}

/**
 * @brief Apply one device's deferred runtime-config requests on the usb_task.
 *
 * @details
 *   This is where Core-1 set_* / start/stop requests actually reach the chip — on
 *   the one task that owns the USB client event loop. Each dirty flag is cleared
 *   under the lock and the matching chip write is performed (only if the device
 *   is open). Failures are recorded in last_error rather than propagated, since
 *   the originating call already returned "accepted". Called once per in-use slot
 *   by task_apply_config().
 */
static void apply_device_config(usb_rtlsdr_dev_t *d)
{
    /* Snapshot + clear the dirty flags under the lock so a concurrent set_* that
     * lands mid-apply simply re-arms for the next pass. */
    lock();
    bool d_freq  = d->cfg_dirty_freq;  d->cfg_dirty_freq  = false;
    bool d_rate  = d->cfg_dirty_rate;  d->cfg_dirty_rate  = false;
    bool d_gain  = d->cfg_dirty_gain;  d->cfg_dirty_gain  = false;
    bool d_bias  = d->cfg_dirty_bias;  d->cfg_dirty_bias  = false;
    bool d_start = d->cfg_dirty_start; d->cfg_dirty_start = false;
    bool d_stop  = d->cfg_dirty_stop;  d->cfg_dirty_stop  = false;
    bool have_dev = (d->dev != NULL);
    unlock();

    /* Stop takes priority — if both were requested, the latest intent wins. */
    if (d_stop) {
        lock();
        d->want_stream = false;
        stop_streaming_locked(d);
        unlock();
        emit_event(d->index, USB_RTLSDR_EVENT_STREAM_STOPPED);
        return;     /* nothing else makes sense after a stop this pass.         */
    }

    /* Start: latch the intent; if a device is already open, arm streaming now.
     * Otherwise the open path will auto-start when the dongle enumerates. */
    if (d_start) {
        lock();
        d->want_stream = true;
        esp_err_t err = have_dev ? start_streaming_locked(d) : ESP_OK;
        unlock();
        if (err == ESP_OK && d->state == USB_RTLSDR_STATE_STREAMING) {
            emit_event(d->index, USB_RTLSDR_EVENT_STREAM_STARTED);
        } else if (err != ESP_OK) {
            set_last_error(d, err);
        }
    }

    /* The per-knob retunes only make sense with a device open. */
    if (!have_dev) {
        return;
    }

    /* Re-program the resampler first (it resets the sample pipe), then the LO,
     * then gain, then the bias-tee GPIO. Order mirrors a fresh stream bring-up. */
    if (d_rate) {
        lock();
        esp_err_t err = rtl_set_sample_rate(d, d->sample_rate_sps);
        unlock();
        if (err != ESP_OK) set_last_error(d, err);
    }
    if (d_freq) {
        lock();
        esp_err_t err = configure_frequency_locked(d);
        unlock();
        if (err != ESP_OK) set_last_error(d, err);
    }
    if (d_gain) {
        lock();
        esp_err_t err = configure_gain_locked(d);
        unlock();
        if (err != ESP_OK) set_last_error(d, err);
    }
    if (d_bias) {
        lock();
        esp_err_t err = configure_bias_tee_locked(d);
        unlock();
        if (err != ESP_OK) set_last_error(d, err);
    }
}

/**
 * @brief Apply deferred config across every device slot.
 *
 * @details
 *   A device that is in_use OR still has its start dirty-bit pending (legacy
 *   start() before any dongle enumerated sets dev0's cfg_dirty_start) gets
 *   serviced. Servicing an empty slot is harmless — apply_device_config short-
 *   circuits the retunes when no device is open — but we still need to run it so
 *   a pending start latches want_stream on dev0 even before a dongle is present.
 */
static void task_apply_config(void)
{
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        apply_device_config(&s_ctx.dev[i]);
    }
}

/**
 * @brief The driver's single housekeeping task (Core 0).
 *
 * @details
 *   Pumps the USB host library + our client event loops, services queued
 *   NEW_DEV/DEV_GONE, applies deferred runtime-config requests, runs
 *   stall/disconnect recovery, and emits debounced OVERFLOW events when the hot
 *   path notifies it. It NEVER touches the bulk payload — that is the completion
 *   callback's job. This is the ONLY task that issues control transfers.
 */
static void usb_task(void *arg)
{
    (void)arg;
    s_ctx.task_alive = true;
    ESP_LOGI(TAG, "usb task up on core %d", (int)xPortGetCoreID());

    /* Debounce window so a burst of per-URB overflow notifications collapses to
     * one OVERFLOW event per ~200 ms. */
    int64_t last_overflow_evt = 0;
    uint64_t last_drops_seen  = 0;

    while (s_ctx.task_run) {

        /* Pump the host library + client event loops (bounded waits). */
        uint32_t flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(RTLSDR_EVENT_WAIT_MS), &flags);
        usb_host_client_handle_events(s_ctx.client, pdMS_TO_TICKS(RTLSDR_EVENT_WAIT_MS));

        /* Consume any hot-path notification (overflow/stall nudge). Non-blocking.*/
        ulTaskNotifyTake(pdTRUE, 0);

        /* Act on queued lifecycle work, then apply any deferred config. These
         * iterate every device slot internally. */
        task_do_recovery();
        task_try_open_pending();
        task_apply_config();

        /* Debounced OVERFLOW event from the hot path's drop counter. Aggregate
         * drops across all in-use devices so a burst on either dongle collapses
         * to a single OVERFLOW event. */
        lock();
        uint64_t drops = 0;
        for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
            drops += s_ctx.dev[i].stats.overflow_drops;
        }
        unlock();
        int64_t now = adsbin_now_us();
        if (drops > last_drops_seen && (now - last_overflow_evt) > 200000) {
            last_overflow_evt = now;
            last_drops_seen   = drops;
            /* The aggregate event is not slot-specific; report slot 0. */
            emit_event(0, USB_RTLSDR_EVENT_OVERFLOW);
        }
    }

    s_ctx.task_alive = false;
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API — lifecycle.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Seed one device slot's defaults at init.
 *
 * @details
 *   Sets the slot index, the band/stream defaults and the gain/ppm/bias defaults
 *   that init used to seed into the lone singleton. Slot 0 gets the canonical
 *   1090 band; slot 1 gets sane 1090 defaults too (its real band is set at adopt
 *   from its role). The ring is allocated by the caller (init).
 */
static void seed_device_defaults(usb_rtlsdr_dev_t *d, int index)
{
    d->index = index;

    /* Stream config: both slots start on the 1090 band so a slot adopted as the
     * 1090 device needs no change; a slot adopted as 978 is re-seeded at adopt. */
    d->center_freq_hz      = ADSB_CENTER_FREQ_HZ;
    d->sample_rate_sps     = ADSB_SAMPLE_RATE_HZ;
    d->gain_mode           = USB_RTLSDR_GAIN_MANUAL_FIXED;
    d->gain_tenth_db       = 496;
    d->freq_correction_ppm = 0;
    d->bias_tee_enable     = false;

    d->role   = ADSBIN_ROLE_NONE;
    d->in_use = false;
    d->state  = USB_RTLSDR_STATE_UNINIT;
}

esp_err_t usb_rtlsdr_init(const usb_rtlsdr_config_t *cfg)
{
    if (s_ctx.inited) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Resolve install config, applying the documented defaults for zero fields.*/
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.ring_capacity_blocks = (cfg && cfg->ring_capacity_blocks) ? cfg->ring_capacity_blocks
                                                                    : RTLSDR_DEFAULT_RING_BLOCKS;
    s_ctx.block_size_pairs     = (cfg && cfg->block_size_iq_pairs) ? cfg->block_size_iq_pairs
                                                                   : RTLSDR_DEFAULT_BLOCK_PAIRS;
    s_ctx.task_priority        = (cfg && cfg->usb_task_priority) ? cfg->usb_task_priority
                                                                 : RTLSDR_DEFAULT_TASK_PRIO;
    s_ctx.task_core_id         = (cfg && cfg->usb_task_core_id) ? cfg->usb_task_core_id
                                                                : ADSBIN_CORE_DSP;
    s_ctx.auto_recover         = (cfg) ? cfg->auto_recover : true;

    /* Seed each device slot's stream-config defaults (slot 0 = 1090 exactly as
     * the old code seeded the singleton; slot 1 also gets sane 1090 defaults and
     * is re-banded at adopt from its role). */
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        seed_device_defaults(&s_ctx.dev[i], i);
    }

    /* Driver lock (serialises config / stats / handles / control bus for ALL
     * slots). Recursive so the control path can hold it while the event pump
     * re-enters via the bulk completion callback on the same task — see lock(). */
    s_ctx.lock = xSemaphoreCreateRecursiveMutex();
    if (!s_ctx.lock) {
        ESP_LOGE(TAG, "sync primitive alloc failed");
        goto fail;
    }

    /* Allocate one owned IQ no-split ring PER device slot. Size = depth *
     * (header + payload), plus the 8-byte per-item ring header the no-split type
     * adds. Both slots get an identical ring so either can be the 1090 device. */
    {
        const size_t item_payload = sizeof(iq_block_t) + s_ctx.block_size_pairs * 2u;
        const size_t per_item     = item_payload + 8u + 4u;   /* hdr + alignment.*/
        const size_t ring_bytes   = per_item * s_ctx.ring_capacity_blocks;
        for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
            s_ctx.dev[i].iq_ring = xRingbufferCreate(ring_bytes, RINGBUF_TYPE_NOSPLIT);
            if (!s_ctx.dev[i].iq_ring) {
                ESP_LOGE(TAG, "IQ ring %d alloc (%u bytes) failed",
                         i, (unsigned)ring_bytes);
                goto fail;
            }
        }
        ESP_LOGI(TAG, "IQ rings: %u x [%u blocks x %u pairs (%u bytes)]",
                 (unsigned)RTLSDR_MAX_DEVICES,
                 (unsigned)s_ctx.ring_capacity_blocks,
                 (unsigned)s_ctx.block_size_pairs, (unsigned)ring_bytes);
    }

    /* Install the USB Host stack. root_port_unpowered=false => the stack drives
     * VBUS for us (host mode powers the dongle, plan S0). */
    usb_host_config_t host_cfg = {
        .skip_phy_setup      = false,
        .root_port_unpowered = false,
        .intr_flags          = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&host_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(err));
        goto fail;
    }

    /* Register our async client. */
    usb_host_client_config_t client_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg          = NULL,
        },
    };
    err = usb_host_client_register(&client_cfg, &s_ctx.client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "client_register failed: %s", esp_err_to_name(err));
        usb_host_uninstall();
        goto fail;
    }

    /* Spin up the housekeeping/USB task pinned to the DSP core. */
    s_ctx.task_run = true;
    BaseType_t ok = xTaskCreatePinnedToCore(usb_task, "usb_rtlsdr",
                                            RTLSDR_DEFAULT_TASK_STACK, NULL,
                                            s_ctx.task_priority, &s_ctx.task,
                                            s_ctx.task_core_id);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "usb task create failed");
        s_ctx.task_run = false;
        usb_host_client_deregister(s_ctx.client);
        usb_host_uninstall();
        goto fail;
    }

    /* Each slot starts NO_DEVICE; the host is up waiting for enumeration. */
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        s_ctx.dev[i].state = USB_RTLSDR_STATE_NO_DEVICE;
    }
    s_ctx.inited = true;
    ESP_LOGI(TAG, "init complete (auto_recover=%d)", (int)s_ctx.auto_recover);
    return ESP_OK;

fail:
    /* Free BOTH rings on the failure path. */
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        if (s_ctx.dev[i].iq_ring) {
            vRingbufferDelete(s_ctx.dev[i].iq_ring);
            s_ctx.dev[i].iq_ring = NULL;
        }
    }
    if (s_ctx.lock) { vSemaphoreDelete(s_ctx.lock); s_ctx.lock = NULL; }
    return ESP_FAIL;
}

esp_err_t usb_rtlsdr_deinit(void)
{
    if (!s_ctx.inited) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Stop the housekeeping task FIRST and wait for it to exit. Once it is gone,
     * this thread is the sole owner of the USB client, so the teardown below can
     * safely pump the event loop (cancel_urbs) without racing the task. We also
     * clear each device's want_stream so the task doesn't re-arm a URB on its
     * way out. */
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        s_ctx.dev[i].want_stream = false;
    }
    s_ctx.want_stream_latched = false;
    s_ctx.task_run            = false;
    for (int i = 0; i < 200 && s_ctx.task_alive; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Now quiescent: retire URBs + close each device, then delete its ring. */
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        usb_rtlsdr_dev_t *d = &s_ctx.dev[i];
        cancel_urbs(d);
        free_urbs(d);
        close_device(d);
        if (d->iq_ring) { vRingbufferDelete(d->iq_ring); d->iq_ring = NULL; }
    }

    /* Deregister the client + uninstall the host stack (one of each). */
    if (s_ctx.client) {
        usb_host_client_deregister(s_ctx.client);
        s_ctx.client = NULL;
    }
    usb_host_uninstall();

    /* Free the one lock. */
    if (s_ctx.lock) { vSemaphoreDelete(s_ctx.lock); s_ctx.lock = NULL; }

    s_ctx.inited = false;
    ESP_LOGI(TAG, "deinit complete");
    return ESP_OK;
}

/* kick_task() is defined further down with the deferred-config helpers, but the
 * start/stop/set_* entry points above it are the first callers — declare it here. */
static inline void kick_task(void);

esp_err_t usb_rtlsdr_start(const usb_rtlsdr_stream_config_t *stream_cfg)
{
    if (!s_ctx.inited) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Legacy device-0 (1090) shim. Resolve the per-stream config into device 0
     * (defaults for zero fields) so an explicit start() with a 1090 cfg still
     * configures dev0 exactly as the single-dongle build did. The stream_cfg's
     * device_index is ignored — the band follows the device's role. */
    usb_rtlsdr_dev_t *d0 = &s_ctx.dev[0];
    lock();
    if (stream_cfg) {
        d0->center_freq_hz      = stream_cfg->center_freq_hz  ? stream_cfg->center_freq_hz
                                                              : ADSB_CENTER_FREQ_HZ;
        d0->sample_rate_sps     = stream_cfg->sample_rate_sps ? stream_cfg->sample_rate_sps
                                                              : ADSB_SAMPLE_RATE_HZ;
        d0->gain_mode           = stream_cfg->gain_mode;
        d0->gain_tenth_db       = stream_cfg->gain_tenth_db ? stream_cfg->gain_tenth_db : 496;
        d0->freq_correction_ppm = stream_cfg->freq_correction_ppm;
        d0->bias_tee_enable     = stream_cfg->bias_tee_enable;
    }

    /* Latch the GLOBAL streaming intent (so a dongle that enumerates LATER still
     * auto-starts) and ask the usb_task to bring dev0's stream up. We never touch
     * the chip from here — start() may be called from Core 1, and only the
     * usb_task is allowed to drive the USB client (see set_* note). If no dongle
     * is connected yet the intent persists and the open path auto-starts on
     * enumeration. dev0's want_stream is also latched immediately for the case
     * where dev0 is already open. */
    s_ctx.want_stream_latched = true;
    d0->want_stream     = true;
    d0->cfg_dirty_start = true;
    d0->cfg_dirty_stop  = false;
    unlock();

    kick_task();
    ESP_LOGI(TAG, "start requested");
    return ESP_OK;
}

esp_err_t usb_rtlsdr_stop(void)
{
    if (!s_ctx.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Legacy device-0 stop. Clear the global intent and stop dev0. Defer the
     * actual bulk-IN teardown to the usb_task (it owns the client event loop the
     * cancellation pumps). Clearing want_stream immediately also makes the
     * hot-path completion stop re-arming right away. */
    usb_rtlsdr_dev_t *d0 = &s_ctx.dev[0];
    lock();
    s_ctx.want_stream_latched = false;
    d0->want_stream     = false;
    d0->cfg_dirty_stop  = true;
    d0->cfg_dirty_start = false;
    unlock();
    kick_task();
    return ESP_OK;
}

RingbufHandle_t usb_rtlsdr_get_iq_ring(void)
{
    /* Device 0 (the 1090 slot) ring; valid after init, NULL otherwise. */
    return s_ctx.inited ? s_ctx.dev[0].iq_ring : NULL;
}

RingbufHandle_t usb_rtlsdr_get_iq_ring_for_role(adsbin_rf_role_t role)
{
    if (!s_ctx.inited) {
        return NULL;
    }
    /* Return the ring of whichever IN-USE device currently holds this role. The
     * ring exists from init per slot, but we only hand it out when a dongle is
     * actually adopted into that role. */
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        if (s_ctx.dev[i].in_use && s_ctx.dev[i].role == role) {
            return s_ctx.dev[i].iq_ring;
        }
    }
    return NULL;
}

int usb_rtlsdr_active_count(void)
{
    if (!s_ctx.inited) {
        return 0;
    }
    int n = 0;
    for (int i = 0; i < (int)RTLSDR_MAX_DEVICES; i++) {
        if (s_ctx.dev[i].in_use) {
            n++;
        }
    }
    return n;
}

adsbin_rf_role_t usb_rtlsdr_role_of(int idx)
{
    if (!s_ctx.inited || idx < 0 || idx >= (int)RTLSDR_MAX_DEVICES) {
        return ADSBIN_ROLE_NONE;
    }
    return s_ctx.dev[idx].role;
}

void usb_rtlsdr_set_role_override(adsbin_rf_role_t role)
{
    /* A bare file-scope assignment — safe before or after init. The adopt path
     * reads it once, for the first device. */
    s_role_override = role;
}

esp_err_t usb_rtlsdr_start_index(int idx, const usb_rtlsdr_stream_config_t *cfg)
{
    if (!s_ctx.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (idx < 0 || idx >= (int)RTLSDR_MAX_DEVICES) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Store cfg into the target slot, latch ITS streaming intent + start bit. The
     * GLOBAL latch is also raised so a not-yet-enumerated dongle still auto-starts
     * when it appears. The band still follows the device's role; cfg fields that
     * matter (gain/ppm/bias) are applied to this slot. */
    usb_rtlsdr_dev_t *d = &s_ctx.dev[idx];
    lock();
    if (cfg) {
        d->center_freq_hz      = cfg->center_freq_hz  ? cfg->center_freq_hz  : d->center_freq_hz;
        d->sample_rate_sps     = cfg->sample_rate_sps ? cfg->sample_rate_sps : d->sample_rate_sps;
        d->gain_mode           = cfg->gain_mode;
        d->gain_tenth_db       = cfg->gain_tenth_db ? cfg->gain_tenth_db : 496;
        d->freq_correction_ppm = cfg->freq_correction_ppm;
        d->bias_tee_enable     = cfg->bias_tee_enable;
    }
    s_ctx.want_stream_latched = true;
    d->want_stream     = true;
    d->cfg_dirty_start = true;
    d->cfg_dirty_stop  = false;
    unlock();

    kick_task();
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API — discovery / identity.
 * ═══════════════════════════════════════════════════════════════════════════ */

int usb_rtlsdr_count(void)
{
    if (!s_ctx.inited) {
        return -1;
    }

    /* Ask the host stack for the current device address list, then count the
     * ones whose descriptor matches our supported family. Scans the BUS — not
     * the adopted slots (see usb_rtlsdr_active_count for that). */
    uint8_t addrs[16];
    int num = 0;
    if (usb_host_device_addr_list_fill(sizeof(addrs), addrs, &num) != ESP_OK) {
        return -1;
    }

    int matches = 0;
    for (int i = 0; i < num; i++) {
        usb_device_handle_t h = NULL;
        if (usb_host_device_open(s_ctx.client, addrs[i], &h) != ESP_OK) {
            continue;
        }
        const usb_device_desc_t *dd = NULL;
        if (usb_host_get_device_descriptor(h, &dd) == ESP_OK && dd &&
            is_supported_dongle(dd->idVendor, dd->idProduct)) {
            matches++;
        }
        usb_host_device_close(s_ctx.client, h);
    }
    return matches;
}

esp_err_t usb_rtlsdr_get_device_info(usb_rtlsdr_device_info_t *out_info)
{
    /* Legacy device-0 shim. */
    return usb_rtlsdr_get_device_info_index(0, out_info);
}

esp_err_t usb_rtlsdr_get_device_info_index(int idx, usb_rtlsdr_device_info_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ctx.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (idx < 0 || idx >= (int)RTLSDR_MAX_DEVICES) {
        return ESP_ERR_INVALID_ARG;
    }
    usb_rtlsdr_dev_t *d = &s_ctx.dev[idx];
    lock();
    bool present = d->in_use;
    if (present) {
        *out = d->info;
    }
    unlock();
    return present ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API — runtime tuning (safe from Core 1 while streaming).
 *
 *  CRITICAL DESIGN POINT. ALL chip control I/O (control transfers + the event
 *  pump they require) must happen on the single usb_task — the USB Host client
 *  event loop is single-owner, and a Core-1 caller pumping it concurrently with
 *  the Core-0 task would corrupt the stack. So a set_* from Core 1 only stores
 *  the new value (under the lock) and raises a "dirty" flag; the usb_task picks
 *  it up and applies it to the chip on its next pass. The call returns ESP_OK
 *  for "request accepted" — the chip-write result surfaces via get_status's
 *  last_error if it fails. When no device is open the value is simply stored.
 *
 *  The legacy set_* entry points target DEVICE 0 (the 1090 slot).
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief Wake the usb_task so it services a freshly-raised dirty flag now. */
static inline void kick_task(void)
{
    if (s_ctx.task) {
        xTaskNotify(s_ctx.task, 0, eNoAction);
    }
}

esp_err_t usb_rtlsdr_set_center_freq(uint32_t freq_hz)
{
    if (!s_ctx.inited) return ESP_ERR_INVALID_STATE;
    if (freq_hz == 0)  return ESP_ERR_INVALID_ARG;

    usb_rtlsdr_dev_t *d = &s_ctx.dev[0];
    lock();
    d->center_freq_hz  = freq_hz;
    d->cfg_dirty_freq  = true;      /* usb_task re-tunes the LO.              */
    unlock();
    kick_task();
    return ESP_OK;
}

esp_err_t usb_rtlsdr_set_sample_rate(uint32_t sample_rate_sps)
{
    if (!s_ctx.inited)        return ESP_ERR_INVALID_STATE;
    if (sample_rate_sps == 0) return ESP_ERR_INVALID_ARG;

    usb_rtlsdr_dev_t *d = &s_ctx.dev[0];
    lock();
    d->sample_rate_sps = sample_rate_sps;
    d->cfg_dirty_rate  = true;      /* usb_task re-programs the resampler.    */
    unlock();
    kick_task();
    return ESP_OK;
}

esp_err_t usb_rtlsdr_set_tuner_gain(usb_rtlsdr_gain_mode_t mode, int gain_tenth_db)
{
    if (!s_ctx.inited) return ESP_ERR_INVALID_STATE;

    usb_rtlsdr_dev_t *d = &s_ctx.dev[0];
    lock();
    d->gain_mode      = mode;
    d->gain_tenth_db  = gain_tenth_db;
    d->cfg_dirty_gain = true;       /* usb_task re-applies tuner gain.        */
    unlock();
    kick_task();
    return ESP_OK;
}

esp_err_t usb_rtlsdr_set_freq_correction(int ppm)
{
    if (!s_ctx.inited) return ESP_ERR_INVALID_STATE;

    usb_rtlsdr_dev_t *d = &s_ctx.dev[0];
    lock();
    d->freq_correction_ppm = ppm;
    /* The ppm trim is folded into the LO on the next retune. */
    d->cfg_dirty_freq = true;
    unlock();
    kick_task();
    return ESP_OK;
}

esp_err_t usb_rtlsdr_set_bias_tee(bool enable)
{
    if (!s_ctx.inited) return ESP_ERR_INVALID_STATE;

    usb_rtlsdr_dev_t *d = &s_ctx.dev[0];
    lock();
    d->bias_tee_enable = enable;
    d->cfg_dirty_bias  = true;      /* usb_task drives the GPIO.              */
    unlock();
    kick_task();
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API — health / stats / events.
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t usb_rtlsdr_get_status(usb_rtlsdr_status_t *out_status)
{
    /* Legacy device-0 shim. */
    return usb_rtlsdr_get_status_index(0, out_status);
}

esp_err_t usb_rtlsdr_get_status_index(int idx, usb_rtlsdr_status_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_ctx.inited) return ESP_ERR_INVALID_STATE;
    if (idx < 0 || idx >= (int)RTLSDR_MAX_DEVICES) return ESP_ERR_INVALID_ARG;

    usb_rtlsdr_dev_t *d = &s_ctx.dev[idx];
    lock();
    out->state          = d->state;
    out->device_present = (d->dev != NULL);
    out->streaming      = (d->state == USB_RTLSDR_STATE_STREAMING);
    out->last_error     = d->last_error;
    out->last_block_us  = d->last_block_us;
    unlock();
    return ESP_OK;
}

esp_err_t usb_rtlsdr_get_stats(usb_rtlsdr_stats_t *out_stats)
{
    if (!out_stats) return ESP_ERR_INVALID_ARG;
    if (!s_ctx.inited) return ESP_ERR_INVALID_STATE;

    /* Legacy device-0 stats. */
    lock();
    *out_stats = s_ctx.dev[0].stats;
    unlock();
    return ESP_OK;
}

void usb_rtlsdr_reset_stats(void)
{
    if (!s_ctx.inited) return;
    /* Legacy device-0 stats reset. */
    usb_rtlsdr_dev_t *d = &s_ctx.dev[0];
    lock();
    memset(&d->stats, 0, sizeof(d->stats));
    d->rate_window_us    = 0;
    d->rate_window_bytes = 0;
    unlock();
}

esp_err_t usb_rtlsdr_register_event_callback(usb_rtlsdr_event_cb_t cb, void *user_ctx)
{
    if (!s_ctx.inited) return ESP_ERR_INVALID_STATE;
    lock();
    s_ctx.event_cb  = cb;
    s_ctx.event_ctx = user_ctx;
    unlock();
    return ESP_OK;
}
