/**
 * @file    usb_rtlsdr_internal.h
 * @brief   Private state, register map and helpers for the RTL-SDR USB driver.
 *
 * @details
 *   This header is INTERNAL to components/usb_rtlsdr — nothing outside the
 *   component includes it. It keeps the public usb_rtlsdr.h purely a contract
 *   while giving the .c a single home for:
 *
 *     - the module-private singleton (::usb_rtlsdr_ctx_t),
 *     - the RTL2832U + R820T2 register addresses and bit meanings, all
 *       CLEAN-ROOMED from the public Realtek RTL2832U and Rafael Micro R820T2
 *       datasheets / register-description documents (NOT copied from the GPLv2
 *       librtlsdr source), and
 *     - the compile-time geometry of the bulk-IN pipeline.
 *
 *   WHY THE REGISTER MEANINGS LIVE HERE. The two chips are configured by poking
 *   named fields, not magic numbers. Documenting each address + bit alongside
 *   its datasheet purpose is what makes this a clean-room implementation rather
 *   than a transliteration of someone else's driver: a reader can check every
 *   write against the published datasheet meaning.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "usb/usb_host.h"        /* ESP-IDF USB Host Library + ch9 types.        */
#include "usb/usb_helpers.h"     /* Descriptor parse helpers + EP/intf structs.  */

#include "usb_rtlsdr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  USB identity & endpoint geometry
 *
 *  Realtek RTL2832U-based dongles all share VID 0x0BDA; the PID varies per
 *  vendor (the Nooelec / generic "RTL2838" units report 0x2838). We accept the
 *  whole family by VID and a small PID allow-list — the tuner is probed over
 *  I2C after enumeration, so the PID is only a coarse first filter.
 * ═══════════════════════════════════════════════════════════════════════════ */
#define RTLSDR_VID_REALTEK        0x0BDAu  /**< Realtek Semiconductor vendor ID. */
#define RTLSDR_PID_2832           0x2832u  /**< RTL2832U generic DVB-T.          */
#define RTLSDR_PID_2838           0x2838u  /**< RTL2838 (most RTL-SDR sticks).   */
#define RTLSDR_PID_NESDR_SMART    0x2832u  /**< Nooelec variants reuse 0x2832/38.*/

/* The DVB-T data path is a single bulk-IN endpoint. On every Realtek stick this
 * is endpoint 1 IN (address 0x81); we still verify it by parsing the interface
 * descriptor rather than hard-trusting the address. */
#define RTLSDR_BULK_IN_EP_ADDR    0x81u

/* The dongle exposes exactly one interface (bInterfaceNumber 0, alt-setting 0)
 * carrying the DVB-T transport-stream bulk endpoint we repurpose for raw IQ. */
#define RTLSDR_INTF_NUMBER        0u
#define RTLSDR_INTF_ALT           0u

/* ═══════════════════════════════════════════════════════════════════════════
 *  RTL2832U vendor-request register access (clean-room, from the RTL2832U
 *  datasheet "USB control" + register-block description).
 *
 *  The RTL2832U is controlled by USB vendor control transfers. A 16-bit address
 *  goes in wValue, a "block" selector in the high byte of wIndex, and the
 *  read/write direction in bmRequestType. Each demod/system block is a small
 *  register file. These block IDs and the request value are the documented USB
 *  control interface, not librtlsdr internals.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* bmRequestType for a vendor request to the device (type=vendor, recip=device).*/
#define RTL_CTRL_IN   (USB_BM_REQUEST_TYPE_DIR_IN  | USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_RECIP_DEVICE)
#define RTL_CTRL_OUT  (USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_RECIP_DEVICE)

/* The single vendor bRequest the RTL2832U uses for register block access. The
 * direction (read vs write) is carried by bmRequestType, not bRequest. */
#define RTL_VENDOR_REQUEST        0u

/* Register "blocks" selected via the high byte of wIndex. Names follow the
 * datasheet's functional grouping. */
#define RTL_BLK_DEMOD             0x0000u  /**< OFDM/DVB-T demodulator regs.     */
#define RTL_BLK_USB               0x0100u  /**< USB controller / FIFO regs.      */
#define RTL_BLK_SYS               0x0200u  /**< System / GPIO / clock regs.      */
#define RTL_BLK_I2C               0x0600u  /**< I2C master (tuner bus) window.   */

/* ── USB-block registers (RTL_BLK_USB) ─────────────────────────────────────── */
#define RTL_USB_SYSCTL            0x2000u  /**< USB system control.              */
#define RTL_USB_EPA_CTL           0x2148u  /**< Bulk-IN endpoint A control.      */
#define RTL_USB_EPA_MAXPKT        0x2158u  /**< Endpoint A max packet size.      */

/* ── SYS-block registers (RTL_BLK_SYS) ─────────────────────────────────────── */
#define RTL_SYS_DEMOD_CTL         0x3000u  /**< Demod control: ADC/clk/IR power. */
#define RTL_SYS_GPO               0x3001u  /**< GPIO output value.               */
#define RTL_SYS_GPI               0x3002u  /**< GPIO input value.                */
#define RTL_SYS_GPOE              0x3003u  /**< GPIO output enable.              */
#define RTL_SYS_GPD               0x3004u  /**< GPIO direction.                  */
#define RTL_SYS_DEMOD_CTL_1       0x300Bu  /**< Secondary demod control.         */

/* DEMOD_CTL bit meanings (clean-room from the datasheet's demod-control field).
 * Bit layout: ADC enable for I and Q, the on-chip clock-out gate, and reset. */
#define RTL_DEMOD_CTL_ADC_Q       0x08u    /**< Power the Q-channel ADC.         */
#define RTL_DEMOD_CTL_ADC_I       0x10u    /**< Power the I-channel ADC.         */
#define RTL_DEMOD_CTL_RESET_DGTL  0x04u    /**< Hold the digital core in reset.  */

/* GPIO bit that gates the bias-tee FET on the antenna port. On RTL-SDR.com v3
 * style hardware the 4.5 V bias-tee is switched by GPIO0; some Nooelec units use
 * GPIO0 as well. We drive GPIO0 high to enable. Exposed via set_bias_tee. */
#define RTL_GPIO_BIAS_TEE         0x01u    /**< GPIO0 mask.                      */

/* ── DEMOD-block registers (RTL_BLK_DEMOD) ─────────────────────────────────── */
/* The demod block is paged: the low byte of the 16-bit address is the in-page
 * offset, the page number rides in the LOW byte of wIndex when this block is
 * selected. Page 1, offset 0x01 holds the I2C-repeater enable bit (datasheet:
 * "set IIC_repeat=1 before talking to the tuner"). */
#define RTL_DEMOD_PAGE_SHIFT      8
#define RTL_DEMOD_OFF_MASK        0xFFu

#define RTL_DEMOD_IIC_REPEAT      0x0101u  /**< page1/off1: I2C repeater enable. */
#define RTL_DEMOD_IIC_REPEAT_ON   0x18u    /**< Datasheet value enabling repeat. */
#define RTL_DEMOD_IIC_REPEAT_OFF  0x10u    /**< Repeater disabled.               */

/* Resampler ratio registers (page1) that set the effective output sample rate
 * from the 28.8 MHz crystal. The 28-bit ratio is split across four byte regs. */
#define RTL_DEMOD_RSAMP_RATIO0    0x019Fu  /**< Resample ratio [27:24..16].      */
#define RTL_DEMOD_RSAMP_RATIO1    0x01A0u  /**< Resample ratio [15:8..0].        */

/* Soft reset of the demod sample pipe (page1/off1, bit values per datasheet). */
#define RTL_DEMOD_SOFT_RST        0x0101u
#define RTL_DEMOD_SOFT_RST_ON     0x14u
#define RTL_DEMOD_SOFT_RST_OFF    0x10u

/* The RTL2832U master crystal. Both the demod resampler and (via the I2C-fed
 * tuner reference) the R820T2 PLL are derived from this 28.8 MHz reference. */
#define RTL_XTAL_HZ               28800000u

/* ═══════════════════════════════════════════════════════════════════════════
 *  R820T2 tuner (clean-room, from the Rafael Micro R820T2 register-description
 *  document). The tuner sits on the RTL2832U's I2C bus at slave address 0x1A.
 *
 *  Registers 0x00..0x04 are read-only status (chip id, PLL lock); the host
 *  writes 0x05..0x1F. We keep a shadow of the writable registers because the
 *  chip has no per-register read-modify-write — you must remember prior bits.
 * ═══════════════════════════════════════════════════════════════════════════ */
#define R820T_I2C_ADDR            0x1Au    /**< Fixed 7-bit I2C slave address.   */
#define R820T_NUM_REGS            32       /**< 0x00..0x1F total register file.  */
#define R820T_WRITE_START         5        /**< First host-writable register.    */

/* Read-only identity (datasheet): a successful chip probe reads back a known
 * chip-id pattern in reg 0x00's low bits. We use it only to confirm presence. */
#define R820T_REG_CHIPID          0x00u

/* Writable function registers (subset we touch), with the datasheet meaning of
 * the fields we set. Each name maps to a real R820T2 register address. */
#define R820T_REG_LNA_GAIN        0x05u  /**< LNA gain + loop-through + AGC mode.*/
#define R820T_REG_PWR_PDET        0x06u  /**< Power-detect / filter power.       */
#define R820T_REG_MIXER_GAIN      0x07u  /**< Mixer gain + mixer AGC mode.       */
#define R820T_REG_MIXBUF          0x08u  /**< Mixer buffer power.                */
#define R820T_REG_IFFILT          0x09u  /**< IF filter power / current.         */
#define R820T_REG_FILT_CAL        0x0Au  /**< Filter auto-cal + VGA mode.        */
#define R820T_REG_FILT_BW         0x0Bu  /**< Channel filter bandwidth select.   */
#define R820T_REG_VGA_GAIN        0x0Cu  /**< VGA (IF) gain + VGA AGC mode.      */
#define R820T_REG_LNA_TOP         0x0Du  /**< LNA AGC loop top voltage.          */
#define R820T_REG_MIX_TOP         0x0Eu  /**< Mixer AGC loop top voltage.        */
#define R820T_REG_PLL_VCO         0x10u  /**< Reference divider + VCO settings.  */
#define R820T_REG_PLL_FRAC_LO     0x11u  /**< Sigma-delta fractional [7:0].      */
#define R820T_REG_PLL_FRAC_HI     0x12u  /**< Sigma-delta fractional [15:8].     */
#define R820T_REG_PLL_NINT        0x13u  /**< Integer divider Nint + frac msb.   */
#define R820T_REG_PLL_VCO_CTRL    0x14u  /**< VCO band / divider control.        */
#define R820T_REG_TF_FILTER       0x1Au  /**< Tracking-filter / PLL auto.        */
#define R820T_REG_FILTER_GATE     0x1Bu  /**< Filter gate + tracking band.       */

/* AGC-mode bits (datasheet: each gain register's bit4 selects auto vs manual
 * for that stage). Convention used here, matching the R820T2 register
 * description: bit4 = 1 puts the stage under the chip's AGC loop (AUTO); bit4 = 0
 * leaves the low-nibble gain step in force (MANUAL). For ADS-B we want MANUAL
 * fixed gain on all three stages. */
#define R820T_AGC_AUTO_BIT        0x10u  /**< bit4: 1=AGC auto, 0=manual.        */
#define R820T_GAIN_STEP_MASK      0x0Fu  /**< Low nibble: per-stage gain step.   */

/* The R820T2 intermediate frequency the RTL2832U expects. The tuner mixes the
 * RF down to this IF; the demod resampler then brings it to baseband. Standard
 * Realtek value used for the OFDM path. */
#define R820T_IF_FREQ_HZ          3570000u

/* ═══════════════════════════════════════════════════════════════════════════
 *  Bulk pipeline geometry & defaults
 * ═══════════════════════════════════════════════════════════════════════════ */

/* One delivered IQ block holds this many IQ pairs by default. 2.4 Msps means
 * 2.4 M pairs/s; at ~16384 pairs/block that is ~146 blocks/s — fine ring churn
 * with low per-block overhead. (block_size_iq_pairs in the config overrides.) */
#define RTLSDR_DEFAULT_BLOCK_PAIRS   16384u

/* Default ring depth in blocks if the caller passes 0. 16 blocks at the default
 * block size buffers ~110 ms of IQ — plenty for the Core-0 demod to absorb a
 * scheduling hiccup without an overflow. */
#define RTLSDR_DEFAULT_RING_BLOCKS   16u

/* USB bulk transfer (URB) sizing. We keep several large transfers in flight so
 * the host controller is never starved between completions. Each URB carries a
 * multiple of the 512-byte HS bulk max-packet so short-packet handling is rare.
 * 256 KiB per URB at 4.8 MB/s drains in ~53 ms; with this many in flight the
 * pipe stays saturated even if a completion is briefly delayed. */
#define RTLSDR_URB_SIZE              (256u * 1024u)
#define RTLSDR_NUM_URBS              8u

/* HS bulk max-packet for the RTL2832U data endpoint (USB 2.0 high speed). */
#define RTLSDR_BULK_MPS_HS           512u

/* Default driver task knobs. The USB host + bulk path are the hardest real-time
 * thing on Core 0, so the housekeeping task runs at a high priority but below
 * the demod that consumes from us is irrelevant here (separate task). */
#define RTLSDR_DEFAULT_TASK_PRIO     21
#define RTLSDR_DEFAULT_TASK_STACK    5120

/* String-descriptor scratch: longest serial/product we will read from EEPROM. */
#define RTLSDR_STRDESC_MAX           128

/* ═══════════════════════════════════════════════════════════════════════════
 *  The module-private singleton.
 *
 *  One dongle per firmware image. All cross-thread access to mutable config /
 *  stats / device handles goes through s_ctx.lock so a Core-1 set_*/get_* call
 *  cannot tear a value the Core-0 completion path is updating.
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* ---- resolved install config (post-defaults) ---- */
    size_t  ring_capacity_blocks;   /**< IQ ring depth in blocks.               */
    size_t  block_size_pairs;       /**< IQ pairs per delivered block.          */
    int     task_priority;          /**< Housekeeping/USB task priority.        */
    int     task_core_id;           /**< Core pinned to (ADSBIN_CORE_DSP).      */
    bool    auto_recover;           /**< Re-enumerate on stall/disconnect.      */

    /* ---- resolved stream config (post-defaults) ---- */
    uint32_t center_freq_hz;        /**< Current tuned centre frequency.        */
    uint32_t sample_rate_sps;       /**< Current sample rate.                   */
    usb_rtlsdr_gain_mode_t gain_mode;
    int      gain_tenth_db;         /**< Requested fixed gain (tenths of dB).   */
    int      freq_correction_ppm;   /**< Frequency-correction trim.             */
    bool     bias_tee_enable;       /**< Antenna-port bias-tee state.           */
    int      device_index;          /**< Which enumerated dongle.               */

    /* ---- USB host handles ---- */
    usb_host_client_handle_t client;     /**< Our USB Host client.              */
    usb_device_handle_t      dev;        /**< Open device (NULL if none).        */
    uint8_t                  dev_addr;   /**< Bus address of the open device.    */
    uint8_t                  bulk_ep;    /**< Bulk-IN endpoint address.          */
    uint16_t                 bulk_mps;   /**< Bulk-IN max packet size.           */

    /* ---- control-transfer plumbing (one shared, serialised by `lock`) ---- */
    usb_transfer_t   *ctrl_xfer;         /**< Reusable EP0 control transfer.     */
    volatile bool     ctrl_complete;     /**< Raised by the control callback.    */

    /* ---- bulk-IN transfers (URBs) kept continuously in flight ---- */
    usb_transfer_t   *urb[RTLSDR_NUM_URBS];
    volatile uint32_t urbs_inflight;     /**< How many URBs are submitted now.   */

    /* ---- the owned IQ ring (no-split; producer = Core-0 completion) ---- */
    RingbufHandle_t  iq_ring;
    uint32_t         block_seq;          /**< Monotonic iq_block_t.seq counter.  */

    /* ---- R820T2 writable-register shadow (no RMW on the chip) ---- */
    uint8_t          r82_shadow[R820T_NUM_REGS];

    /* ---- task / lifecycle ---- */
    TaskHandle_t     task;               /**< USB host + housekeeping task.      */
    volatile bool    task_run;           /**< Task keeps looping while true.     */
    volatile bool    task_alive;         /**< Set on entry, cleared on exit.     */
    volatile usb_rtlsdr_state_t state;   /**< Liveness state machine.            */
    volatile bool    want_stream;        /**< Streaming requested by start().    */
    volatile bool    do_recover;         /**< Recovery requested by callback.    */
    volatile uint8_t pending_new_addr;   /**< NEW_DEV address awaiting open.     */
    volatile bool    have_pending_new;   /**< A NEW_DEV is queued for the task.   */

    /* ---- deferred runtime-tune requests (set by Core-1 set_*; applied by the
     *      single usb_task so ALL USB I/O stays on one task). Each bit asks the
     *      task to push the matching already-stored config field to the chip. -- */
    volatile bool    cfg_dirty_freq;     /**< Re-tune the LO.                    */
    volatile bool    cfg_dirty_rate;     /**< Re-program the resampler.          */
    volatile bool    cfg_dirty_gain;     /**< Re-apply tuner gain.               */
    volatile bool    cfg_dirty_bias;     /**< Re-drive the bias-tee GPIO.        */
    volatile bool    cfg_dirty_start;    /**< Start streaming when possible.     */
    volatile bool    cfg_dirty_stop;     /**< Stop streaming.                    */

    /* ---- identity of the open device (filled at open) ---- */
    usb_rtlsdr_device_info_t info;

    /* ---- effective-rate measurement (windowed) ---- */
    int64_t          rate_window_us;     /**< Start of the current 1 s window.   */
    uint64_t         rate_window_bytes;  /**< Bytes seen in the current window.  */

    /* ---- stats + status (guarded by `lock`) ---- */
    usb_rtlsdr_stats_t  stats;
    esp_err_t           last_error;
    int64_t             last_block_us;

    /* ---- async event callback ---- */
    usb_rtlsdr_event_cb_t event_cb;
    void                 *event_ctx;

    SemaphoreHandle_t lock;              /**< Guards mutable config/stats/handles*/
    bool inited;                         /**< usb_rtlsdr_init() succeeded.       */
} usb_rtlsdr_ctx_t;

#ifdef __cplusplus
}
#endif
