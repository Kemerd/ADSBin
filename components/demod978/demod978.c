/**
 * @file    demod978.c
 * @brief   978 MHz UAT demodulator (FSK) — Core-0 DSP + host-testable core.
 *
 * @details
 *   Pulls 8-bit I/Q from the 978-role IQ ring, runs an FM discriminator to recover
 *   the UAT bit stream (binary CPFSK, 1.041667 Mbps), correlates the 36-bit ADS-B
 *   and uplink sync words, captures the coded bytes that follow, and FEC-decodes
 *   them with the clean-room Reed-Solomon layer (uat_fec). Successful messages are
 *   emitted: ADS-B frames by value onto a uat_frame_t queue, uplink frames by
 *   reference into a uat_uplink_t ring.
 *
 *   The DSP "core" (discriminator + bit recovery + sync + FEC dispatch) is written
 *   WITHOUT FreeRTOS/ESP-IDF so the identical code links into a host unit test that
 *   feeds it a synthesized FSK waveform. The FreeRTOS task is a thin shell around it.
 *
 *   REAL-TIME CONTRACT (same as demod1090): the task runs on Core 0 and must never
 *   block; on output backpressure it drops-with-counter and immediately moves on.
 *   RS decode runs only on a sync hit (a few times/sec), so its cost is amortized.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 *
 * Clean-room from the public DO-282B UAT spec + published sync words. No code
 * adapted from dump978 / any GPL source.
 */

#include <string.h>
#include <stdlib.h>

#include "demod978_internal.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  PURE CORE — no FreeRTOS / ESP-IDF below this banner (host-testable).
 * ═══════════════════════════════════════════════════════════════════════════ */

/* RTL2832U offset-binary mid-scale: a code of ~127.4 on I and Q is "zero". We use
 * 127 (integer) for the discriminator; the half-LSB bias is irrelevant to the SIGN
 * of the instantaneous frequency, which is all a 2-FSK slicer needs. */
#define UAT_DC_BIAS  127

uint32_t uat_discriminator(const uint8_t *iq, uint32_t n_pairs, int16_t *out)
{
    /* Need at least two pairs to take a difference. */
    if (n_pairs < 2) {
        return 0;
    }

    /* Previous centred sample (I,Q). Seed from pair 0. */
    int prev_i = (int)iq[0] - UAT_DC_BIAS;
    int prev_q = (int)iq[1] - UAT_DC_BIAS;

    uint32_t w = 0;
    for (uint32_t n = 1; n < n_pairs; n++) {
        int ci = (int)iq[2 * n]     - UAT_DC_BIAS;
        int cq = (int)iq[2 * n + 1] - UAT_DC_BIAS;

        /* Instantaneous frequency ∝ imaginary part of (conj(prev) * cur):
         *   q = prev_i*cur_q - prev_q*cur_i.
         * Its SIGN is the FSK bit; its magnitude (scaled down to fit int16) is a
         * soft confidence the sync correlator can weight if desired. The product
         * of two centred 8-bit values fits in ~15 bits; >>4 keeps it well inside
         * int16 with headroom for the burst. */
        int cross = prev_i * cq - prev_q * ci;
        int v = cross >> 4;
        if (v > 32767)  v = 32767;
        if (v < -32768) v = -32768;
        out[w++] = (int16_t)v;

        prev_i = ci;
        prev_q = cq;
    }
    return w;
}

/* ── Sync-word fuzzy match: popcount of the 36-bit XOR, against a tolerance. ─── */

/** @brief Count set bits in a 64-bit word (portable, no builtins for the test). */
static inline int popcount64(uint64_t x)
{
    int c = 0;
    while (x) {
        x &= (x - 1);
        c++;
    }
    return c;
}

/**
 * @brief Slice one bit from the discriminator at a fractional sample position.
 *
 * @details
 *   The bit value is simply the SIGN of the discriminator at the bit centre. We
 *   integrate over a small window around the centre for noise immunity (a cheap
 *   integrate-and-dump), summing the soft values and taking the sign of the sum.
 *
 * @param disc     Discriminator buffer.
 * @param nsamps   Length of @p disc.
 * @param fp_pos   Fractional sample position (32.32) of the bit centre.
 * @param fp_per_bit  Samples per bit (32.32), for sizing the integration window.
 * @return 1 or 0 (the recovered bit), or -1 if @p fp_pos is past the buffer.
 */
static int slice_bit(const int16_t *disc, uint32_t nsamps,
                     uint64_t fp_pos, uint64_t fp_per_bit)
{
    /* Integration half-window = a quarter bit, in whole samples (>=1). */
    uint32_t half = (uint32_t)((fp_per_bit >> UAT_FP_SHIFT) / 4);
    if (half == 0) {
        half = 1;
    }

    uint32_t centre = (uint32_t)(fp_pos >> UAT_FP_SHIFT);
    if (centre >= nsamps) {
        return -1;
    }

    /* Sum the soft discriminator across [centre-half, centre+half]. */
    long sum = 0;
    uint32_t lo = (centre > half) ? (centre - half) : 0;
    uint32_t hi = centre + half;
    if (hi >= nsamps) {
        hi = nsamps - 1;
    }
    for (uint32_t s = lo; s <= hi; s++) {
        sum += disc[s];
    }
    return (sum >= 0) ? 1 : 0;
}

/**
 * @brief Read @p nbits bits starting at fractional sample @p fp_start into an MSB-
 *        first 64-bit accumulator (used to test a candidate sync word).
 *
 * @return true if all bits were in range; false if the window ran off the buffer.
 */
static bool read_bits_u64(const int16_t *disc, uint32_t nsamps,
                          uint64_t fp_start, uint64_t fp_per_bit,
                          int nbits, uint64_t *out_bits)
{
    uint64_t acc = 0;
    uint64_t fp = fp_start;
    for (int i = 0; i < nbits; i++) {
        int b = slice_bit(disc, nsamps, fp, fp_per_bit);
        if (b < 0) {
            return false;
        }
        acc = (acc << 1) | (uint64_t)b;
        fp += fp_per_bit;
    }
    *out_bits = acc;
    return true;
}

/**
 * @brief Read @p nbytes coded bytes (MSB-first) starting at fractional sample
 *        @p fp_start into @p out.
 *
 * @return true if all bytes were in range.
 */
static bool read_bytes(const int16_t *disc, uint32_t nsamps,
                       uint64_t fp_start, uint64_t fp_per_bit,
                       int nbytes, uint8_t *out)
{
    uint64_t fp = fp_start;
    for (int i = 0; i < nbytes; i++) {
        uint8_t byte = 0;
        for (int bit = 0; bit < 8; bit++) {
            int b = slice_bit(disc, nsamps, fp, fp_per_bit);
            if (b < 0) {
                return false;
            }
            byte = (uint8_t)((byte << 1) | (uint8_t)b);
            fp += fp_per_bit;
        }
        out[i] = byte;
    }
    return true;
}

/* ── UAT message-type discrimination from the decoded payload. ───────────────
 *  After a Basic (RS(30,18)) decode succeeds we inspect the payload's leading
 *  bits: the UAT "Message Type" (the top 5 bits of the first payload byte) tells
 *  us whether the transmission is actually a LONG message. dump978-class decoders
 *  use the rule "MSG TYPE 0 => short; otherwise long". We re-derive that from the
 *  public message-format: type-codes >= 1 carry a long payload, so on a long type
 *  we re-decode over the longer coded span. */
static bool uat_type_is_long(const uint8_t *payload18)
{
    uint8_t mt = (uint8_t)(payload18[0] >> 3);   /* top 5 bits = message type. */
    return mt != 0;
}

int uat_core_process(const int16_t *disc, uint32_t nsamps,
                     uint64_t fp_per_bit, int sync_max_err,
                     uat_core_emit_fn emit, void *user,
                     uint64_t *out_adsb_hits, uint64_t *out_uplink_hits,
                     uint64_t *out_rs_fail)
{
    int emitted = 0;
    if (nsamps < UAT_SYNC_BITS) {
        return 0;
    }

    /* Slide a candidate sync window one sample at a time. At each start sample we
     * read 36 bits and compare (fuzzy) against both sync words. A real receiver
     * could correlate more cleverly, but a per-sample 36-bit read is cheap enough
     * here and is exactly what the host test wants to validate. */
    for (uint32_t start = 0; start < nsamps; start++) {
        /* Fractional sample position of this candidate's first sync bit. */
        uint64_t fp_start = (uint64_t)start * fp_per_bit;

        uint64_t sync = 0;
        if (!read_bits_u64(disc, nsamps, fp_start, fp_per_bit, UAT_SYNC_BITS, &sync)) {
            break;   /* ran off the end; no more candidates fit. */
        }
        sync &= UAT_SYNC_MASK;

        int adsb_err   = popcount64((sync ^ UAT_ADSB_SYNC_WORD)   & UAT_SYNC_MASK);
        int uplink_err = popcount64((sync ^ UAT_UPLINK_SYNC_WORD) & UAT_SYNC_MASK);

        bool is_adsb   = (adsb_err   <= sync_max_err);
        bool is_uplink = (uplink_err <= sync_max_err);
        if (!is_adsb && !is_uplink) {
            continue;
        }
        /* If both somehow match (shouldn't, they are complements), prefer the
         * closer one. */
        if (is_adsb && is_uplink) {
            if (adsb_err <= uplink_err) { is_uplink = false; } else { is_adsb = false; }
        }

        /* The coded bytes begin right after the 36-bit sync word. */
        uint64_t fp_data = fp_start + (uint64_t)UAT_SYNC_BITS * fp_per_bit;

        if (is_adsb) {
            if (out_adsb_hits) { (*out_adsb_hits)++; }

            /* A UAT ADS-B transmission is EITHER a Basic (RS(30,18)) or a Long
             * (RS(48,34)) frame, and the two coded spans differ, so we cannot know
             * which it is until the FEC succeeds. We try BOTH and accept the one
             * that decodes with a self-consistent message type:
             *   - A Basic frame's 30 coded bytes are NOT a valid RS(48,34) prefix,
             *     and vice-versa, so at most one decode succeeds for a real frame.
             *   - The decoded payload's message type (top 5 bits) must agree with
             *     the frame length (type 0 => short, type != 0 => long); this
             *     rejects a stray RS "success" on the wrong-length capture. ─────── */
            bool handled = false;

            /* Attempt 1: Basic RS(30,18). */
            uint8_t cws[UAT_ADSB_SHORT_CODED_BYTES];
            if (read_bytes(disc, nsamps, fp_data, fp_per_bit,
                           UAT_ADSB_SHORT_CODED_BYTES, cws)) {
                int rc = uat_fec_adsb_short_decode(cws);
                if (rc >= 0 && !uat_type_is_long(cws)) {
                    if (emit) {
                        uat_decoded_msg_t m;
                        m.kind = UAT_MSG_ADSB_SHORT;
                        m.len  = UAT_ADSB_SHORT_BYTES;
                        m.rs_errors = rc;
                        m.block_errors = 0;
                        memcpy(m.data, cws, UAT_ADSB_SHORT_BYTES);
                        emit(&m, user);
                    }
                    handled = true;
                }
            }

            /* Attempt 2: Long RS(48,34), if Basic did not claim the frame. */
            if (!handled) {
                uint8_t cwl[UAT_ADSB_LONG_CODED_BYTES];
                if (read_bytes(disc, nsamps, fp_data, fp_per_bit,
                               UAT_ADSB_LONG_CODED_BYTES, cwl)) {
                    int rcl = uat_fec_adsb_long_decode(cwl);
                    if (rcl >= 0 && uat_type_is_long(cwl)) {
                        if (emit) {
                            uat_decoded_msg_t m;
                            m.kind = UAT_MSG_ADSB_LONG;
                            m.len  = UAT_ADSB_LONG_BYTES;
                            m.rs_errors = rcl;
                            m.block_errors = 0;
                            memcpy(m.data, cwl, UAT_ADSB_LONG_BYTES);
                            emit(&m, user);
                        }
                        handled = true;
                    }
                }
            }

            if (!handled) {
                if (out_rs_fail) { (*out_rs_fail)++; }
                continue;
            }
            emitted++;
            /* Skip past this frame so we do not re-trigger inside its body. The
             * for-loop's own start++ then advances one further; fp_start is
             * recomputed from `start` at the top of the next iteration. */
            start += (uint32_t)((UAT_SYNC_BITS + UAT_ADSB_SHORT_CODED_BYTES * 8)
                                * (fp_per_bit >> UAT_FP_SHIFT));
        } else { /* is_uplink */
            if (out_uplink_hits) { (*out_uplink_hits)++; }

            uint8_t frame[UAT_UPLINK_CODED_BYTES];
            if (!read_bytes(disc, nsamps, fp_data, fp_per_bit,
                            UAT_UPLINK_CODED_BYTES, frame)) {
                continue;
            }
            uint8_t payload[UAT_UPLINK_PAYLOAD_BYTES];
            int total = 0, blkerr = 0;
            bool ok = uat_fec_uplink_decode(frame, payload, &total, &blkerr);
            if (!ok) {
                if (out_rs_fail) { (*out_rs_fail)++; }
                /* Still emit best-effort? No — a bad uplink must not reach the EFB. */
                continue;
            }
            if (emit) {
                uat_decoded_msg_t m;
                m.kind = UAT_MSG_UPLINK;
                m.len  = UAT_UPLINK_PAYLOAD_BYTES;
                m.rs_errors = total;
                m.block_errors = blkerr;
                memcpy(m.data, payload, UAT_UPLINK_PAYLOAD_BYTES);
                emit(&m, user);
            }
            emitted++;
            start += (uint32_t)((UAT_SYNC_BITS + UAT_UPLINK_CODED_BYTES * 8)
                                * (fp_per_bit >> UAT_FP_SHIFT));
        }
    }

    return emitted;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  FreeRTOS task shell — only compiled into the firmware (not the host test).
 * ═══════════════════════════════════════════════════════════════════════════ */
#ifndef UAT_HOST_TEST

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "demod978";

/* The one and only demodulator instance. */
static demod978_ctx_t s_ctx;

/* How long the task waits on the ring before re-checking `running`. */
#define UAT_RING_WAIT_MS  20

/* ── Stats helpers (mirror demod1090) ──────────────────────────────────────── */
static inline void stats_lock(void)   { if (s_ctx.stats_mux) xSemaphoreTake(s_ctx.stats_mux, portMAX_DELAY); }
static inline void stats_unlock(void) { if (s_ctx.stats_mux) xSemaphoreGive(s_ctx.stats_mux); }

/* Context carried into the emit callback so it can enqueue + count. */
typedef struct {
    int64_t rx_time_us;
} uat_emit_ctx_t;

/**
 * @brief Emit callback (runs inline in the core, on the demod task). Enqueues a
 *        decoded message to the right output and bumps the right counter.
 */
static void demod978_emit(const uat_decoded_msg_t *m, void *user)
{
    uat_emit_ctx_t *ec = (uat_emit_ctx_t *)user;

    if (m->kind == UAT_MSG_UPLINK) {
        /* Uplink: reserve header+payload contiguously in the no-split ring and
         * fill a self-describing uat_uplink_t (same trick as the IQ ring). On a
         * full ring we drop-and-count — never block the Core-0 path. */
        const size_t need = sizeof(uat_uplink_t) + UAT_UPLINK_PAYLOAD_BYTES;
        void *slot = NULL;
        if (s_ctx.out_uplink_ring &&
            xRingbufferSendAcquire(s_ctx.out_uplink_ring, &slot, need, 0) == pdTRUE &&
            slot) {
            uat_uplink_t *up = (uat_uplink_t *)slot;
            up->payload      = (const uint8_t *)slot + sizeof(uat_uplink_t);
            up->payload_len  = UAT_UPLINK_PAYLOAD_BYTES;
            up->rs_errors    = (uint8_t)(m->rs_errors > 255 ? 255 : m->rs_errors);
            up->block_errors = (uint8_t)m->block_errors;
            up->rx_time_us   = ec->rx_time_us;
            memcpy((void *)up->payload, m->data, UAT_UPLINK_PAYLOAD_BYTES);
            xRingbufferSendComplete(s_ctx.out_uplink_ring, slot);
            stats_lock(); s_ctx.stats.uplinks_emitted++; stats_unlock();
        } else {
            stats_lock(); s_ctx.stats.queue_overflows++; stats_unlock();
        }
        return;
    }

    /* ADS-B short/long: by-value onto the frame queue. */
    uat_frame_t f;
    memset(&f, 0, sizeof(f));
    f.len_bytes    = (uint8_t)m->len;
    f.rs_errors    = (uint8_t)(m->rs_errors > 255 ? 255 : m->rs_errors);
    f.signal_level = 0;
    f.rx_time_us   = ec->rx_time_us;
    memcpy(f.data, m->data, m->len);

    if (s_ctx.out_frame_q && xQueueSend(s_ctx.out_frame_q, &f, 0) == pdTRUE) {
        stats_lock(); s_ctx.stats.frames_emitted++; stats_unlock();
    } else {
        stats_lock(); s_ctx.stats.queue_overflows++; stats_unlock();
    }
}

/** @brief Ensure the discriminator scratch holds at least @p n samples. */
static bool ensure_disc_cap(uint32_t n)
{
    if (s_ctx.disc_cap >= n) {
        return true;
    }
    int16_t *p = (int16_t *)heap_caps_realloc(s_ctx.disc, (size_t)n * sizeof(int16_t),
                                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!p) {
        return false;
    }
    s_ctx.disc     = p;
    s_ctx.disc_cap = n;
    return true;
}

/** @brief Process one IQ block: discriminator -> core -> emit. */
static void process_block(const iq_block_t *blk)
{
    uint32_t n_pairs = blk->n_bytes / 2u;
    if (n_pairs < 2) {
        return;
    }
    if (!ensure_disc_cap(n_pairs)) {
        stats_lock(); s_ctx.stats.iq_blocks_dropped++; stats_unlock();
        return;
    }

    uint32_t ndisc = uat_discriminator(blk->samples, n_pairs, s_ctx.disc);

    uint64_t adsb_hits = 0, uplink_hits = 0, rs_fail = 0;
    uat_emit_ctx_t ec = { .rx_time_us = blk->t_capture_us };
    uat_core_process(s_ctx.disc, ndisc, s_ctx.fp_samp_per_bit,
                     s_ctx.sync_max_errors, demod978_emit, &ec,
                     &adsb_hits, &uplink_hits, &rs_fail);

    stats_lock();
    s_ctx.stats.iq_blocks_consumed++;
    s_ctx.stats.adsb_sync_hits   += adsb_hits;
    s_ctx.stats.uplink_sync_hits += uplink_hits;
    s_ctx.stats.rs_uncorrectable += rs_fail;
    stats_unlock();
}

/** @brief The Core-0 demod task: pop IQ blocks, demodulate, emit. */
static void demod978_task(void *arg)
{
    (void)arg;
    s_ctx.task_alive = true;
    ESP_LOGI(TAG, "demod978 task up on core %d", (int)xPortGetCoreID());

    while (s_ctx.running) {
        size_t item_size = 0;
        iq_block_t *blk = (iq_block_t *)xRingbufferReceive(s_ctx.iq_ring, &item_size,
                                                           pdMS_TO_TICKS(UAT_RING_WAIT_MS));
        if (!blk) {
            continue;   /* timeout: re-check running and loop. */
        }

        /* Track ring sequence gaps as dropped blocks (we couldn't keep up). */
        if (s_ctx.have_seq && blk->seq != s_ctx.last_seq + 1) {
            uint32_t gap = blk->seq - s_ctx.last_seq - 1;
            stats_lock(); s_ctx.stats.iq_blocks_dropped += gap; stats_unlock();
        }
        s_ctx.last_seq = blk->seq;
        s_ctx.have_seq = true;

        process_block(blk);

        /* Return the borrowed item immediately — never retain blk->samples. */
        vRingbufferReturnItem(s_ctx.iq_ring, blk);
    }

    s_ctx.task_alive = false;
    vTaskDelete(NULL);
}

/* ── Public lifecycle ───────────────────────────────────────────────────────── */

esp_err_t demod978_init(const demod978_config_t *cfg)
{
    if (s_ctx.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(&s_ctx, 0, sizeof(s_ctx));

    s_ctx.sample_rate_hz  = (cfg && cfg->sample_rate_hz) ? cfg->sample_rate_hz : UAT_SAMPLE_RATE_HZ;
    s_ctx.task_core_id    = (cfg && cfg->task_core_id)   ? cfg->task_core_id   : ADSBIN_CORE_DSP;
    s_ctx.task_priority   = (cfg && cfg->task_priority)  ? cfg->task_priority  : UAT_DEFAULT_PRIORITY;
    s_ctx.task_stack_size = (cfg && cfg->task_stack_size)? cfg->task_stack_size: UAT_DEFAULT_STACK;
    s_ctx.sync_max_errors = (cfg && cfg->sync_max_errors)? cfg->sync_max_errors: UAT_DEFAULT_SYNC_MAX_ERR;

    /* Precompute samples-per-bit in 32.32 fixed point: fs / bit_rate. */
    s_ctx.fp_samp_per_bit = ((uint64_t)s_ctx.sample_rate_hz << UAT_FP_SHIFT) / UAT_BIT_RATE_HZ;

    /* Make sure the RS field tables are built before the hot path runs. */
    uat_fec_init();

    s_ctx.stats_mux = xSemaphoreCreateMutex();
    if (!s_ctx.stats_mux) {
        return ESP_ERR_NO_MEM;
    }

    s_ctx.inited = true;
    ESP_LOGI(TAG, "init: %u sps, %llu fp samp/bit, sync_max_err=%u",
             s_ctx.sample_rate_hz, (unsigned long long)s_ctx.fp_samp_per_bit,
             s_ctx.sync_max_errors);
    return ESP_OK;
}

esp_err_t demod978_start(RingbufHandle_t iq_ring, QueueHandle_t out_frame_queue,
                         RingbufHandle_t out_uplink_ring)
{
    if (!s_ctx.inited || s_ctx.running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!iq_ring || !out_frame_queue || !out_uplink_ring) {
        return ESP_ERR_INVALID_ARG;
    }

    s_ctx.iq_ring         = iq_ring;
    s_ctx.out_frame_q     = out_frame_queue;
    s_ctx.out_uplink_ring = out_uplink_ring;
    s_ctx.have_seq        = false;
    s_ctx.running         = true;

    BaseType_t ok = xTaskCreatePinnedToCore(demod978_task, "demod978",
                                            s_ctx.task_stack_size, NULL,
                                            s_ctx.task_priority, &s_ctx.task,
                                            s_ctx.task_core_id);
    if (ok != pdPASS) {
        s_ctx.running = false;
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t demod978_stop(void)
{
    if (!s_ctx.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx.running = false;
    for (int i = 0; i < 200 && s_ctx.task_alive; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_ctx.task = NULL;
    return ESP_OK;
}

void demod978_deinit(void)
{
    if (!s_ctx.inited) {
        return;
    }
    demod978_stop();
    if (s_ctx.disc) {
        heap_caps_free(s_ctx.disc);
        s_ctx.disc = NULL;
        s_ctx.disc_cap = 0;
    }
    if (s_ctx.stats_mux) {
        vSemaphoreDelete(s_ctx.stats_mux);
        s_ctx.stats_mux = NULL;
    }
    s_ctx.inited = false;
}

void demod978_get_stats(demod978_stats_t *out)
{
    if (!out) {
        return;
    }
    stats_lock();
    *out = s_ctx.stats;
    stats_unlock();
}

void demod978_reset_stats(void)
{
    stats_lock();
    memset(&s_ctx.stats, 0, sizeof(s_ctx.stats));
    stats_unlock();
}

#endif /* !UAT_HOST_TEST */
