/**
 * @file    modes_decode.c
 * @brief   Mode-S / ADS-B frame decoder with internal CPR position resolution.
 *
 * @details
 *   This is the Stage-3 worker of the ADSBin pipeline. It takes raw, CRC-unchecked
 *   candidate frames produced by demod1090, validates the 24-bit Mode-S parity,
 *   gates to the ADS-B downlink formats (DF17 / DF18), parses the message body
 *   (identification, airborne position, velocity), and — crucially — RESOLVES the
 *   position locally. It owns a fixed-size per-ICAO even/odd CPR pairing cache so
 *   the emitted ::adsb_msg_t already carries an absolute WGS-84 latitude/longitude
 *   when @c has_position is set; the traffic manager never sees raw CPR.
 *
 *   Everything here runs on Core 1 (::ADSBIN_CORE_DECODE) inside a single decode
 *   task. The pairing cache is therefore single-owner and is deliberately NOT
 *   internally locked — that is a documented invariant, not an oversight. The
 *   only cross-task reader is modes_decode_get_stats(), which takes a cheap
 *   word-by-word snapshot of the counters.
 *
 * @par CRC provenance
 *   The 24-bit Mode-S CRC uses the standard generator polynomial 0xFFF409. The
 *   table-driven implementation and the DF17/18 parity→ICAO recovery are adapted
 *   from antirez's BSD-licensed dump1090; the BSD notice is preserved in cpr.c
 *   and applies equally to the CRC routine reproduced here. No GPL fork code was
 *   consulted.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "modes_decode.h"
#include "modes_internal.h"
#include "adsbin_types.h"
#include "ownship.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

/* M_PI is not in ISO C, only POSIX; define our own so the velocity vector math
 * compiles cleanly under -std=c11 on any toolchain. */
#ifndef MODES_PI
#define MODES_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration defaults (see modes_types.h modes_decode_cfg_t).
 * ═══════════════════════════════════════════════════════════════════════════ */

#define MODES_DEFAULT_PAIR_AGE_US   10000000u   /**< 10 s, per dump1090.          */
#define MODES_DEFAULT_CACHE_SLOTS   256u        /**< Per-ICAO pairing entries.    */
#define MODES_CACHE_SLOTS_MAX       4096u       /**< Hard ceiling, sanity clamp.  */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Module-private state — one decoder instance per firmware (single owner).
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief One per-ICAO slot in the CPR pairing cache.
 *
 * We retain the most recent even AND odd airborne-position frame for an aircraft
 * so a later opposite-parity frame can be globally paired without any malloc.
 * @c valid_even / @c valid_odd say which halves currently hold real data.
 */
typedef struct {
    uint32_t          icao;         /**< Aircraft key; 0 == empty slot.           */
    bool              valid_even;   /**< @c even holds a usable frame.            */
    bool              valid_odd;    /**< @c odd  holds a usable frame.            */
    modes_cpr_frame_t even;         /**< Last even-format frame seen.             */
    modes_cpr_frame_t odd;          /**< Last odd-format  frame seen.             */
    int64_t           last_used_us; /**< For LRU eviction when the table is full. */
} cpr_slot_t;

/**
 * @brief Everything modes_decode owns between init and deinit.
 */
typedef struct {
    bool                 inited;     /**< Guards double-init / use-before-init.   */
    modes_decode_cfg_t   cfg;        /**< Effective (defaulted) configuration.    */
    cpr_slot_t          *slots;      /**< Heap-allocated pairing cache.           */
    uint16_t             n_slots;    /**< Number of entries in @c slots.          */
    modes_decode_stats_t stats;      /**< Cumulative decode counters.             */
} modes_state_t;

/* The single instance. Zero-initialised => inited == false until init runs. */
static modes_state_t g_modes;

/* ═══════════════════════════════════════════════════════════════════════════
 *  24-bit Mode-S CRC
 *
 *  Mode-S uses a 24-bit CRC with generator polynomial 0xFFF409 (the leading
 *  implicit bit dropped). For DF17/18 the 24-bit AP field at the tail equals the
 *  bare CRC of the preceding bits — so a correct extended-squitter frame CRCs to
 *  zero over its whole length. We build a 256-entry byte table once at init.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define MODES_CRC_POLY  0xFFF409u   /**< Mode-S 24-bit generator polynomial.      */

static uint32_t g_crc_table[256];   /**< Byte-wise CRC lookup, seeded at init.    */
static bool     g_crc_ready;        /**< True once g_crc_table is populated.       */

/**
 * @brief Populate the 256-entry CRC lookup table for polynomial 0xFFF409.
 *
 * Standard MSB-first table construction: for each possible leading byte, shift
 * the byte into the high end of a 24-bit register and divide by the polynomial
 * eight times, recording the residue.
 */
static void modes_crc_build_table(void)
{
    for (uint32_t byte = 0; byte < 256; ++byte) {
        // Seed the 24-bit register with this byte in its most-significant slot.
        uint32_t crc = byte << 16;

        // Eight rounds of polynomial division (one per bit of the byte).
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x800000u) {
                crc = (crc << 1) ^ MODES_CRC_POLY;   // top bit set => subtract poly
            } else {
                crc = (crc << 1);                    // otherwise just shift up
            }
        }
        // Keep only the low 24 bits — Mode-S CRC is 24-bit wide.
        g_crc_table[byte] = crc & 0xFFFFFFu;
    }
    g_crc_ready = true;
}

/**
 * @brief Compute the 24-bit Mode-S CRC over the message bits, excluding the
 *        trailing 24-bit parity/AP field.
 *
 * @param frame      MSB-first frame bytes.
 * @param frame_len  7 or 14.
 * @return The 24-bit CRC residue of the message portion (bytes [0 .. len-4]).
 */
static uint32_t modes_crc_compute(const uint8_t *frame, size_t frame_len)
{
    uint32_t crc = 0;

    // The last 3 bytes are the parity field; CRC covers everything before them.
    const size_t n = frame_len - 3;

    // Byte-at-a-time table division. Each step folds the next message byte into
    // the running 24-bit residue.
    for (size_t i = 0; i < n; ++i) {
        uint8_t idx = (uint8_t)(((crc >> 16) ^ frame[i]) & 0xFFu);
        crc = ((crc << 8) ^ g_crc_table[idx]) & 0xFFFFFFu;
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Lifecycle
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t modes_decode_init(const modes_decode_cfg_t *cfg)
{
    // Idempotent guard: a second init without deinit is a no-op success so a
    // caller cannot accidentally leak the first cache allocation.
    if (g_modes.inited) {
        return ESP_OK;
    }

    // ── Resolve effective configuration (NULL => all defaults). ──────────────
    modes_decode_cfg_t eff = {
        .max_cpr_pair_age_us = MODES_DEFAULT_PAIR_AGE_US,
        .enable_local_cpr    = true,
        .cpr_cache_slots     = (uint16_t)MODES_DEFAULT_CACHE_SLOTS,
    };
    if (cfg != NULL) {
        // Honour caller overrides, but defend the cache size against zero/huge.
        if (cfg->max_cpr_pair_age_us != 0) {
            eff.max_cpr_pair_age_us = cfg->max_cpr_pair_age_us;
        }
        eff.enable_local_cpr = cfg->enable_local_cpr;
        if (cfg->cpr_cache_slots != 0) {
            eff.cpr_cache_slots = cfg->cpr_cache_slots;
        }
    }
    // Clamp to a sane ceiling so a bad config cannot exhaust PSRAM.
    if (eff.cpr_cache_slots > MODES_CACHE_SLOTS_MAX) {
        eff.cpr_cache_slots = (uint16_t)MODES_CACHE_SLOTS_MAX;
    }

    // ── Allocate the per-ICAO pairing cache (zeroed => all slots empty). ─────
    cpr_slot_t *slots = (cpr_slot_t *)calloc(eff.cpr_cache_slots, sizeof(cpr_slot_t));
    if (slots == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // ── Seed the CRC table once (idempotent across re-inits). ────────────────
    if (!g_crc_ready) {
        modes_crc_build_table();
    }

    // ── Commit state. ────────────────────────────────────────────────────────
    memset(&g_modes, 0, sizeof(g_modes));
    g_modes.cfg     = eff;
    g_modes.slots   = slots;
    g_modes.n_slots = eff.cpr_cache_slots;
    g_modes.inited  = true;
    return ESP_OK;
}

void modes_decode_deinit(void)
{
    if (!g_modes.inited) {
        return;
    }
    // Release the pairing cache and wipe the instance back to the empty state.
    free(g_modes.slots);
    memset(&g_modes, 0, sizeof(g_modes));
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  DF gate + CRC + ICAO
 * ═══════════════════════════════════════════════════════════════════════════ */

modes_df_t modes_decode_df(const uint8_t *frame, size_t frame_len)
{
    // The downlink format is the top 5 bits of the first byte. We do not even
    // need a valid length to classify — but a missing buffer is unsupported.
    if (frame == NULL || frame_len == 0) {
        return MODES_DF_UNSUPPORTED;
    }

    const uint8_t df = (uint8_t)(frame[0] >> 3);

    // Only the two extended-squitter formats interest the ADS-B decoder.
    switch (df) {
        case 17: return MODES_DF_ADSB;
        case 18: return MODES_DF_TISB;
        default: return MODES_DF_UNSUPPORTED;
    }
}

modes_crc_t modes_decode_check_crc(const uint8_t *frame, size_t frame_len, uint32_t *out_icao)
{
    // Length gate first: Mode-S frames are exactly short (7) or long (14) bytes.
    if (frame == NULL) {
        return MODES_CRC_BAD_LEN;
    }
    if (frame_len != MODES_SHORT_BYTES && frame_len != MODES_LONG_BYTES) {
        return MODES_CRC_BAD_LEN;
    }

    // Lazily seed the table so the granular helpers work even before init (the
    // host bench harness calls check_crc without a full modes_decode_init).
    if (!g_crc_ready) {
        modes_crc_build_table();
    }

    // ── Recover the transmitted parity field (last 3 bytes, MSB-first). ──────
    const uint32_t parity =
        ((uint32_t)frame[frame_len - 3] << 16) |
        ((uint32_t)frame[frame_len - 2] << 8)  |
        ((uint32_t)frame[frame_len - 1]);

    // ── Compute the expected CRC over the message portion. ───────────────────
    const uint32_t crc = modes_crc_compute(frame, frame_len);

    // For DF17/DF18 (extended squitter) the AP field carries no overlaid
    // address: a good frame satisfies crc == parity, and the ICAO is read
    // straight from the AA field. The XOR of the two recovers the ICAO in the
    // general overlaid case and is zero when they match — exactly what we want.
    const uint32_t recovered = (crc ^ parity) & 0xFFFFFFu;

    const uint8_t df = (uint8_t)(frame[0] >> 3);
    if (df == 17 || df == 18) {
        // ADS-B / TIS-B: parity must validate cleanly (no address overlay).
        if (recovered != 0) {
            return MODES_CRC_FAIL;
        }
        // ICAO is the 24-bit AA field, bytes [1..3].
        if (out_icao != NULL) {
            *out_icao = ((uint32_t)frame[1] << 16) |
                        ((uint32_t)frame[2] << 8)  |
                        ((uint32_t)frame[3]);
        }
        return MODES_CRC_OK;
    }

    // Any other DF is not something we validate here; report the residue as the
    // (overlaid-address) recovery so callers that care can inspect it, but the
    // ADS-B path treats non-17/18 as unsupported well before reaching here.
    if (out_icao != NULL) {
        *out_icao = recovered;
    }
    return (recovered == 0) ? MODES_CRC_OK : MODES_CRC_FAIL;
}

modes_result_t modes_decode_extract_icao(const uint8_t *frame, size_t frame_len, uint32_t *out_icao)
{
    if (frame == NULL || out_icao == NULL) {
        return MODES_ERR_NULL_ARG;
    }
    if (frame_len != MODES_SHORT_BYTES && frame_len != MODES_LONG_BYTES) {
        return MODES_ERR_BAD_LEN;
    }

    // Only DF17/18 expose the ICAO directly in the AA field; otherwise the
    // address is parity-overlaid and out of scope for the MVP decoder.
    const uint8_t df = (uint8_t)(frame[0] >> 3);
    if (df != 17 && df != 18) {
        return MODES_ERR_UNSUPPORTED_DF;
    }

    // AA field = bytes [1..3], MSB-first.
    *out_icao = ((uint32_t)frame[1] << 16) |
                ((uint32_t)frame[2] << 8)  |
                ((uint32_t)frame[3]);
    return MODES_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Type code classification
 *
 *  For DF17/18 the 112-bit frame carries a 56-bit ME ("message, extended
 *  squitter") field starting at byte 4. Its top 5 bits are the type code, which
 *  selects how the rest of the ME is interpreted.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief Read the ME type code (ME[0] >> 3 == top 5 bits of byte 4). */
static inline uint8_t modes_typecode_raw(const uint8_t *frame)
{
    return (uint8_t)(frame[4] >> 3);
}

modes_msg_kind_t modes_decode_typecode(const uint8_t *frame)
{
    if (frame == NULL) {
        return MODES_MSG_UNKNOWN;
    }

    const uint8_t tc = modes_typecode_raw(frame);

    // Map the ADS-B type-code ranges to high-level kinds (DO-260B Table 2-9).
    if (tc >= 1 && tc <= 4)   return MODES_MSG_IDENT;        // identification + category
    if (tc >= 5 && tc <= 8)   return MODES_MSG_SURFACE_POS;  // surface position
    if (tc >= 9 && tc <= 18)  return MODES_MSG_AIRBORNE_POS; // airborne position (baro)
    if (tc == 19)             return MODES_MSG_AIRBORNE_VEL; // airborne velocity
    if (tc >= 20 && tc <= 22) return MODES_MSG_AIRBORNE_POS; // airborne position (GNSS alt)
    if (tc == 31)             return MODES_MSG_OPSTATUS;     // operational status

    return MODES_MSG_UNKNOWN;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TC 1-4 — Identification (callsign + emitter category)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief The 6-bit-character alphabet used by ADS-B identification (DO-260B).
 *
 * Index by the 6-bit code: 1-26 => A-Z, 32 => space, 48-57 => 0-9. Everything
 * else is reserved and rendered as a space so the callsign stays printable.
 */
static const char IDENT_CHARSET[] =
    "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####"   /* 0-31  (0 and 27-31 invalid -> '#')  */
    " ###############0123456789######";  /* 32-63 (32 space, 48-57 digits)      */

modes_result_t modes_decode_identification(const uint8_t *frame,
                                           char out_callsign[ADSB_CALLSIGN_LEN],
                                           adsb_emitter_category_t *out_category)
{
    if (frame == NULL || out_callsign == NULL) {
        return MODES_ERR_NULL_ARG;
    }

    // Confirm we are actually looking at an identification message.
    const uint8_t tc = modes_typecode_raw(frame);
    if (tc < 1 || tc > 4) {
        return MODES_ERR_UNSUPPORTED_TC;
    }

    // ── Emitter category. ────────────────────────────────────────────────────
    // The 3-bit CA field sits just below the type code in ME byte 0 (bits 6-8).
    // Together with the type code it forms the GDL90/Annex-10 category set. For
    // the ADSBin product we surface the CA directly via the shared enum, whose
    // numeric values intentionally line up with the wire encoding for TC==4.
    const uint8_t ca = (uint8_t)(frame[4] & 0x07u);
    if (out_category != NULL) {
        // Category is only meaningful for TC 1-4; a 0 (NO_INFO) is a valid value.
        // The enum's ordinals match the CA codes, so a direct cast is correct.
        *out_category = (adsb_emitter_category_t)ca;
    }

    // ── Callsign: eight 6-bit characters packed across ME bytes 1-6. ─────────
    // The first character starts at frame bit 40 (the byte-5 boundary); reading
    // eight 6-bit fields walks exactly 48 bits to the end of the ME body.
    for (int i = 0; i < 8; ++i) {
        // Bit 40 is the MSB of ME byte 1 (== frame byte 5). Each char is 6 bits.
        const unsigned start = 40u + (unsigned)(i * 6);
        const uint32_t code  = modes_get_bits(frame, start, 6) & 0x3Fu;

        // Decode through the charset; reserved codes map to '#', trailing pads
        // are spaces which we trim below.
        out_callsign[i] = IDENT_CHARSET[code];
    }
    out_callsign[8] = '\0';

    // ── Trim trailing pad spaces so "DAL123  " becomes "DAL123". ─────────────
    for (int i = 7; i >= 0; --i) {
        if (out_callsign[i] == ' ') {
            out_callsign[i] = '\0';
        } else {
            break;  // hit a real glyph; stop trimming
        }
    }

    return MODES_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Altitude decoding (airborne position ME)
 *
 *  The 12-bit AC field has two encodings selected by the "Q bit":
 *    - Q == 1: a 25-foot binary code (modern, almost universal for ADS-B).
 *    - Q == 0: legacy Gillham (Gray-coded) 100-foot Mode-C, which we decode for
 *              completeness so old transponders still report altitude.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Convert a Gillham (Gray-coded Mode-C) altitude to feet.
 *
 * @details
 *   Mode-C altitude is split into two Gray-coded parts:
 *     - C1 C2 C4 encode the 100-ft sub-position within a 500-ft band, mapping
 *       (via a fixed table) to 1..5; codes 0/6/7 are illegal.
 *     - D2 D4 A1 A2 A4 B1 B2 B4 are a Gray-coded 500-ft band count.
 *   Every odd 500-ft band runs its 100-ft sub-positions in REVERSE so adjacent
 *   bands meet without a discontinuity (the classic Gillham fold). Altitude is
 *   then band*500 + (sub-1)*100 referenced to the -1200 ft Mode-C datum.
 *
 *   Reproduces the ICAO Annex 10 Mode-C decode (same algorithm as antirez's BSD
 *   dump1090 decodeAC13Field/Gillham path).
 *
 * @param d2,d4,a1,a2,a4,b1,b2,b4,c1,c2,c4  The labelled Mode-C interrogation bits.
 * @return Altitude in feet, or INT32_MIN if the code is illegal.
 */
static int32_t modes_gillham_to_ft(uint32_t d2, uint32_t d4,
                                   uint32_t a1, uint32_t a2, uint32_t a4,
                                   uint32_t b1, uint32_t b2, uint32_t b4,
                                   uint32_t c1, uint32_t c2, uint32_t c4)
{
    // ── 100-ft sub-position from the C Gray triple. ──────────────────────────
    // Lookup indexed by (C1<<2 | C2<<1 | C4): 0/3/5 are illegal codes (-1).
    static const int8_t C_TO_HUNDREDS[8] = { -1, 1, 5, -1, 2, -1, 4, 3 };
    const uint32_t c = (c1 << 2) | (c2 << 1) | c4;
    int hundreds = C_TO_HUNDREDS[c & 7u];
    if (hundreds < 0) {
        return INT32_MIN;                   // illegal C code => no altitude
    }

    // ── 500-ft band from the D/A/B Gray code (MSB-first D2..B4). ─────────────
    uint32_t band_gray = (d2 << 7) | (d4 << 6) |
                         (a1 << 5) | (a2 << 4) | (a4 << 3) |
                         (b1 << 2) | (b2 << 1) | (b4 << 0);

    // Binary-reflected Gray -> binary.
    uint32_t band = band_gray;
    band ^= band >> 4;
    band ^= band >> 2;
    band ^= band >> 1;

    // Odd bands count their 100-ft sub-positions in reverse (the fold).
    if (band & 1u) {
        hundreds = 6 - hundreds;
    }

    // Combine the band and sub-position, referenced to the -1200 ft datum.
    return (int32_t)band * 500 + (hundreds - 1) * 100 - 1200;
}

/**
 * @brief Decode the 12-bit AC altitude field to feet (Q-bit aware).
 *
 * @param ac12     The raw 12-bit AC field, right-justified.
 * @param out_ft   Receives altitude in feet on success.
 * @return true if a valid altitude was decoded; false for the "no altitude"
 *         code (all zero) or an illegal Gillham value.
 */
static bool modes_decode_altitude_ac12(uint32_t ac12, int32_t *out_ft)
{
    // All-zero AC means the transmitter is not reporting altitude.
    if (ac12 == 0) {
        return false;
    }

    // The Q bit is AC bit 4 (zero-based bit index 4 from the LSB end == the
    // bit at value 0x10 within the 12-bit field).
    const bool q_bit = (ac12 & 0x10u) != 0;

    if (q_bit) {
        // ── 25-foot binary code. ─────────────────────────────────────────────
        // Remove the Q bit and re-pack the surrounding 11 bits into one integer
        // N, then altitude = N*25 - 1000 ft.
        const uint32_t lower = ac12 & 0x0Fu;          // bits below Q
        const uint32_t upper = (ac12 >> 5) & 0x7Fu;   // bits above Q
        const uint32_t n = (upper << 4) | lower;      // 11-bit binary count
        *out_ft = (int32_t)n * 25 - 1000;
        return true;
    }

    // ── Q == 0 : legacy Gillham (Mode-C) 100-ft code. ────────────────────────
    // The 12 AC bits are laid out, MSB (bit 11) to LSB (bit 0), as:
    //   C1 A1 C2 A2 C4 A4 [Q] B1 D1 B2 D2 B4
    // (D4 is not transmitted in the 12-bit field and is taken as 0.) Pull each
    // labelled bit straight out of its position and hand them to the decoder.
    const uint32_t c1 = (ac12 >> 11) & 1u;
    const uint32_t a1 = (ac12 >> 10) & 1u;
    const uint32_t c2 = (ac12 >> 9)  & 1u;
    const uint32_t a2 = (ac12 >> 8)  & 1u;
    const uint32_t c4 = (ac12 >> 7)  & 1u;
    const uint32_t a4 = (ac12 >> 6)  & 1u;
    /* bit 5 is the Q bit (0 in this branch) */
    const uint32_t b1 = (ac12 >> 4)  & 1u;
    const uint32_t d1 = (ac12 >> 3)  & 1u;   /* D1 unused by the altitude code   */
    const uint32_t b2 = (ac12 >> 2)  & 1u;
    const uint32_t d2 = (ac12 >> 1)  & 1u;
    const uint32_t b4 = (ac12 >> 0)  & 1u;
    const uint32_t d4 = 0u;                   /* D4 absent in the 12-bit field    */
    (void)d1;                                 /* documented but not used          */

    const int32_t ft = modes_gillham_to_ft(d2, d4, a1, a2, a4, b1, b2, b4, c1, c2, c4);
    if (ft == INT32_MIN) {
        return false;   // illegal Gillham code — treat as no altitude
    }
    *out_ft = ft;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TC 9-18 / 20-22 — Airborne position (raw CPR + altitude)
 * ═══════════════════════════════════════════════════════════════════════════ */

modes_result_t modes_decode_airborne_position(const uint8_t *frame, modes_cpr_frame_t *out_cpr,
                                              int32_t *out_altitude_ft, bool *out_altitude_valid)
{
    if (frame == NULL || out_cpr == NULL) {
        return MODES_ERR_NULL_ARG;
    }

    // Only airborne-position type codes carry this layout.
    const uint8_t tc = modes_typecode_raw(frame);
    const bool baro_pos = (tc >= 9 && tc <= 18);
    const bool gnss_pos = (tc >= 20 && tc <= 22);
    if (!baro_pos && !gnss_pos) {
        return MODES_ERR_UNSUPPORTED_TC;
    }

    // ── Altitude: the 12-bit AC field occupies ME bits 9-20. ─────────────────
    // ME starts at frame bit 32; AC starts 8 bits in => frame bit 40, width 12.
    // (For TC 20-22 this same field is geometric/GNSS altitude in 25-ft steps.)
    const uint32_t ac12 = modes_get_bits(frame, 40, 12);
    int32_t alt_ft = 0;
    bool    alt_ok = modes_decode_altitude_ac12(ac12, &alt_ft);
    if (out_altitude_ft != NULL) {
        *out_altitude_ft = alt_ok ? alt_ft : 0;
    }
    if (out_altitude_valid != NULL) {
        *out_altitude_valid = alt_ok;
    }

    // ── CPR fields. ──────────────────────────────────────────────────────────
    // Format bit F is ME bit 22 (frame bit 53). Then 17-bit lat (frame bit 54)
    // and 17-bit lon (frame bit 71) follow contiguously.
    out_cpr->odd     = (modes_get_bits(frame, 53, 1) != 0);
    out_cpr->lat_cpr = modes_get_bits(frame, 54, 17);
    out_cpr->lon_cpr = modes_get_bits(frame, 71, 17);
    out_cpr->surface = false;                 // airborne path only for the MVP
    out_cpr->rx_time_us = 0;                  // stamped by the caller at receipt

    return MODES_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TC 19 — Airborne velocity
 * ═══════════════════════════════════════════════════════════════════════════ */

modes_result_t modes_decode_velocity(const uint8_t *frame, adsb_velocity_t *out_vel)
{
    if (frame == NULL || out_vel == NULL) {
        return MODES_ERR_NULL_ARG;
    }

    const uint8_t tc = modes_typecode_raw(frame);
    if (tc != 19) {
        return MODES_ERR_UNSUPPORTED_TC;
    }

    // Zero the output so any sub-type we do not fully populate stays defined.
    memset(out_vel, 0, sizeof(*out_vel));

    // ── Sub-type selects ground-speed vs airspeed encoding. ──────────────────
    // The 3-bit sub-type is ME bits 6-8 (frame bits 37-39). 1/2 = ground speed
    // (subsonic / supersonic), 3/4 = airspeed+heading. We decode the common
    // ground-speed sub-types fully; airspeed sub-types still yield vertical rate.
    const uint32_t subtype = modes_get_bits(frame, 37, 3);

    // ── Vertical rate is common to all sub-types (ME bits 38-46 region). ─────
    // Sign bit at frame bit 68, 9-bit magnitude at frame bit 69. The encoded
    // value is in 64-ft/min units, offset by 1 (0 == no info).
    const uint32_t vr_sign = modes_get_bits(frame, 68, 1);
    const uint32_t vr_raw  = modes_get_bits(frame, 69, 9);
    if (vr_raw != 0) {
        int32_t vr = (int32_t)(vr_raw - 1) * 64;       // ft/min magnitude
        out_vel->vertical_rate_fpm = (int16_t)(vr_sign ? -vr : vr);
    }
    // Vertical-rate source bit (ME bit 36, frame bit 67): 0 = baro, 1 = GNSS.
    out_vel->gnss_baro_alt = (modes_get_bits(frame, 67, 1) != 0);

    if (subtype == 1 || subtype == 2) {
        // ── Ground-speed sub-types. ──────────────────────────────────────────
        // East/West and North/South velocity components, each a sign + 10-bit
        // magnitude. Supersonic (subtype 2) scales the magnitude by 4.
        const uint32_t ew_sign = modes_get_bits(frame, 45, 1);
        const uint32_t ew_raw  = modes_get_bits(frame, 46, 10);
        const uint32_t ns_sign = modes_get_bits(frame, 56, 1);
        const uint32_t ns_raw  = modes_get_bits(frame, 57, 10);

        // A raw magnitude of 0 means "no velocity information" for that axis.
        if (ew_raw == 0 || ns_raw == 0) {
            // No usable horizontal velocity; vertical rate (above) may still be
            // valid, but the contract's velocity fields stay zero.
            return MODES_OK;
        }

        // Decode to knots. Value is (raw-1), supersonic multiplies by 4.
        const int scale = (subtype == 2) ? 4 : 1;
        double vx = (double)((int32_t)ew_raw - 1) * scale;   // east(+)/west(-)
        double vy = (double)((int32_t)ns_raw - 1) * scale;   // north(+)/south(-)
        if (ew_sign) vx = -vx;     // sign bit 1 => westbound
        if (ns_sign) vy = -vy;     // sign bit 1 => southbound

        // Resultant ground speed and true track from the vector components.
        double speed = sqrt(vx * vx + vy * vy);
        out_vel->ground_speed_kt = (uint16_t)(speed + 0.5);

        // atan2 of (east, north) gives bearing clockwise from true north.
        double track = atan2(vx, vy) * (180.0 / MODES_PI);
        if (track < 0.0) track += 360.0;          // fold into [0,360)
        out_vel->track_deg = (uint16_t)(track + 0.5);
        if (out_vel->track_deg >= 360) out_vel->track_deg = 0;

        return MODES_OK;
    }

    // Sub-types 3/4 (airspeed + heading) are not fully decoded for the MVP, but
    // we have already populated the vertical rate, which is the field traffic
    // most needs from them. Report success so that data is not discarded.
    return MODES_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  CPR pairing cache + position resolution
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Find the cache slot for @p icao, or claim a free/LRU slot for it.
 *
 * Linear probe over the fixed table. A zero @c icao marks an empty slot; if the
 * table is full we evict the least-recently-used entry. No allocation — the
 * table was sized once at init.
 *
 * @param icao  Aircraft key (24-bit, non-zero).
 * @param now_us Current time, for stamping/LRU.
 * @return Pointer to the owning slot (always non-NULL).
 */
static cpr_slot_t *cpr_cache_get_slot(uint32_t icao, int64_t now_us)
{
    cpr_slot_t *lru = &g_modes.slots[0];

    // First pass: exact hit or remember the LRU candidate as we go.
    for (uint16_t i = 0; i < g_modes.n_slots; ++i) {
        cpr_slot_t *s = &g_modes.slots[i];
        if (s->icao == icao && icao != 0) {
            s->last_used_us = now_us;
            return s;                       // existing entry for this aircraft
        }
        if (s->icao == 0) {
            // Empty slot — claim it immediately for this aircraft.
            s->icao         = icao;
            s->valid_even   = false;
            s->valid_odd    = false;
            s->last_used_us = now_us;
            return s;
        }
        if (s->last_used_us < lru->last_used_us) {
            lru = s;                        // track the oldest in case we evict
        }
    }

    // Table full: evict the LRU slot and repurpose it for this aircraft.
    lru->icao         = icao;
    lru->valid_even   = false;
    lru->valid_odd    = false;
    lru->last_used_us = now_us;
    return lru;
}

modes_result_t modes_decode_position_cpr(uint32_t icao, const modes_cpr_frame_t *frame,
                                         int64_t rx_time_us, const ownship_ref_t *ref,
                                         double *out_lat, double *out_lon)
{
    if (frame == NULL || out_lat == NULL || out_lon == NULL) {
        return MODES_ERR_NULL_ARG;
    }
    if (!g_modes.inited) {
        return MODES_ERR_NULL_ARG;          // cache not allocated => cannot pair
    }

    // ── Stash this frame into its parity half of the per-ICAO slot. ──────────
    cpr_slot_t *slot = cpr_cache_get_slot(icao, rx_time_us);
    if (frame->odd) {
        slot->odd       = *frame;
        slot->odd.rx_time_us = rx_time_us;
        slot->valid_odd = true;
    } else {
        slot->even       = *frame;
        slot->even.rx_time_us = rx_time_us;
        slot->valid_even = true;
    }

    // ── Try a GLOBAL decode if we now hold a fresh opposite-parity pair. ─────
    if (slot->valid_even && slot->valid_odd) {
        // Reject the pair if the two halves are too far apart in time — an old
        // frame paired with a new one yields a wildly wrong position.
        const int64_t age =
            (slot->even.rx_time_us > slot->odd.rx_time_us)
                ? (slot->even.rx_time_us - slot->odd.rx_time_us)
                : (slot->odd.rx_time_us - slot->even.rx_time_us);

        if ((uint64_t)age <= g_modes.cfg.max_cpr_pair_age_us) {
            // latest_is_odd anchors longitude on whichever frame is newer.
            const bool latest_is_odd = (slot->odd.rx_time_us >= slot->even.rx_time_us);
            double lat = 0.0, lon = 0.0;
            if (cpr_global_decode(&slot->even, &slot->odd, latest_is_odd, &lat, &lon) == 0) {
                *out_lat = lat;
                *out_lon = lon;
                g_modes.stats.positions_global++;
                return MODES_OK;
            }
            // Math rejected the pair (NL mismatch / range): count and fall
            // through to a possible local decode below.
            g_modes.stats.cpr_rejects++;
        } else {
            // The partner is stale; keep the just-stored frame and report so.
            // (We do not clear the stale half — a newer opposite frame may yet
            //  arrive and pair with THIS one.)
            if (!(g_modes.cfg.enable_local_cpr && ref != NULL && ref->valid)) {
                return MODES_ERR_STALE_PAIR;
            }
        }
    }

    // ── Fall back to LOCAL decode against ownship, if permitted & available. ─
    if (g_modes.cfg.enable_local_cpr && ref != NULL && ref->valid) {
        double lat = 0.0, lon = 0.0;
        if (cpr_local_decode(frame, ref->lat_deg, ref->lon_deg, &lat, &lon) == 0) {
            *out_lat = lat;
            *out_lon = lon;
            g_modes.stats.positions_local++;
            return MODES_OK;
        }
        g_modes.stats.cpr_rejects++;
        return MODES_ERR_CPR_REJECT;
    }

    // Only one parity seen and no usable reference: we need the partner frame.
    return MODES_ERR_NO_POSITION;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Top-level entry — the only thing the Core-1 decode task calls
 * ═══════════════════════════════════════════════════════════════════════════ */

modes_result_t modes_decode_frame(const uint8_t *frame, size_t frame_len,
                                  int64_t rx_time_us, const ownship_ref_t *ref,
                                  adsb_msg_t *out_msg)
{
    if (frame == NULL || out_msg == NULL) {
        return MODES_ERR_NULL_ARG;
    }

    // Count every frame that reaches us, regardless of outcome.
    g_modes.stats.frames_in++;

    // ── Length gate. ─────────────────────────────────────────────────────────
    if (frame_len != MODES_SHORT_BYTES && frame_len != MODES_LONG_BYTES) {
        return MODES_ERR_BAD_LEN;
    }

    // ── DF gate (cheap, pre-CRC). Only DF17/DF18 proceed. ────────────────────
    const modes_df_t df = modes_decode_df(frame, frame_len);
    if (df != MODES_DF_ADSB && df != MODES_DF_TISB) {
        g_modes.stats.df_dropped++;
        return MODES_ERR_UNSUPPORTED_DF;
    }

    // ADS-B extended squitter is always a long (112-bit) frame; a short DF17/18
    // is malformed and we drop it rather than read past the data.
    if (frame_len != MODES_LONG_BYTES) {
        return MODES_ERR_BAD_LEN;
    }

    // ── CRC + ICAO recovery. ─────────────────────────────────────────────────
    uint32_t icao = 0;
    const modes_crc_t crc = modes_decode_check_crc(frame, frame_len, &icao);
    if (crc != MODES_CRC_OK) {
        g_modes.stats.crc_fail++;
        return MODES_ERR_BAD_CRC;
    }
    g_modes.stats.crc_ok++;

    // ── Initialise the output record. ────────────────────────────────────────
    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->icao            = icao & 0x00FFFFFFu;
    out_msg->rx_time_us      = rx_time_us;
    out_msg->downlink_format = (uint8_t)((df == MODES_DF_ADSB) ? 17 : 18);
    out_msg->type_code       = modes_typecode_raw(frame);
    out_msg->signal_level    = -1;            // unknown here; sinks may fill later

    // ── Dispatch on the high-level message kind. ─────────────────────────────
    const modes_msg_kind_t kind = modes_decode_typecode(frame);
    switch (kind) {

    case MODES_MSG_IDENT: {
        // Identification: callsign + emitter category.
        char cs[ADSB_CALLSIGN_LEN];
        adsb_emitter_category_t cat = ADSB_CAT_NO_INFO;
        if (modes_decode_identification(frame, cs, &cat) == MODES_OK) {
            memcpy(out_msg->callsign, cs, sizeof(out_msg->callsign));
            out_msg->has_callsign     = (cs[0] != '\0');
            out_msg->has_category     = true;
            out_msg->emitter_category = cat;
        }
        return MODES_OK;
    }

    case MODES_MSG_AIRBORNE_POS: {
        // Airborne position: altitude (always) + CPR-resolved lat/lon (maybe).
        modes_cpr_frame_t cpr;
        int32_t alt_ft = 0;
        bool    alt_ok = false;
        const modes_result_t pr =
            modes_decode_airborne_position(frame, &cpr, &alt_ft, &alt_ok);
        if (pr != MODES_OK) {
            return pr;
        }

        // Surface the altitude regardless of whether position resolves.
        if (alt_ok) {
            out_msg->has_altitude          = true;
            out_msg->altitude_ft           = alt_ft;
            // TC 20-22 carry GNSS/geometric altitude; 9-18 are barometric.
            out_msg->altitude_is_geometric = (out_msg->type_code >= 20 &&
                                              out_msg->type_code <= 22);
        }

        // Attempt CPR resolution through the pairing cache (+ optional ownship).
        cpr.rx_time_us = rx_time_us;
        double lat = 0.0, lon = 0.0;
        const modes_result_t cr =
            modes_decode_position_cpr(out_msg->icao, &cpr, rx_time_us, ref, &lat, &lon);
        if (cr == MODES_OK) {
            out_msg->has_position = true;
            out_msg->lat_deg      = lat;
            out_msg->lon_deg      = lon;
            out_msg->on_ground    = false;
        }
        // Even without a resolved position, the altitude (and the fact that we
        // heard this aircraft) is useful, so we always return MODES_OK here.
        return MODES_OK;
    }

    case MODES_MSG_AIRBORNE_VEL: {
        // Velocity: ground speed / track / vertical rate.
        adsb_velocity_t vel;
        if (modes_decode_velocity(frame, &vel) == MODES_OK) {
            // Ground speed/track are only meaningful when speed was decoded.
            if (vel.ground_speed_kt != 0 || vel.track_deg != 0) {
                out_msg->has_velocity    = true;
                out_msg->ground_speed_kt = vel.ground_speed_kt;
                out_msg->track_deg       = vel.track_deg;
            }
            // Vertical rate is independent of horizontal velocity.
            if (vel.vertical_rate_fpm != 0) {
                out_msg->has_vertical_rate = true;
                out_msg->vertical_rate_fpm = vel.vertical_rate_fpm;
            }
        }
        return MODES_OK;
    }

    case MODES_MSG_SURFACE_POS:
    case MODES_MSG_OPSTATUS:
        // Recognised but not decoded in the MVP; the ICAO/DF tagging above is
        // still valid output for the traffic table to register the contact.
        return MODES_ERR_UNSUPPORTED_TC;

    case MODES_MSG_UNKNOWN:
    default:
        return MODES_ERR_UNSUPPORTED_TC;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Diagnostics
 * ═══════════════════════════════════════════════════════════════════════════ */

const char *modes_result_str(modes_result_t result)
{
    // Static strings; safe to return from any context (sink_debug calls this).
    switch (result) {
        case MODES_OK:                  return "MODES_OK";
        case MODES_ERR_BAD_LEN:         return "MODES_ERR_BAD_LEN";
        case MODES_ERR_BAD_CRC:         return "MODES_ERR_BAD_CRC";
        case MODES_ERR_UNSUPPORTED_DF:  return "MODES_ERR_UNSUPPORTED_DF";
        case MODES_ERR_UNSUPPORTED_TC:  return "MODES_ERR_UNSUPPORTED_TC";
        case MODES_ERR_NO_POSITION:     return "MODES_ERR_NO_POSITION";
        case MODES_ERR_CPR_REJECT:      return "MODES_ERR_CPR_REJECT";
        case MODES_ERR_STALE_PAIR:      return "MODES_ERR_STALE_PAIR";
        case MODES_ERR_NULL_ARG:        return "MODES_ERR_NULL_ARG";
        default:                        return "MODES_ERR_UNKNOWN";
    }
}

void modes_decode_get_stats(modes_decode_stats_t *out_stats)
{
    if (out_stats == NULL) {
        return;
    }
    // Word-by-word copy of the counter block. Each field is a single 32-bit
    // load/store, so a status-task read sees individually coherent values even
    // without a lock — exactly the "atomic-ish snapshot" the header documents.
    *out_stats = g_modes.stats;
}
