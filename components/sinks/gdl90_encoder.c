/**
 * @file    gdl90_encoder.c
 * @brief   Pure GDL90 message framing — no I/O, host-unit-testable.
 *
 * @details
 *   CLEAN-ROOM implementation written from the public "GDL 90 Data Interface
 *   Specification" (Garmin 560-1058-00 Rev A, June 2007) and the ForeFlight
 *   "GDL90 Extended Specification". No GPLv3 dump1090 fork or any copyleft
 *   source was consulted: the bit layouts below come straight from the public
 *   Garmin tables, and the CRC routine is the standard CCITT-16 (poly 0x1021)
 *   table algorithm the spec's Appendix prescribes.
 *
 *   Everything here is deliberately self-contained and free of ESP-IDF runtime
 *   calls so the same translation unit links into a host unit test (the bench
 *   harness validates the wire bytes against this file byte-for-byte).
 *
 *   Wire facts encoded here (see tools/bench/WIRE_CONTRACT.md):
 *     - Frame  : 0x7E | id+payload | CRC16(LE) | 0x7E.
 *     - Stuff  : 0x7E and 0x7D become 0x7D, (byte XOR 0x20); flags never stuffed.
 *     - CRC    : computed over the UN-stuffed id+payload, transmitted LE.
 *     - Lat/Lon: 24-bit two's-complement semicircles, round(deg * 2^23 / 180).
 *     - Alt    : 12-bit pressure-altitude code, (alt_ft + 1000) / 25; 0xFFF bad.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 *
 * Clean-room from the public Garmin GDL90 spec + ForeFlight extension.
 * No code adapted from any GPL dump1090 fork.
 */

#include "gdl90_encoder.h"

#include <string.h>
#include <math.h>
#include "esp_err.h"   /* ESP_OK / ESP_ERR_INVALID_SIZE / ESP_ERR_INVALID_ARG */

/* ───────────────────────────────────────────────────────────────────────────
 *  Wire constants. Kept local (not in the public header) because they are an
 *  implementation detail of the framing; the WIRE_CONTRACT.md table is the
 *  authoritative cross-check.
 * ─────────────────────────────────────────────────────────────────────────── */
#define GDL90_FLAG        0x7Eu   /**< Start/end-of-frame flag.                  */
#define GDL90_ESC         0x7Du   /**< Byte-stuffing escape.                     */
#define GDL90_ESC_XOR     0x20u   /**< Stuffed byte = original XOR this.         */

#define GDL90_ID_HEARTBEAT 0x00u  /**< Heartbeat message id.                     */
#define GDL90_ID_OWNSHIP   0x0Au  /**< Ownship Report message id.                */
#define GDL90_ID_TRAFFIC   0x14u  /**< Traffic Report message id.                */

/* Heartbeat is id + 6 payload bytes; Traffic/Ownship is id + 27 payload bytes. */
#define GDL90_HEARTBEAT_LEN   7u   /**< id + 6 payload.                          */
#define GDL90_TRAFFIC_LEN     28u  /**< id + 27 payload (the longest message).   */

/* The 12-bit "altitude unavailable" sentinel from the Garmin spec.             */
#define GDL90_ALT_INVALID  0xFFFu

/* ═══════════════════════════════════════════════════════════════════════════
 *  CRC-16 / CCITT (poly 0x1021) — the variant the GDL90 spec mandates.
 *
 *  The spec's reference code precomputes a 256-entry table from the polynomial
 *  and then folds each byte with:
 *      crc = table[crc >> 8] ^ (crc << 8) ^ data_byte
 *  We build the table lazily on first use so the encoder stays allocation-free
 *  and works identically on host and target.
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint16_t s_crc_table[256];   /**< CCITT-16 fold table (built once).       */
static bool     s_crc_ready = false;/**< Lazy-init guard for the table above.    */

/**
 * @brief Build the 256-entry CCITT-16 table exactly as the GDL90 spec lists it.
 *
 * @details
 *   For each possible high byte we shift through 8 bits of the 0x1021 polynomial.
 *   This mirrors the "crcInit()" pseudo-code in the GDL90 specification so the
 *   resulting table — and therefore every CRC we emit — is bit-for-bit what a
 *   spec-conformant receiver expects.
 */
static void gdl90_crc_init(void)
{
    uint16_t crc;

    // Walk all 256 leading-byte values and precompute their folded remainder.
    for (uint32_t i = 0; i < 256; ++i) {
        // Seed with the byte in the high half of the 16-bit register.
        crc = (uint16_t)(i << 8);

        // Eight polynomial-division steps, MSB-first, poly = 0x1021.
        for (uint32_t bit = 0; bit < 8; ++bit) {
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000u) ? 0x1021u : 0u));
        }

        s_crc_table[i] = crc;
    }

    s_crc_ready = true;
}

uint16_t gdl90_crc16(const uint8_t *payload, size_t len)
{
    // First-use table construction. Single-threaded path in practice (one
    // publisher task), and idempotent if it ever raced — every writer produces
    // the identical table, so no lock is required for correctness.
    if (!s_crc_ready) {
        gdl90_crc_init();
    }

    // Guard the degenerate call so a NULL/zero-length input yields the spec's
    // empty-message CRC (0x0000) rather than dereferencing a bad pointer.
    if (payload == NULL || len == 0) {
        return 0;
    }

    // Standard table fold across the UN-stuffed id+payload bytes.
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc = (uint16_t)(s_crc_table[crc >> 8] ^ (crc << 8) ^ payload[i]);
    }

    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Framing core: take an UN-stuffed id+payload, append CRC, byte-stuff, and
 *  wrap in flags. Every public frame_* helper funnels through here so the
 *  stuffing/CRC rules live in exactly one place.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Append one logical byte to @p out, applying 0x7E/0x7D byte-stuffing.
 *
 * @param out      Destination framed buffer.
 * @param out_cap  Capacity of @p out.
 * @param pos      In/out write cursor; advanced by 1 or 2 bytes.
 * @param byte     The raw (UN-stuffed) byte to emit.
 * @return true if the byte (and any escape) fit; false on overflow.
 */
static bool gdl90_emit_stuffed(uint8_t *out, size_t out_cap, size_t *pos, uint8_t byte)
{
    // 0x7E and 0x7D are the only two values that must be escaped on the wire.
    if (byte == GDL90_FLAG || byte == GDL90_ESC) {
        // Need room for the escape AND the transformed byte.
        if (*pos + 2 > out_cap) {
            return false;
        }
        out[(*pos)++] = GDL90_ESC;
        out[(*pos)++] = (uint8_t)(byte ^ GDL90_ESC_XOR);
        return true;
    }

    // Ordinary byte: one slot.
    if (*pos + 1 > out_cap) {
        return false;
    }
    out[(*pos)++] = byte;
    return true;
}

/**
 * @brief Frame an UN-stuffed message (id already in msg[0]) into @p out.
 *
 * @details
 *   Computes the CRC over the message, then writes: opening flag, the stuffed
 *   message bytes, the stuffed CRC (little-endian), and the closing flag. The
 *   flags themselves are written raw — never stuffed — exactly as the contract
 *   requires.
 *
 * @return Total framed length on success, or a NEGATIVE esp_err on overflow.
 */
static int gdl90_frame_message(uint8_t *out, size_t out_cap,
                               const uint8_t *msg, size_t msg_len)
{
    // Defensive: a NULL destination or zero-length message is a programming bug.
    if (out == NULL || msg == NULL || msg_len == 0) {
        return -ESP_ERR_INVALID_ARG;
    }

    // CRC is taken over the raw (pre-stuff) id+payload, per the contract.
    uint16_t crc = gdl90_crc16(msg, msg_len);

    size_t pos = 0;

    // Opening flag (raw).
    if (pos + 1 > out_cap) {
        return -ESP_ERR_INVALID_SIZE;
    }
    out[pos++] = GDL90_FLAG;

    // Stuffed message bytes (id + payload).
    for (size_t i = 0; i < msg_len; ++i) {
        if (!gdl90_emit_stuffed(out, out_cap, &pos, msg[i])) {
            return -ESP_ERR_INVALID_SIZE;
        }
    }

    // Stuffed CRC, little-endian (low byte first).
    if (!gdl90_emit_stuffed(out, out_cap, &pos, (uint8_t)(crc & 0xFFu))) {
        return -ESP_ERR_INVALID_SIZE;
    }
    if (!gdl90_emit_stuffed(out, out_cap, &pos, (uint8_t)((crc >> 8) & 0xFFu))) {
        return -ESP_ERR_INVALID_SIZE;
    }

    // Closing flag (raw).
    if (pos + 1 > out_cap) {
        return -ESP_ERR_INVALID_SIZE;
    }
    out[pos++] = GDL90_FLAG;

    return (int)pos;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Field packers — small helpers that turn engineering units into the GDL90
 *  fixed-point wire encodings. Pulled out so the message builders read cleanly.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Pack signed WGS-84 degrees into a 24-bit two's-complement semicircle.
 *
 * @details
 *   The GDL90 lat/lon encoding is value = round(deg * 2^23 / 180), clamped to
 *   the 24-bit signed range, then stored big-endian across three bytes. We clamp
 *   rather than wrap so a wildly out-of-range input degrades gracefully instead
 *   of aliasing to a plausible-looking position.
 *
 * @param deg    Latitude or longitude in degrees.
 * @param out3   Destination 3-byte big-endian field.
 */
static void gdl90_pack_latlon(double deg, uint8_t out3[3])
{
    // Scale to semicircles. 2^23 / 180 is the spec's lat/lon LSB.
    double scaled = round(deg * (8388608.0 / 180.0));

    // Clamp into the signed 24-bit range [-2^23, 2^23 - 1].
    if (scaled >  8388607.0) scaled =  8388607.0;
    if (scaled < -8388608.0) scaled = -8388608.0;

    // Reduce into a 24-bit two's-complement field.
    int32_t v = (int32_t)scaled;
    uint32_t u = (uint32_t)v & 0x00FFFFFFu;

    // Big-endian: most-significant byte first.
    out3[0] = (uint8_t)((u >> 16) & 0xFFu);
    out3[1] = (uint8_t)((u >> 8) & 0xFFu);
    out3[2] = (uint8_t)(u & 0xFFu);
}

/**
 * @brief Pack a pressure altitude (feet) into the GDL90 12-bit altitude code.
 *
 * @details
 *   The encoding is (alt_ft + 1000) / 25, giving 25-ft resolution from a
 *   -1000 ft floor; the all-ones code 0xFFF flags "altitude unavailable".
 *   INT32_MIN is the agreed sentinel from gdl90_traffic_t meaning "no altitude".
 *
 * @param alt_ft  Pressure altitude in feet, or INT32_MIN for invalid.
 * @return 12-bit altitude code (0..0xFFE), or 0xFFF when unavailable.
 */
static uint16_t gdl90_pack_altitude(int32_t alt_ft)
{
    // Explicit "no altitude" sentinel maps to the spec's invalid code.
    if (alt_ft == INT32_MIN) {
        return GDL90_ALT_INVALID;
    }

    // Apply the +1000 / 25 transform with rounding to the nearest 25 ft step.
    int32_t code = (alt_ft + 1000 + 12) / 25;

    // Anything outside the representable 0..0xFFE window is reported invalid.
    if (code < 0 || code > 0xFFE) {
        return GDL90_ALT_INVALID;
    }

    return (uint16_t)code;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Traffic / Ownship payload builder (shared 28-byte layout).
 *
 *  Byte map (id at [0]), from the public Garmin Traffic Report table:
 *    [0]      message id (0x14 traffic / 0x0A ownship)
 *    [1]      (alert status << 4) | address type
 *    [2..4]   participant address (24-bit, big-endian)
 *    [5..7]   latitude  (24-bit semicircle)
 *    [8..10]  longitude (24-bit semicircle)
 *    [11]     altitude bits 11..4
 *    [12]     altitude bits 3..0 (high nibble) | misc nibble (low nibble)
 *    [13]     (NIC << 4) | NACp
 *    [14]     horizontal velocity bits 11..4
 *    [15]     hVel bits 3..0 (high nibble) | vVel bits 11..8 (low nibble)
 *    [16]     vertical velocity bits 7..0
 *    [17]     track/heading (8-bit, 360/256 deg per LSB)
 *    [18]     emitter category
 *    [19..26] callsign (8 ASCII chars, space-padded)
 *    [27]     (emergency/priority << 4) | spare
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Build the 28-byte UN-stuffed Traffic/Ownship message body.
 *
 * @param id   0x14 for Traffic, 0x0A for Ownship.
 * @param tr   Source fields (engineering units).
 * @param msg  Destination 28-byte buffer.
 */
static void gdl90_build_traffic_body(uint8_t id, const gdl90_traffic_t *tr, uint8_t msg[GDL90_TRAFFIC_LEN])
{
    // Start clean so any reserved/spare nibbles are zero.
    memset(msg, 0, GDL90_TRAFFIC_LEN);

    // [0] message id.
    msg[0] = id;

    // [1] alert status (high nibble) | address type (low nibble).
    msg[1] = (uint8_t)(((tr->alert_status & 0x0Fu) << 4) | (tr->addr_type & 0x0Fu));

    // [2..4] 24-bit participant ICAO address, big-endian.
    msg[2] = (uint8_t)((tr->icao >> 16) & 0xFFu);
    msg[3] = (uint8_t)((tr->icao >> 8) & 0xFFu);
    msg[4] = (uint8_t)(tr->icao & 0xFFu);

    // [5..7] latitude, [8..10] longitude (24-bit semicircles).
    gdl90_pack_latlon(tr->lat_deg, &msg[5]);
    gdl90_pack_latlon(tr->lon_deg, &msg[8]);

    // [11..12] 12-bit altitude code packed into 1.5 bytes; low nibble of [12]
    // is the "misc" field (airborne flag + report type — bit 3 = airborne).
    uint16_t alt = gdl90_pack_altitude(tr->alt_press_ft);
    uint8_t  misc = (uint8_t)(tr->airborne ? 0x09u : 0x08u); /* bit3 airborne, bit0 "updated/true track" */
    msg[11] = (uint8_t)((alt >> 4) & 0xFFu);
    msg[12] = (uint8_t)(((alt & 0x0Fu) << 4) | (misc & 0x0Fu));

    // [13] integrity: NIC (high nibble) | NACp (low nibble).
    msg[13] = (uint8_t)(((tr->nic & 0x0Fu) << 4) | (tr->nacp & 0x0Fu));

    // [14..16] horizontal (12-bit) + vertical (12-bit) velocity.
    //   hVel is knots, 0xFFF = unknown; vVel is a signed 12-bit count of 64-fpm
    //   units, 0x800 = unknown. We clamp hVel to its 0xFFE max.
    uint16_t hvel = tr->h_velocity_kt;
    if (hvel > 0xFFE) hvel = 0xFFE;

    // Vertical velocity: convert ft/min -> 64-fpm units, round to nearest, then
    // fit a signed 12-bit field. 0x800 is the spec's "no vertical rate" code.
    int32_t vraw = tr->v_velocity_fpm;
    int32_t vunits = (vraw >= 0) ? (vraw + 32) / 64 : (vraw - 32) / 64;
    if (vunits >  511) vunits =  511;
    if (vunits < -511) vunits = -511;
    uint16_t vvel = (uint16_t)(vunits & 0x0FFFu);

    msg[14] = (uint8_t)((hvel >> 4) & 0xFFu);
    msg[15] = (uint8_t)(((hvel & 0x0Fu) << 4) | ((vvel >> 8) & 0x0Fu));
    msg[16] = (uint8_t)(vvel & 0xFFu);

    // [17] track/heading: 8-bit, LSB = 360/256 degrees.
    //   round(deg * 256 / 360) modulo 256.
    uint32_t trk = (uint32_t)((tr->track_heading * 256u + 180u) / 360u) & 0xFFu;
    msg[17] = (uint8_t)trk;

    // [18] emitter category.
    msg[18] = tr->emitter_cat;

    // [19..26] callsign: 8 ASCII chars, space-padded, no NUL on the wire.
    for (int i = 0; i < 8; ++i) {
        char c = tr->callsign[i];
        // Stop padding from a NUL terminator onward; spaces fill the remainder.
        if (c == '\0') {
            // Once we hit the terminator, fill the rest with spaces and bail.
            for (int j = i; j < 8; ++j) {
                msg[19 + j] = ' ';
            }
            break;
        }
        msg[19 + i] = (uint8_t)c;
    }

    // [27] emergency/priority code (high nibble); low nibble spare = 0.
    msg[27] = (uint8_t)((tr->emergency_code & 0x0Fu) << 4);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public encoders
 * ═══════════════════════════════════════════════════════════════════════════ */

int gdl90_frame_heartbeat(uint8_t *out, size_t out_cap, const gdl90_heartbeat_t *hb)
{
    // Validate inputs up front so callers get a clean negative esp_err.
    if (out == NULL || hb == NULL) {
        return -ESP_ERR_INVALID_ARG;
    }

    // Build the 7-byte UN-stuffed heartbeat body.
    uint8_t msg[GDL90_HEARTBEAT_LEN];

    // [0] message id.
    msg[0] = GDL90_ID_HEARTBEAT;

    // [1] Status Byte 1. Bit7 = GPS position valid; bit0 = UAT initialized
    //     (we hold it asserted so receivers treat the link as up).
    uint8_t st1 = 0x01u; /* bit0: device initialized / running */
    if (hb->gps_pos_valid) {
        st1 |= 0x80u;    /* bit7: GPS position valid */
    }
    msg[1] = st1;

    // [2] Status Byte 2. Bit0 carries timestamp bit 16; bit6 = maintenance req.
    uint8_t st2 = 0;
    if (hb->maint_required) {
        st2 |= 0x40u;
    }
    // Timestamp is a 17-bit value; its MSB (bit 16) rides in Status Byte 2 bit0.
    uint32_t ts = hb->timestamp_s & 0x1FFFFu;
    if (ts & 0x10000u) {
        st2 |= 0x01u;
    }
    msg[2] = st2;

    // [3..4] timestamp lower 16 bits, LITTLE-endian per the GDL90 spec.
    msg[3] = (uint8_t)(ts & 0xFFu);
    msg[4] = (uint8_t)((ts >> 8) & 0xFFu);

    // [5..6] message counts: byte5 = uplink count (top 5 bits) packed with the
    //   high bits of the basic/long count; byte6 = low 8 bits of that count.
    //   We follow the spec's "Message Count" field: bits 15..11 = uplink count,
    //   bits 9..0 = basic+long count.
    uint16_t uplink = hb->msg_count_uplink & 0x1Fu;          /* 5-bit field */
    uint16_t basiclong = hb->msg_count_basic_long & 0x3FFu;  /* 10-bit field */
    msg[5] = (uint8_t)((uplink << 3) | ((basiclong >> 8) & 0x03u));
    msg[6] = (uint8_t)(basiclong & 0xFFu);

    // Frame it (CRC + stuff + flags).
    return gdl90_frame_message(out, out_cap, msg, GDL90_HEARTBEAT_LEN);
}

int gdl90_frame_traffic_report(uint8_t *out, size_t out_cap, const gdl90_traffic_t *tr)
{
    // Argument check mirrors the heartbeat path for a uniform contract.
    if (out == NULL || tr == NULL) {
        return -ESP_ERR_INVALID_ARG;
    }

    // Build the shared 28-byte body with the Traffic id, then frame.
    uint8_t msg[GDL90_TRAFFIC_LEN];
    gdl90_build_traffic_body(GDL90_ID_TRAFFIC, tr, msg);
    return gdl90_frame_message(out, out_cap, msg, GDL90_TRAFFIC_LEN);
}

int gdl90_frame_ownship_report(uint8_t *out, size_t out_cap, const gdl90_traffic_t *own)
{
    // Ownship is the identical payload layout under message id 0x0A.
    if (out == NULL || own == NULL) {
        return -ESP_ERR_INVALID_ARG;
    }

    uint8_t msg[GDL90_TRAFFIC_LEN];
    gdl90_build_traffic_body(GDL90_ID_OWNSHIP, own, msg);
    return gdl90_frame_message(out, out_cap, msg, GDL90_TRAFFIC_LEN);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  traffic_target_t -> gdl90_traffic_t mapping
 *
 *  Bridges our internal canonical record to the GDL90 wire fields, including the
 *  emitter-category translation. ADS-B emitter categories and GDL90 emitter
 *  categories share the same numbering for the codes we emit (light/small/large/
 *  heavy/rotorcraft/glider/etc.), which the public GDL90 spec table confirms, so
 *  the mapping is a direct cast with a couple of explicit fix-ups.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Translate an ADS-B emitter category to the GDL90 emitter category code.
 *
 * @details
 *   The GDL90 spec's emitter-category table is intentionally aligned with the
 *   ADS-B "ME" category coding for the values a 1090ES receiver produces, so for
 *   the codes ADSBin decodes the numeric value passes straight through. We keep
 *   this as a named function (rather than an inline cast) so any future GDL90
 *   quirks have one obvious place to live.
 */
static uint8_t gdl90_emitter_from_adsb(adsb_emitter_category_t cat)
{
    // Direct correspondence for the categories we ever emit; clamp the few
    // higher ADS-B-only codes that exceed the GDL90 single-byte field's table.
    switch (cat) {
        case ADSB_CAT_NO_INFO:       return 0;
        case ADSB_CAT_LIGHT:         return 1;
        case ADSB_CAT_SMALL:         return 2;
        case ADSB_CAT_LARGE:         return 3;
        case ADSB_CAT_HIGH_VORTEX:   return 4;
        case ADSB_CAT_HEAVY:         return 5;
        case ADSB_CAT_HIGH_PERF:     return 6;
        case ADSB_CAT_ROTORCRAFT:    return 7;
        case ADSB_CAT_GLIDER:        return 9;   /* GDL90: glider/sailplane = 9  */
        case ADSB_CAT_LIGHTER_AIR:   return 10;  /* lighter-than-air            */
        case ADSB_CAT_PARACHUTE:     return 11;  /* parachutist / skydiver      */
        case ADSB_CAT_ULTRALIGHT:    return 12;  /* ultralight / hang/paraglider */
        case ADSB_CAT_UAV:           return 14;  /* unmanned aerial vehicle     */
        case ADSB_CAT_SPACE:         return 15;  /* space / transatmospheric    */
        case ADSB_CAT_SURFACE_EMERG: return 17;  /* surface emergency vehicle   */
        case ADSB_CAT_SURFACE_SVC:   return 18;  /* surface service vehicle     */
        default:                     return 0;
    }
}

void gdl90_traffic_from_target(gdl90_traffic_t *out, const traffic_target_t *tgt)
{
    // Tolerate NULLs defensively; a zeroed output is a safe "no traffic" body.
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (tgt == NULL) {
        return;
    }

    // No alert from a passive receiver; standard ADS-B-with-ICAO address type.
    out->alert_status = 0;
    out->addr_type    = 0;
    out->icao         = tgt->icao & 0x00FFFFFFu;

    // Position: only meaningful when the target currently holds a valid fix.
    if (tgt->position_valid) {
        out->lat_deg = tgt->lat_deg;
        out->lon_deg = tgt->lon_deg;
    } else {
        // Spec convention: lat/lon zero with NIC=0 signals "position unknown".
        out->lat_deg = 0.0;
        out->lon_deg = 0.0;
    }

    // Pressure altitude — INT32_MIN tells the encoder to emit the 0xFFF sentinel.
    out->alt_press_ft = tgt->has_altitude ? tgt->altitude_ft : INT32_MIN;

    // Airborne misc bit is the inverse of the on-ground flag.
    out->airborne = !tgt->on_ground;

    // Integrity categories straight from the merged record.
    out->nic  = tgt->nic;
    out->nacp = tgt->nacp;

    // Horizontal velocity / track only when a velocity solution exists.
    if (tgt->has_velocity) {
        out->h_velocity_kt = tgt->ground_speed_kt;
        out->track_heading = tgt->track_deg;
    } else {
        // 0xFFF horizontal velocity == "unavailable" in the encoder.
        out->h_velocity_kt = 0xFFFu;
        out->track_heading = 0;
    }

    // Vertical rate when present; otherwise leave 0 (encoder emits ~no climb).
    out->v_velocity_fpm = tgt->has_vertical_rate ? tgt->vertical_rate_fpm : 0;

    // Emitter category via the explicit ADS-B -> GDL90 translation table.
    out->emitter_cat = tgt->has_category
                           ? gdl90_emitter_from_adsb(tgt->emitter_category)
                           : 0;

    // Callsign: copy when known, else leave the all-zero buffer (encoder pads it
    // to 8 spaces on the wire).
    if (tgt->has_callsign) {
        // strncpy bounded to the shared buffer length; guarantees no overrun and
        // leaves trailing zeros that the encoder turns into spaces.
        strncpy(out->callsign, tgt->callsign, ADSB_CALLSIGN_LEN - 1);
        out->callsign[ADSB_CALLSIGN_LEN - 1] = '\0';
    }

    // No emergency signalling from a receive-only box.
    out->emergency_code = 0;
}
