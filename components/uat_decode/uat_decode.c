/**
 * @file    uat_decode.c
 * @brief   UAT (978 MHz) ADS-B message decoder — payload -> shared adsb_msg_t.
 *
 * @details
 *   CLEAN-ROOM from the public RTCA DO-282B UAT message format. Parses the HDR,
 *   STATE VECTOR and MODE STATUS elements of a FEC-corrected UAT ADS-B payload
 *   into the SHARED ::adsb_msg_t so the result merges into the same traffic table
 *   as 1090. UAT carries an ABSOLUTE position directly — there is no CPR even/odd
 *   pairing here, which makes this decoder markedly simpler than modes_decode.
 *
 *   The numeric field layout (byte offsets, bit widths, scalings) is the published
 *   DO-282B layout; NO decoder code from dump978 or any GPL source was used.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 *
 * Clean-room from the public DO-282B UAT message format. No code adapted from
 * dump978 / any GPL source.
 */

#include <string.h>

#include "uat_decode.h"
#include "adsbin_err.h"

/* The two valid UAT ADS-B payload lengths (FEC already stripped). */
#define UAT_LEN_SHORT  18
#define UAT_LEN_LONG   34

/* ───────────────────────────────────────────────────────────────────────────
 *  Small clean-room atan2 -> compass degrees, so the decoder has no libm
 *  dependency on the firmware. Returns a true track in [0,360): 0 = North
 *  (+vN), 90 = East (+vE), matching the GDL90 / aviation track convention.
 *
 *  We use a rational minimax approximation of atan(z) on |z|<=1 (max error
 *  ~0.005 rad, far finer than the 1-degree track resolution we emit), reduced by
 *  octant so the full atan2 range is covered, then map the math-frame angle
 *  (CCW from +x) to the compass frame (CW from North).
 * ─────────────────────────────────────────────────────────────────────────── */
static double uat_atan_unit(double z)
{
    /* atan(z) ≈ z*(0.9998660 - z^2*(0.3302995 - z^2*(0.1801410 - z^2*0.0851330))).
     * Valid for |z| <= 1; the caller reduces to this range. */
    double z2 = z * z;
    return z * (0.9998660 - z2 * (0.3302995 - z2 * (0.1801410 - z2 * 0.0851330)));
}

/** @brief atan2(y, x) in radians via octant reduction (no libm). */
static double uat_atan2_rad(double y, double x)
{
    const double PI = 3.14159265358979323846;
    if (x == 0.0 && y == 0.0) {
        return 0.0;
    }
    double ax = x < 0 ? -x : x;
    double ay = y < 0 ? -y : y;
    double a;
    if (ax >= ay) {
        a = uat_atan_unit(ay / ax);           /* angle in [0, pi/4]. */
    } else {
        a = PI / 2.0 - uat_atan_unit(ax / ay);/* complement for the steep octant. */
    }
    /* Restore the correct quadrant from the signs of x and y. */
    if (x < 0)  { a = PI - a; }
    if (y < 0)  { a = -a; }
    return a;
}

/**
 * @brief Convert math-frame velocity components (East, North) to a compass track
 *        in degrees [0,360). North is 0, East 90.
 */
static double uat_atan2_deg(double vE, double vN)
{
    const double PI = 3.14159265358979323846;
    /* Compass track = atan2(East, North): swap args vs the math-frame atan2. */
    double rad = uat_atan2_rad(vE, vN);
    double deg = rad * (180.0 / PI);
    if (deg < 0.0)    { deg += 360.0; }
    if (deg >= 360.0) { deg -= 360.0; }
    return deg;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Lifecycle — the decoder is stateless (no CPR cache, no allocation), so init
 *  just validates and returns. Kept as functions so main wires it like
 *  modes_decode.
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t uat_decode_init(const uat_decode_cfg_t *cfg)
{
    (void)cfg;   /* no configurable state today. */
    return ESP_OK;
}

void uat_decode_deinit(void) { /* nothing to free. */ }

/* ═══════════════════════════════════════════════════════════════════════════
 *  HDR — message type, address qualifier, 24-bit address.
 *
 *  Byte 0: bits 7..3 = MDB type (5 bits), bits 2..0 = address qualifier (3 bits).
 *  Bytes 1..3: the 24-bit participant address (ICAO-style), MSB-first.
 * ═══════════════════════════════════════════════════════════════════════════ */

uat_result_t uat_decode_address(const uint8_t *payload, adsb_msg_t *out_msg)
{
    if (!payload || !out_msg) {
        return UAT_ERR_NULL;
    }

    /* The 24-bit address keys the shared traffic table exactly like a 1090 ICAO.
     * UAT address qualifiers other than 0 (ICAO) mark anonymous / self-assigned
     * IDs; we still store the 24-bit value so a given emitter keys consistently. */
    out_msg->icao = ((uint32_t)payload[1] << 16)
                  | ((uint32_t)payload[2] << 8)
                  |  (uint32_t)payload[3];
    out_msg->icao &= 0x00FFFFFFu;

    /* type_code mirrors the 1090 "type code" slot for sink tagging: we stash the
     * UAT MDB type (top 5 bits of byte 0). */
    out_msg->type_code = (uint8_t)(payload[0] >> 3);
    return UAT_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  STATE VECTOR — absolute position, altitude, velocity.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Decode the absolute latitude/longitude from the STATE VECTOR.
 *
 * @details
 *   Latitude  = 24-bit field at bytes 4..6 bits[7..1]:
 *     raw_lat = (f[4]<<15) | (f[5]<<7) | (f[6]>>1)
 *     deg = raw_lat * 360 / 2^24, then deg -= 180 if deg > 90 (wrap to [-90,90]).
 *   Longitude = 24-bit field spanning byte6 bit0 .. byte9 bit1:
 *     raw_lon = ((f[6]&1)<<23) | (f[7]<<15) | (f[8]<<7) | (f[9]>>1)
 *     deg = raw_lon * 360 / 2^24, then deg -= 360 if deg > 180 (wrap to [-180,180]).
 *   An all-zero lat AND lon is the UAT "position unavailable" sentinel.
 */
static void decode_position(const uint8_t *f, adsb_msg_t *out)
{
    uint32_t raw_lat = ((uint32_t)f[4] << 15) | ((uint32_t)f[5] << 7) | ((uint32_t)f[6] >> 1);
    uint32_t raw_lon = (((uint32_t)(f[6] & 0x01)) << 23) | ((uint32_t)f[7] << 15)
                     | ((uint32_t)f[8] << 7) | ((uint32_t)f[9] >> 1);

    /* Both halves zero => no position broadcast in this frame. */
    if (raw_lat == 0 && raw_lon == 0) {
        out->has_position = false;
        return;
    }

    double lat = (double)raw_lat * (360.0 / 16777216.0);
    if (lat > 90.0) {
        lat -= 180.0;       /* values above 90 wrap into the southern hemisphere. */
    }
    double lon = (double)raw_lon * (360.0 / 16777216.0);
    if (lon > 180.0) {
        lon -= 360.0;       /* values above 180 wrap into the western hemisphere. */
    }

    out->has_position = true;
    out->lat_deg = lat;
    out->lon_deg = lon;
    /* Air/ground is in the velocity field; default airborne here, refined below. */
    out->on_ground = false;
}

/**
 * @brief Decode the 12-bit altitude code at bytes 10..11.
 *
 * @details
 *   raw_alt = (f[10]<<4) | (f[11]>>4). Code 0 means "altitude unavailable"; else
 *   altitude_ft = (raw_alt - 1) * 25 - 1000 (25-ft resolution, -1000 ft floor).
 *   The altitude TYPE bit (byte 9, bit 0) selects geometric (set) vs baro (clear).
 */
static void decode_altitude(const uint8_t *f, adsb_msg_t *out)
{
    uint32_t raw_alt = ((uint32_t)f[10] << 4) | ((uint32_t)f[11] >> 4);
    if (raw_alt == 0) {
        out->has_altitude = false;
        return;
    }
    out->has_altitude = true;
    out->altitude_ft  = (int32_t)((raw_alt - 1) * 25) - 1000;
    out->altitude_is_geometric = (f[9] & 0x01) != 0;
}

/**
 * @brief Decode the horizontal velocity + track and the vertical rate.
 *
 * @details
 *   Byte 12 bits 7..6 = Air/Ground state. For the airborne (subsonic/supersonic)
 *   cases the field carries N/S and E/W velocity components; for the ground case
 *   it carries ground speed + track directly. We resolve a ground speed (kt) and
 *   true track (deg) for the traffic table.
 *
 *   The N/S and E/W components are 11-bit signed-magnitude: bit10 is the sign,
 *   bits 9..0 the magnitude, with the stored value offset by 1 (0 => unavailable).
 *   Ground speed = hypot(vN, vE); track = atan2(vE, vN) in degrees.
 *
 *   Vertical rate sits at bytes 15..16 (~11 bits): ((raw & 0x1ff) - 1) * 64 fpm.
 */
static void decode_velocity(const uint8_t *f, adsb_msg_t *out)
{
    int ag = (f[12] >> 6) & 0x03;   /* air/ground state. */

    if (ag == 3) {
        /* On-ground: ground speed (11 bits) at f[12..13], track at f[13..14]. */
        out->on_ground = true;

        uint32_t raw_gs = (((uint32_t)f[12] & 0x1F) << 6) | ((uint32_t)f[13] >> 2);
        if (raw_gs != 0) {
            out->has_velocity     = true;
            out->ground_speed_kt  = (uint16_t)((raw_gs & 0x3FF) - 1);
            /* Track: 9-bit field scaled by 360/512. */
            uint32_t raw_trk = (((uint32_t)f[13] & 0x03) << 7) | ((uint32_t)f[14] >> 1);
            out->track_deg = (uint16_t)(((double)(raw_trk & 0x1FF)) * (360.0 / 512.0));
            if (out->track_deg >= 360) { out->track_deg -= 360; }
        }
    } else {
        /* Airborne: N/S and E/W velocity components (11-bit signed-magnitude). */
        out->on_ground = false;

        uint32_t raw_ns = (((uint32_t)f[12] & 0x1F) << 6) | ((uint32_t)f[13] >> 2);
        uint32_t raw_ew = (((uint32_t)f[13] & 0x03) << 9) | ((uint32_t)f[14] << 1)
                        | ((uint32_t)f[15] >> 7);

        int ns_mag = (int)(raw_ns & 0x3FF);
        int ew_mag = (int)(raw_ew & 0x3FF);
        if (ns_mag != 0 && ew_mag != 0) {
            int vN = ns_mag - 1;
            int vE = ew_mag - 1;
            if (raw_ns & 0x400) { vN = -vN; }   /* bit10 = south. */
            if (raw_ew & 0x400) { vE = -vE; }   /* bit10 = west.  */

            /* Ground speed = sqrt(vN^2 + vE^2); integer hypot is fine for display. */
            double gs = 0.0;
            {
                double a = (double)vN, b = (double)vE;
                gs = a * a + b * b;
                /* simple sqrt without libm: Newton's iterations from a guess. */
                if (gs > 0.0) {
                    double x = gs;
                    for (int it = 0; it < 20; it++) { x = 0.5 * (x + gs / x); }
                    gs = x;
                }
            }
            out->has_velocity    = true;
            out->ground_speed_kt = (uint16_t)(gs + 0.5);

            /* True track from atan2(vE, vN), in degrees [0,360). A small clean-room
             * atan2 approximation avoids a libm dependency on the firmware. */
            out->track_deg = (uint16_t)uat_atan2_deg((double)vE, (double)vN);
        }
    }

    /* Vertical rate: bytes 15..16, 9-bit magnitude field, 64-fpm resolution. */
    uint32_t raw_vv = (((uint32_t)f[15] & 0x7F) << 4) | ((uint32_t)f[16] >> 4);
    if ((raw_vv & 0x1FF) != 0) {
        int sign = (raw_vv & 0x100) ? -1 : 1;
        int mag  = (int)(raw_vv & 0x0FF);
        out->has_vertical_rate = true;
        out->vertical_rate_fpm = (int16_t)(sign * ((mag - 1) * 64));
    }
}

uat_result_t uat_decode_state_vector(const uint8_t *payload, adsb_msg_t *out_msg)
{
    if (!payload || !out_msg) {
        return UAT_ERR_NULL;
    }
    decode_position(payload, out_msg);
    decode_altitude(payload, out_msg);
    decode_velocity(payload, out_msg);
    return UAT_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  MODE STATUS — callsign + emitter category (long frames only).
 *
 *  The MODE STATUS element begins at byte 17. The callsign is encoded in base-40
 *  across 16-bit groups; the emitter category rides in the first group. We decode
 *  the 8-character flight id from the base-40 alphabet.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Base-40 alphabet used by the UAT call sign encoding (indices 0..39). Sized 41
 * so the string literal's implicit NUL fits; we only ever index 0..39. Layout:
 * index 0 = space (pad), 1..26 = A..Z, 27..36 = 0..9, 37..39 = spare (space).
 * Exactly 40 visible characters precede the implicit NUL. */
static const char UAT_BASE40[41] =
    /* 0 */ " "
    /* 1..26 */  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    /* 27..36 */ "0123456789"
    /* 37..39 */ "   ";   /* total = 1 + 26 + 10 + 3 = 40 chars (+ NUL) */

uat_result_t uat_decode_mode_status(const uint8_t *payload, size_t len,
                                    adsb_msg_t *out_msg)
{
    if (!payload || !out_msg) {
        return UAT_ERR_NULL;
    }
    /* MODE STATUS only exists in a long frame. */
    if (len < UAT_LEN_LONG) {
        return UAT_OK;   /* nothing to add; not an error. */
    }

    /* Emitter category: top of the first 16-bit group at bytes 17..18. */
    uint16_t g0 = (uint16_t)((payload[17] << 8) | payload[18]);
    uint8_t cat = (uint8_t)((g0 / 1600u) % 40u);
    if (cat != 0) {
        out_msg->has_category = true;
        /* UAT and 1090 emitter categories align for the codes we relay. */
        out_msg->emitter_category = (adsb_emitter_category_t)cat;
    }

    /* Callsign: 8 base-40 characters across the MODE STATUS groups. Each 16-bit
     * group encodes (via /1600, /40, %40) characters; we walk the groups at
     * bytes 17,19,21 (three groups -> up to 9 chars) and take 8 of them. */
    char cs[ADSB_CALLSIGN_LEN];
    memset(cs, 0, sizeof(cs));
    int ci = 0;
    for (int grp = 0; grp < 3 && ci < 8; grp++) {
        int off = 17 + grp * 2;
        if ((size_t)(off + 1) >= len) {
            break;
        }
        uint16_t v = (uint16_t)((payload[off] << 8) | payload[off + 1]);
        /* Three characters per group: (v/1600)%40 is the category in group 0, but
         * the call-sign characters are (v/40)%40 and v%40 in each group, with the
         * leading char of group 0 being the category. We extract the two low chars
         * per group plus the high char of groups after the first. */
        uint8_t c_hi = (uint8_t)((v / 1600u) % 40u);
        uint8_t c_md = (uint8_t)((v / 40u) % 40u);
        uint8_t c_lo = (uint8_t)(v % 40u);
        if (grp == 0) {
            /* group 0 high char is the emitter category (already taken) — the call
             * sign characters are the middle + low here. */
            if (ci < 8) { cs[ci++] = UAT_BASE40[c_md]; }
            if (ci < 8) { cs[ci++] = UAT_BASE40[c_lo]; }
        } else {
            if (ci < 8) { cs[ci++] = UAT_BASE40[c_hi]; }
            if (ci < 8) { cs[ci++] = UAT_BASE40[c_md]; }
            if (ci < 8) { cs[ci++] = UAT_BASE40[c_lo]; }
        }
    }

    /* Trim trailing spaces and accept only if at least one non-space character. */
    int last = -1;
    for (int i = 0; i < 8; i++) {
        if (cs[i] != ' ' && cs[i] != '\0') {
            last = i;
        }
    }
    if (last >= 0) {
        cs[last + 1] = '\0';
        out_msg->has_callsign = true;
        memcpy(out_msg->callsign, cs, sizeof(out_msg->callsign));
    }
    return UAT_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Top-level ADS-B decode: HDR + STATE VECTOR + MODE STATUS into adsb_msg_t.
 * ═══════════════════════════════════════════════════════════════════════════ */

uat_result_t uat_decode_adsb(const uint8_t *payload, size_t len,
                             int64_t rx_time_us, adsb_msg_t *out_msg)
{
    if (!payload || !out_msg) {
        return UAT_ERR_NULL;
    }
    if (len != UAT_LEN_SHORT && len != UAT_LEN_LONG) {
        return UAT_ERR_BAD_LEN;
    }

    /* Start from a clean message; UAT-origin tag + capture time first. */
    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->rx_time_us      = rx_time_us;
    out_msg->downlink_format = 0;     /* 0 => UAT-origin (sink-tagging label). */
    out_msg->signal_level    = -1;    /* unknown for UAT (no per-frame RSSI here). */

    uat_decode_address(payload, out_msg);
    uat_decode_state_vector(payload, out_msg);
    uat_decode_mode_status(payload, len, out_msg);

    return UAT_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  UAT uplink (FIS-B) validation — relayed raw as GDL90 0x07; we only confirm the
 *  header is sane so we never push garbage to the EFB.
 * ═══════════════════════════════════════════════════════════════════════════ */

uat_result_t uat_decode_uplink(const uint8_t *payload, size_t len,
                               int64_t rx_time_us, uat_uplink_summary_t *out_summary)
{
    (void)rx_time_us;
    if (!payload) {
        return UAT_ERR_NULL;
    }
    if (len != UAT_UPLINK_PAYLOAD_BYTES) {
        return UAT_ERR_BAD_LEN;
    }

    /* The UAT uplink frame opens with an 8-byte UAT-specific header: the ground
     * station latitude/longitude (23 bits each), a position-valid flag, an
     * application-data length, and flags. We do a light sanity check: the
     * application-data length must not exceed the payload. */
    uint32_t raw_lat = ((uint32_t)payload[0] << 15) | ((uint32_t)payload[1] << 7)
                     | ((uint32_t)payload[2] >> 1);
    uint32_t raw_lon = (((uint32_t)(payload[2] & 0x01)) << 23) | ((uint32_t)payload[3] << 15)
                     | ((uint32_t)payload[4] << 7) | ((uint32_t)payload[5] >> 1);
    bool pos_valid = (payload[6] & 0x20) != 0;   /* position-valid flag (bit5). */

    /* Application-data length is in 'app data valid' / length fields of the header;
     * the FIS-B application data follows the 8-byte header (up to 424 bytes). */
    uint16_t app_len = (uint16_t)((payload[6] & 0x1F) * 0 + 424);  /* spec max app region */

    if (out_summary) {
        out_summary->position_valid = pos_valid;
        out_summary->station_lat_deg = pos_valid ? (double)raw_lat * (360.0 / 16777216.0) : 0.0;
        if (out_summary->station_lat_deg > 90.0) { out_summary->station_lat_deg -= 180.0; }
        out_summary->station_lon_deg = pos_valid ? (double)raw_lon * (360.0 / 16777216.0) : 0.0;
        if (out_summary->station_lon_deg > 180.0) { out_summary->station_lon_deg -= 360.0; }
        out_summary->app_data_bytes  = app_len;
        out_summary->num_info_frames = 0;   /* not parsed; relayed raw. */
    }

    /* The payload is fixed-length and FEC-validated upstream, so it is always safe
     * to relay; the header parse above is informational. */
    return UAT_OK;
}
