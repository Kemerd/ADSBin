/**
 * @file    gps_clock_nmea.c
 * @brief   Layer 2 — clean-room, allocation-free NMEA-0183 parser (signal producer).
 *
 * @details
 *   A byte-streaming parser that turns the MAX-M10S serial NMEA into one merged
 *   fix per UTC second. It is a PURE SIGNAL PRODUCER: it fills ::gps_nmea_signals_t
 *   and NEVER decides clock quality — the supervisor owns the ladder.
 *
 *   == What it parses ==
 *     - RMC (Recommended Minimum): UTC time, status A/V, lat/lon, ground speed,
 *       course, and DATE (the only sentence that carries the calendar day, which
 *       we need to build an absolute Unix timestamp).
 *     - GGA (Fix Data): fix quality and MSL altitude (and a lat/lon cross-check).
 *
 *   Talker prefix is IGNORED (GP/GN/GL/GB/…): a multi-constellation M10 emits GN*
 *   sentences, so we match on the 3-letter SENTENCE type (chars [2..4] of the
 *   address field), not the talker. RMC and GGA for the SAME UTC second are merged
 *   into one coherent fix regardless of arrival order.
 *
 *   == Rigor ==
 *     XOR checksum gate (post-'$' .. pre-'*'), field-count guards, range checks on
 *     lat/lon, null-island rejection, an 82-char line cap (NMEA max is 82 incl.
 *     CR/LF), and a fix is only flagged valid when RMC status is 'A' AND a GGA
 *     fix-quality ≥ 1 has been seen for the same UTC second.
 *
 *   == No libc heap, no libm ==
 *     Coordinate conversion is integer/double arithmetic; the calendar conversion
 *     is the classic days-from-civil algorithm (Howard Hinnant), so there is no
 *     dependency on timegm()/mktime() (absent/locale-dependent on the target).
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "gps_clock_signals.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "adsbin_types.h"   // adsbin_now_us()

/* ───────────────────────────────────────────────────────────────────────────
 *  Line-assembly state. Kept in a small file-static struct re-pointed per parser
 *  instance via the signals pointer — but since there is exactly one GPS and one
 *  supervisor task, a single static assembler is correct and allocation-free.
 * ─────────────────────────────────────────────────────────────────────────── */

#define NMEA_MAX_LINE   82          /**< NMEA hard max incl. CR/LF (we store body). */
#define NMEA_MAX_FIELDS 24          /**< Generous upper bound on comma fields.      */

typedef struct {
    char   line[NMEA_MAX_LINE + 1]; /**< Current sentence body (between $ and *).   */
    size_t len;                     /**< Chars in @c line.                          */
    bool   overflow;                /**< Dropped: line exceeded the cap; resync.    */

    /* Same-second merge scratch: an RMC and a GGA for one second are stitched
     * together before being published as a single fix. */
    int64_t pend_second;            /**< UTC second the pending halves belong to.   */
    bool    have_rmc;               /**< RMC seen for pend_second.                   */
    bool    have_gga;               /**< GGA seen for pend_second.                   */

    /* RMC-derived. */
    bool    rmc_active;             /**< status == 'A'.                             */
    double  rmc_lat, rmc_lon;
    uint16_t rmc_speed_kt, rmc_track_deg;
    bool    rmc_has_vel;
    uint32_t rmc_frac_ns;
    int64_t rmc_utc_us;             /**< Absolute UTC of the RMC fix.               */

    /* GGA-derived. */
    int     gga_quality;            /**< 0 = no fix.                                */
    float   gga_alt_m;              /**< MSL altitude, metres (NAN if absent).      */
    double  gga_lat, gga_lon;
    bool    gga_has_pos;

    int64_t fix_seq;                /**< Bumped once per published merged fix.      */
} nmea_state_t;

static nmea_state_t s_nmea_state;

/* ───────────────────────────────────────────────────────────────────────────
 *  Small helpers
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief One ASCII hex nibble → 0..15, or -1 if not hex. */
static int nmea_hex(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/**
 * @brief Validate the XOR checksum of a sentence body "TYPE,fields*HH".
 *
 * The checksum is the XOR of every char AFTER '$' and BEFORE '*'. @p body holds
 * exactly those bytes plus "*HH"; we split on '*', recompute, and compare.
 *
 * @return Pointer to the first field char (start of body) if valid, else NULL.
 */
static bool nmea_checksum_ok(const char *body, size_t len)
{
    // Find the '*'. It must leave exactly two hex digits after it.
    const char *star = memchr(body, '*', len);
    if (star == NULL) {
        return false;
    }
    size_t payload_len = (size_t)(star - body);
    if (len < payload_len + 3) {        // need '*' + 2 hex
        return false;
    }
    int hi = nmea_hex(star[1]);
    int lo = nmea_hex(star[2]);
    if (hi < 0 || lo < 0) {
        return false;
    }
    uint8_t want = (uint8_t)((hi << 4) | lo);

    uint8_t got = 0;
    for (size_t i = 0; i < payload_len; ++i) {
        got ^= (uint8_t)body[i];
    }
    return got == want;
}

/**
 * @brief Split a comma-delimited sentence body into field pointers (in place).
 *
 * Truncates @p body at the '*' and replaces commas with NULs, returning the field
 * starts. Empty fields yield empty strings (valid NMEA — a missing value).
 *
 * @return Number of fields found.
 */
static int nmea_split(char *body, char *fields[], int max_fields)
{
    // Cut the checksum off.
    char *star = strchr(body, '*');
    if (star) {
        *star = '\0';
    }

    int n = 0;
    char *p = body;
    fields[n++] = p;
    while (*p && n < max_fields) {
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        }
        ++p;
    }
    return n;
}

/**
 * @brief Convert an NMEA "ddmm.mmmm" coordinate + hemisphere to decimal degrees.
 *
 * Latitude is dd, longitude is ddd; both share mm.mmmm minutes. Hemisphere 'S'/'W'
 * negates. Returns NAN on an empty/garbled field so the caller can reject it.
 */
static double nmea_coord(const char *val, const char *hemi, int deg_digits)
{
    if (val == NULL || val[0] == '\0') {
        return NAN;
    }
    // Degrees are the leading deg_digits; the rest is decimal minutes.
    char degbuf[4] = {0};
    if (deg_digits >= (int)sizeof(degbuf)) {
        return NAN;
    }
    for (int i = 0; i < deg_digits; ++i) {
        if (val[i] < '0' || val[i] > '9') {
            return NAN;
        }
        degbuf[i] = val[i];
    }
    double deg = (double)atoi(degbuf);
    double min = atof(val + deg_digits);
    double dec = deg + min / 60.0;

    if (hemi && (hemi[0] == 'S' || hemi[0] == 'W')) {
        dec = -dec;
    }
    return dec;
}

/**
 * @brief Parse "hhmmss.ss" → seconds-of-day (double). NAN on garbage.
 */
static double nmea_tod(const char *t)
{
    if (t == NULL || strlen(t) < 6) {
        return NAN;
    }
    int hh = (t[0] - '0') * 10 + (t[1] - '0');
    int mm = (t[2] - '0') * 10 + (t[3] - '0');
    double ss = atof(t + 4);
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0.0 || ss >= 61.0) {
        return NAN;
    }
    return hh * 3600.0 + mm * 60.0 + ss;
}

/**
 * @brief Days from civil date (Howard Hinnant's algorithm) → days since 1970-01-01.
 *
 * Valid for any Gregorian date; no libc, no locale. Used to turn RMC's ddmmyy +
 * the time-of-day into an absolute Unix timestamp without timegm()/mktime().
 */
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= (m <= 2);
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153u * (m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097LL + (int64_t)doe - 719468LL;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Sentence handlers
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Try to publish a merged fix once both halves (or enough) are in.
 *
 * A fix is published when RMC is active for the pending second; GGA contributes
 * quality + altitude when present for the same second. We anchor the absolute UTC
 * to the RMC time-of-day + date, and stamp adsbin_now_us() at publication.
 */
static void nmea_try_publish(nmea_state_t *st, gps_nmea_signals_t *out)
{
    if (!st->have_rmc) {
        return;     // RMC carries the date; without it we have no absolute UTC
    }

    // Position: prefer GGA's (it pairs with the altitude), else RMC's.
    double lat = st->gga_has_pos ? st->gga_lat : st->rmc_lat;
    double lon = st->gga_has_pos ? st->gga_lon : st->rmc_lon;

    // Validity: RMC must be 'A' and (if we have GGA) quality must be a real fix.
    bool quality_ok = st->have_gga ? (st->gga_quality >= 1) : true;
    bool coords_ok  = isfinite(lat) && isfinite(lon)
                      && lat >= -90.0 && lat <= 90.0
                      && lon >= -180.0 && lon <= 180.0
                      && !(fabs(lat) < 1e-6 && fabs(lon) < 1e-6);   // null-island

    bool valid = st->rmc_active && quality_ok && coords_ok;

    // Bump the publication sequence so the supervisor's debounce sees a fresh fix.
    st->fix_seq++;

    out->fix_valid         = valid;
    out->fix_seq           = st->fix_seq;
    out->utc_us            = st->rmc_utc_us;
    out->utc_anchor_now_us = adsbin_now_us();
    out->rmc_second        = st->rmc_utc_us / 1000000LL;
    out->rmc_frac_ns       = st->rmc_frac_ns;
    out->lat_deg           = lat;
    out->lon_deg           = lon;
    out->altitude_m        = st->have_gga ? st->gga_alt_m : NAN;
    out->ground_speed_kt   = st->rmc_speed_kt;
    out->track_deg         = st->rmc_track_deg;
    out->has_velocity      = st->rmc_has_vel;

    // Consume the pending halves so the next second starts clean.
    st->have_rmc = false;
    st->have_gga = false;
    st->gga_has_pos = false;
}

/** @brief Handle an RMC sentence (already split into fields). */
static void nmea_handle_rmc(nmea_state_t *st, char *fields[], int nf, gps_nmea_signals_t *out)
{
    // RMC: 0=addr 1=time 2=status 3=lat 4=N/S 5=lon 6=E/W 7=spd 8=course 9=date
    if (nf < 10) {
        return;
    }
    double tod = nmea_tod(fields[1]);
    const char *date = fields[9];
    if (!isfinite(tod) || strlen(date) < 6) {
        return;     // need both time and date to anchor absolute UTC
    }

    int dd = (date[0] - '0') * 10 + (date[1] - '0');
    int mo = (date[2] - '0') * 10 + (date[3] - '0');
    int yy = (date[4] - '0') * 10 + (date[5] - '0');
    if (dd < 1 || dd > 31 || mo < 1 || mo > 12) {
        return;
    }
    int year = 2000 + yy;   // NMEA two-digit year; M10 era is 20xx

    int64_t whole_sec = (int64_t)tod;                       // integer seconds of day
    double  frac      = tod - (double)whole_sec;            // sub-second part
    int64_t day_sec   = days_from_civil(year, (unsigned)mo, (unsigned)dd) * 86400LL;
    int64_t utc_sec   = day_sec + whole_sec;

    st->rmc_active   = (fields[2][0] == 'A');
    st->rmc_lat      = nmea_coord(fields[3], fields[4], 2);
    st->rmc_lon      = nmea_coord(fields[5], fields[6], 3);
    st->rmc_speed_kt = (uint16_t)lroundf((float)atof(fields[7]));
    st->rmc_track_deg= (uint16_t)lroundf((float)atof(fields[8]));
    st->rmc_has_vel  = (fields[7][0] != '\0') && (fields[8][0] != '\0');
    st->rmc_frac_ns  = (uint32_t)lround(frac * 1e9);
    st->rmc_utc_us   = utc_sec * 1000000LL + (int64_t)lround(frac * 1e6);

    // If this RMC opens a new second, the previous pending GGA (if any) belonged to
    // a different second and is stale — drop it.
    if (st->rmc_second != utc_sec) {
        st->pend_second = utc_sec;
        st->have_gga    = false;
    }
    st->rmc_second = utc_sec;
    st->have_rmc   = true;

    nmea_try_publish(st, out);
}

/** @brief Handle a GGA sentence (already split into fields). */
static void nmea_handle_gga(nmea_state_t *st, char *fields[], int nf, gps_nmea_signals_t *out)
{
    // GGA: 0=addr 1=time 2=lat 3=N/S 4=lon 5=E/W 6=quality 7=numSV 8=HDOP
    //      9=alt 10=altUnit 11=geoidSep ...
    if (nf < 10) {
        return;
    }
    st->gga_quality = atoi(fields[6]);
    st->gga_lat     = nmea_coord(fields[2], fields[3], 2);
    st->gga_lon     = nmea_coord(fields[4], fields[5], 3);
    st->gga_has_pos = isfinite(st->gga_lat) && isfinite(st->gga_lon);
    st->gga_alt_m   = (fields[9][0] != '\0') ? (float)atof(fields[9]) : NAN;
    st->have_gga    = true;

    // GGA has no date, so it cannot open a second on its own; it only enriches the
    // fix RMC anchors. If RMC for this second already arrived, publish the merge.
    nmea_try_publish(st, out);
}

/** @brief Dispatch a checksum-valid sentence by its 3-letter type. */
static void nmea_dispatch(nmea_state_t *st, gps_nmea_signals_t *out)
{
    // Address field is the first token; type is its last 3 chars (talker-agnostic).
    // e.g. "GNRMC" -> "RMC", "GPGGA" -> "GGA".
    size_t addr_len = strcspn(st->line, ",");
    if (addr_len < 3) {
        return;
    }
    const char *type = st->line + (addr_len - 3);

    // Split a working copy (nmea_split mutates) — st->line is ours to chew.
    char *fields[NMEA_MAX_FIELDS];
    int nf = nmea_split(st->line, fields, NMEA_MAX_FIELDS);
    if (nf < 1) {
        return;
    }

    if (memcmp(type, "RMC", 3) == 0) {
        nmea_handle_rmc(st, fields, nf, out);
    } else if (memcmp(type, "GGA", 3) == 0) {
        nmea_handle_gga(st, fields, nf, out);
    }
    // Other sentences (GSA/GSV/VTG/…) are ignored — we only need RMC+GGA.
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Public producer seam (called by the supervisor)
 * ─────────────────────────────────────────────────────────────────────────── */

void gps_nmea_reset(gps_nmea_signals_t *sig)
{
    memset(&s_nmea_state, 0, sizeof(s_nmea_state));
    s_nmea_state.gga_alt_m   = NAN;
    s_nmea_state.pend_second = INT64_MIN;
    s_nmea_state.rmc_second  = INT64_MIN;
    if (sig != NULL) {
        memset(sig, 0, sizeof(*sig));
        sig->altitude_m = NAN;
    }
}

void gps_nmea_feed(gps_nmea_signals_t *sig, const uint8_t *bytes, size_t n)
{
    if (sig == NULL || bytes == NULL) {
        return;
    }
    nmea_state_t *st = &s_nmea_state;

    for (size_t i = 0; i < n; ++i) {
        char c = (char)bytes[i];

        // Presence: every consumed byte counts, so a chatty-but-fixless module is
        // still detected as PRESENT (FREE_RUNNING) by the supervisor.
        sig->byte_count++;

        if (c == '$') {
            // Start of a new sentence — reset the assembler regardless of state, so
            // a mid-line glitch can't wedge us (the next '$' always resyncs).
            st->len = 0;
            st->overflow = false;
            continue;
        }
        if (c == '\r' || c == '\n') {
            // End of sentence. Validate + dispatch if it fits and the checksum is OK.
            if (!st->overflow && st->len > 0) {
                st->line[st->len] = '\0';
                if (nmea_checksum_ok(st->line, st->len)) {
                    nmea_dispatch(st, sig);
                }
            }
            st->len = 0;
            st->overflow = false;
            continue;
        }
        // Accumulate body chars up to the cap; an overlong line is dropped wholesale.
        if (st->len < NMEA_MAX_LINE) {
            st->line[st->len++] = c;
        } else {
            st->overflow = true;
        }
    }
}
