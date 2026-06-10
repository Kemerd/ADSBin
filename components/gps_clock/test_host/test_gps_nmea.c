/* Host unit test for the pure NMEA-0183 parser (../gps_clock_nmea.c).
 *
 * Includes the parser source directly so the test can reach its file-static state
 * and exercise gps_nmea_feed()/gps_nmea_reset() with crafted sentence streams.
 * Asserts the resulting gps_nmea_signals_t (fix validity, lat/lon/alt, UTC) for:
 *   - a good GGA + RMC pair (multi-constellation GN talker)
 *   - a bad-checksum sentence (rejected)
 *   - the null-island (0,0) position (rejected as invalid)
 *   - a UTC date/time → absolute Unix-µs conversion (incl. a known epoch)
 *   - RMC status 'V' (void) → fix not valid
 * NOT part of the firmware build. */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* Pull the parser in directly (static symbols + state become visible to us). */
#include "../gps_clock_nmea.c"

static int g_fail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s\n", msg); g_fail++; } \
    else         { printf("ok  : %s\n", msg); } } while (0)
#define CLOSE(a, b, eps) (fabs((a) - (b)) <= (eps))

/* Append the NMEA XOR checksum + CRLF to "$BODY" and feed the whole line in. */
static void feed_sentence(gps_nmea_signals_t *sig, const char *body)
{
    /* body is "GNRMC,..."—everything between '$' and '*'. Compute the checksum. */
    uint8_t ck = 0;
    for (const char *p = body; *p; ++p) {
        ck ^= (uint8_t)*p;
    }
    char line[128];
    int n = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, ck);
    gps_nmea_feed(sig, (const uint8_t *)line, (size_t)n);
}

/* Feed a line with a DELIBERATELY WRONG checksum (so the parser must reject it). */
static void feed_bad_checksum(gps_nmea_signals_t *sig, const char *body)
{
    char line[128];
    int n = snprintf(line, sizeof(line), "$%s*00\r\n", body);   /* 00 won't match */
    gps_nmea_feed(sig, (const uint8_t *)line, (size_t)n);
}

int main(void)
{
    /* 1) Good GGA + RMC for the same second (GN multi-constellation talker). */
    {
        gps_nmea_signals_t sig;
        gps_nmea_reset(&sig);

        /* GGA: time 123519, lat 4807.038 N, lon 01131.000 E, quality 1, alt 545.4 m */
        feed_sentence(&sig,
            "GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
        /* RMC: time 123519, status A, same lat/lon, speed 22.4 kt, course 84.4,
         * date 230324 (23 Mar 2024 — the M10 emits 20xx dates). */
        feed_sentence(&sig,
            "GNRMC,123519.00,A,4807.038,N,01131.000,E,22.4,084.4,230324,,,A");

        CHECK(sig.fix_valid, "good GGA+RMC => fix_valid");
        CHECK(CLOSE(sig.lat_deg, 48.0 + 7.038/60.0, 1e-6), "latitude decoded");
        CHECK(CLOSE(sig.lon_deg, 11.0 + 31.000/60.0, 1e-6), "longitude decoded");
        CHECK(CLOSE(sig.altitude_m, 545.4, 0.05), "MSL altitude from GGA (merged by time-of-day)");
        CHECK(sig.ground_speed_kt == 22, "ground speed (rounded kt)");
        CHECK(sig.track_deg == 84, "track (rounded deg)");
        CHECK(sig.has_velocity, "velocity present");
        /* 2024-03-23 12:35:19 UTC = 1711197319 s since epoch. */
        CHECK(sig.utc_us == 1711197319LL * 1000000LL, "absolute UTC µs (civil-from-days)");
    }

    /* 2) Bad checksum is rejected (no fresh fix published). */
    {
        gps_nmea_signals_t sig;
        gps_nmea_reset(&sig);
        int64_t before = sig.fix_seq;
        feed_bad_checksum(&sig,
            "GNRMC,123519.00,A,4807.038,N,01131.000,E,22.4,084.4,230394,,,A");
        CHECK(sig.fix_seq == before, "bad-checksum sentence ignored (no new fix)");
        CHECK(!sig.fix_valid, "bad-checksum leaves fix invalid");
    }

    /* 3) Null-island (0,0) position is rejected even with status 'A'. */
    {
        gps_nmea_signals_t sig;
        gps_nmea_reset(&sig);
        feed_sentence(&sig,
            "GNGGA,000000.00,0000.000,N,00000.000,E,1,04,1.0,0.0,M,0.0,M,,");
        feed_sentence(&sig,
            "GNRMC,000000.00,A,0000.000,N,00000.000,E,0.0,0.0,060180,,,A");
        CHECK(!sig.fix_valid, "null-island (0,0) rejected as invalid");
    }

    /* 4) Southern/Western hemisphere sign handling. */
    {
        gps_nmea_signals_t sig;
        gps_nmea_reset(&sig);
        feed_sentence(&sig,
            "GPGGA,010101.00,3351.000,S,15112.000,E,1,06,1.0,10.0,M,0.0,M,,");
        feed_sentence(&sig,
            "GPRMC,010101.00,A,3351.000,S,15112.000,E,0.0,0.0,010220,,,A");
        CHECK(sig.fix_valid, "S/E fix valid");
        CHECK(sig.lat_deg < 0, "southern latitude is negative");
        CHECK(sig.lon_deg > 0, "eastern longitude is positive");
        CHECK(CLOSE(sig.lat_deg, -(33.0 + 51.0/60.0), 1e-6), "S latitude magnitude");
    }

    /* 5) RMC status 'V' (navigation receiver warning) => not a valid fix. */
    {
        gps_nmea_signals_t sig;
        gps_nmea_reset(&sig);
        feed_sentence(&sig,
            "GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
        feed_sentence(&sig,
            "GNRMC,123519.00,V,4807.038,N,01131.000,E,22.4,084.4,230394,,,N");
        CHECK(!sig.fix_valid, "RMC status 'V' => fix not valid");
    }

    /* 6) Byte-count tracks every consumed byte (presence signal for the ladder). */
    {
        gps_nmea_signals_t sig;
        gps_nmea_reset(&sig);
        const char *blob = "garbage\r\n";
        gps_nmea_feed(&sig, (const uint8_t *)blob, strlen(blob));
        CHECK(sig.byte_count == strlen(blob), "byte_count counts all consumed bytes");
    }

    printf("\n%s (%d failures)\n", g_fail ? "FAILURES" : "ALL PASS", g_fail);
    return g_fail ? 1 : 0;
}
