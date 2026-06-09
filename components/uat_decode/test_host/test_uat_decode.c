/* Host unit test for the UAT ADS-B decoder (../uat_decode.c). Builds UAT payloads
 * with known field values (address, absolute lat/lon, altitude, velocity) per the
 * DO-282B bit layout, decodes them, and asserts the resulting adsb_msg_t matches.
 * Also checks the uplink header validation. NOT part of the firmware build. */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "uat_decode.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); g_fail++; } else { printf("ok  : %s\n", msg); } } while (0)
#define CLOSE(a, b, eps) (fabs((a) - (b)) <= (eps))

/* Encode degrees into the 24-bit UAT semicircle (inverse of the decoder). */
static uint32_t enc_lat(double deg)
{
    if (deg < 0) deg += 180.0;   /* inverse of the decoder's ">90 => -180" wrap. */
    return (uint32_t)lround(deg * (16777216.0 / 360.0)) & 0xFFFFFF;
}
static uint32_t enc_lon(double deg)
{
    if (deg < 0) deg += 360.0;
    return (uint32_t)lround(deg * (16777216.0 / 360.0)) & 0xFFFFFF;
}

int main(void)
{
    /* 1) A long UAT ADS-B frame with a known address, position, altitude. */
    {
        uint8_t f[34];
        memset(f, 0, sizeof(f));

        /* HDR: byte0 bits7..3 = MDB type (1 => long), bits2..0 = addr qualifier 0.
         * Address 0xA1B2C3 at bytes 1..3. */
        f[0] = (uint8_t)((1 << 3) | 0);
        f[1] = 0xA1; f[2] = 0xB2; f[3] = 0xC3;

        /* Position: lat 44.0, lon -93.0 (near KMSP). Pack into bytes 4..9. */
        uint32_t rl = enc_lat(44.0);
        uint32_t ro = enc_lon(-93.0);
        f[4] = (uint8_t)((rl >> 15) & 0xFF);
        f[5] = (uint8_t)((rl >> 7) & 0xFF);
        f[6] = (uint8_t)(((rl << 1) & 0xFE) | ((ro >> 23) & 0x01));
        f[7] = (uint8_t)((ro >> 15) & 0xFF);
        f[8] = (uint8_t)((ro >> 7) & 0xFF);
        f[9] = (uint8_t)((ro << 1) & 0xFE);   /* bit0 = altitude type (0 => baro). */

        /* Altitude 5000 ft: raw = (5000 + 1000)/25 + 1 = 241. 12-bit at f[10..11]. */
        uint32_t raw_alt = (uint32_t)((5000 + 1000) / 25 + 1);
        f[10] = (uint8_t)((raw_alt >> 4) & 0xFF);
        f[11] = (uint8_t)((raw_alt << 4) & 0xF0);

        adsb_msg_t m;
        uat_result_t r = uat_decode_adsb(f, 34, 123456, &m);
        CHECK(r == UAT_OK, "long frame decodes UAT_OK");
        CHECK(m.icao == 0xA1B2C3, "address decoded");
        CHECK(m.downlink_format == 0, "tagged as UAT-origin (df=0)");
        CHECK(m.has_position && CLOSE(m.lat_deg, 44.0, 0.001), "latitude ~44.0");
        CHECK(m.has_position && CLOSE(m.lon_deg, -93.0, 0.001), "longitude ~-93.0");
        CHECK(m.has_altitude && m.altitude_ft == 5000, "altitude 5000 ft");
        CHECK(!m.altitude_is_geometric, "altitude is barometric");
        CHECK(m.rx_time_us == 123456, "rx_time carried through");
    }

    /* 2) Geometric altitude bit set. */
    {
        uint8_t f[34];
        memset(f, 0, sizeof(f));
        f[0] = (1 << 3);
        uint32_t rl = enc_lat(10.0), ro = enc_lon(20.0);
        f[4] = (rl >> 15) & 0xFF; f[5] = (rl >> 7) & 0xFF;
        f[6] = ((rl << 1) & 0xFE) | ((ro >> 23) & 0x01);
        f[7] = (ro >> 15) & 0xFF; f[8] = (ro >> 7) & 0xFF;
        f[9] = ((ro << 1) & 0xFE) | 0x01;   /* altitude type = geometric. */
        uint32_t raw_alt = (uint32_t)((10000 + 1000) / 25 + 1);
        f[10] = (raw_alt >> 4) & 0xFF; f[11] = (raw_alt << 4) & 0xF0;

        adsb_msg_t m;
        uat_decode_adsb(f, 34, 0, &m);
        CHECK(m.has_altitude && m.altitude_ft == 10000, "geometric altitude 10000 ft");
        CHECK(m.altitude_is_geometric, "altitude flagged geometric");
    }

    /* 3) Short frame (18 bytes) decodes address + position without MODE STATUS. */
    {
        uint8_t f[18];
        memset(f, 0, sizeof(f));
        f[0] = (0 << 3);   /* type 0 => short. */
        f[1] = 0x12; f[2] = 0x34; f[3] = 0x56;
        uint32_t rl = enc_lat(-33.0), ro = enc_lon(151.0);  /* Sydney-ish. */
        f[4] = (rl >> 15) & 0xFF; f[5] = (rl >> 7) & 0xFF;
        f[6] = ((rl << 1) & 0xFE) | ((ro >> 23) & 0x01);
        f[7] = (ro >> 15) & 0xFF; f[8] = (ro >> 7) & 0xFF; f[9] = (ro << 1) & 0xFE;

        adsb_msg_t m;
        uat_result_t r = uat_decode_adsb(f, 18, 0, &m);
        CHECK(r == UAT_OK && m.icao == 0x123456, "short frame address");
        CHECK(m.has_position && CLOSE(m.lat_deg, -33.0, 0.001), "short frame lat ~-33");
        CHECK(m.has_position && CLOSE(m.lon_deg, 151.0, 0.001), "short frame lon ~151");
        CHECK(!m.has_callsign, "short frame has no callsign (no MODE STATUS)");
    }

    /* 4) Zero position => has_position false (UAT 'unavailable' sentinel). */
    {
        uint8_t f[18];
        memset(f, 0, sizeof(f));
        f[0] = 0; f[1] = 1; f[2] = 2; f[3] = 3;   /* address, no position. */
        adsb_msg_t m;
        uat_decode_adsb(f, 18, 0, &m);
        CHECK(!m.has_position, "all-zero lat/lon => no position");
        CHECK(m.icao == 0x010203, "address still decoded with no position");
    }

    /* 5) Bad length rejected. */
    {
        uint8_t f[20] = {0};
        adsb_msg_t m;
        CHECK(uat_decode_adsb(f, 20, 0, &m) == UAT_ERR_BAD_LEN, "bad length rejected");
        CHECK(uat_decode_adsb(NULL, 18, 0, &m) == UAT_ERR_NULL, "NULL payload rejected");
    }

    /* 6) Uplink validation: correct length passes; wrong length fails. */
    {
        uint8_t up[432];
        memset(up, 0, sizeof(up));
        up[6] |= 0x20;   /* position-valid flag. */
        uat_uplink_summary_t s;
        uat_result_t r = uat_decode_uplink(up, 432, 0, &s);
        CHECK(r == UAT_OK, "valid-length uplink passes");
        CHECK(s.position_valid, "uplink position-valid flag parsed");

        uint8_t bad[100] = {0};
        CHECK(uat_decode_uplink(bad, 100, 0, NULL) == UAT_ERR_BAD_LEN, "short uplink rejected");
    }

    printf("\n%s (%d failures)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
