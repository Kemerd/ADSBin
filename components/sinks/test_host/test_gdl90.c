/* Host unit test for the pure GDL90 encoder. Validates the wire bytes
 * byte-for-byte using an INDEPENDENT reference implementation of the CRC,
 * stuffing, and field packing — written here from the same public spec, kept
 * deliberately separate from the production code so a bug in one is caught by
 * the other. NOT part of the firmware build. */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#include "gdl90_encoder.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); g_fail++; } else { printf("ok  : %s\n", msg); } } while (0)

/* ---- Independent reference CRC ---------------------------------------------
 * The GDL90 spec's CRC is the CCITT "table variant" whose fold is
 *   crc = T[crc>>8] ^ (crc<<8) ^ b
 * which is NOT the same as the plain bit-by-bit CCITT. We build the table
 * independently here (no shared code with the encoder) and additionally pin it
 * against the published spec heartbeat vector so a bug in BOTH would have to
 * coincide to slip through. */
static uint16_t ref_tbl[256];
static int ref_tbl_ready = 0;
static void ref_tbl_init(void)
{
    for (int i = 0; i < 256; ++i) {
        uint16_t c = (uint16_t)(i << 8);
        for (int b = 0; b < 8; ++b)
            c = (uint16_t)((c << 1) ^ ((c & 0x8000) ? 0x1021 : 0));
        ref_tbl[i] = c;
    }
    ref_tbl_ready = 1;
}
static uint16_t ref_crc16(const uint8_t *p, size_t n)
{
    if (!ref_tbl_ready) ref_tbl_init();
    uint16_t crc = 0;
    for (size_t i = 0; i < n; ++i)
        crc = (uint16_t)(ref_tbl[crc >> 8] ^ (crc << 8) ^ p[i]);
    return crc;
}

/* ---- Independent reference framing ---------------------------------------- */
static size_t ref_frame(const uint8_t *msg, size_t mlen, uint8_t *out)
{
    uint16_t crc = ref_crc16(msg, mlen);
    uint8_t body[64];
    size_t bl = 0;
    memcpy(body, msg, mlen); bl = mlen;
    body[bl++] = (uint8_t)(crc & 0xFF);
    body[bl++] = (uint8_t)(crc >> 8);

    size_t o = 0;
    out[o++] = 0x7E;
    for (size_t i = 0; i < bl; ++i) {
        uint8_t c = body[i];
        if (c == 0x7E || c == 0x7D) { out[o++] = 0x7D; out[o++] = (uint8_t)(c ^ 0x20); }
        else out[o++] = c;
    }
    out[o++] = 0x7E;
    return o;
}

int main(void)
{
    /* 1) CRC table-vs-bitwise agreement over many inputs. */
    {
        bool ok = true;
        for (int len = 0; len <= 40; ++len) {
            uint8_t buf[40];
            for (int i = 0; i < len; ++i) buf[i] = (uint8_t)(i * 31 + len);
            if (gdl90_crc16(buf, (size_t)len) != ref_crc16(buf, (size_t)len)) { ok = false; break; }
        }
        CHECK(ok, "crc16 table == independent reference");
    }

    /* 1b) Pin the CRC to the PUBLISHED GDL90 spec heartbeat vector. The spec
     *     example frame is 7E 00 81 41 DB D0 08 02 B3 8B 7E, i.e. CRC over the
     *     7 body bytes = 0x8BB3, transmitted little-endian as B3 8B. */
    {
        uint8_t spec[] = { 0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02 };
        uint16_t c = gdl90_crc16(spec, sizeof(spec));
        CHECK(c == 0x8BB3, "crc16 matches published GDL90 spec vector (0x8BB3)");
    }

    /* 2) Heartbeat frame matches the independent reference body+frame. */
    {
        gdl90_heartbeat_t hb = { .gps_pos_valid = true, .maint_required = false,
                                 .timestamp_s = 12345, .msg_count_uplink = 0,
                                 .msg_count_basic_long = 0 };
        uint8_t got[80]; int n = gdl90_frame_heartbeat(got, sizeof(got), &hb);
        CHECK(n > 0, "heartbeat frame returns positive length");
        /* Rebuild the expected 7-byte body the way the encoder documents it. */
        uint32_t ts = 12345u & 0x1FFFF;
        uint8_t body[7];
        body[0] = 0x00;
        body[1] = 0x81;                 /* bit0 init + bit7 gps valid */
        body[2] = (ts & 0x10000) ? 0x01 : 0x00;
        body[3] = (uint8_t)(ts & 0xFF);
        body[4] = (uint8_t)((ts >> 8) & 0xFF);
        body[5] = 0x00; body[6] = 0x00;
        uint8_t exp[80]; size_t en = ref_frame(body, 7, exp);
        CHECK((size_t)n == en && memcmp(got, exp, en) == 0, "heartbeat bytes match reference");
        CHECK(got[0] == 0x7E && got[n-1] == 0x7E, "heartbeat starts/ends with flag");
    }

    /* 3) Traffic report: known lat/lon/alt packing + full-frame match. */
    {
        gdl90_traffic_t tr; memset(&tr, 0, sizeof(tr));
        tr.icao = 0xAB1234; tr.lat_deg = 44.90417; tr.lon_deg = -93.22; /* near KMSP */
        tr.alt_press_ft = 5000; tr.airborne = true; tr.nic = 8; tr.nacp = 9;
        tr.h_velocity_kt = 425; tr.v_velocity_fpm = 640; tr.track_heading = 90;
        tr.emitter_cat = 3; strcpy(tr.callsign, "DAL123"); tr.emergency_code = 0;

        uint8_t got[80]; int n = gdl90_frame_traffic_report(got, sizeof(got), &tr);
        CHECK(n > 0, "traffic frame returns positive length");

        /* Build the expected 28-byte body independently. */
        uint8_t b[28]; memset(b, 0, 28);
        b[0] = 0x14;
        b[1] = 0x00;
        b[2] = 0xAB; b[3] = 0x12; b[4] = 0x34;
        /* lat */
        int32_t la = (int32_t)lround(44.90417 * (8388608.0/180.0)) & 0xFFFFFF;
        b[5] = (la>>16)&0xFF; b[6] = (la>>8)&0xFF; b[7] = la&0xFF;
        int32_t lo = (int32_t)lround(-93.22 * (8388608.0/180.0)) & 0xFFFFFF;
        b[8] = (lo>>16)&0xFF; b[9] = (lo>>8)&0xFF; b[10] = lo&0xFF;
        /* alt code (5000+1000+12)/25 = 240 -> 0x0F0 */
        uint16_t alt = (5000+1000+12)/25;
        b[11] = (alt>>4)&0xFF; b[12] = (uint8_t)(((alt&0x0F)<<4) | 0x09);
        b[13] = (8<<4)|9;
        uint16_t hv = 425;
        int32_t vu = (640+32)/64; /* 10 */
        uint16_t vv = (uint16_t)(vu & 0xFFF);
        b[14] = (hv>>4)&0xFF; b[15] = (uint8_t)(((hv&0x0F)<<4) | ((vv>>8)&0x0F)); b[16] = vv&0xFF;
        b[17] = (uint8_t)((90*256+180)/360); /* 64 */
        b[18] = 3;
        const char *cs = "DAL123  ";
        for (int i=0;i<8;i++) b[19+i] = (uint8_t)cs[i];
        b[27] = 0x00;
        uint8_t exp[80]; size_t en = ref_frame(b, 28, exp);
        CHECK((size_t)n == en && memcmp(got, exp, en) == 0, "traffic bytes match reference");
    }

    /* 4) Byte-stuffing: craft a payload whose CRC or bytes include 0x7E/0x7D. */
    {
        gdl90_traffic_t tr; memset(&tr, 0, sizeof(tr));
        tr.icao = 0x7E7D7E; tr.alt_press_ft = INT32_MIN; tr.h_velocity_kt = 0xFFF;
        uint8_t got[80]; int n = gdl90_frame_traffic_report(got, sizeof(got), &tr);
        /* No interior byte (between the two flags) may be a raw 0x7E. */
        bool clean = true;
        for (int i = 1; i < n-1; ++i) if (got[i] == 0x7E) { clean = false; break; }
        CHECK(clean, "no interior raw 0x7E after stuffing");
        /* Every 0x7D must be followed by a stuffed (orig^0x20) byte. */
        bool good = true;
        for (int i = 1; i < n-1; ++i) {
            if (got[i] == 0x7D) {
                if (i+1 >= n-1) { good = false; break; }
                uint8_t orig = got[i+1] ^ 0x20;
                if (orig != 0x7E && orig != 0x7D) { good = false; break; }
                i++;
            }
        }
        CHECK(good, "every 0x7D escape is well-formed");
    }

    /* 5) Overflow returns a negative esp_err. */
    {
        gdl90_heartbeat_t hb; memset(&hb, 0, sizeof(hb));
        uint8_t tiny[3];
        int n = gdl90_frame_heartbeat(tiny, sizeof(tiny), &hb);
        CHECK(n < 0, "tiny buffer -> negative error");
    }

    printf("\n%s (%d failures)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
