/* Host unit test for the GDL90 Uplink Data (0x07) encoder — the FIS-B weather
 * relay used by the 978 UAT path. Validates the framed wire bytes byte-for-byte
 * against an INDEPENDENT reference framer (its own CRC table + stuffing, written
 * here from the same public spec and kept separate from the production code so a
 * bug in one is caught by the other). NOT part of the firmware build. */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "gdl90_encoder.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); g_fail++; } else { printf("ok  : %s\n", msg); } } while (0)

/* ---- Independent reference CRC (same fold as the encoder, built separately) -- */
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

/* ---- Independent reference framing (large body: up to 436 + 2 CRC bytes) ----
 * Mirrors the production framer: CRC over the UN-stuffed body, then stuff the
 * body+CRC, wrapped in 0x7E flags. The output buffer must be sized for the
 * worst case (every byte stuffed) — the caller passes a generous buffer. */
static size_t ref_frame(const uint8_t *msg, size_t mlen, uint8_t *out)
{
    uint16_t crc = ref_crc16(msg, mlen);

    /* body = msg + CRC (little-endian), pre-stuffing. */
    uint8_t body[440];                 /* 436 max msg + 2 CRC, with headroom.    */
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

/* Build the expected UN-stuffed uplink body: id 0x07 + 3-byte big-endian Time of
 * Reception + payload. This is the reference for what the encoder must assemble. */
static size_t ref_uplink_body(uint32_t tor, const uint8_t *payload, size_t plen,
                              uint8_t *body)
{
    body[0] = 0x07;
    body[1] = (uint8_t)((tor >> 16) & 0xFF);
    body[2] = (uint8_t)((tor >> 8) & 0xFF);
    body[3] = (uint8_t)(tor & 0xFF);
    memcpy(&body[4], payload, plen);
    return 4 + plen;
}

int main(void)
{
    /* 1) Full 432-byte uplink with a non-trivial Time of Reception: the framed
     *    output must equal the independent reference byte-for-byte. */
    {
        uint8_t payload[GDL90_UPLINK_PAYLOAD_MAX];
        for (size_t i = 0; i < sizeof(payload); ++i) {
            payload[i] = (uint8_t)(i * 37 + 11);   /* deterministic, varied bytes */
        }
        uint32_t tor = 0x0ABCDE;                   /* 24-bit time of reception     */

        uint8_t got[GDL90_UPLINK_FRAME_MAX];
        int n = gdl90_frame_uplink(got, sizeof(got), tor, payload, sizeof(payload));
        CHECK(n > 0, "full uplink frame returns positive length");

        uint8_t body[4 + GDL90_UPLINK_PAYLOAD_MAX];
        size_t blen = ref_uplink_body(tor, payload, sizeof(payload), body);
        uint8_t exp[GDL90_UPLINK_FRAME_MAX];
        size_t en = ref_frame(body, blen, exp);

        CHECK((size_t)n == en && memcmp(got, exp, en) == 0,
              "full uplink bytes match independent reference");
        CHECK(got[0] == 0x7E && got[n - 1] == 0x7E,
              "uplink starts/ends with flag");
    }

    /* 2) The 24-bit Time of Reception is packed big-endian and masked to 24 bits
     *    (high byte of a 32-bit input is discarded). */
    {
        uint8_t payload[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
        uint32_t tor = 0xFF112233;                 /* top byte must be dropped     */
        uint8_t got[GDL90_UPLINK_FRAME_MAX];
        int n = gdl90_frame_uplink(got, sizeof(got), tor, payload, sizeof(payload));
        CHECK(n > 0, "small uplink frame returns positive length");
        /* Frame layout after the opening flag: [0]=0x7E, [1]=id 0x07, then the
         * 3 ToR bytes at [2..4]. None of 0x07/0x11/0x22/0x33 need stuffing, so
         * they appear verbatim. */
        CHECK(got[1] == 0x07 &&
              got[2] == 0x11 && got[3] == 0x22 && got[4] == 0x33,
              "Time of Reception packed big-endian, masked to 24 bits");
    }

    /* 3) Time of Reception = 0 (the "unknown" sentinel) is accepted and framed. */
    {
        uint8_t payload[16];
        memset(payload, 0xA5, sizeof(payload));
        uint8_t got[GDL90_UPLINK_FRAME_MAX];
        int n = gdl90_frame_uplink(got, sizeof(got), 0, payload, sizeof(payload));
        /* [1]=id 0x07, [2..4]=ToR all zero. */
        CHECK(n > 0 && got[1] == 0x07 &&
              got[2] == 0x00 && got[3] == 0x00 && got[4] == 0x00,
              "zero Time of Reception accepted");
    }

    /* 4) Byte-stuffing holds even for a payload full of raw flag/escape bytes. */
    {
        uint8_t payload[64];
        for (size_t i = 0; i < sizeof(payload); ++i) {
            payload[i] = (i & 1) ? 0x7E : 0x7D;    /* worst-case stuffing input    */
        }
        uint8_t got[GDL90_UPLINK_FRAME_MAX];
        int n = gdl90_frame_uplink(got, sizeof(got), 0x7E7D7E, payload, sizeof(payload));
        CHECK(n > 0, "flag-heavy uplink frames");

        bool clean = true;
        for (int i = 1; i < n - 1; ++i) if (got[i] == 0x7E) { clean = false; break; }
        CHECK(clean, "no interior raw 0x7E after stuffing");

        bool good = true;
        for (int i = 1; i < n - 1; ++i) {
            if (got[i] == 0x7D) {
                if (i + 1 >= n - 1) { good = false; break; }
                uint8_t orig = got[i + 1] ^ 0x20;
                if (orig != 0x7E && orig != 0x7D) { good = false; break; }
                i++;
            }
        }
        CHECK(good, "every 0x7D escape is well-formed");
    }

    /* 5) Argument / size guards: NULL out, NULL payload, zero len, oversized len,
     *    and a too-small destination all return a NEGATIVE esp_err. */
    {
        uint8_t payload[16] = {0};
        uint8_t got[GDL90_UPLINK_FRAME_MAX];
        CHECK(gdl90_frame_uplink(NULL, sizeof(got), 0, payload, 16) < 0,
              "NULL out -> negative error");
        CHECK(gdl90_frame_uplink(got, sizeof(got), 0, NULL, 16) < 0,
              "NULL payload -> negative error");
        CHECK(gdl90_frame_uplink(got, sizeof(got), 0, payload, 0) < 0,
              "zero length -> negative error");
        CHECK(gdl90_frame_uplink(got, sizeof(got), 0, payload,
                                 GDL90_UPLINK_PAYLOAD_MAX + 1) < 0,
              "oversized payload -> negative error");

        /* A 432-byte payload cannot fit in a 100-byte destination. */
        uint8_t full[GDL90_UPLINK_PAYLOAD_MAX]; memset(full, 0x7E, sizeof(full));
        uint8_t tiny[100];
        CHECK(gdl90_frame_uplink(tiny, sizeof(tiny), 0, full, sizeof(full)) < 0,
              "too-small destination -> negative error");
    }

    printf("\n%s (%d failures)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
