/* Host unit test for the clean-room UAT Reed-Solomon FEC (../uat_fec.c).
 *
 * Strategy: the SAME module provides a systematic encoder, so each test encodes a
 * known message into a valid codeword, injects a controlled number of byte errors
 * at chosen positions, and asserts:
 *   - at t errors (t = (n-k)/2): EXACT recovery of the original message, and the
 *     reported corrected-count equals the number of injected errors;
 *   - at t+1 errors: the decoder reports uncorrectable (-1) OR miscorrects to
 *     something != the original (it must NOT silently return the wrong message as
 *     if correct). We require it to either fail or, if it "succeeds", to have
 *     actually restored the original — never a false success.
 * Plus: GF(256) table sanity, the all-clean (zero-error) fast path, and the full
 * 6-block uplink deinterleave round-trip. NOT part of the firmware build. */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "uat_fec.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); g_fail++; } else { printf("ok  : %s\n", msg); } } while (0)

/* A small deterministic PRNG so the test is reproducible across runs/platforms. */
static uint32_t rng_state = 0x1234abcdu;
static uint8_t rng_byte(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (uint8_t)(rng_state >> 24);
}
static int rng_below(int n)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (int)((rng_state >> 16) % (uint32_t)n);
}

/* Fill the first k bytes of a codeword with pseudo-random data, then encode. */
static void make_codeword(uint8_t *cw, int n, int k)
{
    for (int i = 0; i < k; i++) {
        cw[i] = rng_byte();
    }
    uat_fec_rs_encode(cw, n, k);
}

/* Inject `count` byte errors at DISTINCT positions, returning the original bytes
 * so the caller can verify exact recovery. */
static void inject_errors(uint8_t *cw, int n, int count)
{
    int used[92] = {0};
    int placed = 0;
    while (placed < count) {
        int p = rng_below(n);
        if (used[p]) continue;
        used[p] = 1;
        uint8_t e = rng_byte();
        if (e == 0) e = 1;          /* ensure a real error (non-zero XOR) */
        cw[p] ^= e;
        placed++;
    }
}

/* Run many encode→corrupt-at-t→decode trials for one code size. */
static void exercise_code(const char *name, int n, int k, int trials)
{
    int t = (n - k) / 2;
    int recover_fail = 0;
    int count_mismatch = 0;

    for (int trial = 0; trial < trials; trial++) {
        uint8_t orig[92], cw[92];
        make_codeword(orig, n, k);
        memcpy(cw, orig, (size_t)n);

        /* Inject exactly t errors at distinct positions. */
        int errs = t;
        /* vary the error count from 0..t to also exercise the easy cases */
        errs = rng_below(t + 1);
        inject_errors(cw, n, errs);

        int corrected = uat_fec_rs_decode(cw, n, k);
        if (corrected < 0) {
            recover_fail++;
            continue;
        }
        if (memcmp(cw, orig, (size_t)k) != 0) {
            recover_fail++;            /* data part not restored */
        }
        if (corrected != errs) {
            count_mismatch++;
        }
    }

    char buf[96];
    snprintf(buf, sizeof(buf), "%s: recovers all 0..t error patterns", name);
    CHECK(recover_fail == 0, buf);
    snprintf(buf, sizeof(buf), "%s: reported corrected-count == injected count", name);
    CHECK(count_mismatch == 0, buf);
}

/* Verify the decoder does not FALSELY succeed at t+1 errors: it must either
 * report -1, or (rarely) "succeed" only if it genuinely restored the original. */
static void exercise_beyond_t(const char *name, int n, int k, int trials)
{
    int t = (n - k) / 2;
    int false_success = 0;

    for (int trial = 0; trial < trials; trial++) {
        uint8_t orig[92], cw[92];
        make_codeword(orig, n, k);
        memcpy(cw, orig, (size_t)n);

        inject_errors(cw, n, t + 1);   /* one beyond the guarantee */

        int corrected = uat_fec_rs_decode(cw, n, k);
        if (corrected >= 0) {
            /* It claimed success — only acceptable if the data is truly restored. */
            if (memcmp(cw, orig, (size_t)k) != 0) {
                false_success++;
            }
        }
    }

    char buf[96];
    snprintf(buf, sizeof(buf), "%s: never FALSE-succeeds at t+1 errors", name);
    CHECK(false_success == 0, buf);
}

int main(void)
{
    uat_fec_init();

    /* 1) GF(256) sanity: alpha^0 = 1; the cycle has order 255 (alpha^255 == 1);
     *    a*inv(a) == 1 for all non-zero a. We exercise field arithmetic indirectly
     *    by round-tripping an all-zero-error codeword (must report 0 corrected). */
    {
        uint8_t cw[30];
        make_codeword(cw, 30, 18);
        int c = uat_fec_rs_decode(cw, 30, 18);
        CHECK(c == 0, "clean codeword decodes with 0 corrections");
    }

    /* 1b) A single deliberate 1-byte error in a known codeword is corrected and
     *     the exact original byte is restored. */
    {
        uint8_t orig[48], cw[48];
        make_codeword(orig, 48, 34);
        memcpy(cw, orig, 48);
        cw[5] ^= 0x5A;
        int c = uat_fec_rs_decode(cw, 48, 34);
        CHECK(c == 1 && memcmp(cw, orig, 34) == 0, "single-byte error corrected exactly");
    }

    /* 2) Full-strength trials for each UAT code size. */
    exercise_code("RS(30,18) basic ADS-B", 30, 18, 2000);
    exercise_code("RS(48,34) long ADS-B",  48, 34, 2000);
    exercise_code("RS(92,72) uplink block", 92, 72, 2000);

    /* 3) No false successes beyond the correction guarantee. */
    exercise_beyond_t("RS(30,18)", 30, 18, 2000);
    exercise_beyond_t("RS(48,34)", 48, 34, 2000);
    exercise_beyond_t("RS(92,72)", 92, 72, 2000);

    /* 4) Full uplink frame: build 6 valid RS(92,72) codewords, interleave them on
     *    the air the way the demod will see them, corrupt up to t per block, then
     *    deinterleave+decode and assert the 432-byte payload is restored. */
    {
        uint8_t blocks[6][92];
        uint8_t expect[432];
        for (int b = 0; b < 6; b++) {
            make_codeword(blocks[b], 92, 72);
            memcpy(&expect[b * 72], blocks[b], 72);
        }

        /* Interleave: on-air[i*6 + b] = blocks[b][i]. */
        uint8_t frame[552];
        for (int b = 0; b < 6; b++) {
            for (int i = 0; i < 92; i++) {
                frame[i * 6 + b] = blocks[b][i];
            }
        }
        /* Corrupt up to t=10 bytes in a couple of blocks (still correctable). */
        /* Block 2: 10 errors; block 5: 7 errors. Inject by transforming back to
         * on-air positions. */
        {
            int used2[92] = {0}, placed = 0;
            while (placed < 10) { int p = rng_below(92); if (used2[p]) continue; used2[p]=1;
                uint8_t e = rng_byte(); if(!e)e=1; frame[p*6 + 2] ^= e; placed++; }
            int used5[92] = {0}; placed = 0;
            while (placed < 7) { int p = rng_below(92); if (used5[p]) continue; used5[p]=1;
                uint8_t e = rng_byte(); if(!e)e=1; frame[p*6 + 5] ^= e; placed++; }
        }

        uint8_t out[432];
        int total = -1, blkerr = -1;
        bool ok = uat_fec_uplink_decode(frame, out, &total, &blkerr);
        CHECK(ok && blkerr == 0, "uplink: all 6 interleaved blocks decode");
        CHECK(memcmp(out, expect, 432) == 0, "uplink: 432-byte payload restored exactly");
        CHECK(total == 17, "uplink: total corrected count == 10 + 7");
    }

    /* 5) Uplink with an UNCORRECTABLE block (t+1 errors) is reported as a block
     *    failure rather than a silent pass. */
    {
        uint8_t blocks[6][92];
        for (int b = 0; b < 6; b++) make_codeword(blocks[b], 92, 72);
        uint8_t frame[552];
        for (int b = 0; b < 6; b++)
            for (int i = 0; i < 92; i++)
                frame[i * 6 + b] = blocks[b][i];
        /* 11 errors in block 0 — beyond t=10. */
        int used[92] = {0}, placed = 0;
        while (placed < 11) { int p = rng_below(92); if (used[p]) continue; used[p]=1;
            uint8_t e = rng_byte(); if(!e)e=1; frame[p*6 + 0] ^= e; placed++; }

        uint8_t out[432];
        int total = 0, blkerr = 0;
        bool ok = uat_fec_uplink_decode(frame, out, &total, &blkerr);
        CHECK(!ok && blkerr >= 1, "uplink: uncorrectable block flagged, not silent");
    }

    printf("\n%s (%d failures)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
