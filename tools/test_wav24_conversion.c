/*
 * test_wav24_conversion.c
 *
 * Exhaustively verifies the 24-bit -> 16-bit PCM conversion used by
 * DaisyPod3/main.cpp (LoadWavToPad's chunked reader and the QSPI/SD boot
 * loader) against an independently derived reference formula, across every
 * one of the 2^24 possible 24-bit little-endian sample values.
 *
 * Background: an earlier code review flagged this conversion as dropping
 * the LSB byte and sign-extending incorrectly. Hand analysis suggested the
 * shift-based formula already in main.cpp is in fact correct (dropping the
 * LSB is an unavoidable, intentional part of any 24->16 bit reduction; the
 * arithmetic right shift on a signed int32_t propagates the sign bit
 * correctly). This test settles it exhaustively instead of by argument.
 *
 * Build & run:
 *   gcc -O2 -Wall -o /tmp/test_wav24 tools/test_wav24_conversion.c && /tmp/test_wav24
 *
 * Exit code 0 and "ALL 16777216 CASES MATCH" means the production formula
 * is bit-exact against the reference for the entire 24-bit input space.
 */
#include <stdint.h>
#include <stdio.h>

/* ---- Verbatim from DaisyPod3/main.cpp (both call sites, mono path) ---- */
static int16_t ProductionConvert24to16(uint8_t s0, uint8_t s1, uint8_t s2)
{
    int32_t sample = (int32_t)(((uint32_t)s0 << 8) | ((uint32_t)s1 << 16)
                               | ((uint32_t)s2 << 24));
    sample >>= 16;
    return (int16_t)(sample < -32768 ? -32768
                     : (sample > 32767 ? 32767 : sample));
}

/* ---- Independent reference: build the natural 24-bit value, sign-extend
 * from bit 23 explicitly, then reduce with an arithmetic shift (floor
 * division by 256, matching audio truncation semantics — NOT C's
 * truncate-toward-zero integer division, which would wrongly disagree with
 * the production formula on negative non-multiples of 256). This is the
 * "canonical" form originally proposed as the fix. ---- */
static int16_t ReferenceConvert24to16(uint8_t s0, uint8_t s1, uint8_t s2)
{
    int32_t v = (int32_t)((uint32_t)s0 | ((uint32_t)s1 << 8)
                          | ((uint32_t)s2 << 16));
    if(v & 0x800000) v |= 0xFF000000u; /* sign-extend from bit 23 */
    int32_t sample = v >> 8;
    return (int16_t)(sample < -32768 ? -32768
                     : (sample > 32767 ? 32767 : sample));
}

int main(void)
{
    uint32_t mismatches = 0;
    uint32_t checked = 0;
    uint8_t s2 = 0;
    do {
        uint16_t s1 = 0;
        do {
            for(uint16_t s0 = 0; s0 < 256; ++s0){
                int16_t prod = ProductionConvert24to16((uint8_t)s0,
                                                        (uint8_t)s1, s2);
                int16_t ref = ReferenceConvert24to16((uint8_t)s0,
                                                      (uint8_t)s1, s2);
                checked++;
                if(prod != ref){
                    if(mismatches < 20)
                        fprintf(stderr,
                            "MISMATCH s0=%u s1=%u s2=%u -> "
                            "production=%d reference=%d\n",
                            (unsigned)s0, (unsigned)s1, (unsigned)s2,
                            prod, ref);
                    mismatches++;
                }
            }
            s1++;
        } while(s1 < 256);
        s2++;
    } while(s2 != 0); /* wraps after 255 -> covers all 256 values */

    /* Spot-check documented edge cases explicitly, with expected values
     * derived by hand (see the accompanying analysis): */
    struct { uint8_t s0, s1, s2; int16_t expect; const char* label; } cases[] = {
        {0x00, 0x00, 0x00, 0,      "zero"},
        {0xFF, 0xFF, 0xFF, -1,     "24-bit -1 (0xFFFFFF)"},
        {0x00, 0x00, 0x80, -32768, "24-bit min (0x800000)"},
        {0xFF, 0xFF, 0x7F, 32767,  "24-bit max (0x7FFFFF)"},
        {0x00, 0x00, 0x01, 256,    "0x010000 -> 256"},
        {0x80, 0x00, 0x00, 0,      "0x000080 truncates to 0"},
    };
    int edge_failures = 0;
    for(size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i){
        int16_t got = ProductionConvert24to16(cases[i].s0, cases[i].s1,
                                              cases[i].s2);
        if(got != cases[i].expect){
            fprintf(stderr, "EDGE CASE FAILED [%s]: got %d expected %d\n",
                   cases[i].label, got, cases[i].expect);
            edge_failures++;
        } else {
            printf("edge case OK  [%-28s] -> %d\n", cases[i].label, got);
        }
    }

    printf("\nChecked %u of 16777216 24-bit input combinations.\n", checked);
    if(mismatches == 0 && edge_failures == 0 && checked == 16777216u){
        printf("ALL 16777216 CASES MATCH — production formula is bit-exact "
               "with the reference for the full 24-bit input space.\n");
        return 0;
    }
    printf("FAILED: %u mismatches, %d edge-case failures, %u/%u checked.\n",
           mismatches, edge_failures, checked, 16777216u);
    return 1;
}
