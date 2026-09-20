#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>

typedef struct {
    uint64_t assertions;
    uint32_t tests;
    uint32_t case_index;
} test_counts_t;
#define CHECK(c) do { \
    ++counts.assertions; \
    if (!(c)) { \
        fprintf(stderr, "%s:%d case=%" PRIu32 ": %s\n", __FILE__, __LINE__, counts.case_index, #c); \
        exit(EXIT_FAILURE); \
    } \
} while (0)
#define RUN(f) do { ++counts.tests; f(); } while (0)
#define SUMMARY(name) printf("%s: %" PRIu32 " tests, %" PRIu64 " assertions PASS\n", \
                            name, counts.tests, counts.assertions)
#define REQUIRE(c) do { \
    if (!(c)) { \
        fprintf(stderr, "Dependency contract %s:%d: %s\n", __FILE__, __LINE__, #c); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

/* Independent polynomial-division oracle: reverse each UART byte, divide the
 * resulting polynomial by x^8+x^2+x+1. No production CRC helpers/constants.
 * Derived from Rev.1.08 §4.2 (LSB-first input, initial remainder zero).
 */
static inline uint8_t reference_crc(const uint8_t *bytes, size_t length)
{
    unsigned remainder = 0u;
    for (size_t i = 0u; i < length; ++i) {
        unsigned reflected = 0u;
        for (unsigned bit = 0u; bit < 8u; ++bit)
            reflected |= (((unsigned)bytes[i] >> bit) & 1u) << (7u - bit);
        remainder = (remainder ^ reflected) << 8u;
        for (unsigned bit = 16u; bit-- > 8u;)
            if ((remainder & (1u << bit)) != 0u) remainder ^= 0x107u << (bit - 8u);
    }
    return (uint8_t)remainder;
}

static inline void reference_reply(uint8_t reg, uint32_t value, uint8_t out[8])
{
    out[0] = 5u; out[1] = 255u; out[2] = reg;
    out[3] = (uint8_t)(value >> 24u); out[4] = (uint8_t)(value >> 16u);
    out[5] = (uint8_t)(value >> 8u); out[6] = (uint8_t)value;
    out[7] = reference_crc(out, 7u);
}

static inline uint32_t seeded_random(uint32_t *seed)
{
    uint32_t x = *seed;
    x ^= x << 13u; x ^= x >> 17u; x ^= x << 5u;
    *seed = x;
    return x;
}
#endif
