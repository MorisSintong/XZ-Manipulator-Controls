#include "CRC.h"
#include "Functions.h"
#include "API_Header.h"
#include "test_support.h"

static uint8_t reference_crc(const uint8_t *data, size_t size, uint8_t polynomial,
                             bool reflected)
{
    unsigned accumulator = 0U;
    for (size_t i = 0U; i < size; ++i) {
        for (unsigned j = 0U; j < 8U; ++j) {
            unsigned bit = reflected ? j : 7U - j;
            unsigned feedback = ((accumulator / 128U) ^ (data[i] >> bit)) & 1U;
            accumulator = (accumulator * 2U) % 256U;
            if (feedback != 0U) {
                accumulator ^= polynomial;
            }
        }
    }
    return (uint8_t)accumulator;
}

static void test_crc(void)
{
    uint8_t output = 0xA5U;
    uint8_t poly = 0x5AU;
    bool reflect = true;
    uint8_t bytes[32];
    STATUS(tmc_CRC8(NULL, 0U, 0U, &output), TMC2240_ERROR_NOT_INITIALIZED);
    STATUS(tmc_tableGetPolynomial(0U, &poly), TMC2240_ERROR_NOT_INITIALIZED);
    STATUS(tmc_tableIsReflected(0U, &reflect), TMC2240_ERROR_NOT_INITIALIZED);
    STATUS(tmc_CRC8(NULL, 1U, 0U, &output), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc_CRC8(bytes, 1U, 0U, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc_CRC8(NULL, 0U, CRC_TABLE_COUNT, &output), TMC2240_ERROR_RANGE);
    STATUS(tmc_tableGetPolynomial(0U, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc_tableIsReflected(0U, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc_tableGetPolynomial(CRC_TABLE_COUNT, &poly), TMC2240_ERROR_RANGE);
    STATUS(tmc_tableIsReflected(CRC_TABLE_COUNT, &reflect), TMC2240_ERROR_RANGE);
    STATUS(tmc_fillCRC8Table(7U, true, CRC_TABLE_COUNT), TMC2240_ERROR_RANGE);
    CHECK(output == 0xA5U && poly == 0x5AU && reflect);

    for (unsigned polynomial = 0U; polynomial < 256U; ++polynomial) {
        for (unsigned reflected = 0U; reflected < 2U; ++reflected) {
            uint8_t index = (uint8_t)(polynomial % CRC_TABLE_COUNT);
            OK(tmc_fillCRC8Table((uint8_t)polynomial, reflected != 0U, index));
            OK(tmc_tableGetPolynomial(index, &poly));
            OK(tmc_tableIsReflected(index, &reflect));
            CHECK(poly == polynomial && reflect == (reflected != 0U));
            for (unsigned b = 0U; b < 256U; ++b) {
                bytes[0] = (uint8_t)b;
                OK(tmc_CRC8(bytes, 1U, index, &output));
                CHECK(output == reference_crc(bytes, 1U, (uint8_t)polynomial, reflect));
            }
            for (unsigned length = 0U; length <= sizeof(bytes); ++length) {
                for (unsigned i = 0U; i < length; ++i) {
                    bytes[i] = (uint8_t)test_random();
                }
                OK(tmc_CRC8(bytes, length, index, &output));
                CHECK(output == reference_crc(bytes, length, (uint8_t)polynomial, reflect));
            }
            OK(tmc_CRC8(NULL, 0U, index, &output));
            CHECK(output == 0U);
        }
    }
    static const uint8_t golden[] = {0x05U,0x00U,0x00U};
    OK(tmc_fillCRC8Table(7U, true, 0U));
    OK(tmc_CRC8(golden, sizeof(golden), 0U, &output));
    CHECK(output == 0x48U);
}

static uint32_t reference_sqrt(uint32_t value)
{
    uint32_t lo = 0U, hi = 46341U;
    while (lo + 1U < hi) {
        uint32_t mid = lo + (hi - lo) / 2U;
        if ((uint64_t)mid * mid > value) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    return lo;
}

static void test_math(void)
{
    CHECK(tmc_limitInt(INT32_MIN, -10, 10) == -10);
    CHECK(tmc_limitInt(INT32_MAX, -10, 10) == 10);
    CHECK(tmc_limitInt(0, -10, 10) == 0);
    CHECK(tmc_limitInt(10, 10, 10) == 10);
    CHECK(tmc_limitInt(INT32_MAX, INT32_MIN, INT32_MAX) == INT32_MAX);
    CHECK(tmc_limitS64(INT64_MIN, -10, 10) == -10);
    CHECK(tmc_limitS64(INT64_MAX, -10, 10) == 10);
    CHECK(tmc_limitS64(0, -10, 10) == 0);
    CHECK(tmc_limitS64(10, 10, 10) == 10);
    CHECK(tmc_limitS64(INT64_MAX, INT64_MIN, INT64_MAX) == INT64_MAX);
    CHECK(tmc_sqrti(-1) == -1 && tmc_sqrti(INT32_MIN) == -1);
    for (int32_t i = 0; i < 65536; ++i) {
        CHECK(tmc_sqrti(i) == (int32_t)reference_sqrt((uint32_t)i));
    }
    for (unsigned bit = 16U; bit < 31U; ++bit) {
        for (int32_t offset = -1; offset <= 1; ++offset) {
            int32_t input = (int32_t)(1U << bit) + offset;
            CHECK(tmc_sqrti(input) == (int32_t)reference_sqrt((uint32_t)input));
        }
    }
    CHECK(tmc_sqrti(INT32_MAX) == 46340);
    for (unsigned i = 0U; i < 100000U; ++i) {
        uint32_t input = test_random() & INT32_MAX;
        int32_t actual = tmc_sqrti((int32_t)input);
        CHECK(actual == (int32_t)reference_sqrt(input));
    }
}

static void test_filter(void)
{
    int64_t accumulator = 0;
    int32_t output = 123;
    STATUS(tmc_filterPT1(NULL, 0, 0, 0U, 0U, &output), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc_filterPT1(&accumulator, 0, 0, 0U, 0U, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc_filterPT1(&accumulator, 0, 0, 0U, 32U, &output), TMC2240_ERROR_RANGE);
    STATUS(tmc_filterPT1(&accumulator, 0, 0, 1U, 0U, &output), TMC2240_ERROR_RANGE);
    CHECK(accumulator == 0 && output == 123);
    OK(tmc_filterPT1(&accumulator, 100, 0, 2U, 8U, &output));
    CHECK(accumulator == 6400 && output == 25);
    OK(tmc_filterPT1(&accumulator, -100, 25, 2U, 8U, &output));
    CHECK(accumulator == -1600 && output == -7);
    accumulator = -4;
    OK(tmc_filterPT1(&accumulator, 0, 0, 1U, 1U, &output));
    CHECK(output == -2);
    accumulator = INT64_MAX;
    output = 55;
    STATUS(tmc_filterPT1(&accumulator, 1, 0, 0U, 0U, &output), TMC2240_ERROR_RANGE);
    CHECK(accumulator == INT64_MAX && output == 55);
    accumulator = INT64_MIN;
    STATUS(tmc_filterPT1(&accumulator, -1, 0, 0U, 0U, &output), TMC2240_ERROR_RANGE);
    CHECK(accumulator == INT64_MIN);
    STATUS(tmc_filterPT1(&accumulator, 0, 0, 31U, 31U, &output), TMC2240_ERROR_RANGE);
    accumulator = INT64_MAX;
    STATUS(tmc_filterPT1(&accumulator, 0, 0, 31U, 31U, &output), TMC2240_ERROR_RANGE);
    accumulator = 0;
    STATUS(tmc_filterPT1(&accumulator, INT32_MAX, INT32_MIN, 0U, 31U, &output),
           TMC2240_ERROR_RANGE);
    STATUS(tmc_filterPT1(&accumulator, INT32_MIN, INT32_MAX, 0U, 31U, &output),
           TMC2240_ERROR_RANGE);
    CHECK(accumulator == 0 && output == 55);
    OK(tmc_filterPT1(&accumulator, INT32_MAX, INT32_MIN, 31U, 31U, &output));
    CHECK(accumulator == INT64_C(4294967295) && output == 1);
    accumulator = 0;
    OK(tmc_filterPT1(&accumulator, INT32_MIN, 0, 0U, 31U, &output));
    CHECK(output == INT32_MIN);
    accumulator = 0;
    OK(tmc_filterPT1(&accumulator, INT32_MAX, 0, 0U, 31U, &output));
    CHECK(output == INT32_MAX);
}

static void test_macros(void)
{
    uint8_t large_array[300];
    CHECK(ARRAY_SIZE(large_array) == 300U);
    CHECK(MIN(1,2) == 1 && MAX(1,2) == 2);
    CHECK(FIELD_GET(0x12345678U,0xFF00U,8U) == 0x56U);
    CHECK(FIELD_SET(0x12345678U,0xFF00U,8U,0xABU) == 0x1234AB78U);
    CHECK(TMC_ADDRESS(0xFFU) == 127U);
    for (unsigned width = 1U; width <= 32U; ++width) {
        uint64_t modulus = UINT64_C(1) << width;
        for (unsigned sample = 0U; sample < 256U; ++sample) {
            uint32_t word = test_random();
            int64_t expected = word & (modulus - 1U);
            if ((uint64_t)expected >= modulus / 2U) {
                expected -= (int64_t)modulus;
            }
            CHECK(CAST_Sn_TO_S32(word,width) == expected);
        }
    }
    CHECK(COMBINE_8_16(0x12U,0x34U) == 0x1234U);
    CHECK(COMBINE_8_32(0x89U,0xABU,0xCDU,0xEFU) == 0x89ABCDEFU);
    CHECK(COMBINE_16_32(0x89ABU,0xCDEFU) == 0x89ABCDEFU);
    for (unsigned n = 0U; n < 16U; ++n) {
        CHECK(NIBBLE(UINT64_C(0x0123456789ABCDEF),n) == 15U-n);
    }
    for (unsigned n = 0U; n < 8U; ++n) {
        CHECK(BYTE(UINT64_C(0x0706050403020100),n) == n);
    }
    for (unsigned n = 0U; n < 4U; ++n) {
        CHECK(SHORT(UINT64_C(0x0003000200010000),n) == n);
    }
    CHECK(WORD(UINT64_C(0x0123456789ABCDEF),0U) == 0x89ABCDEFU);
    CHECK(WORD(UINT64_C(0x0123456789ABCDEF),1U) == 0x01234567U);
    for (unsigned permission = 0U; permission < 256U; ++permission) {
        CHECK(TMC_IS_READABLE(permission) == ((permission & 1U) != 0U));
        CHECK(TMC_IS_WRITABLE(permission) == ((permission & 2U) != 0U));
        CHECK(TMC_IS_DIRTY(permission) == ((permission & 8U) != 0U));
        CHECK(TMC_IS_PRESET(permission) == ((permission & 64U) != 0U));
        CHECK(TMC_IS_RESETTABLE(permission) ==
              (((permission & 2U) != 0U) && ((permission & 64U) == 0U)));
        CHECK(TMC_IS_RESTORABLE(permission) == (((permission & 2U) != 0U) &&
              (((permission & 64U) == 0U) || ((permission & 8U) != 0U))));
    }
}

int main(void)
{
    test_macros();
    test_crc();
    test_math();
    test_filter();
    test_summary("TMC2240 shared helpers", 100000U, 0U);
    return EXIT_SUCCESS;
}
