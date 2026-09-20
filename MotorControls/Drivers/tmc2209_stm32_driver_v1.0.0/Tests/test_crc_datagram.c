/** Links the production LL/core. The oracle is polynomial division, not a copy. */
#include "tmc2209.h"
#include "test_support.h"
static test_counts_t counts;

static void test_golden(void)
{
    static const uint8_t requests[][4] = {
        {5, 0, 0, 0x48}, {5, 0, 6, 0x6F}, {5, 0, 2, 0x8F}
    };
    static const uint8_t writes[][8] = {
        {5, 0, 0x80, 0, 0, 0, 0xC0, 0x40},
        {5, 2, 0xEC, 0x10, 0, 0, 0x53, 0xEA},
        {5, 1, 0x90, 0, 7, 4, 4, 0x3C}
    };
    static const uint32_t values[] = {0xC0u, 0x10000053u, 0x70404u};
    for (unsigned i = 0u; i < 3u; ++i) {
        uint8_t frame[8];
        CHECK(tmc2209_pack_request(0, requests[i][2], frame) == TMC2209_OK);
        CHECK(memcmp(frame, requests[i], 4u) == 0);
        CHECK(reference_crc(requests[i], 3u) == requests[i][3]);
        CHECK(tmc2209_pack_write(writes[i][1], (uint8_t)(writes[i][2] & 127u),
                                  values[i], frame) == TMC2209_OK);
        CHECK(memcmp(frame, writes[i], 8u) == 0);
        CHECK(reference_crc(writes[i], 7u) == writes[i][7]);
    }
    uint8_t reply[] = {5, 255, 6, 0x21, 0, 0, 0, 0x41};
    uint32_t value = 0;
    CHECK(tmc2209_unpack_reply(reply, 6, &value) == TMC2209_OK);
    CHECK(value == 0x21000000u);
    CHECK(reference_crc(reply, 7u) == 0x41u);
    for (unsigned a = 0; a < 256u; ++a) {
        uint8_t prefix[] = {5, (uint8_t)a};
        CHECK(tmc2209_crc_start((uint8_t)a) ==
              (a < 4u ? reference_crc(prefix, 2u) : 0u));
    }
    uint8_t prefix[] = {5, 255};
    CHECK(TMC2209_CRC_START_REPLY == reference_crc(prefix, 2u));
}

static void test_crc_finite_domain(void)
{
    for (unsigned first = 0u; first < 256u; ++first) {
        uint8_t bytes[] = {(uint8_t)first, 0u};
        uint8_t start = reference_crc(bytes, 1u);
        for (unsigned byte = 0u; byte < 256u; ++byte) {
            bytes[1] = (uint8_t)byte;
            CHECK(tmc2209_crc_byte((uint8_t)byte, start) == reference_crc(bytes, 2u));
        }
    }
}

static void test_parameters(void)
{
    uint8_t frame[8] = {0};
    uint32_t value = 0xBADC0DEu;
    CHECK(tmc2209_pack_write(0, 0, 0, NULL) == TMC2209_ERR_PARAM);
    CHECK(tmc2209_pack_request(0, 0, NULL) == TMC2209_ERR_PARAM);
    CHECK(tmc2209_unpack_reply(NULL, 0, &value) == TMC2209_ERR_PARAM);
    CHECK(tmc2209_unpack_reply(frame, 0, NULL) == TMC2209_ERR_PARAM);
    CHECK(tmc2209_unpack_reply(frame, 128, &value) == TMC2209_ERR_PARAM);
    CHECK(value == 0xBADC0DEu);
    for (unsigned a = 0; a < 256u; ++a) {
        CHECK(tmc2209_pack_write((uint8_t)a, 0, 0, frame) ==
              (a < 4u ? TMC2209_OK : TMC2209_ERR_PARAM));
        CHECK(tmc2209_pack_request((uint8_t)a, 0, frame) ==
              (a < 4u ? TMC2209_OK : TMC2209_ERR_PARAM));
    }
    for (unsigned reg = 0; reg < 256u; ++reg) {
        CHECK(tmc2209_pack_write(0, (uint8_t)reg, 0, frame) ==
              (reg < 128u ? TMC2209_OK : TMC2209_ERR_PARAM));
        CHECK(tmc2209_pack_request(0, (uint8_t)reg, frame) ==
              (reg < 128u ? TMC2209_OK : TMC2209_ERR_PARAM));
    }
}

static void test_all_single_bit_corruptions(void)
{
    for (unsigned addr = 0; addr < 4u; ++addr) {
        for (unsigned reg = 0; reg < 128u; ++reg) {
            uint8_t reply[8];
            reference_reply((uint8_t)reg, 0xA55A1234u ^ addr, reply);
            for (unsigned bit = 0; bit < 64u; ++bit) {
                uint32_t value = 0xDEADBEEFu;
                reply[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
                CHECK(tmc2209_unpack_reply(reply, (uint8_t)reg, &value) != TMC2209_OK);
                CHECK(value == 0xDEADBEEFu);
                reply[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
            }
        }
    }
}

static void test_seeded_properties(void)
{
    uint32_t seed = 0x2209C0DEu, completed = 0u;
    printf("Protocol property seed=0x%08" PRIX32 "\n", seed);
    for (counts.case_index = 0u; counts.case_index < 100000u; ++counts.case_index) {
        uint32_t value = seeded_random(&seed);
        uint8_t reg = (uint8_t)(seeded_random(&seed) & 127u);
        uint8_t addr = (uint8_t)(seeded_random(&seed) & 3u);
        uint8_t write[8], req[4], reply[8], payload[4];
        CHECK(tmc2209_pack_write(addr, reg, value, write) == TMC2209_OK);
        CHECK(write[0] == 5u && write[1] == addr && write[2] == (reg | 128u));
        CHECK(write[3] == (uint8_t)(value >> 24u) && write[4] == (uint8_t)(value >> 16u));
        CHECK(write[5] == (uint8_t)(value >> 8u) && write[6] == (uint8_t)value);
        CHECK(write[7] == reference_crc(write, 7u));
        memcpy(payload, &write[3], sizeof(payload));
        CHECK(tmc2209_crc_u32(value, 0u) == reference_crc(payload, sizeof(payload)));
        CHECK(tmc2209_pack_request(addr, reg, req) == TMC2209_OK);
        CHECK(req[0] == 5u && req[1] == addr && req[2] == reg);
        CHECK(req[3] == reference_crc(req, 3u));
        reference_reply(reg, value, reply);
        uint32_t decoded = 0u;
        CHECK(tmc2209_unpack_reply(reply, reg, &decoded) == TMC2209_OK);
        CHECK(decoded == value);
        ++completed;
    }
    CHECK(completed == 100000u);
    printf("Protocol properties completed=%" PRIu32 " final_state=0x%08" PRIX32 "\n",
           completed, seed);
}

int main(void)
{
    RUN(test_golden);
    RUN(test_crc_finite_domain);
    RUN(test_parameters);
    RUN(test_all_single_bit_corruptions);
    RUN(test_seeded_properties);
    SUMMARY("Production protocol");
    return EXIT_SUCCESS;
}
