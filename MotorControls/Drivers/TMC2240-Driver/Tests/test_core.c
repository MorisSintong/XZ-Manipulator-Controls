#include "model_core.h"
#include "model_cache.h"
#include "test_support.h"

#if TMC2240_CACHE && TMC2240_ENABLE_TMC_CACHE
#define TEST_ICS TMC2240_IC_CACHE_COUNT
#else
#define TEST_ICS 4U
#endif

static void fresh(void)
{
    core_model_reset();
    custom_cache_reset_faults();
    OK(tmc2240_initCache(TEST_ICS));
}

static void test_register_matrix(void)
{
    TMC2240RegisterInfo info;
    uint32_t data;
    fresh();
    STATUS(tmc2240_getRegisterInfo(0U, NULL), TMC2240_ERROR_ARGUMENT);
    CHECK(TMC2240_REGISTER_COUNT == 128U && TMC2240_ADDRESS_MASK == 127U);
    CHECK(TMC2240_WRITE_BIT == 128U && TMC2240_NODECONF == 3U && TMC2240_SLAVECONF == 3U);
    for (unsigned a = 0U; a < 256U; ++a) {
        const SpecRegister *expected = model_register((uint8_t)a);
        core_trace_reset();
        if (expected == NULL) {
            STATUS(tmc2240_getRegisterInfo((uint8_t)a, &info), TMC2240_ERROR_ADDRESS);
            STATUS(tmc2240_readRegister(0U, (uint8_t)a, &data), TMC2240_ERROR_ADDRESS);
            STATUS(tmc2240_writeRegister(0U, (uint8_t)a, 0U), TMC2240_ERROR_ADDRESS);
            STATUS(tmc2240_writeRegisterVerified(0U, (uint8_t)a, 0U), TMC2240_ERROR_ADDRESS);
            STATUS(tmc2240_updateRegister(0U, (uint8_t)a, 1U, 0U), TMC2240_ERROR_ADDRESS);
            CHECK(core_frame_count == 0U);
            continue;
        }
        OK(tmc2240_getRegisterInfo((uint8_t)a, &info));
        CHECK(info.read_mask == expected->readable && info.write_mask == expected->writable);
        CHECK(info.reset_value == expected->reset);
        CHECK(info.write_one_to_clear == (a == 1U || a == 0x3BU));
        CHECK((info.reset_value & ~info.reset_mask) == 0U);
        model_devices[0].reg[a] = UINT32_MAX;
        OK(tmc2240_readRegister(0U, (uint8_t)a, &data));
        CHECK(data == UINT32_MAX);
        CHECK(core_frame_count == 2U);
        if (expected->writable == 0U) {
            STATUS(tmc2240_writeRegister(0U, (uint8_t)a, 0U), TMC2240_ERROR_ACCESS);
            STATUS(tmc2240_updateRegister(0U, (uint8_t)a, 1U, 0U), TMC2240_ERROR_ACCESS);
        } else {
            OK(tmc2240_writeRegister(0U, (uint8_t)a, expected->reset & expected->writable));
            for (unsigned bit = 0U; bit < 32U; ++bit) {
                if ((expected->writable & (UINT32_C(1) << bit)) == 0U) {
                    unsigned before = core_frame_count;
                    STATUS(tmc2240_writeRegister(0U, (uint8_t)a, UINT32_C(1) << bit),
                           TMC2240_ERROR_RANGE);
                    CHECK(core_frame_count == before);
                }
            }
        }
    }
    for (unsigned gs = 0U; gs < 256U; ++gs) {
        TMC2240Status expected = gs != 0U && gs < 32U ? TMC2240_ERROR_RANGE : TMC2240_OK;
        STATUS(tmc2240_validateRegisterValue(0x0BU, gs), expected);
    }
    for (unsigned delay = 0U; delay < 16U; ++delay) {
        STATUS(tmc2240_validateRegisterValue(3U, delay << 8U),
               delay % 2U != 0U ? TMC2240_ERROR_RANGE : TMC2240_OK);
    }
    STATUS(tmc2240_validateRegisterValue(3U, 255U), TMC2240_ERROR_RANGE);
    for (unsigned mres = 0U; mres < 16U; ++mres) {
        STATUS(tmc2240_validateRegisterValue(0x6CU, mres << 24U),
               mres > 8U ? TMC2240_ERROR_RANGE : TMC2240_OK);
    }
    STATUS(tmc2240_validateRegisterValue(0x6CU, 1U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_validateRegisterValue(0x6CU, 0x8001U), TMC2240_ERROR_RANGE);
    OK(tmc2240_validateRegisterValue(0x6CU, 0x10001U));
    STATUS(tmc2240_validateRegisterValue(0x70U, 0x40000U), TMC2240_ERROR_RANGE);
    OK(tmc2240_validateRegisterValue(0x70U, 0x1040000U));
    OK(tmc2240_validateRegisterValue(0x70U, 0U));
}

static void test_fields(void)
{
    typedef struct {
        const char *name;
        RegisterField field;
        uint32_t mask;
        uint8_t shift, address;
        bool is_signed;
    } FieldCase;
    const FieldCase cases[] = {
#define F(n,m,s,a,b) {#n,TMC2240_##n##_FIELD,m,s,a,b},
#include "field_cases.h"
#undef F
    };
    int64_t extracted;
    uint32_t updated;
    fresh();
    for (size_t i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        const FieldCase *c = &cases[i];
        CHECK(c->field.mask == c->mask && c->field.shift == c->shift);
        CHECK(c->field.address == c->address && c->field.isSigned == c->is_signed);
        int64_t full = c->mask >> c->shift;
        int64_t max = c->is_signed ? full / 2 : full;
        int64_t min = c->is_signed ? -(max + 1) : 0;
        int64_t values[3] = { min, 0, max };
        for (unsigned j = 0U; j < 3U; ++j) {
            OK(tmc2240_fieldUpdate(0x55AA55AAU, c->field, values[j], &updated));
            uint32_t bits = (uint32_t)(((uint64_t)values[j] & (uint64_t)full) << c->shift);
            CHECK(updated == ((0x55AA55AAU & ~c->mask) | bits));
            OK(tmc2240_fieldExtract(updated, c->field, &extracted));
            CHECK(extracted == values[j]);
        }
        STATUS(tmc2240_fieldUpdate(0U, c->field, min - 1, &updated), TMC2240_ERROR_RANGE);
        STATUS(tmc2240_fieldUpdate(0U, c->field, max + 1, &updated), TMC2240_ERROR_RANGE);
        core_trace_reset();
        model_devices[0].reg[c->address] = c->mask;
        OK(tmc2240_fieldRead(0U, c->field, &extracted));
        CHECK(extracted == (c->is_signed ? -1 : full));
        CHECK(core_frame_count == 2U);
        /* Restore a valid base value for dependent chopper/PWM fields. */
        model_devices[0].reg[c->address] = model_register(c->address)->reset;
        core_trace_reset();
        if ((c->mask & ~model_register(c->address)->writable) != 0U) {
            STATUS(tmc2240_fieldWrite(0U, c->field, 0), TMC2240_ERROR_ACCESS);
            CHECK(core_frame_count == 0U);
        } else {
            int64_t valid = strcmp(c->name, "PWM_REG") == 0 ? 4 : 0;
            OK(tmc2240_fieldWrite(0U, c->field, valid));
            CHECK(core_frame_count == 1U || core_frame_count == 3U);
        }
    }
    const RegisterField malformed[] = {
        {0U,0U,0x39U,false}, {1U,32U,0x39U,false}, {0xEU,0U,0x39U,false},
        {5U,0U,0x39U,false}, {15U,1U,0x39U,false}, {1U,0U,0x7FU,false},
        {0x200U,9U,0x38U,false}
    };
    for (size_t i = 0U; i < sizeof(malformed) / sizeof(malformed[0]); ++i) {
        core_trace_reset();
        extracted = 123;
        updated = 456U;
        CHECK(tmc2240_fieldExtract(0U, malformed[i], &extracted) != TMC2240_OK);
        CHECK(tmc2240_fieldUpdate(0U, malformed[i], 0, &updated) != TMC2240_OK);
        CHECK(tmc2240_fieldRead(0U, malformed[i], &extracted) != TMC2240_OK);
        CHECK(tmc2240_fieldWrite(0U, malformed[i], 0) != TMC2240_OK);
        CHECK(extracted == 123 && updated == 456U && core_frame_count == 0U);
    }
    STATUS(tmc2240_fieldExtract(0U, TMC2240_X_ENC_FIELD, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_fieldUpdate(0U, TMC2240_X_ENC_FIELD, 0, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_fieldRead(0U, TMC2240_X_ENC_FIELD, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_updateRegister(0U, 0x39U, 0U, 0U), TMC2240_ERROR_ACCESS);
    STATUS(tmc2240_updateRegister(0U, 0x39U, 1U, 2U), TMC2240_ERROR_RANGE);
    core_trace_reset();
    STATUS(tmc2240_fieldWrite(0U, TMC2240_MRES_FIELD, 9), TMC2240_ERROR_RANGE);
    CHECK(core_frame_count == 0U);
}

static void test_protocol_and_failures(void)
{
    uint32_t value;
    int64_t field;
    static const uint8_t spi_write[] = {0xB9U,0x89U,0xABU,0xCDU,0xEFU};
    static const uint8_t spi_read[] = {0x39U,0U,0U,0U,0U};
    static const uint8_t uart_read_gconf[] = {0x05U,0x00U,0x00U,0x48U};
    fresh();
    OK(tmc2240_writeRegister(0U, 0x39U, 0x89ABCDEFU));
    CHECK(core_frame_count == 1U && memcmp(core_frames[0].bytes, spi_write, 5U) == 0);
    OK(tmc2240_readRegister(0U, 0x39U, &value));
    CHECK(value == 0x89ABCDEFU && core_frame_count == 3U);
    CHECK(memcmp(core_frames[1].bytes, spi_read, 5U) == 0);
    CHECK(memcmp(core_frames[2].bytes, spi_read, 5U) == 0);

    for (unsigned accepted = 0U; accepted < 2U; ++accepted) {
        for (unsigned phase = 1U; phase <= 2U; ++phase) {
            core_trace_reset();
            core_fail_at = phase;
            core_fail_after_accept = accepted != 0U;
            value = 0xCAFEBABEU;
            STATUS(tmc2240_readRegister(0U, 0x39U, &value), TMC2240_ERROR_TIMEOUT);
            CHECK(value == 0xCAFEBABEU && core_frame_count == phase);
            core_trace_reset();
            core_fail_at = phase;
            field = 123;
            STATUS(tmc2240_fieldRead(0U, TMC2240_CUR_A_FIELD, &field), TMC2240_ERROR_TIMEOUT);
            CHECK(field == 123 && core_frame_count == phase);
            core_trace_reset();
            core_fail_at = phase;
            uint32_t before = model_devices[0].writes;
            STATUS(tmc2240_fieldWrite(0U, TMC2240_SHAFT_FIELD, 1), TMC2240_ERROR_TIMEOUT);
            CHECK(core_frame_count == phase && model_devices[0].writes == before);
        }
        core_trace_reset();
        core_fail_at = 3U;
        STATUS(tmc2240_fieldWrite(0U, TMC2240_SHAFT_FIELD, 1), TMC2240_ERROR_TIMEOUT);
    }
    for (unsigned phase = 1U; phase <= 3U; ++phase) {
        core_trace_reset();
        core_fail_at = phase;
        STATUS(tmc2240_writeRegisterVerified(0U, 0x39U, 0x01020304U), TMC2240_ERROR_TIMEOUT);
        CHECK(core_frame_count == phase);
    }
    core_trace_reset();
    model_devices[0].ignore_write = true;
    STATUS(tmc2240_writeRegisterVerified(0U, 0x39U, 0x76543210U), TMC2240_ERROR_VERIFY);
    model_devices[0].ignore_write = false;
    OK(tmc2240_writeRegisterVerified(0U, 0x39U, UINT32_MAX));
    OK(tmc2240_readRegister(0U, 0x39U, &value));
    CHECK(value == UINT32_MAX);
    OK(tmc2240_writeRegisterVerified(0U, 0x39U, 0U));

    core_trace_reset();
    model_devices[0].reg[1] = 0x1FU;
    OK(tmc2240_fieldWrite(0U, TMC2240_RESET_FIELD, 1));
    CHECK(core_frame_count == 1U && model_devices[0].reg[1] == 0x1EU);
    CHECK(core_frames[0].bytes[0] == 0x81U && core_frames[0].bytes[4] == 1U);
    STATUS(tmc2240_writeRegisterVerified(0U, 1U, 1U), TMC2240_ERROR_ACCESS);
    model_devices[0].reg[0x3B] = 1U;
    OK(tmc2240_fieldWrite(0U, TMC2240_N_EVENT_FIELD, 0));
    CHECK(model_devices[0].reg[0x3B] == 1U);
    OK(tmc2240_fieldWrite(0U, TMC2240_N_EVENT_FIELD, 1));
    CHECK(model_devices[0].reg[0x3B] == 0U);

    core_trace_reset();
    core_buses[0] = IC_BUS_UART;
    OK(tmc2240_readRegister(0U, 0U, &value));
    CHECK(core_frame_count == 1U && memcmp(core_frames[0].bytes, uart_read_gconf, 4U) == 0);
    for (unsigned node = 0U; node < 255U; ++node) {
        core_nodes[0] = (uint8_t)node;
        core_trace_reset();
        OK(tmc2240_writeRegister(0U, 0x39U, node * 0x01010101U));
        OK(tmc2240_readRegister(0U, 0x39U, &value));
        CHECK(value == node * 0x01010101U);
        CHECK(core_frames[0].bytes[1] == node && core_frames[1].bytes[1] == node);
        CHECK(core_frame_count == 2U);
    }
    core_nodes[0] = 0U;
    for (int bit = 0; bit < 64; ++bit) {
        core_trace_reset();
        core_corrupt_bit = bit;
        value = 0x42U;
        CHECK(tmc2240_readRegister(0U, 0x39U, &value) != TMC2240_OK);
        CHECK(value == 0x42U);
    }
    core_trace_reset();
    core_reply_reserved = 0xA0U;
    /* Request reserved bits are don't-care; REPLY reserved bits are zero
     * (Rev.2 p.25, Table 6), even if a malformed reply has a correct CRC. */
    STATUS(tmc2240_readRegister(0U, 0x39U, &value), TMC2240_ERROR_PROTOCOL);
    core_trace_reset();
    core_fail_at = 1U;
    STATUS(tmc2240_readRegister(0U, 0x39U, &value), TMC2240_ERROR_TIMEOUT);
    core_trace_reset();
    core_fail_at = 1U;
    STATUS(tmc2240_writeRegister(0U, 0x39U, 0U), TMC2240_ERROR_TIMEOUT);
}

static void test_invalid_ids_and_callbacks(void)
{
    uint32_t value = 9U;
    TMC2240CacheEntry entry;
    fresh();
    STATUS(tmc2240_readRegister(0U, 0U, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_readRegister(UINT16_MAX, 0U, &value), TMC2240_ERROR_ID);
    STATUS(tmc2240_writeRegister(UINT16_MAX, 0U, 0U), TMC2240_ERROR_ID);
    STATUS(tmc2240_invalidateCache(UINT16_MAX), TMC2240_ERROR_ID);
    STATUS(tmc2240_getCachedRegister(UINT16_MAX, 0U, &entry), TMC2240_ERROR_ID);
    STATUS(tmc2240_getCachedRegister(0U, 0x7FU, &entry), TMC2240_ERROR_ADDRESS);
    STATUS(tmc2240_getCachedRegister(0U, 0U, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_initCache(0U), TMC2240_ERROR_ARGUMENT);
#if TMC2240_CACHE && TMC2240_ENABLE_TMC_CACHE
    STATUS(tmc2240_initCache(TMC2240_IC_CACHE_COUNT + 1U), TMC2240_ERROR_RANGE);
#endif
    CHECK(core_frame_count == 0U && value == 9U);
    core_buses[0] = IC_BUS_WLAN;
    STATUS(tmc2240_readRegister(0U, 0U, &value), TMC2240_ERROR_UNSUPPORTED);
    STATUS(tmc2240_writeRegister(0U, 0U, 0U), TMC2240_ERROR_UNSUPPORTED);
    core_bus_failure = TMC2240_ERROR_NOT_INITIALIZED;
    STATUS(tmc2240_readRegister(0U, 0U, &value), TMC2240_ERROR_NOT_INITIALIZED);
    STATUS(tmc2240_writeRegister(0U, 0U, 0U), TMC2240_ERROR_NOT_INITIALIZED);
    core_bus_failure = TMC2240_OK;
    core_count = 0U;
    STATUS(tmc2240_readRegister(0U, 0U, &value), TMC2240_ERROR_ID);
    core_count = 4U;
    core_buses[0] = IC_BUS_UART;
    core_node_failure = TMC2240_ERROR_IO;
    STATUS(tmc2240_readRegister(0U, 0U, &value), TMC2240_ERROR_IO);
    STATUS(tmc2240_writeRegister(0U, 0U, 0U), TMC2240_ERROR_IO);
    core_node_failure = TMC2240_OK;
    core_nodes[0] = 255U;
    STATUS(tmc2240_readRegister(0U, 0U, &value), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_writeRegister(0U, 0U, 0U), TMC2240_ERROR_RANGE);
    CHECK(core_frame_count == 0U);
}

static void test_cache(void)
{
    TMC2240CacheEntry entry;
    uint32_t actual;
    fresh();
#if TMC2240_CACHE
    for (uint16_t id = 0U; id < TEST_ICS; ++id) {
        core_trace_reset();
        OK(tmc2240_getCachedRegister(id, 0x60U, &entry));
        CHECK(!entry.desired_valid && !entry.confirmed_valid && !entry.dirty);
        OK(tmc2240_readRegister(id, 0x60U, &actual));
        CHECK(actual == 0xAAAAB554U && core_frame_count == 2U);
        OK(tmc2240_getCachedRegister(id, 0x60U, &entry));
        CHECK(!entry.desired_valid && entry.confirmed_valid && !entry.dirty);
        core_trace_reset();
        core_fail_at = 1U;
        STATUS(tmc2240_writeRegister(id, 0x60U, UINT32_MAX - id), TMC2240_ERROR_TIMEOUT);
        OK(tmc2240_getCachedRegister(id, 0x60U, &entry));
        CHECK(entry.desired == UINT32_MAX - id && entry.desired_valid);
        CHECK(!entry.confirmed_valid && entry.dirty);
        core_trace_reset();
        OK(tmc2240_writeRegister(id, 0x60U, UINT32_MAX - id));
        OK(tmc2240_getCachedRegister(id, 0x60U, &entry));
        CHECK(entry.dirty && !entry.confirmed_valid);
        OK(tmc2240_readRegister(id, 0x60U, &actual));
        OK(tmc2240_getCachedRegister(id, 0x60U, &entry));
        CHECK(entry.confirmed_valid && !entry.dirty && entry.confirmed == UINT32_MAX - id);
        OK(tmc2240_invalidateCache(id));
        OK(tmc2240_getCachedRegister(id, 0x60U, &entry));
        CHECK(!entry.confirmed_valid && entry.dirty && entry.desired_valid);
        OK(tmc2240_readRegister(id, 0x60U, &actual));
    }
    OK(tmc2240_initCache(1U));
    for (uint16_t id = 0U; id < TEST_ICS; ++id) {
        OK(tmc2240_getCachedRegister(id, 0x60U, &entry));
#if TMC2240_ENABLE_TMC_CACHE
        CHECK(!entry.desired_valid && !entry.confirmed_valid && !entry.dirty);
#else
        if (id == 0U) {
            CHECK(!entry.desired_valid);
        }
#endif
    }
    STATUS(tmc2240_cache(UINT16_MAX, TMC2240_CACHE_GET, 0U, &entry), TMC2240_ERROR_ID);
    STATUS(tmc2240_cache(0U, TMC2240_CACHE_GET, 0x80U, &entry), TMC2240_ERROR_ADDRESS);
    STATUS(tmc2240_cache(0U, TMC2240_CACHE_GET, 0x7FU, &entry), TMC2240_ERROR_ADDRESS);
    STATUS(tmc2240_cache(0U, TMC2240_CACHE_GET, 0U, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_cache(0U, (TMC2240CacheOp)99, 0U, &entry), TMC2240_ERROR_ARGUMENT);
    OK(tmc2240_cache(0U, TMC2240_CACHE_INVALIDATE, 0U, NULL));
    core_trace_reset();
    OK(tmc2240_writeRegisterVerified(0U, 0x39U, 17U));
    model_devices[0].reg[0x39] = 18U;
    OK(tmc2240_readRegister(0U, 0x39U, &actual));
    OK(tmc2240_getCachedRegister(0U, 0x39U, &entry));
    CHECK(entry.confirmed == 18U && entry.desired == 17U && entry.dirty);
    core_trace_reset();
    core_fail_at = 1U;
    STATUS(tmc2240_readRegister(0U, 0x39U, &actual), TMC2240_ERROR_TIMEOUT);
    OK(tmc2240_getCachedRegister(0U, 0x39U, &entry));
    CHECK(!entry.confirmed_valid && entry.dirty);
    core_trace_reset();
    OK(tmc2240_writeRegisterVerified(0U, 0x39U, 17U));
    model_reset_device(0U);
    OK(tmc2240_readRegister(0U, 1U, &actual));
    OK(tmc2240_getCachedRegister(0U, 0x39U, &entry));
    CHECK(!entry.confirmed_valid && entry.desired == 17U && entry.dirty);
    OK(tmc2240_getCachedRegister(0U, 1U, &entry));
    CHECK(!entry.desired_valid); /* W1C is a command, not a desired register image. */
    OK(tmc2240_writeRegister(0U, 1U, 0x1FU));
    OK(tmc2240_getCachedRegister(0U, 1U, &entry));
    CHECK(!entry.desired_valid && !entry.confirmed_valid);
#else
    STATUS(tmc2240_getCachedRegister(0U, 0U, &entry), TMC2240_ERROR_UNSUPPORTED);
    OK(tmc2240_invalidateCache(0U));
    OK(tmc2240_readRegister(0U, 0x60U, &actual));
    CHECK(actual == 0xAAAAB554U);
#endif

#if TMC2240_CACHE && !TMC2240_ENABLE_TMC_CACHE
    fresh();
    custom_cache_fail_op = TMC2240_CACHE_CLEAR;
    STATUS(tmc2240_initCache(4U), TMC2240_ERROR_IO);
    custom_cache_fail_op = TMC2240_CACHE_DESIRE;
    STATUS(tmc2240_writeRegister(0U, 0x39U, 123U), TMC2240_ERROR_IO);
    CHECK(core_frame_count == 0U);
    custom_cache_fail_op = TMC2240_CACHE_OBSERVE;
    actual = 42U;
    STATUS(tmc2240_readRegister(0U, 0x39U, &actual), TMC2240_ERROR_IO);
    CHECK(actual == 42U);
    custom_cache_fail_op = TMC2240_CACHE_GET;
    memset(&entry, 0, sizeof(entry));
    STATUS(tmc2240_getCachedRegister(0U, 0x39U, &entry), TMC2240_ERROR_IO);
    CHECK(entry.desired == 0U && entry.confirmed == 0U && !entry.dirty);
    custom_cache_fail_op = TMC2240_CACHE_RESET;
    STATUS(tmc2240_invalidateCache(0U), TMC2240_ERROR_IO);
    core_trace_reset();
    model_devices[0].status_pipeline = 1U;
    STATUS(tmc2240_readRegister(0U, 0U, &actual), TMC2240_ERROR_IO);
    CHECK(core_frame_count == 1U);
    core_buses[0] = IC_BUS_UART;
    model_devices[0].reg[1] = 8U;
    STATUS(tmc2240_readRegister(0U, 1U, &actual), TMC2240_ERROR_IO);
    custom_cache_fail_op = TMC2240_CACHE_INVALIDATE;
    core_trace_reset();
    core_fail_at = 1U;
    STATUS(tmc2240_readRegister(0U, 0U, &actual), TMC2240_ERROR_TIMEOUT);
    custom_cache_reset_faults();
#endif
}

static void test_seeded_properties(void)
{
    fresh();
    for (unsigned i = 0U; i < 100000U; ++i) {
        unsigned id = i % TEST_ICS;
        uint32_t word = test_random();
        uint32_t output = ~word;
        core_trace_reset();
        core_buses[id] = (i & 1U) != 0U ? IC_BUS_UART : IC_BUS_SPI;
        core_nodes[id] = (uint8_t)(i % 255U);
        OK(tmc2240_writeRegister((uint16_t)id, 0x39U, word));
        OK(tmc2240_readRegister((uint16_t)id, 0x39U, &output));
        CHECK(output == word);
        unsigned shift = test_random() % 32U;
        unsigned width = 1U + test_random() % (32U - shift);
        uint64_t modulus = UINT64_C(1) << width;
        uint32_t mask = (uint32_t)((modulus - 1U) << shift);
        bool sign = (test_random() & 1U) != 0U;
        RegisterField field = { mask, (uint8_t)shift, 0x39U, sign };
        int64_t expected = (word >> shift) & (modulus - 1U);
        if (sign && (uint64_t)expected >= modulus / 2U) {
            expected -= (int64_t)modulus;
        }
        int64_t actual = INT64_MAX;
        OK(tmc2240_fieldExtract(word, field, &actual));
        CHECK(actual == expected);
        uint32_t base = test_random();
        OK(tmc2240_fieldUpdate(base, field, actual, &output));
        CHECK((output & mask) == (word & mask));
        CHECK((output & ~mask) == (base & ~mask));
        STATUS(tmc2240_fieldUpdate(base, field,
            sign ? (int64_t)(modulus / 2U) : (int64_t)modulus, &output), TMC2240_ERROR_RANGE);
    }
}

static void test_recovery(void)
{
    fresh();
    uint32_t expected[4] = {0U};
    for (unsigned i = 0U; i < 10000U; ++i) {
        uint16_t id = (uint16_t)(i % TEST_ICS);
        uint32_t before = model_devices[id].reg[0x39];
        uint32_t after = test_random();
        uint32_t actual = 0xABCDU;
        core_trace_reset();
        core_fail_at = 1U + (i % 2U);
        core_fail_after_accept = (i % 3U) != 0U;
        STATUS(tmc2240_readRegister(id, 0x39U, &actual), TMC2240_ERROR_TIMEOUT);
        CHECK(actual == 0xABCDU && model_devices[id].reg[0x39] == before);
        core_trace_reset();
        OK(tmc2240_writeRegisterVerified(id, 0x39U, after));
        expected[id] = after;
        CHECK(model_devices[id].reg[0x39] == after);
#if TMC2240_CACHE
        TMC2240CacheEntry entry;
        OK(tmc2240_getCachedRegister(id, 0x39U, &entry));
        CHECK(entry.confirmed_valid && entry.desired_valid && !entry.dirty);
#endif
        for (unsigned other = 0U; other < TEST_ICS; ++other) {
            CHECK(model_devices[other].activations == 0U);
            CHECK(model_devices[other].reg[0x39] == expected[other]);
        }
    }
}

int main(void)
{
    test_register_matrix();
    test_fields();
    test_protocol_and_failures();
    test_invalid_ids_and_callbacks();
    test_cache();
    test_seeded_properties();
    test_recovery();
    test_summary("TMC2240 core", 100000U, 10000U);
    return EXIT_SUCCESS;
}
