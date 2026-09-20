#include "fake_core.h"
#include "test_support.h"
static test_counts_t counts;

static void start(tmc2209_bus_t *bus, tmc2209_unit_t *unit)
{
    core_reset();
    memset(bus, 0, sizeof(*bus));
    memset(unit, 0, sizeof(*unit));
    CHECK(core_bus_init(bus) == TMC2209_OK);
    CHECK(TMC2209_UnitInit(unit, bus, 0u) == TMC2209_OK);
    CHECK(core_fake.tx_calls == 0u);
}
static void finish(tmc2209_bus_t *bus, tmc2209_unit_t *unit)
{
    CHECK(TMC2209_UnitDeinit(unit) == TMC2209_OK);
    CHECK(TMC2209_BusDeinit(bus) == TMC2209_OK);
    CHECK(!core_fake.allocated && !core_fake.locked);
}

static void test_lifecycle(void)
{
    core_reset();
    tmc2209_bus_t bus = {0}, zero = {0};
    tmc2209_unit_t unit = {0}, other = {0};
    CHECK(TMC2209_BusInit(NULL, core_uart, 115200u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_BusInit(&bus, NULL, 115200u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_BusInit(&bus, core_uart, 0u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_BusInit(&bus, core_uart, 8999u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_BusInit(&bus, core_uart, 500001u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_BusDeinit(NULL) == TMC2209_OK);
    CHECK(TMC2209_BusDeinit(&bus) == TMC2209_OK);
    CHECK(TMC2209_UnitDeinit(NULL) == TMC2209_OK);
    CHECK(TMC2209_UnitDeinit(&unit) == TMC2209_OK);
    CHECK(TMC2209_UnitInit(NULL, &bus, 0) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_UnitInit(&unit, NULL, 0) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_UnitInit(&unit, &bus, 0) == TMC2209_ERR_PARAM);
    core_fake.init_result = TMC2209_ERR_PORT;
    core_fake.deinit_result = TMC2209_ERR_OS;
    CHECK(core_bus_init(&bus) == TMC2209_ERR_PORT);
    CHECK(core_fake.deinit_calls == 1u);
    CHECK(memcmp(&bus, &zero, sizeof(bus)) == 0);
    core_reset();
#if TMC2209_OS_CREATE_MUTEX
    core_fake.create_result = TMC2209_ERR_OS;
    core_fake.deinit_result = TMC2209_ERR_PORT;
    CHECK(core_bus_init(&bus) == TMC2209_ERR_OS);
    CHECK(core_fake.deinit_calls == 1u && core_fake.delete_calls == 0u);
    CHECK(memcmp(&bus, &zero, sizeof(bus)) == 0);
    core_reset();
#elif TMC2209_OS_FREERTOS
    CHECK(TMC2209_BusInit(&bus, core_uart, 115200u) == TMC2209_ERR_PARAM);
#endif
    CHECK(core_bus_init(&bus) == TMC2209_OK);
    CHECK(core_bus_init(&bus) == TMC2209_ERR_STATE);
    CHECK(bus.port.tx_timeout_ms == TMC2209_UART_TX_TIMEOUT_MS);
    CHECK(bus.port.rx_timeout_ms == TMC2209_UART_RX_TIMEOUT_MS);
    CHECK(bus.port.bus_idle_us == TMC2209_BUS_IDLE_US);
    for (unsigned a = 0u; a < 256u; ++a) {
        CHECK(TMC2209_UnitInit(&unit, &bus, (uint8_t)a) ==
              (a < 4u ? TMC2209_OK : TMC2209_ERR_PARAM));
        if (a < 4u) {
            CHECK(TMC2209_UnitInit(&other, &bus, (uint8_t)a) == TMC2209_ERR_STATE);
            CHECK(TMC2209_UnitInit(&unit, &bus, (uint8_t)a) == TMC2209_ERR_STATE);
            CHECK(TMC2209_BusDeinit(&bus) == TMC2209_ERR_BUSY);
            CHECK(TMC2209_UnitDeinit(&unit) == TMC2209_OK);
            CHECK(unit.bus == NULL && unit.desired_valid == 0u);
        }
    }
    CHECK(core_fake.tx_calls == 0u);
    core_fake.deinit_result = TMC2209_ERR_PORT;
    CHECK(TMC2209_BusDeinit(&bus) == TMC2209_ERR_PORT);
    CHECK(!bus.initialized && bus.port.huart == NULL && !core_fake.allocated);
    CHECK(TMC2209_BusDeinit(&bus) == TMC2209_OK);
    core_reset();
    int external = 0;
#if TMC2209_OS_FREERTOS
    CHECK(TMC2209_BusInitWithLock(&bus, core_uart, 115200u, NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_BusInitWithLock(&bus, core_uart, 115200u, &external) == TMC2209_OK);
    CHECK(bus.lock == &external);
#else
    CHECK(TMC2209_BusInitWithLock(&bus, core_uart, 115200u, &external) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_BusInitWithLock(&bus, core_uart, 115200u, NULL) == TMC2209_OK);
#endif
    CHECK(!bus.owns_lock && core_fake.create_calls == 0u);
    CHECK(TMC2209_BusDeinit(&bus) == TMC2209_OK);
    CHECK(core_fake.delete_calls == 0u);
}

static void test_raw_diagnostics(void)
{
    /* Rev.1.08 sections 5.1, 5.2, 5.4 and 5.5; not derived from driver metadata.
     * MSCURACT and PWM_SCALE samples include negative nine-bit encodings.
     */
    const struct { uint8_t address; uint32_t sample, field_mask; } diagnostics[] = {
        {0x05u, 0x00A15E37u, 0x00FFFFFFu},
        {0x07u, 0x00000215u, 0x0000031Fu},
        {0x12u, 0x000ABCDEu, 0x000FFFFFu},
        {0x6Au, 0x000003FFu, 0x000003FFu},
        {0x6Bu, 0x010101FFu, 0x01FF01FFu},
        {0x71u, 0x010100A5u, 0x01FF00FFu},
        {0x72u, 0x00AD0056u, 0x00FF00FFu}
    };
    const uint8_t named_registers[] = {
        TMC2209_REG_OTP_READ, TMC2209_REG_FACTORY_CONF, TMC2209_REG_TSTEP,
        TMC2209_REG_MSCNT, TMC2209_REG_MSCURACT, TMC2209_REG_PWM_SCALE,
        TMC2209_REG_PWM_AUTO
    };
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    start(&bus, &unit);
    for (uint8_t address = 0u; address < 4u; ++address) {
        if (address != 0u) {
            CHECK(TMC2209_UnitDeinit(&unit) == TMC2209_OK);
            CHECK(TMC2209_UnitInit(&unit, &bus, address) == TMC2209_OK);
        }
        unsigned char before[sizeof(unit)];
        memcpy(before, &unit, sizeof(unit));
        for (size_t i = 0u; i < sizeof(diagnostics) / sizeof(diagnostics[0]); ++i) {
            CHECK(named_registers[i] == diagnostics[i].address);
            const uint32_t values[] = {diagnostics[i].sample ^ address,
                                       diagnostics[i].field_mask, 0u, UINT32_MAX};
            uint32_t data = 0u;
            for (size_t j = 0u; j < sizeof(values) / sizeof(values[0]); ++j) {
                core_fake.regs[address][diagnostics[i].address] = values[j];
                unsigned locks = core_fake.lock_calls, unlocks = core_fake.unlock_calls;
                CHECK(TMC2209_ReadReg(&unit, diagnostics[i].address, &data) == TMC2209_OK);
                CHECK(data == values[j]);
                uint8_t request[] = {5u, address, diagnostics[i].address, 0u};
                request[3] = reference_crc(request, 3u);
                CHECK(core_fake.last_len == 4u);
                CHECK(memcmp(core_fake.last_frame, request, sizeof(request)) == 0);
                CHECK(core_fake.lock_calls == locks + 1u);
                CHECK(core_fake.unlock_calls == unlocks + 1u && !core_fake.locked);
            }
            unsigned tx = core_fake.tx_calls, rx = core_fake.rx_calls;
            const uint32_t writes[] = {0u, diagnostics[i].sample, UINT32_MAX};
            for (size_t j = 0u; j < sizeof(writes) / sizeof(writes[0]); ++j) {
                CHECK(TMC2209_WriteReg(&unit, diagnostics[i].address, writes[j]) == TMC2209_ERR_PARAM);
                CHECK(TMC2209_WriteRegVerified(&unit, diagnostics[i].address, writes[j]) == TMC2209_ERR_PARAM);
            }
            CHECK(core_fake.tx_calls == tx && core_fake.rx_calls == rx);
            CHECK(core_fake.regs[address][diagnostics[i].address] == UINT32_MAX);
            CHECK(memcmp(before, &unit, sizeof(unit)) == 0);
            data = 0xDEADBEEFu;
            core_fake.tx_result = TMC2209_ERR_TIMEOUT;
            core_fake.unlock_result = TMC2209_ERR_OS;
            CHECK(TMC2209_ReadReg(&unit, diagnostics[i].address, &data) == TMC2209_ERR_TIMEOUT);
            core_fake.tx_result = TMC2209_OK; core_fake.unlock_result = TMC2209_OK;
            CHECK(data == 0xDEADBEEFu && !core_fake.locked);
            core_fake.rx_result = TMC2209_ERR_TIMEOUT;
            CHECK(TMC2209_ReadReg(&unit, diagnostics[i].address, &data) == TMC2209_ERR_TIMEOUT);
            core_fake.rx_result = TMC2209_OK;
            CHECK(data == 0xDEADBEEFu && !core_fake.locked);
            core_fake.corrupt_crc = true;
            CHECK(TMC2209_ReadReg(&unit, diagnostics[i].address, &data) == TMC2209_ERR_CRC);
            core_fake.corrupt_crc = false; core_fake.reply_reg_xor = 0x80u;
            CHECK(TMC2209_ReadReg(&unit, diagnostics[i].address, &data) == TMC2209_ERR_NODEV);
            core_fake.reply_reg_xor = 0u;
            core_fake.lock_result = TMC2209_ERR_BUSY;
            CHECK(TMC2209_ReadReg(&unit, diagnostics[i].address, &data) == TMC2209_ERR_BUSY);
            core_fake.lock_result = TMC2209_OK;
            core_fake.unlock_result = TMC2209_ERR_OS;
            CHECK(TMC2209_ReadReg(&unit, diagnostics[i].address, &data) == TMC2209_ERR_OS);
            core_fake.unlock_result = TMC2209_OK;
            CHECK(data == 0xDEADBEEFu && !core_fake.locked);
            CHECK(memcmp(before, &unit, sizeof(unit)) == 0);
        }
    }
    CHECK(core_fake.write_calls == 0u && core_fake.active_writes == 0u);
    CHECK(TMC2209_WriteReg(&unit, 0x04u, 0xBD00u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WriteRegVerified(&unit, 0x04u, 0xBD00u) == TMC2209_ERR_PARAM);
    finish(&bus, &unit);
}

static void test_invalid_unit_and_access(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit, bad = {0};
    start(&bus, &unit);
    uint32_t data = 0xFEEDu;
    CHECK(TMC2209_WriteReg(NULL, 0, 0) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WriteReg(&bad, 0, 0) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_ReadReg(NULL, 0, &data) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_ReadReg(&unit, 0, NULL) == TMC2209_ERR_PARAM);
    bad = unit; bad.address = 4u;
    CHECK(TMC2209_ReadReg(&bad, 0, &data) == TMC2209_ERR_PARAM);
    bad.address = 1u;
    CHECK(TMC2209_ReadReg(&bad, 0, &data) == TMC2209_ERR_PARAM);
    bus.initialized = false;
    CHECK(TMC2209_ReadReg(&unit, 0, &data) == TMC2209_ERR_PARAM);
    bus.initialized = true;
    CHECK(data == 0xFEEDu);
    for (unsigned r = 0u; r < 256u; ++r) {
        bool writable = r == 0x00u || r == 0x01u || r == 0x03u || r == 0x10u ||
            r == 0x11u || r == 0x13u || r == 0x14u || r == 0x22u || r == 0x40u || r == 0x42u ||
            r == 0x6Cu || r == 0x70u;
        bool readable = r == 0u || r == 1u || r == 2u || r == 5u || r == 6u ||
            r == 7u || r == 0x12u || r == 0x41u || r == 0x6Au || r == 0x6Bu ||
            r == 0x6Cu || r == 0x6Fu || r == 0x70u || r == 0x71u || r == 0x72u;
        CHECK(TMC2209_WriteReg(&unit, (uint8_t)r, 0u) ==
              (writable ? TMC2209_OK : TMC2209_ERR_PARAM));
        CHECK(TMC2209_ReadReg(&unit, (uint8_t)r, &data) ==
              (readable ? TMC2209_OK : TMC2209_ERR_PARAM));
    }
    CHECK(TMC2209_WriteReg(&unit, 0, 1u << 9u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WriteCHOPCONF(&unit, 0x09000000u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WriteCHOPCONF(&unit, 1u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WriteCHOPCONF(&unit, 3u) == TMC2209_ERR_STATE);
    CHECK(TMC2209_WritePWMCONF(&unit, 1u << 19u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WritePWMCONF(&unit, 1u << 18u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WriteRegVerified(&unit, 0x80u, 0u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WriteReg(&unit, TMC2209_REG_VACTUAL, 1u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WriteReg(&unit, TMC2209_REG_VACTUAL, 0xFFFFFFu) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_WriteReg(&unit, TMC2209_REG_VACTUAL, 0x1000000u) == TMC2209_ERR_PARAM);
    finish(&bus, &unit);
    CHECK(TMC2209_ReadReg(&unit, 0, &data) == TMC2209_ERR_PARAM);
}

typedef tmc2209_status_t (*write_fn)(tmc2209_unit_t *, uint32_t);
static const struct { write_fn fn; uint8_t reg; uint32_t valid, mask; } writers[] = {
    {TMC2209_WriteGCONF, 0, 0x1FFu, 0x1FFu},
    {TMC2209_WriteSLAVECONF, 3, 0xF00u, 0xF00u},
    {TMC2209_WriteIHOLD_IRUN, 0x10, 0xF1F1Fu, 0xF1F1Fu},
    {TMC2209_WriteCHOPCONF, 0x6C, 0x18010050u, 0xFF0387FFu},
    {TMC2209_WritePWMCONF, 0x70, 0xFF3FFFFFu, 0xFF3FFFFFu},
    {TMC2209_WriteCOOLCONF, 0x42, 0xEF6Fu, 0xEF6Fu},
    {TMC2209_WriteTCOOLTHRS, 0x14, 0xFFFFFu, 0xFFFFFu},
    {TMC2209_WriteTPWMTHRS, 0x13, 0xFFFFFu, 0xFFFFFu},
    {TMC2209_WriteSGTHRS, 0x40, 0xFFu, 0xFFu},
    {TMC2209_WriteTPOWERDOWN, 0x11, 0xFFu, 0xFFu}
};

static void test_desired_mirrors(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    start(&bus, &unit);
    for (size_t i = 0u; i < sizeof(writers) / sizeof(writers[0]); ++i) {
        CHECK(writers[i].fn(NULL, 0) == TMC2209_ERR_PARAM);
        CHECK(writers[i].fn(&unit, UINT32_MAX) == TMC2209_ERR_STATE);
        core_fake.tx_result = TMC2209_ERR_TIMEOUT;
        CHECK(writers[i].fn(&unit, writers[i].valid) == TMC2209_ERR_TIMEOUT);
        CHECK(!core_fake.locked);
        core_fake.tx_result = TMC2209_OK;
        CHECK(writers[i].fn(&unit, UINT32_MAX) == TMC2209_OK);
        CHECK(core_fake.regs[0][writers[i].reg] == writers[i].valid);
        CHECK(core_fake.last_frame[2] == (writers[i].reg | 128u));
        for (unsigned bit = 0u; bit < 32u; ++bit) {
            if ((writers[i].mask & (1u << bit)) == 0u) {
                CHECK(writers[i].fn(&unit, 1u << bit) == TMC2209_ERR_PARAM);
                CHECK(writers[i].fn(&unit, UINT32_MAX) == TMC2209_OK);
                CHECK(core_fake.regs[0][writers[i].reg] == writers[i].valid);
            }
        }
        CHECK(TMC2209_WriteReg(&unit, writers[i].reg, 0u) == TMC2209_OK);
        CHECK(writers[i].fn(&unit, UINT32_MAX) == TMC2209_OK);
        CHECK(core_fake.regs[0][writers[i].reg] == 0u);
    }
    finish(&bus, &unit);
}

static void test_reads_and_failures(void)
{
    typedef tmc2209_status_t (*read_fn)(tmc2209_unit_t *, uint32_t *);
    const read_fn functions[] = {TMC2209_ReadIOIN, TMC2209_ReadIFCNT,
        TMC2209_ReadSG_RESULT, TMC2209_ReadGSTAT, TMC2209_ReadDRV_STATUS};
    const uint8_t regs[] = {6u, 2u, 0x41u, 1u, 0x6Fu};
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    start(&bus, &unit);
    for (unsigned i = 0u; i < 5u; ++i) {
        uint32_t value = 123u;
        CHECK(functions[i](NULL, &value) == TMC2209_ERR_PARAM);
        CHECK(functions[i](&unit, NULL) == TMC2209_ERR_PARAM);
        core_fake.rx_result = TMC2209_ERR_TIMEOUT;
        CHECK(functions[i](&unit, &value) == TMC2209_ERR_TIMEOUT);
        CHECK(value == 123u);
        core_fake.rx_result = TMC2209_OK;
        core_fake.regs[0][regs[i]] = UINT32_MAX;
        CHECK(functions[i](&unit, &value) == TMC2209_OK);
        CHECK(value == UINT32_MAX);
    }
    CHECK(unit.ioin.UINT32 == UINT32_MAX && unit.ifcnt == 255u);
    CHECK(unit.sg_result == 1023u && unit.gstat.UINT8 == 7u);
    CHECK(unit.drv_status.UINT32 == UINT32_MAX);
    CHECK(core_fake.regs[0][1] == UINT32_MAX); /* Observing GSTAT never acknowledges. */
    bool present = true;
    CHECK(TMC2209_Available(NULL, &present) == TMC2209_ERR_PARAM && !present);
    CHECK(TMC2209_Available(&unit, NULL) == TMC2209_ERR_PARAM);
    for (unsigned version = 0u; version < 256u; ++version) {
        core_fake.regs[0][6] = ((uint32_t)version << 24u) | 0x3DDu;
        CHECK(TMC2209_Available(&unit, &present) ==
              (version == 0x21u ? TMC2209_OK : TMC2209_ERR_NODEV));
        CHECK(present == (version == 0x21u));
    }
    uint32_t data = 0x12345678u;
    core_fake.lock_result = TMC2209_ERR_OS;
    CHECK(TMC2209_ReadReg(&unit, 6, &data) == TMC2209_ERR_OS);
    CHECK(TMC2209_WriteGCONF(&unit, 0xC0u) == TMC2209_ERR_OS);
    core_fake.lock_result = TMC2209_ERR_TIMEOUT;
    CHECK(TMC2209_ReadReg(&unit, 6, &data) == TMC2209_ERR_TIMEOUT);
    core_fake.lock_result = TMC2209_OK;
    core_fake.tx_result = TMC2209_ERR_TIMEOUT;
    core_fake.unlock_result = TMC2209_ERR_OS;
    CHECK(TMC2209_ReadReg(&unit, 6, &data) == TMC2209_ERR_TIMEOUT);
    CHECK(TMC2209_WriteGCONF(&unit, 0xC0u) == TMC2209_ERR_TIMEOUT);
    core_fake.tx_result = TMC2209_OK;
    CHECK(TMC2209_ReadReg(&unit, 6, &data) == TMC2209_ERR_OS);
    CHECK(TMC2209_WriteGCONF(&unit, 0xC0u) == TMC2209_ERR_OS);
    CHECK(data == 0x12345678u);
    core_fake.corrupt_crc = true;
    CHECK(TMC2209_ReadReg(&unit, 6, &data) == TMC2209_ERR_CRC);
    core_fake.corrupt_crc = false; core_fake.corrupt_identity = true;
    CHECK(TMC2209_ReadReg(&unit, 6, &data) == TMC2209_ERR_NODEV);
    core_fake.corrupt_identity = false; core_fake.reply_reg_xor = 0x80u;
    CHECK(TMC2209_ReadReg(&unit, 6, &data) == TMC2209_ERR_NODEV);
    CHECK(data == 0x12345678u && !core_fake.locked);
    core_fake.unlock_result = TMC2209_OK;
    finish(&bus, &unit);
}

static void test_verified_writes(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    start(&bus, &unit);
    core_fake.regs[0][2] = 255u;
    CHECK(TMC2209_WriteRegVerified(&unit, 0, 0xC0) == TMC2209_OK);
    CHECK(core_fake.regs[0][2] == 0u);
    CHECK(core_fake.lock_calls == 1u && core_fake.unlock_calls == 1u);
    CHECK(core_fake.tx_calls == 4u && core_fake.rx_calls == 3u);
    core_fake.drop_write_call = core_fake.write_calls + 1u;
    CHECK(TMC2209_WriteRegVerified(&unit, 0, 0xC1) == TMC2209_ERR_VERIFY);
    core_fake.drop_write_call = 0u; core_fake.extra_ifcnt = 1u;
    CHECK(TMC2209_WriteRegVerified(&unit, 0, 0xC1) == TMC2209_ERR_VERIFY);
    core_fake.extra_ifcnt = 0u; core_fake.bad_readback_reg = 0u;
    CHECK(TMC2209_WriteRegVerified(&unit, 0, 0xC0) == TMC2209_ERR_VERIFY);
    core_fake.bad_readback_reg = 256u;
    unsigned reads = core_fake.rx_calls;
    CHECK(TMC2209_WriteRegVerified(&unit, 0x10, 0x70404u) == TMC2209_OK);
    CHECK(core_fake.rx_calls - reads == 2u);
    core_fake.regs[0][1] = 7u;
    CHECK(TMC2209_ClearGSTAT(&unit, 1u) == TMC2209_OK);
    CHECK(core_fake.regs[0][1] == 6u);
    CHECK(TMC2209_ClearGSTAT(&unit, 2u) == TMC2209_OK);
    CHECK(core_fake.regs[0][1] == 4u);
    CHECK(TMC2209_ClearGSTAT(&unit, 4u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_ClearGSTAT(NULL, 1u) == TMC2209_ERR_PARAM);
    core_fake.lock_result = TMC2209_ERR_BUSY;
    CHECK(TMC2209_WriteRegVerified(&unit, 0, 0xC0) == TMC2209_ERR_BUSY);
    core_fake.lock_result = TMC2209_OK;
    finish(&bus, &unit);
    for (unsigned phase = 1u; phase <= 4u; ++phase) {
        start(&bus, &unit);
        core_fake.tx_result = TMC2209_ERR_PORT;
        core_fake.fail_tx_call = phase;
        core_fake.unlock_result = TMC2209_ERR_OS;
        CHECK(TMC2209_WriteRegVerified(&unit, 0, 0xC0) == TMC2209_ERR_PORT);
        CHECK(core_fake.tx_calls == phase && core_fake.unlock_calls == 1u);
        core_fake.unlock_result = TMC2209_OK;
        finish(&bus, &unit);
    }
    for (unsigned phase = 1u; phase <= 3u; ++phase) {
        start(&bus, &unit);
        core_fake.rx_result = TMC2209_ERR_TIMEOUT;
        core_fake.fail_rx_call = phase;
        CHECK(TMC2209_WriteRegVerified(&unit, 0, 0xC0) == TMC2209_ERR_TIMEOUT);
        CHECK(core_fake.rx_calls == phase && core_fake.unlock_calls == 1u);
        finish(&bus, &unit);
    }
    start(&bus, &unit);
    core_fake.unlock_result = TMC2209_ERR_OS;
    CHECK(TMC2209_WriteRegVerified(&unit, 0, 0xC0) == TMC2209_ERR_OS);
    core_fake.unlock_result = TMC2209_OK;
    finish(&bus, &unit);
}

static void test_config_and_activation(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    start(&bus, &unit);
    tmc2209_motor_config_t cfg = core_config();
    CHECK(TMC2209_Configure(NULL, &cfg) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Configure(&unit, NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Activate(NULL, 3) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Activate(&unit, 0) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Activate(&unit, 16) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Activate(&unit, 3) == TMC2209_ERR_STATE);
    CHECK(TMC2209_Deactivate(NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Deactivate(&unit) == TMC2209_ERR_STATE);
    for (unsigned fault = 0u; fault < 22u; ++fault) {
        tmc2209_motor_config_t bad = cfg;
        switch (fault) {
        case 0: bad.gconf |= 1u << 9u; break;
        case 1: bad.slaveconf |= 1u; break;
        case 2: bad.ihold_irun |= 1u << 5u; break;
        case 3: bad.chopconf |= 1u << 11u; break;
        case 4: bad.pwmconf |= 1u << 22u; break;
        case 5: bad.coolconf |= 1u << 4u; break;
        case 6: bad.tcoolthrs |= 1u << 20u; break;
        case 7: bad.tpwmthrs |= 1u << 20u; break;
        case 8: bad.sgthrs = 256u; break;
        case 9: bad.tpowerdown = 256u; break;
        case 10: bad.gconf = 0x80u; break;
        case 11: bad.gconf = 0x40u; break;
        case 12: bad.chopconf |= 3u; break;
        case 13: bad.slaveconf = 0u; break;
        case 14: bad.slaveconf = 0x100u; break;
        case 15: bad.gconf |= 0x100u; bad.chopconf |= 1u << 29u; break;
        case 16: bad.pwmconf &= ~(1u << 18u); break;
        case 17: bad.tpowerdown = 1u; break;
        case 18: bad.pwmconf &= ~(15u << 24u); break;
        case 19: bad.coolconf = 1u; bad.ihold_irun = 9u << 8u; break;
        case 20: bad.coolconf = 0x8001u; bad.ihold_irun = 19u << 8u; break;
        default: bad.chopconf = 9u << 24u; break;
        }
        CHECK(TMC2209_Configure(&unit, &bad) == TMC2209_ERR_PARAM);
        CHECK(core_fake.tx_calls == 0u && !unit.configured);
    }
    core_fake.regs[0][6] = 0x20000000u;
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_NODEV);
    CHECK(core_fake.write_calls == 0u);
    core_fake.regs[0][6] = 0x21000000u;
    core_fake.regs[0][0x22] = 0x100u; /* Prior owner used the internal pulse generator. */
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    CHECK(core_fake.regs[0][0x22] == 0u);
    CHECK(core_fake.active_writes == 0u && unit.configured);
    CHECK(core_fake.write_calls == 11u);
    core_fake.regs[0][6] = 0x20000000u;
    CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_ERR_NODEV);
    CHECK(!unit.configured && core_fake.active_writes == 0u);
    core_fake.regs[0][6] = 0x21000000u;
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    bool present = true;
    core_fake.regs[0][6] = 0x20000000u;
    CHECK(TMC2209_Available(&unit, &present) == TMC2209_ERR_NODEV);
    CHECK(!unit.configured && !present);
    core_fake.regs[0][6] = 0x21000000u;
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    for (uint8_t toff = 1u; toff <= 15u; ++toff) {
        CHECK(TMC2209_Activate(&unit, toff) == TMC2209_OK);
        CHECK((core_fake.regs[0][0x6C] & 15u) == toff);
        CHECK(unit.configured);
    }
    core_fake.corrupt_crc = true;
    CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_ERR_CRC);
    CHECK(!unit.configured);
    core_fake.corrupt_crc = false;
    CHECK(TMC2209_Deactivate(&unit) == TMC2209_OK);
    CHECK((core_fake.regs[0][0x6C] & 15u) == 0u && !unit.configured);
    CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_ERR_STATE);
    cfg.coolconf = 1u; cfg.ihold_irun = 10u << 8u;
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    cfg.coolconf = 0x8001u; cfg.ihold_irun = 20u << 8u;
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    core_fake.drop_write_call = core_fake.write_calls + 1u;
    CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_ERR_VERIFY);
    CHECK(!unit.configured);
    core_fake.drop_write_call = 0u;
    CHECK(TMC2209_Deactivate(&unit) == TMC2209_OK);
    cfg = core_config();
    cfg.pwmconf &= ~((1u << 18u) | (1u << 19u));
    cfg.gconf |= 0x100u;
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    CHECK(TMC2209_WriteGCONF(&unit, 0xC0u) == TMC2209_OK);
    CHECK(!unit.configured);
    cfg = core_config(); cfg.chopconf |= 1u << 29u;
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    finish(&bus, &unit);
}

static void test_small_field_domains(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    start(&bus, &unit);
    for (uint32_t mres = 0u; mres < 16u; ++mres)
        CHECK(TMC2209_WriteCHOPCONF(&unit, mres << 24u) ==
              (mres <= 8u ? TMC2209_OK : TMC2209_ERR_PARAM));
    for (uint32_t hend = 0u; hend < 16u; ++hend) {
        for (uint32_t hstrt = 0u; hstrt < 8u; ++hstrt) {
            int effective_hysteresis = ((int)hend - 3) + ((int)hstrt + 1);
            CHECK(TMC2209_WriteCHOPCONF(&unit, (hend << 7u) | (hstrt << 4u)) ==
                  (effective_hysteresis <= 16 ? TMC2209_OK : TMC2209_ERR_PARAM));
        }
    }
    for (uint32_t run = 0u; run < 32u; ++run) {
        for (uint32_t hold = 0u; hold < 32u; ++hold) {
            uint32_t data = (run << 8u) | hold;
            CHECK(TMC2209_WriteIHOLD_IRUN(&unit, data) == TMC2209_OK);
            CHECK(core_fake.regs[0][0x10] == data && unit.ihold_irun.UINT32 == data);
        }
    }
    for (uint32_t n = 0u; n < 256u; ++n) {
        CHECK(TMC2209_WriteSGTHRS(&unit, n) == TMC2209_OK);
        CHECK(unit.sgthrs == n);
        CHECK(TMC2209_WriteTPOWERDOWN(&unit, n) == TMC2209_OK);
        CHECK(unit.tpowerdown == n);
        core_fake.regs[0][2] = n;
        CHECK(TMC2209_WriteRegVerified(&unit, 3u, 0x200u) == TMC2209_OK);
        CHECK(core_fake.regs[0][2] == ((n + 1u) & 255u));
    }
    for (uint32_t n = 0u; n < 16u; ++n) {
        CHECK(TMC2209_WriteSLAVECONF(&unit, n << 8u) == TMC2209_OK);
        CHECK(TMC2209_WriteIHOLD_IRUN(&unit, n << 16u) == TMC2209_OK);
        CHECK(TMC2209_WriteCOOLCONF(&unit, n | (n << 8u)) == TMC2209_OK);
    }
    finish(&bus, &unit);
}

static void test_reset_after_configuration(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    start(&bus, &unit);
    tmc2209_motor_config_t cfg = core_config();
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    core_chip_reset(0u);
    CHECK(core_fake.regs[0][6] == 0x21000000u);
    CHECK(core_fake.regs[0][0x10] != cfg.ihold_irun);
    tmc2209_status_t st = TMC2209_Activate(&unit, 3u);
    printf("Reset-before-activation: status=%u active_writes=%u GSTAT=0x%08" PRIX32
           " current=0x%08" PRIX32 "\n", (unsigned)st, core_fake.active_writes,
           core_fake.regs[0][1], core_fake.regs[0][0x10]);
    CHECK(st == TMC2209_ERR_FAULT);
    CHECK(core_fake.active_writes == 0u && !unit.configured);
    CHECK((core_fake.regs[0][1] & 1u) != 0u);
    finish(&bus, &unit);
}

static tmc2209_status_t attempt_activation(tmc2209_unit_t *unit, unsigned method)
{
    uint32_t chop = (unit->chopconf.UINT32 & ~UINT32_C(15)) | 3u;
    switch (method) {
    case 0u: return TMC2209_Activate(unit, 3u);
    case 1u: return TMC2209_WriteCHOPCONF(unit, chop);
    case 2u: return TMC2209_WriteReg(unit, TMC2209_REG_CHOPCONF, chop);
    default: return TMC2209_WriteRegVerified(unit, TMC2209_REG_CHOPCONF, chop);
    }
}

static void test_fault_acknowledgement_order(void)
{
    tmc2209_motor_config_t cfg = core_config();
    for (unsigned path = 0u; path < 3u; ++path) {
        tmc2209_bus_t bus;
        tmc2209_unit_t unit;
        start(&bus, &unit);
        core_chip_reset(0u);
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_FAULT);
        CHECK(core_fake.write_calls == 0u && core_fake.gstat_writes == 0u);
        uint32_t status = 0u;
        CHECK(TMC2209_ReadGSTAT(&unit, &status) == TMC2209_OK && status == 1u);
        CHECK(TMC2209_ClearGSTAT(&unit, 1u) == TMC2209_OK);
        CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_ERR_STATE);
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
        core_chip_reset(0u);
        tmc2209_status_t st;
        if (path == 0u) st = TMC2209_ClearGSTAT(&unit, 1u);
        else if (path == 1u) st = TMC2209_WriteReg(&unit, 1u, 1u);
        else st = TMC2209_WriteRegVerified(&unit, 1u, 1u);
        CHECK(st == TMC2209_OK && !unit.configured);
        CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_ERR_STATE);
        CHECK(core_fake.active_writes == 0u && core_fake.gstat_writes == 2u);
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
        CHECK(core_fake.regs[0][0x10] == cfg.ihold_irun);
        CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_OK);
        CHECK(core_fake.regs[0][0x10] == cfg.ihold_irun);
        CHECK(core_fake.gstat_writes == 2u);
        core_fake.regs[0][1] |= 2u;
        core_fake.regs[0][0x6F] |= 4u;
        CHECK(TMC2209_ClearGSTAT(&unit, 2u) == TMC2209_OK);
        CHECK((core_fake.regs[0][1] & 2u) != 0u); /* Active cause cannot be acknowledged away. */
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_FAULT);
        CHECK(TMC2209_Deactivate(&unit) == TMC2209_OK);
        CHECK((core_fake.regs[0][0x6F] & 4u) == 0u);
        CHECK((core_fake.regs[0][1] & 2u) != 0u);
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_FAULT);
        CHECK(TMC2209_ClearGSTAT(&unit, 2u) == TMC2209_OK);
        core_chip_reset(0u); /* A new reset after acknowledgement must not be silently cleared. */
        unsigned acknowledgements = core_fake.gstat_writes;
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_FAULT);
        CHECK(core_fake.gstat_writes == acknowledgements && !unit.configured);
        finish(&bus, &unit);
    }
}

static void test_fault_masks_and_activation_paths(void)
{
    CHECK(TMC2209_GSTAT_FAULT_MASK == 7u);
    CHECK(TMC2209_DRV_STATUS_FAULT_MASK == 0x3Fu);
    tmc2209_motor_config_t cfg = core_config();
    for (unsigned fault = 0u; fault < 9u; ++fault) {
        uint32_t gst = fault < 3u ? 1u << fault : 0u;
        uint32_t drv = fault >= 3u ? 1u << (fault - 3u) : 0u;
        tmc2209_bus_t bus;
        tmc2209_unit_t unit;
        start(&bus, &unit);
        core_fake.regs[0][1] = gst;
        core_fake.regs[0][0x6F] = drv;
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_FAULT);
        CHECK(core_fake.write_calls == 0u && core_fake.gstat_writes == 0u);
        CHECK(core_fake.regs[0][1] == gst && core_fake.regs[0][0x6F] == drv);
        finish(&bus, &unit);
        for (unsigned method = 0u; method < 4u; ++method) {
            start(&bus, &unit);
            CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
            core_fake.regs[0][1] = gst;
            core_fake.regs[0][0x6F] = drv;
            CHECK(attempt_activation(&unit, method) == TMC2209_ERR_FAULT);
            CHECK(!unit.configured && core_fake.active_writes == 0u);
            CHECK(core_fake.gstat_writes == 0u);
            CHECK(core_fake.regs[0][1] == gst && core_fake.regs[0][0x6F] == drv);
            finish(&bus, &unit);
        }
    }
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    start(&bus, &unit);
    core_fake.regs[0][0x6F] = 0xC01F0FC0u; /* Informative flags are not activation faults. */
    core_fake.regs[0][1] = 0xFFFFFFF8u;    /* Reserved bits are ignored. */
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_OK);
    CHECK(core_fake.gstat_writes == 0u);
    finish(&bus, &unit);
}

static void arm_event(unsigned kind, unsigned boundary, unsigned phase)
{
    core_fake.event_reset = kind == 0u;
    core_fake.event_gstat = kind == 0u ? 0u : 2u;
    core_fake.event_drv_status = kind == 0u ? 0u : 4u;
    if (boundary == 0u) core_fake.event_before_tx = core_fake.tx_calls + phase;
    else if (boundary == 1u) core_fake.event_before_rx = core_fake.rx_calls + phase;
    else core_fake.event_after_rx = core_fake.rx_calls + phase;
}

static void test_configuration_event_boundaries(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    tmc2209_motor_config_t cfg = core_config();
    start(&bus, &unit);
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    unsigned tx_count = core_fake.tx_calls, rx_count = core_fake.rx_calls, completed = 0u;
    finish(&bus, &unit);
    for (unsigned kind = 0u; kind < 2u; ++kind) {
        for (unsigned boundary = 0u; boundary < 3u; ++boundary) {
            unsigned limit = boundary == 0u ? tx_count : rx_count;
            for (unsigned phase = 1u; phase <= limit; ++phase) {
                start(&bus, &unit);
                arm_event(kind, boundary, phase);
                tmc2209_status_t st = TMC2209_Configure(&unit, &cfg);
                CHECK(core_fake.event_count == 1u);
                if (st == TMC2209_OK) {
                    /* Event after the final configuration sample is caught at activation. */
                    CHECK(boundary == 2u && phase == rx_count);
                    CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_ERR_FAULT);
                }
                CHECK(!unit.configured && !core_fake.locked);
                CHECK(core_fake.active_writes == 0u && core_fake.gstat_writes == 0u);
                CHECK((core_fake.regs[0][1] & (kind == 0u ? 1u : 2u)) != 0u);
                finish(&bus, &unit);
                ++completed;
            }
        }
    }
    printf("Configuration event boundaries: %u reset/fault injections (%u TX, %u RX)\n",
           completed, tx_count, rx_count);
}

static void test_activation_event_boundaries(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    tmc2209_motor_config_t cfg = core_config();
    unsigned completed = 0u;
    for (unsigned method = 0u; method < 4u; ++method) {
        start(&bus, &unit);
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
        unsigned tx = core_fake.tx_calls, rx = core_fake.rx_calls;
        CHECK(attempt_activation(&unit, method) == TMC2209_OK);
        unsigned tx_count = core_fake.tx_calls - tx, rx_count = core_fake.rx_calls - rx;
        finish(&bus, &unit);
        for (unsigned kind = 0u; kind < 2u; ++kind) {
            for (unsigned boundary = 0u; boundary < 3u; ++boundary) {
                /* The final status sample ends the observable activation window. */
                unsigned limit = boundary == 0u ? tx_count :
                                 (boundary == 1u ? rx_count : rx_count - 1u);
                for (unsigned phase = 1u; phase <= limit; ++phase) {
                    start(&bus, &unit);
                    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
                    arm_event(kind, boundary, phase);
                    CHECK(attempt_activation(&unit, method) != TMC2209_OK);
                    CHECK(core_fake.event_count == 1u);
                    CHECK(!unit.configured && !core_fake.locked);
                    CHECK(core_fake.gstat_writes == 0u);
                    CHECK((core_fake.regs[0][1] & (kind == 0u ? 1u : 2u)) != 0u);
                    finish(&bus, &unit);
                    ++completed;
                }
            }
        }
    }
    printf("Activation event boundaries: %u reset/fault injections across four write APIs\n",
           completed);
}

static void test_activation_transport_failures(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    tmc2209_motor_config_t cfg = core_config();
    start(&bus, &unit);
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    unsigned tx = core_fake.tx_calls, rx = core_fake.rx_calls, locks = core_fake.lock_calls;
    CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_OK);
    unsigned totals[] = {core_fake.tx_calls - tx, core_fake.rx_calls - rx,
                          core_fake.lock_calls - locks, core_fake.lock_calls - locks};
    finish(&bus, &unit);
    unsigned completed = 0u;
    for (unsigned kind = 0u; kind < 4u; ++kind) {
        for (unsigned phase = 1u; phase <= totals[kind]; ++phase) {
            start(&bus, &unit);
            CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
            tmc2209_status_t expected = TMC2209_ERR_TIMEOUT;
            if (kind == 0u) {
                core_fake.tx_result = expected; core_fake.fail_tx_call = core_fake.tx_calls + phase;
            } else if (kind == 1u) {
                core_fake.rx_result = expected; core_fake.fail_rx_call = core_fake.rx_calls + phase;
            } else if (kind == 2u) {
                expected = TMC2209_ERR_BUSY;
                core_fake.lock_result = expected; core_fake.fail_lock_call = core_fake.lock_calls + phase;
            } else {
                expected = TMC2209_ERR_OS;
                core_fake.unlock_result = expected;
                core_fake.fail_unlock_call = core_fake.unlock_calls + phase;
            }
            CHECK(TMC2209_Activate(&unit, 3u) == expected);
            CHECK(!unit.configured && !core_fake.locked && core_fake.gstat_writes == 0u);
            core_fake.unlock_result = TMC2209_OK;
            finish(&bus, &unit);
            ++completed;
        }
    }
    printf("Activation transport/lock failure phases checked=%u\n", completed);
}

static void test_every_setup_failure(void)
{
    tmc2209_bus_t bus;
    tmc2209_unit_t unit;
    tmc2209_motor_config_t cfg = core_config();
    start(&bus, &unit);
    CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_OK);
    unsigned tx_count = core_fake.tx_calls, rx_count = core_fake.rx_calls;
    unsigned lock_count = core_fake.lock_calls;
    unsigned write_count = core_fake.write_calls;
    finish(&bus, &unit);
    for (unsigned phase = 1u; phase <= tx_count; ++phase) {
        start(&bus, &unit);
        core_fake.tx_result = TMC2209_ERR_TIMEOUT; core_fake.fail_tx_call = phase;
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_TIMEOUT);
        CHECK(core_fake.tx_calls == phase && core_fake.active_writes == 0u);
        CHECK(!unit.configured && !core_fake.locked);
        CHECK(TMC2209_Activate(&unit, 3u) == TMC2209_ERR_STATE);
        finish(&bus, &unit);
    }
    for (unsigned phase = 1u; phase <= rx_count; ++phase) {
        start(&bus, &unit);
        core_fake.rx_result = TMC2209_ERR_TIMEOUT; core_fake.fail_rx_call = phase;
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_TIMEOUT);
        CHECK(core_fake.rx_calls == phase && core_fake.active_writes == 0u);
        CHECK(!unit.configured && !core_fake.locked);
        finish(&bus, &unit);
    }
    for (unsigned phase = 1u; phase <= lock_count; ++phase) {
        start(&bus, &unit);
        core_fake.lock_result = TMC2209_ERR_BUSY; core_fake.fail_lock_call = phase;
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_BUSY);
        CHECK(!unit.configured && core_fake.lock_calls == phase);
        CHECK(core_fake.unlock_calls == phase - 1u);
        finish(&bus, &unit);
        start(&bus, &unit);
        core_fake.unlock_result = TMC2209_ERR_OS; core_fake.fail_unlock_call = phase;
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_OS);
        CHECK(!unit.configured && core_fake.unlock_calls == phase);
        core_fake.unlock_result = TMC2209_OK;
        finish(&bus, &unit);
    }
    for (unsigned phase = 1u; phase <= write_count; ++phase) {
        start(&bus, &unit);
        core_fake.drop_write_call = phase;
        CHECK(TMC2209_Configure(&unit, &cfg) == TMC2209_ERR_VERIFY);
        CHECK(core_fake.write_calls == phase && !unit.configured);
        CHECK(core_fake.active_writes == 0u);
        finish(&bus, &unit);
    }
    printf("Setup phase injection: %u TX, %u RX, %u locks, %u unlocks, %u acceptance drops\n",
           tx_count, rx_count, lock_count, lock_count, write_count);
}

static void test_register_layout(void)
{
#define FIELD(type, word, field, width, shift) do { \
    type v = {0}; v.REG.field = (1u << (width)) - 1u; \
    CHECK((uint32_t)v.word == (((1u << (width)) - 1u) << (shift))); \
} while (0)
    CHECK(sizeof(tmc2209_gconf_t) == 4u && sizeof(tmc2209_slaveconf_t) == 2u);
    CHECK(sizeof(tmc2209_gstat_t) == 1u && sizeof(tmc2209_ioin_t) == 4u);
    FIELD(tmc2209_gconf_t, UINT32, i_scale_analog, 1, 0);
    FIELD(tmc2209_gconf_t, UINT32, internal_rsense, 1, 1);
    FIELD(tmc2209_gconf_t, UINT32, enable_spread_cycle, 1, 2);
    FIELD(tmc2209_gconf_t, UINT32, shaft, 1, 3);
    FIELD(tmc2209_gconf_t, UINT32, index_otpw, 1, 4);
    FIELD(tmc2209_gconf_t, UINT32, index_step, 1, 5);
    FIELD(tmc2209_gconf_t, UINT32, pdn_disable, 1, 6);
    FIELD(tmc2209_gconf_t, UINT32, mstep_reg_select, 1, 7);
    FIELD(tmc2209_gconf_t, UINT32, multistep_filt, 1, 8);
    FIELD(tmc2209_gconf_t, UINT32, test_mode, 1, 9);
    FIELD(tmc2209_gstat_t, UINT8, reset, 1, 0);
    FIELD(tmc2209_gstat_t, UINT8, drv_err, 1, 1);
    FIELD(tmc2209_gstat_t, UINT8, uv_cp, 1, 2);
    FIELD(tmc2209_slaveconf_t, UINT16, senddelay, 4, 8);
    FIELD(tmc2209_ihold_irun_t, UINT32, ihold, 5, 0);
    FIELD(tmc2209_ihold_irun_t, UINT32, irun, 5, 8);
    FIELD(tmc2209_ihold_irun_t, UINT32, iholddelay, 4, 16);
    FIELD(tmc2209_ioin_t, UINT32, enn, 1, 0);
    FIELD(tmc2209_ioin_t, UINT32, ms1, 1, 2);
    FIELD(tmc2209_ioin_t, UINT32, ms2, 1, 3);
    FIELD(tmc2209_ioin_t, UINT32, diag, 1, 4);
    FIELD(tmc2209_ioin_t, UINT32, pdn_serial, 1, 6);
    FIELD(tmc2209_ioin_t, UINT32, step, 1, 7);
    FIELD(tmc2209_ioin_t, UINT32, spread_en, 1, 8);
    FIELD(tmc2209_ioin_t, UINT32, dir, 1, 9);
    FIELD(tmc2209_ioin_t, UINT32, version, 8, 24);
    FIELD(tmc2209_chopconf_t, UINT32, toff, 4, 0);
    FIELD(tmc2209_chopconf_t, UINT32, hstrt, 3, 4);
    FIELD(tmc2209_chopconf_t, UINT32, hend, 4, 7);
    FIELD(tmc2209_chopconf_t, UINT32, tbl, 2, 15);
    FIELD(tmc2209_chopconf_t, UINT32, vsense, 1, 17);
    FIELD(tmc2209_chopconf_t, UINT32, mres, 4, 24);
    FIELD(tmc2209_chopconf_t, UINT32, interpolation, 1, 28);
    FIELD(tmc2209_chopconf_t, UINT32, dedge, 1, 29);
    FIELD(tmc2209_chopconf_t, UINT32, diss2g, 1, 30);
    FIELD(tmc2209_chopconf_t, UINT32, diss2vs, 1, 31);
    FIELD(tmc2209_pwmconf_t, UINT32, pwm_offset, 8, 0);
    FIELD(tmc2209_pwmconf_t, UINT32, pwm_grad, 8, 8);
    FIELD(tmc2209_pwmconf_t, UINT32, pwm_freq, 2, 16);
    FIELD(tmc2209_pwmconf_t, UINT32, pwm_autoscale, 1, 18);
    FIELD(tmc2209_pwmconf_t, UINT32, pwm_autograd, 1, 19);
    FIELD(tmc2209_pwmconf_t, UINT32, freewheel, 2, 20);
    FIELD(tmc2209_pwmconf_t, UINT32, pwm_reg, 4, 24);
    FIELD(tmc2209_pwmconf_t, UINT32, pwm_lim, 4, 28);
    FIELD(tmc2209_coolconf_t, UINT32, semin, 4, 0);
    FIELD(tmc2209_coolconf_t, UINT32, seup, 2, 5);
    FIELD(tmc2209_coolconf_t, UINT32, semax, 4, 8);
    FIELD(tmc2209_coolconf_t, UINT32, sedn, 2, 13);
    FIELD(tmc2209_coolconf_t, UINT32, seimin, 1, 15);
    FIELD(tmc2209_thrs_t, UINT32, threshold, 20, 0);
    FIELD(tmc2209_drv_status_t, UINT32, otpw, 1, 0);
    FIELD(tmc2209_drv_status_t, UINT32, ot, 1, 1);
    FIELD(tmc2209_drv_status_t, UINT32, s2ga, 1, 2);
    FIELD(tmc2209_drv_status_t, UINT32, s2gb, 1, 3);
    FIELD(tmc2209_drv_status_t, UINT32, s2vsa, 1, 4);
    FIELD(tmc2209_drv_status_t, UINT32, s2vsb, 1, 5);
    FIELD(tmc2209_drv_status_t, UINT32, ola, 1, 6);
    FIELD(tmc2209_drv_status_t, UINT32, olb, 1, 7);
    FIELD(tmc2209_drv_status_t, UINT32, t120, 1, 8);
    FIELD(tmc2209_drv_status_t, UINT32, t143, 1, 9);
    FIELD(tmc2209_drv_status_t, UINT32, t150, 1, 10);
    FIELD(tmc2209_drv_status_t, UINT32, t157, 1, 11);
    FIELD(tmc2209_drv_status_t, UINT32, cs_actual, 5, 16);
    FIELD(tmc2209_drv_status_t, UINT32, stealth, 1, 30);
    FIELD(tmc2209_drv_status_t, UINT32, stst, 1, 31);
#undef FIELD
}

int main(void)
{
    RUN(test_lifecycle);
    RUN(test_raw_diagnostics);
    RUN(test_invalid_unit_and_access);
    RUN(test_desired_mirrors);
    RUN(test_reads_and_failures);
    RUN(test_verified_writes);
    RUN(test_config_and_activation);
    RUN(test_reset_after_configuration);
    RUN(test_fault_acknowledgement_order);
    RUN(test_fault_masks_and_activation_paths);
    RUN(test_configuration_event_boundaries);
    RUN(test_activation_event_boundaries);
    RUN(test_activation_transport_failures);
    RUN(test_every_setup_failure);
    RUN(test_small_field_domains);
    RUN(test_register_layout);
    SUMMARY("Production core");
    return EXIT_SUCCESS;
}
