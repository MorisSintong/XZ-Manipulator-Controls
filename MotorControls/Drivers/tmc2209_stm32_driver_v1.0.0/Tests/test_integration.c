#include "tmc2209.h"
#include "fake_hal.h"
#include "test_support.h"
#if TMC2209_OS_FREERTOS
#include "fake_rtos.h"
#endif
static test_counts_t counts;
#if TMC2209_OS_FREERTOS && !TMC2209_OS_CREATE_MUTEX
static fake_semaphore_t supplied_mutex;
#endif

static tmc2209_motor_config_t config(void)
{
    tmc2209_motor_config_t cfg = {0xC0u, 0x200u, 0x71408u, 0x14010050u,
                                  0xC10D0024u, 0u, 0u, 0u, 0u, 20u};
    return cfg;
}
static void bind_all(tmc2209_bus_t *bus, tmc2209_unit_t units[4], uint32_t expected[4])
{
#if TMC2209_OS_FREERTOS && !TMC2209_OS_CREATE_MUTEX
    CHECK(TMC2209_BusInitWithLock(bus, &hal_fake.uart, 115200u, &supplied_mutex) == TMC2209_OK);
#else
    CHECK(TMC2209_BusInit(bus, &hal_fake.uart, 115200u) == TMC2209_OK);
#endif
    tmc2209_motor_config_t cfg = config();
    for (uint8_t a = 0u; a < 4u; ++a) {
        CHECK(TMC2209_UnitInit(&units[a], bus, a) == TMC2209_OK);
        uint32_t gstat = 0u, drv = 0u;
        CHECK(TMC2209_ReadGSTAT(&units[a], &gstat) == TMC2209_OK);
        CHECK(TMC2209_ReadDRV_STATUS(&units[a], &drv) == TMC2209_OK);
        CHECK((drv & 0x3Fu) == 0u && (gstat & 4u) == 0u);
        if ((gstat & 3u) != 0u)
            CHECK(TMC2209_ClearGSTAT(&units[a], (uint8_t)(gstat & 3u)) == TMC2209_OK);
        hal_fake.registers[a][0x22] = 0xFFFFFFu;
        CHECK(TMC2209_Configure(&units[a], &cfg) == TMC2209_OK);
        CHECK((hal_fake.registers[a][0x6C] & 15u) == 0u);
        CHECK(hal_fake.registers[a][0x22] == 0u);
        expected[a] = 0xC0u;
    }
}
static void unbind_all(tmc2209_bus_t *bus, tmc2209_unit_t units[4])
{
    for (uint8_t a = 0u; a < 4u; ++a) CHECK(TMC2209_UnitDeinit(&units[a]) == TMC2209_OK);
    CHECK(TMC2209_BusDeinit(bus) == TMC2209_OK);
}

static void read_diagnostics(tmc2209_unit_t units[4])
{
    ++counts.tests;
    const uint8_t registers[] = {0x05u, 0x07u, 0x12u, 0x6Au, 0x6Bu, 0x71u, 0x72u};
    unsigned writes = hal_fake.accepted_writes, active = hal_fake.active_writes;
    unsigned completed = 0u;
    for (uint8_t address = 0u; address < 4u; ++address) {
        for (size_t i = 0u; i < sizeof(registers); ++i) {
            uint8_t reg = registers[i];
            uint32_t raw = 0x807E5A19u ^ ((uint32_t)address << 24u) ^ reg;
            hal_fake.registers[address][reg] = raw;
            uint32_t actual = 0u;
            CHECK(TMC2209_ReadReg(&units[address], reg, &actual) == TMC2209_OK);
            CHECK(actual == raw);
            CHECK(hal_fake.tx_size == 4u && hal_fake.tx[1] == address && hal_fake.tx[2] == reg);
            CHECK(units[address].configured);
            CHECK(TMC2209_WriteReg(&units[address], reg, 0u) == TMC2209_ERR_PARAM);
            CHECK(TMC2209_WriteRegVerified(&units[address], reg, 0u) == TMC2209_ERR_PARAM);
            ++completed;
        }
    }
    CHECK(hal_fake.accepted_writes == writes && hal_fake.active_writes == active);
    CHECK(completed == 28u);
    printf("Raw diagnostic integration reads=%u\n", completed);
}

static void reset_activation_precondition(tmc2209_unit_t *unit)
{
    ++counts.tests;
    tmc2209_motor_config_t cfg = config();
    unsigned acknowledgements = hal_fake.gstat_writes;
    hal_chip_reset(unit->address);
    CHECK(hal_fake.registers[unit->address][6] == 0x21000000u);
    CHECK(TMC2209_Activate(unit, 3u) == TMC2209_ERR_FAULT);
    CHECK(!unit->configured && hal_fake.active_writes == 0u);
    CHECK(hal_fake.gstat_writes == acknowledgements);
    CHECK(hal_fake.registers[unit->address][0x10] == 0x1F00u);
    CHECK(TMC2209_Configure(unit, &cfg) == TMC2209_ERR_FAULT);
    CHECK(hal_fake.gstat_writes == acknowledgements);
    CHECK(TMC2209_ClearGSTAT(unit, 1u) == TMC2209_OK);
    CHECK(TMC2209_Activate(unit, 3u) == TMC2209_ERR_STATE);
    CHECK(TMC2209_Configure(unit, &cfg) == TMC2209_OK);
    CHECK(hal_fake.registers[unit->address][0x10] == cfg.ihold_irun);
    acknowledgements = hal_fake.gstat_writes;
    hal_fake.registers[unit->address][0x6F] |= 4u;
    hal_fake.registers[unit->address][1] |= 2u;
    CHECK(TMC2209_Activate(unit, 3u) == TMC2209_ERR_FAULT);
    CHECK(hal_fake.gstat_writes == acknowledgements);
    CHECK(TMC2209_ClearGSTAT(unit, 2u) == TMC2209_OK);
    CHECK((hal_fake.registers[unit->address][1] & 2u) != 0u);
    CHECK(TMC2209_Configure(unit, &cfg) == TMC2209_ERR_FAULT);
    CHECK(TMC2209_Deactivate(unit) == TMC2209_OK);
    CHECK((hal_fake.registers[unit->address][1] & 2u) != 0u);
    CHECK(TMC2209_ClearGSTAT(unit, 2u) == TMC2209_OK);
    CHECK(TMC2209_Configure(unit, &cfg) == TMC2209_OK);
    CHECK(hal_fake.active_writes == 0u && unit->configured);
    printf("Same-version reset and latched-driver-fault activation preconditions PASS\n");
}

int main(void)
{
    ++counts.tests;
    hal_reset();
#if TMC2209_OS_FREERTOS
    rtos_reset();
#if !TMC2209_OS_CREATE_MUTEX
    supplied_mutex = (fake_semaphore_t){pdTRUE, pdTRUE, pdTRUE};
#endif
#endif
    tmc2209_bus_t bus = {0};
    tmc2209_unit_t units[4] = {{0}};
    uint32_t expected[4];
    bind_all(&bus, units, expected);
    read_diagnostics(units);
    reset_activation_precondition(&units[0]);
    uint32_t seed = 0x2209A11Cu, failures = 0u, recovered = 0u, reinitializations = 0u;
    unsigned fault_counts[10] = {0};
    printf("Recovery seed=0x%08" PRIX32 "\n", seed);
    for (counts.case_index = 0u; counts.case_index < 10000u; ++counts.case_index) {
        uint32_t rnd = seeded_random(&seed);
        uint8_t addr = (uint8_t)(rnd & 3u);
        unsigned fault = (unsigned)((rnd >> 4u) % 10u);
        uint32_t desired = 0xC0u | ((rnd >> 8u) & 0x3Fu);
        tmc2209_status_t wanted = TMC2209_ERR_TIMEOUT;
        switch (fault) {
        case 0: hal_fake.block_txe_at = (unsigned)((rnd >> 12u) % 4u); break;
        case 1: hal_fake.reply_bytes = (unsigned)((rnd >> 12u) % 8u); break;
        case 2: hal_fake.corrupt_crc = true; wanted = TMC2209_ERR_CRC; break;
        case 3: hal_fake.corrupt_identity = true; wanted = TMC2209_ERR_NODEV; break;
        case 4: hal_fake.drop_write = hal_fake.frames + 2u; wanted = TMC2209_ERR_VERIFY; break;
        case 5: hal_fake.tx_mode_result = HAL_BUSY; wanted = TMC2209_ERR_BUSY; break;
        case 6: hal_fake.stuck_rx = true; break;
        case 7:
#if TMC2209_OS_FREERTOS
            rtos_fake.take_fails = pdTRUE;
#else
            hal_fake.cycle_step = 0u;
#endif
            break;
        case 8: hal_fake.stuck_tc = true; break;
        default:
            hal_fake.rx_mode_result = HAL_ERROR;
            hal_fake.fail_rx_mode_call = hal_fake.rx_mode_calls + 1u;
            wanted = TMC2209_ERR_PORT;
            break;
        }
        CHECK(TMC2209_WriteRegVerified(&units[addr], 0u, desired) == wanted);
        ++failures; ++fault_counts[fault];
        CHECK(units[addr].gconf.UINT32 == desired);
        CHECK(!bus.port.awaiting_reply && !units[addr].configured);
        unsigned tx_before = hal_fake.tx_mode_calls;
        CHECK(TMC2209_Activate(&units[addr], 3u) == TMC2209_ERR_STATE);
        CHECK(hal_fake.tx_mode_calls == tx_before && hal_fake.active_writes == 0u);
        hal_fake.block_txe_at = UINT_MAX; hal_fake.reply_bytes = 8u;
        hal_fake.corrupt_crc = false; hal_fake.corrupt_identity = false;
        hal_fake.drop_write = 0u; hal_fake.tx_mode_result = HAL_OK;
        hal_fake.stuck_rx = false; hal_fake.cycle_step = 18000u;
        hal_fake.stuck_tc = false; hal_fake.rx_mode_result = HAL_OK;
        hal_fake.fail_rx_mode_call = 0u;
#if TMC2209_OS_FREERTOS
        rtos_fake.take_fails = pdFALSE;
        CHECK(((SemaphoreHandle_t)bus.lock)->available == pdTRUE);
#if TMC2209_OS_CREATE_MUTEX
        CHECK(rtos_fake.allocations - rtos_fake.deletions == 1u);
#else
        CHECK(rtos_fake.allocations == 0u && rtos_fake.deletions == 0u);
        CHECK(supplied_mutex.allocated && bus.lock == &supplied_mutex);
#endif
#endif
        CHECK(TMC2209_WriteRegVerified(&units[addr], 0u, desired) == TMC2209_OK);
        expected[addr] = desired;
        uint32_t actual = UINT32_MAX;
        CHECK(TMC2209_ReadReg(&units[addr], 0u, &actual) == TMC2209_OK);
        CHECK(actual == desired);
        for (unsigned a = 0u; a < 4u; ++a) {
            CHECK(hal_fake.registers[a][0] == expected[a]);
            CHECK(units[a].gconf.UINT32 == expected[a]);
            CHECK((hal_fake.registers[a][0x6C] & 15u) == 0u);
        }
        ++recovered;
        if (counts.case_index % 257u == 0u) {
            unbind_all(&bus, units);
            bind_all(&bus, units, expected);
            ++reinitializations;
        }
    }
    CHECK(failures == 10000u && recovered == 10000u);
    for (unsigned f = 0u; f < 10u; ++f) {
        CHECK(fault_counts[f] != 0u);
        printf(" fault[%u]=%u", f, fault_counts[f]);
    }
    puts("");
    tmc2209_motor_config_t cfg = config();
    CHECK(TMC2209_Configure(&units[0], &cfg) == TMC2209_OK);
    CHECK(TMC2209_Activate(&units[0], 3u) == TMC2209_OK);
    CHECK(hal_fake.active_writes == 1u);
    CHECK(TMC2209_Deactivate(&units[0]) == TMC2209_OK);
    CHECK((hal_fake.registers[0][0x6C] & 15u) == 0u);
    unbind_all(&bus, units);
#if TMC2209_OS_FREERTOS
    CHECK(rtos_fake.allocations == rtos_fake.deletions);
    CHECK(rtos_fake.takes == rtos_fake.gives + fault_counts[7]);
#if !TMC2209_OS_CREATE_MUTEX
    CHECK(supplied_mutex.allocated && supplied_mutex.available);
#endif
#endif
    printf("Recovery failed=%" PRIu32 " recovered=%" PRIu32 " reinitializations=%" PRIu32
           " wire_frames=%u accepted_writes=%u final_state=0x%08" PRIX32 "\n",
           failures, recovered, reinitializations, hal_fake.frames,
           hal_fake.accepted_writes, seed);
    SUMMARY("Production core + HAL + OS recovery");
    return EXIT_SUCCESS;
}
