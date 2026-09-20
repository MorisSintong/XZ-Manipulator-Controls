#include "fake_core.h"
#include "test_support.h"

fake_core_t core_fake;
static int uart_storage;
#if TMC2209_OS_FREERTOS
static int mutex_storage;
#endif
UART_HandleTypeDef *const core_uart = (UART_HandleTypeDef *)&uart_storage;

void core_chip_reset(uint8_t address)
{
    REQUIRE(address < 4u);
    uint32_t ioin = core_fake.regs[address][6];
    memset(core_fake.regs[address], 0, sizeof(core_fake.regs[address]));
    core_fake.regs[address][0] = 0x101u;
    core_fake.regs[address][1] = 1u;
    core_fake.regs[address][6] = ioin;
    core_fake.regs[address][0x10] = 0x1F00u; /* IRUN=31; model OTP hold/delay=0. */
    core_fake.regs[address][0x11] = 20u;
    core_fake.regs[address][0x6C] = 0x10000053u;
    core_fake.regs[address][0x70] = 0xC10D0024u;
}

void core_reset(void)
{
    memset(&core_fake, 0, sizeof(core_fake));
    core_fake.bad_readback_reg = 256u;
    for (unsigned a = 0u; a < 4u; ++a) core_fake.regs[a][6] = 0x21000000u;
}

static void inject_chip_event(uint8_t address)
{
    if (core_fake.event_reset) core_chip_reset(address);
    core_fake.regs[address][1] |= core_fake.event_gstat;
    core_fake.regs[address][0x6F] |= core_fake.event_drv_status;
    ++core_fake.event_count;
}

tmc2209_motor_config_t core_config(void)
{
    tmc2209_motor_config_t cfg = {0xC0u, 0x200u, 0x71408u, 0x14010050u,
                                  0xC10D0024u, 0u, 0u, 0u, 0u, 20u};
    return cfg;
}

tmc2209_status_t core_bus_init(tmc2209_bus_t *bus)
{
#if !TMC2209_OS_CREATE_MUTEX && TMC2209_OS_FREERTOS
    return TMC2209_BusInitWithLock(bus, core_uart, 115200u, &mutex_storage);
#else
    return TMC2209_BusInit(bus, core_uart, 115200u);
#endif
}

tmc2209_status_t TMC2209_Port_Init(tmc2209_port_t *port)
{
    ++core_fake.init_calls;
    if (core_fake.init_result == TMC2209_OK) port->initialized = true;
    return core_fake.init_result;
}

tmc2209_status_t TMC2209_Port_Deinit(tmc2209_port_t *port)
{
    ++core_fake.deinit_calls;
    port->initialized = false;
    port->huart = NULL;
    return core_fake.deinit_result;
}

tmc2209_status_t TMC2209_Port_Transmit(tmc2209_port_t *port,
                                     const uint8_t *data, uint16_t len)
{
    REQUIRE(port->initialized && core_fake.locked);
    ++core_fake.tx_calls;
    if (core_fake.tx_result != TMC2209_OK &&
        (core_fake.fail_tx_call == 0u || core_fake.fail_tx_call == core_fake.tx_calls))
        return core_fake.tx_result;
    REQUIRE(len == 4u || len == 8u);
    REQUIRE(reference_crc(data, (size_t)len - 1u) == data[len - 1u]);
    if (core_fake.event_before_tx == core_fake.tx_calls) inject_chip_event(data[1]);
    memcpy(core_fake.last_frame, data, len);
    core_fake.last_len = len;
    core_fake.request_addr = data[1];
    core_fake.request_reg = data[2];
    if (len == 8u) {
        ++core_fake.write_calls;
        uint8_t a = data[1], r = (uint8_t)(data[2] & 127u);
        uint32_t v = ((uint32_t)data[3] << 24u) | ((uint32_t)data[4] << 16u) |
                     ((uint32_t)data[5] << 8u) | data[6];
        if (r == 0x6Cu && (v & 15u) != 0u) ++core_fake.active_writes;
        if (r == 1u) ++core_fake.gstat_writes;
        if (core_fake.drop_write_call != core_fake.write_calls) {
            if (r == 1u) {
                uint32_t clear = v;
                if ((core_fake.regs[a][0x6F] & 0x3Eu) != 0u) clear &= ~UINT32_C(2);
                core_fake.regs[a][r] &= ~clear;
            } else {
                core_fake.regs[a][r] = v;
                if (r == 0x6Cu && (v & 15u) == 0u)
                    core_fake.regs[a][0x6F] &= ~UINT32_C(0x3C); /* Disable clears short latches. */
            }
            core_fake.regs[a][2] =
                (uint8_t)(core_fake.regs[a][2] + 1u + core_fake.extra_ifcnt);
        }
    }
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_Port_Receive(tmc2209_port_t *port, uint8_t *data, uint16_t len)
{
    REQUIRE(port->initialized && core_fake.locked && len == 8u);
    ++core_fake.rx_calls;
    if (core_fake.rx_result != TMC2209_OK &&
        (core_fake.fail_rx_call == 0u || core_fake.fail_rx_call == core_fake.rx_calls))
        return core_fake.rx_result;
    if (core_fake.event_before_rx == core_fake.rx_calls)
        inject_chip_event(core_fake.request_addr);
    uint8_t reg = core_fake.request_reg;
    uint32_t value = core_fake.regs[core_fake.request_addr][reg];
    if (reg == core_fake.bad_readback_reg) value ^= 1u;
    reference_reply((uint8_t)(reg ^ core_fake.reply_reg_xor), value, data);
    if (core_fake.corrupt_identity) data[1] = 0u;
    if (core_fake.corrupt_crc) data[7] ^= 1u;
    if (core_fake.event_after_rx == core_fake.rx_calls)
        inject_chip_event(core_fake.request_addr);
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_Os_MutexCreate(tmc2209_mutex_t *m)
{
    ++core_fake.create_calls;
    *m = NULL;
    if (core_fake.create_result != TMC2209_OK) return core_fake.create_result;
    core_fake.allocated = true;
#if TMC2209_OS_FREERTOS
    *m = &mutex_storage;
#endif
    return TMC2209_OK;
}
void TMC2209_Os_MutexDelete(tmc2209_mutex_t m)
{
    (void)m;
    ++core_fake.delete_calls;
    REQUIRE(core_fake.allocated);
    core_fake.allocated = false;
}
tmc2209_status_t TMC2209_Os_MutexLock(tmc2209_mutex_t m, uint32_t timeout_ms)
{
    (void)m; (void)timeout_ms;
    ++core_fake.lock_calls;
    if (core_fake.lock_result != TMC2209_OK &&
        (core_fake.fail_lock_call == 0u || core_fake.fail_lock_call == core_fake.lock_calls))
        return core_fake.lock_result;
    REQUIRE(!core_fake.locked);
    core_fake.locked = true;
    return TMC2209_OK;
}
tmc2209_status_t TMC2209_Os_MutexUnlock(tmc2209_mutex_t m)
{
    (void)m;
    ++core_fake.unlock_calls;
    REQUIRE(core_fake.locked);
    core_fake.locked = false;
    if (core_fake.fail_unlock_call != 0u &&
        core_fake.fail_unlock_call != core_fake.unlock_calls) return TMC2209_OK;
    return core_fake.unlock_result;
}
