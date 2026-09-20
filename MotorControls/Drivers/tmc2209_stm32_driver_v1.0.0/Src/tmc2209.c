#include "tmc2209_internal.h"
#include <string.h>

#define WRITE_DESC(reg, readable, mask, bit, member) \
    {reg, readable, mask, (uint16_t)(1u << (bit)), \
     offsetof(tmc2209_unit_t, member), sizeof(((tmc2209_unit_t *)0)->member)}
static const tmc2209_reg_desc_t registers[] = {
    WRITE_DESC(TMC2209_REG_GCONF, true, TMC2209_GCONF_WRITE_MASK, 0, gconf.UINT32),
    WRITE_DESC(TMC2209_REG_SLAVECONF, false, TMC2209_SLAVECONF_WRITE_MASK, 1, slaveconf.UINT16),
    WRITE_DESC(TMC2209_REG_IHOLD_IRUN, false, TMC2209_IHOLD_IRUN_WRITE_MASK, 2, ihold_irun.UINT32),
    WRITE_DESC(TMC2209_REG_CHOPCONF, true, TMC2209_CHOPCONF_WRITE_MASK, 3, chopconf.UINT32),
    WRITE_DESC(TMC2209_REG_PWMCONF, true, TMC2209_PWMCONF_WRITE_MASK, 4, pwmconf.UINT32),
    WRITE_DESC(TMC2209_REG_COOLCONF, false, TMC2209_COOLCONF_WRITE_MASK, 5, coolconf.UINT32),
    WRITE_DESC(TMC2209_REG_TCOOLTHRS, false, TMC2209_THRS_WRITE_MASK, 6, tcoolthrs.UINT32),
    WRITE_DESC(TMC2209_REG_TPWMTHRS, false, TMC2209_THRS_WRITE_MASK, 7, tpwmthrs.UINT32),
    WRITE_DESC(TMC2209_REG_SGTHRS, false, TMC2209_BYTE_WRITE_MASK, 8, sgthrs),
    WRITE_DESC(TMC2209_REG_TPOWERDOWN, false, TMC2209_BYTE_WRITE_MASK, 9, tpowerdown),
    {TMC2209_REG_GSTAT, true, TMC2209_GSTAT_WRITE_MASK, 0, 0, 0},
    {TMC2209_REG_VACTUAL, false, TMC2209_VACTUAL_WRITE_MASK, 0, 0, 0},
    {TMC2209_REG_IFCNT, true, 0, 0, 0, 0},
    {TMC2209_REG_OTP_READ, true, 0, 0, 0, 0},
    {TMC2209_REG_IOIN, true, 0, 0, 0, 0},
    {TMC2209_REG_FACTORY_CONF, true, 0, 0, 0, 0},
    {TMC2209_REG_TSTEP, true, 0, 0, 0, 0},
    {TMC2209_REG_SG_RESULT, true, 0, 0, 0, 0},
    {TMC2209_REG_MSCNT, true, 0, 0, 0, 0},
    {TMC2209_REG_MSCURACT, true, 0, 0, 0, 0},
    {TMC2209_REG_DRV_STATUS, true, 0, 0, 0, 0},
    {TMC2209_REG_PWM_SCALE, true, 0, 0, 0, 0},
    {TMC2209_REG_PWM_AUTO, true, 0, 0, 0, 0}
};
#undef WRITE_DESC

bool tmc2209_unit_ready(const tmc2209_unit_t *unit)
{
    return unit != NULL && unit->bus != NULL && unit->address <= 3u &&
           unit->bus->initialized &&
           (unit->bus->bound_addresses & (1u << unit->address)) != 0u;
}

const tmc2209_reg_desc_t *tmc2209_register(uint8_t reg)
{
    for (size_t i = 0u; i < sizeof(registers) / sizeof(registers[0]); ++i)
        if (registers[i].address == reg) return &registers[i];
    return NULL;
}

tmc2209_status_t tmc2209_validate_value(const tmc2209_reg_desc_t *desc, uint32_t data)
{
    if (desc == NULL || desc->write_mask == 0u || (data & ~desc->write_mask) != 0u)
        return TMC2209_ERR_PARAM;
    if (desc->address == TMC2209_REG_VACTUAL && data != 0u)
        return TMC2209_ERR_PARAM;
    if (desc->address == TMC2209_REG_CHOPCONF) {
        if (((data >> 24u) & 15u) > 8u ||
            ((data & 15u) == 1u && ((data >> 15u) & 3u) < 2u) ||
            (((data >> 7u) & 15u) + ((data >> 4u) & 7u)) > 18u)
            return TMC2209_ERR_PARAM;
    }
    if (desc->address == TMC2209_REG_PWMCONF &&
        (((data & (1u << 19u)) != 0u && (data & (1u << 18u)) == 0u) ||
         ((data & (1u << 18u)) != 0u && ((data >> 24u) & 15u) == 0u)))
        return TMC2209_ERR_PARAM;
    return TMC2209_OK;
}

void tmc2209_store_desired(tmc2209_unit_t *unit, const tmc2209_reg_desc_t *desc,
                          uint32_t value)
{
    unsigned char *dst = (unsigned char *)unit + desc->mirror_offset;
    if (desc->mirror_size == 1u) {
        uint8_t v = (uint8_t)value;
        memcpy(dst, &v, sizeof(v));
    } else if (desc->mirror_size == 2u) {
        uint16_t v = (uint16_t)value;
        memcpy(dst, &v, sizeof(v));
    } else if (desc->mirror_size == 4u) {
        memcpy(dst, &value, sizeof(value));
    }
    unit->desired_valid |= desc->mirror_bit;
}

uint32_t tmc2209_load_desired(const tmc2209_unit_t *unit,
                            const tmc2209_reg_desc_t *desc)
{
    const unsigned char *src = (const unsigned char *)unit + desc->mirror_offset;
    if (desc->mirror_size == 1u) {
        uint8_t v;
        memcpy(&v, src, sizeof(v));
        return v;
    }
    if (desc->mirror_size == 2u) {
        uint16_t v;
        memcpy(&v, src, sizeof(v));
        return v;
    }
    uint32_t v;
    memcpy(&v, src, sizeof(v));
    return v;
}

static tmc2209_status_t bus_init(tmc2209_bus_t *bus, UART_HandleTypeDef *huart,
                                uint32_t baud, tmc2209_mutex_t external,
                                bool create)
{
    if (bus == NULL || huart == NULL || baud < 9000u || baud > 500000u)
        return TMC2209_ERR_PARAM;
    if (bus->initialized) return TMC2209_ERR_STATE;
#if TMC2209_OS_FREERTOS
    if (!create && external == NULL) return TMC2209_ERR_PARAM;
#else
    if (external != NULL) return TMC2209_ERR_PARAM;
#endif
    tmc2209_bus_t next = {0};
    next.port.huart = huart;
    next.port.baud = baud;
    next.port.tx_timeout_ms = TMC2209_UART_TX_TIMEOUT_MS;
    next.port.rx_timeout_ms = TMC2209_UART_RX_TIMEOUT_MS;
    next.port.bus_idle_us = TMC2209_BUS_IDLE_US;
    next.lock = external;
    tmc2209_status_t st = TMC2209_Port_Init(&next.port);
    if (st != TMC2209_OK) {
        (void)TMC2209_Port_Deinit(&next.port);
        return st;
    }
    if (create) {
        st = TMC2209_Os_MutexCreate(&next.lock);
        if (st != TMC2209_OK) {
            (void)TMC2209_Port_Deinit(&next.port);
            return st;
        }
        next.owns_lock = true;
    }
    next.initialized = true;
    *bus = next;
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_BusInit(tmc2209_bus_t *bus, UART_HandleTypeDef *huart,
                                uint32_t baud)
{
#if TMC2209_OS_CREATE_MUTEX
    return bus_init(bus, huart, baud, NULL, true);
#else
    return bus_init(bus, huart, baud, NULL, false);
#endif
}

tmc2209_status_t TMC2209_BusInitWithLock(tmc2209_bus_t *bus, UART_HandleTypeDef *huart,
                                       uint32_t baud, tmc2209_mutex_t lock)
{
    return bus_init(bus, huart, baud, lock, false);
}

tmc2209_status_t TMC2209_BusDeinit(tmc2209_bus_t *bus)
{
    if (bus == NULL || !bus->initialized) return TMC2209_OK;
    if (bus->bound_addresses != 0u) return TMC2209_ERR_BUSY;
    tmc2209_status_t st = TMC2209_Port_Deinit(&bus->port);
    if (bus->owns_lock) TMC2209_Os_MutexDelete(bus->lock);
    memset(bus, 0, sizeof(*bus));
    return st;
}

tmc2209_status_t TMC2209_UnitInit(tmc2209_unit_t *unit, tmc2209_bus_t *bus, uint8_t addr)
{
    if (unit == NULL || bus == NULL || addr > 3u || !bus->initialized)
        return TMC2209_ERR_PARAM;
    if (unit->bus != NULL || (bus->bound_addresses & (1u << addr)) != 0u)
        return TMC2209_ERR_STATE;
    memset(unit, 0, sizeof(*unit));
    unit->bus = bus;
    unit->address = addr;
    bus->bound_addresses |= (uint8_t)(1u << addr);
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_UnitDeinit(tmc2209_unit_t *unit)
{
    if (unit == NULL) return TMC2209_OK;
    if (tmc2209_unit_ready(unit))
        unit->bus->bound_addresses &= (uint8_t)~(1u << unit->address);
    memset(unit, 0, sizeof(*unit));
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_Available(tmc2209_unit_t *unit, bool *present)
{
    if (present == NULL) return TMC2209_ERR_PARAM;
    *present = false;
    uint32_t data = 0u;
    tmc2209_status_t st = TMC2209_ReadIOIN(unit, &data);
    if (st != TMC2209_OK) {
        if (unit != NULL) unit->configured = false;
        return st;
    }
    if ((data >> 24u) != TMC2209_IOIN_VERSION) {
        unit->configured = false;
        return TMC2209_ERR_NODEV;
    }
    *present = true;
    return TMC2209_OK;
}

tmc2209_status_t tmc2209_check_faults(tmc2209_unit_t *unit)
{
    uint32_t data = 0u;
    tmc2209_status_t st = TMC2209_ReadDRV_STATUS(unit, &data);
    if (st != TMC2209_OK) return st;
    if ((data & TMC2209_DRV_STATUS_FAULT_MASK) != 0u) return TMC2209_ERR_FAULT;
    /* Sample the latched reset/driver flags last, including events during DRV_STATUS. */
    st = TMC2209_ReadGSTAT(unit, &data);
    if (st != TMC2209_OK) return st;
    return (data & TMC2209_GSTAT_FAULT_MASK) != 0u ? TMC2209_ERR_FAULT : TMC2209_OK;
}

tmc2209_status_t TMC2209_Configure(tmc2209_unit_t *unit,
                                 const tmc2209_motor_config_t *config)
{
    if (!tmc2209_unit_ready(unit) || config == NULL) return TMC2209_ERR_PARAM;
    const uint8_t regs[] = {TMC2209_REG_CHOPCONF, TMC2209_REG_VACTUAL, TMC2209_REG_SLAVECONF,
        TMC2209_REG_GCONF, TMC2209_REG_IHOLD_IRUN, TMC2209_REG_PWMCONF,
        TMC2209_REG_TPOWERDOWN, TMC2209_REG_TPWMTHRS, TMC2209_REG_TCOOLTHRS,
        TMC2209_REG_SGTHRS, TMC2209_REG_COOLCONF};
    const uint32_t values[] = {config->chopconf, 0u, config->slaveconf, config->gconf,
        config->ihold_irun, config->pwmconf, config->tpowerdown, config->tpwmthrs,
        config->tcoolthrs, config->sgthrs, config->coolconf};
    for (size_t i = 0u; i < sizeof(regs); ++i)
        if (tmc2209_validate_value(tmc2209_register(regs[i]), values[i]) != TMC2209_OK)
            return TMC2209_ERR_PARAM;
    uint32_t irun = (config->ihold_irun >> 8u) & 31u;
    if ((config->gconf & 0xC0u) != 0xC0u || (config->chopconf & 15u) != 0u ||
        ((config->slaveconf >> 8u) & 15u) < 2u ||
        ((config->chopconf & (1u << 29u)) != 0u && (config->gconf & 0x100u) != 0u) ||
        ((config->pwmconf & (1u << 18u)) != 0u && config->tpowerdown < 2u) ||
        ((config->coolconf & 15u) != 0u &&
          irun < ((config->coolconf & 0x8000u) != 0u ? 20u : 10u)))
        return TMC2209_ERR_PARAM;
    unit->configured = false;
    bool present = false;
    tmc2209_status_t st = TMC2209_Available(unit, &present);
    if (st != TMC2209_OK) return st;
    st = tmc2209_check_faults(unit);
    if (st != TMC2209_OK) return st;
    for (size_t i = 0u; i < sizeof(regs); ++i) {
        st = TMC2209_WriteRegVerified(unit, regs[i], values[i]);
        if (st != TMC2209_OK) return st;
    }
    st = tmc2209_check_faults(unit);
    if (st == TMC2209_OK) unit->configured = true;
    return st;
}

tmc2209_status_t TMC2209_Activate(tmc2209_unit_t *unit, uint8_t toff)
{
    if (!tmc2209_unit_ready(unit) || toff == 0u || toff > 15u) return TMC2209_ERR_PARAM;
    if (!unit->configured) return TMC2209_ERR_STATE;
    bool present = false;
    tmc2209_status_t st = TMC2209_Available(unit, &present);
    if (st != TMC2209_OK) return st;
    st = TMC2209_WriteRegVerified(unit, TMC2209_REG_CHOPCONF,
                              (unit->chopconf.UINT32 & ~UINT32_C(15)) | toff);
    if (st == TMC2209_OK) unit->configured = true;
    return st;
}

tmc2209_status_t TMC2209_Deactivate(tmc2209_unit_t *unit)
{
    if (!tmc2209_unit_ready(unit)) return TMC2209_ERR_PARAM;
    const tmc2209_reg_desc_t *desc = tmc2209_register(TMC2209_REG_CHOPCONF);
    if ((unit->desired_valid & desc->mirror_bit) == 0u) return TMC2209_ERR_STATE;
    return TMC2209_WriteRegVerified(unit, TMC2209_REG_CHOPCONF,
                                    unit->chopconf.UINT32 & ~UINT32_C(15));
}

static tmc2209_status_t typed_write(tmc2209_unit_t *unit, uint8_t reg, uint32_t value)
{
    if (!tmc2209_unit_ready(unit)) return TMC2209_ERR_PARAM;
    const tmc2209_reg_desc_t *desc = tmc2209_register(reg);
    if (value == UINT32_MAX) {
        if ((unit->desired_valid & desc->mirror_bit) == 0u) return TMC2209_ERR_STATE;
        value = tmc2209_load_desired(unit, desc);
    }
    return TMC2209_WriteReg(unit, reg, value);
}
#define TYPED_WRITE(name) \
    tmc2209_status_t TMC2209_Write##name(tmc2209_unit_t *unit, uint32_t value) \
    { return typed_write(unit, TMC2209_REG_##name, value); }
TYPED_WRITE(GCONF)
TYPED_WRITE(SLAVECONF)
TYPED_WRITE(IHOLD_IRUN)
TYPED_WRITE(CHOPCONF)
TYPED_WRITE(PWMCONF)
TYPED_WRITE(COOLCONF)
TYPED_WRITE(TCOOLTHRS)
TYPED_WRITE(TPWMTHRS)
TYPED_WRITE(SGTHRS)
TYPED_WRITE(TPOWERDOWN)
#undef TYPED_WRITE

#define TYPED_READ(name, member, type, mask) \
    tmc2209_status_t TMC2209_Read##name(tmc2209_unit_t *unit, uint32_t *data) \
    { \
        tmc2209_status_t st = TMC2209_ReadReg(unit, TMC2209_REG_##name, data); \
        if (st == TMC2209_OK) unit->member = (type)(*data & (mask)); \
        return st; \
    }
TYPED_READ(IOIN, ioin.UINT32, uint32_t, UINT32_MAX)
TYPED_READ(SG_RESULT, sg_result, uint16_t, 0x3FFu)
TYPED_READ(IFCNT, ifcnt, uint8_t, 0xFFu)
TYPED_READ(GSTAT, gstat.UINT8, uint8_t, 7u)
TYPED_READ(DRV_STATUS, drv_status.UINT32, uint32_t, UINT32_MAX)
#undef TYPED_READ

tmc2209_status_t TMC2209_ClearGSTAT(tmc2209_unit_t *unit, uint8_t flags)
{
    return TMC2209_WriteRegVerified(unit, TMC2209_REG_GSTAT, flags);
}
