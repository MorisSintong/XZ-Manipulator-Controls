/** UART wire format and access policy, TMC2209 Rev.1.08 §§4-5. */
#include "tmc2209_internal.h"

uint8_t tmc2209_crc_start(uint8_t address)
{
    static const uint8_t starts[4] = {TMC2209_CRC_START_A0, TMC2209_CRC_START_A1,
                                      TMC2209_CRC_START_A2, TMC2209_CRC_START_A3};
    return address < 4u ? starts[address] : 0u;
}

uint8_t tmc2209_crc_byte(uint8_t byte, uint8_t crc)
{
    for (unsigned j = 0u; j < 8u; ++j) {
        crc = (uint8_t)(((uint32_t)crc << 1u) ^
              ((((uint32_t)crc >> 7u) ^ (byte & 1u)) != 0u ? 0x07u : 0u));
        byte >>= 1u;
    }
    return crc;
}

uint8_t tmc2209_crc_u32(uint32_t data, uint8_t crc)
{
    for (unsigned i = 0u; i < 4u; ++i) {
        crc = tmc2209_crc_byte((uint8_t)(data >> 24u), crc);
        data <<= 8u;
    }
    return crc;
}

tmc2209_status_t tmc2209_pack_write(uint8_t addr, uint8_t reg, uint32_t data,
                                   uint8_t out[8])
{
    if (out == NULL || addr > 3u || reg > 0x7Fu) return TMC2209_ERR_PARAM;
    out[0] = TMC2209_SYNC;
    out[1] = addr;
    out[2] = (uint8_t)(reg | TMC2209_WRITE_BIT);
    out[3] = (uint8_t)(data >> 24u);
    out[4] = (uint8_t)(data >> 16u);
    out[5] = (uint8_t)(data >> 8u);
    out[6] = (uint8_t)data;
    out[7] = tmc2209_crc_u32(data, tmc2209_crc_byte(out[2], tmc2209_crc_start(addr)));
    return TMC2209_OK;
}

tmc2209_status_t tmc2209_pack_request(uint8_t addr, uint8_t reg, uint8_t out[4])
{
    if (out == NULL || addr > 3u || reg > 0x7Fu) return TMC2209_ERR_PARAM;
    out[0] = TMC2209_SYNC;
    out[1] = addr;
    out[2] = reg;
    out[3] = tmc2209_crc_byte(reg, tmc2209_crc_start(addr));
    return TMC2209_OK;
}

tmc2209_status_t tmc2209_unpack_reply(const uint8_t reply[8], uint8_t expected_reg,
                                     uint32_t *data)
{
    if (reply == NULL || data == NULL || expected_reg > 0x7Fu)
        return TMC2209_ERR_PARAM;
    if (reply[0] != TMC2209_SYNC || reply[1] != TMC2209_REPLY_ADDR ||
        reply[2] != expected_reg) return TMC2209_ERR_NODEV;
    uint32_t value = ((uint32_t)reply[3] << 24u) | ((uint32_t)reply[4] << 16u) |
                     ((uint32_t)reply[5] << 8u) | reply[6];
    uint8_t crc = tmc2209_crc_u32(value,
                    tmc2209_crc_byte(reply[2], TMC2209_CRC_START_REPLY));
    if (reply[7] != crc) return TMC2209_ERR_CRC;
    *data = value;
    return TMC2209_OK;
}

static tmc2209_status_t release_bus(tmc2209_bus_t *bus, tmc2209_status_t first)
{
    tmc2209_status_t cleanup = TMC2209_Os_MutexUnlock(bus->lock);
    return first == TMC2209_OK ? cleanup : first;
}

static tmc2209_status_t read_locked(tmc2209_unit_t *unit, uint8_t reg,
                                    uint32_t *data)
{
    uint8_t req[4], reply[8];
    (void)tmc2209_pack_request(unit->address, reg, req);
    tmc2209_status_t st = TMC2209_Port_Transmit(&unit->bus->port, req, 4u);
    if (st == TMC2209_OK) st = TMC2209_Port_Receive(&unit->bus->port, reply, 8u);
    if (st == TMC2209_OK) st = tmc2209_unpack_reply(reply, reg, data);
    return st;
}

static tmc2209_status_t write_locked(tmc2209_unit_t *unit, uint8_t reg,
                                     uint32_t data)
{
    uint8_t frame[8];
    (void)tmc2209_pack_write(unit->address, reg, data, frame);
    return TMC2209_Port_Transmit(&unit->bus->port, frame, 8u);
}

static tmc2209_status_t prepare_write(tmc2209_unit_t *unit, uint8_t reg,
                                      uint32_t data)
{
    if (!tmc2209_unit_ready(unit)) return TMC2209_ERR_PARAM;
    const tmc2209_reg_desc_t *desc = tmc2209_register(reg);
    tmc2209_status_t st = tmc2209_validate_value(desc, data);
    if (st != TMC2209_OK) return st;
    if (reg == TMC2209_REG_CHOPCONF && (data & 15u) != 0u && !unit->configured)
        return TMC2209_ERR_STATE;
    tmc2209_store_desired(unit, desc, data);
    unit->configured = false;
    if (reg == TMC2209_REG_CHOPCONF && (data & 15u) != 0u)
        return tmc2209_check_faults(unit);
    return TMC2209_OK;
}

static tmc2209_status_t finish_write(tmc2209_unit_t *unit, uint8_t reg,
                                     uint32_t data, tmc2209_status_t first)
{
    if (first != TMC2209_OK) return first;
    if (reg == TMC2209_REG_CHOPCONF && (data & 15u) != 0u)
        return tmc2209_check_faults(unit);
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_WriteReg(tmc2209_unit_t *unit, uint8_t reg, uint32_t data)
{
    tmc2209_status_t st = prepare_write(unit, reg, data);
    if (st != TMC2209_OK) return st;
    st = TMC2209_Os_MutexLock(unit->bus->lock, TMC2209_BUS_LOCK_TIMEOUT_MS);
    if (st != TMC2209_OK) return st;
    st = release_bus(unit->bus, write_locked(unit, reg, data));
    return finish_write(unit, reg, data, st);
}

tmc2209_status_t TMC2209_WriteRegVerified(tmc2209_unit_t *unit, uint8_t reg,
                                        uint32_t data)
{
    tmc2209_status_t st = prepare_write(unit, reg, data);
    if (st != TMC2209_OK) return st;
    st = TMC2209_Os_MutexLock(unit->bus->lock, TMC2209_BUS_LOCK_TIMEOUT_MS);
    if (st != TMC2209_OK) return st;
    uint32_t before = 0u, after = 0u, actual = 0u;
    st = read_locked(unit, TMC2209_REG_IFCNT, &before);
    if (st == TMC2209_OK) st = write_locked(unit, reg, data);
    if (st == TMC2209_OK) st = read_locked(unit, TMC2209_REG_IFCNT, &after);
    if (st == TMC2209_OK && (uint8_t)(after - before) != 1u)
        st = TMC2209_ERR_VERIFY;
    if (st == TMC2209_OK && tmc2209_register(reg)->readable &&
        reg != TMC2209_REG_GSTAT) {
        st = read_locked(unit, reg, &actual);
        if (st == TMC2209_OK &&
            (actual & tmc2209_register(reg)->write_mask) != data)
            st = TMC2209_ERR_VERIFY;
    }
    return finish_write(unit, reg, data, release_bus(unit->bus, st));
}

tmc2209_status_t TMC2209_ReadReg(tmc2209_unit_t *unit, uint8_t reg, uint32_t *data)
{
    if (!tmc2209_unit_ready(unit) || data == NULL) return TMC2209_ERR_PARAM;
    const tmc2209_reg_desc_t *desc = tmc2209_register(reg);
    if (desc == NULL || !desc->readable) return TMC2209_ERR_PARAM;
    tmc2209_status_t st =
        TMC2209_Os_MutexLock(unit->bus->lock, TMC2209_BUS_LOCK_TIMEOUT_MS);
    if (st != TMC2209_OK) return st;
    uint32_t value = 0u;
    st = release_bus(unit->bus, read_locked(unit, reg, &value));
    if (st == TMC2209_OK) *data = value;
    return st;
}
