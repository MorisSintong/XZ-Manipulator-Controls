/*******************************************************************************
 * Copyright © 2017 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 * Copyright © 2024 Analog Devices, Inc.
 *******************************************************************************/
#include "tmc2240_core.h"
#include <string.h>

/* Rev. 2 (11/23), pp. 80-123. All implemented registers are readable.
 * Reset values are descriptive, never treated as an observation of hardware.
 * IOIN is mixed-access: OUTPUT (bit 12) is writable in UART operation.
 */
#define RW(mask, reset) { mask, mask, reset, mask, false }
#define RO(mask)        { mask, 0U, 0U, 0U, false }
#define RC(mask, reset) { mask, 0U, reset, mask, false }
static const TMC2240RegisterInfo registers[TMC2240_REGISTER_COUNT] = {
    [0x00] = RW(0x0001F7FEU, 0x00000008U),
    [0x01] = { 0x1FU, 0x1FU, 0x1DU, 0x1FU, true },
    [0x02] = RC(0xFFU, 0U),
    [0x03] = RW(0x00000FFFU, 0U),
    [0x04] = { 0xFF07FF7FU, 0x1000U, 0x40001000U, 0xFF071000U, false },
    [0x0A] = RW(0x33U, 0U),
    [0x0B] = RW(0xFFU, 0U),
    [0x10] = RW(0x0F0F1F1FU, 0x04011F08U),
    [0x11] = RW(0xFFU, 10U),
    [0x12] = RC(0x000FFFFFU, 0U),
    [0x13] = RW(0x000FFFFFU, 0U),
    [0x14] = RW(0x000FFFFFU, 0U),
    [0x15] = RW(0x000FFFFFU, 0U),
    [0x2D] = RW(0x01FF01FFU, 0U),
    [0x38] = RW(0x000005FFU, 0U),
    [0x39] = RW(0xFFFFFFFFU, 0U),
    [0x3A] = RW(0xFFFFFFFFU, 0x00010000U),
    [0x3B] = { 1U, 1U, 0U, 1U, true },
    [0x3C] = RC(0xFFFFFFFFU, 0U),
    [0x50] = RO(0x1FFF1FFFU),
    [0x51] = RO(0x00001FFFU),
    [0x52] = RW(0x1FFF1FFFU, 0x0B920F25U),
    [0x60] = RW(0xFFFFFFFFU, 0xAAAAB554U),
    [0x61] = RW(0xFFFFFFFFU, 0x4A9554AAU),
    [0x62] = RW(0xFFFFFFFFU, 0x24492929U),
    [0x63] = RW(0xFFFFFFFFU, 0x10104222U),
    [0x64] = RW(0xFFFFFFFFU, 0xFBFFFFFFU),
    [0x65] = RW(0xFFFFFFFFU, 0xB5BB777DU),
    [0x66] = RW(0xFFFFFFFFU, 0x49295556U),
    [0x67] = RW(0xFFFFFFFFU, 0x00404222U),
    [0x68] = RW(0xFFFFFFFFU, 0xFFFF8056U),
    [0x69] = RW(0xFFFF00FFU, 0x00F70000U),
    [0x6A] = RC(0x000003FFU, 0U),
    [0x6B] = RO(0x01FF01FFU),
    [0x6C] = RW(0xFFFDDFFFU, 0x10410150U),
    [0x6D] = RW(0x017FEF6FU, 0U),
    [0x6F] = RO(0xFF1FF3FFU),
    [0x70] = RW(0xFFFFFFFFU, 0xC40C001DU),
    [0x71] = RC(0x01FF03FFU, 0U),
    [0x72] = RC(0x00FF00FFU, 0U),
    [0x74] = RW(0x000003FFU, 0x00000200U),
    [0x75] = RC(0x000003FFU, 0U),
    [0x76] = RC(0xFFFFFFFFU, 0U)
};
#undef RW
#undef RO
#undef RC

TMC2240Status tmc2240_getRegisterInfo(uint8_t address, TMC2240RegisterInfo *info)
{
    if (info == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    if ((address >= TMC2240_REGISTER_COUNT) || (registers[address].read_mask == 0U)) {
        return TMC2240_ERROR_ADDRESS;
    }
    *info = registers[address];
    return TMC2240_OK;
}

TMC2240Status tmc2240_validateRegisterValue(uint8_t address, uint32_t value)
{
    TMC2240RegisterInfo info;
    TMC2240Status status = tmc2240_getRegisterInfo(address, &info);
    if (status != TMC2240_OK) {
        return status;
    }
    if (info.write_mask == 0U) {
        return TMC2240_ERROR_ACCESS;
    }
    if ((value & ~info.write_mask) != 0U) {
        return TMC2240_ERROR_RANGE;
    }
    if ((address == TMC2240_GLOBAL_SCALER) && (value != 0U) && (value < 32U)) {
        return TMC2240_ERROR_RANGE;
    }
    if ((address == TMC2240_NODECONF) &&
        (((value & 0xFFU) == 255U) || ((value & 0x100U) != 0U))) {
        return TMC2240_ERROR_RANGE;
    }
    if (address == TMC2240_CHOPCONF) {
        uint32_t toff = value & 15U;
        uint32_t tbl = (value >> 15U) & 3U;
        if (((value >> 24U) & 15U) > 8U) {
            return TMC2240_ERROR_RANGE;
        }
        if ((toff == 1U) && (tbl < 2U)) {
            return TMC2240_ERROR_RANGE;
        }
    }
    if ((address == TMC2240_PWMCONF) && ((value & TMC2240_PWM_AUTOSCALE_MASK) != 0U) &&
        ((value & TMC2240_PWM_REG_MASK) == 0U)) {
        return TMC2240_ERROR_RANGE;
    }
    return TMC2240_OK;
}

#if TMC2240_CACHE && TMC2240_ENABLE_TMC_CACHE
static TMC2240CacheEntry cacheEntries[TMC2240_IC_CACHE_COUNT][TMC2240_REGISTER_COUNT];

TMC2240Status tmc2240_cache(uint16_t icID, TMC2240CacheOp operation,
                          uint8_t address, TMC2240CacheEntry *entry)
{
    TMC2240CacheEntry *stored;
    if (icID >= TMC2240_IC_CACHE_COUNT) {
        return TMC2240_ERROR_ID;
    }
    if ((operation == TMC2240_CACHE_RESET) || (operation == TMC2240_CACHE_CLEAR)) {
        for (size_t i = 0U; i < TMC2240_REGISTER_COUNT; ++i) {
            stored = &cacheEntries[icID][i];
            if (operation == TMC2240_CACHE_CLEAR) {
                memset(stored, 0, sizeof(*stored));
            } else {
                stored->confirmed_valid = false;
                stored->dirty = stored->desired_valid;
            }
        }
        return TMC2240_OK;
    }
    if ((address >= TMC2240_REGISTER_COUNT) || (registers[address].read_mask == 0U)) {
        return TMC2240_ERROR_ADDRESS;
    }
    if ((entry == NULL) && (operation != TMC2240_CACHE_INVALIDATE)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    stored = &cacheEntries[icID][address];
    switch (operation) {
    case TMC2240_CACHE_GET:
        *entry = *stored;
        return TMC2240_OK;
    case TMC2240_CACHE_DESIRE:
        stored->desired = entry->desired & registers[address].write_mask;
        stored->desired_valid = true;
        stored->confirmed_valid = false;
        break;
    case TMC2240_CACHE_OBSERVE:
        stored->confirmed = entry->confirmed & registers[address].read_mask;
        stored->confirmed_valid = true;
        break;
    case TMC2240_CACHE_INVALIDATE:
        stored->confirmed_valid = false;
        break;
    default:
        return TMC2240_ERROR_ARGUMENT;
    }
    stored->dirty = stored->desired_valid &&
        (!stored->confirmed_valid ||
         (stored->desired != (stored->confirmed & registers[address].write_mask)));
    return TMC2240_OK;
}
#endif

static TMC2240Status cacheOperation(uint16_t icID, TMC2240CacheOp operation,
                                   uint8_t address, uint32_t value)
{
#if TMC2240_CACHE
    TMC2240CacheEntry entry = { 0 };
    entry.desired = value;
    entry.confirmed = value;
    return tmc2240_cache(icID, operation, address, &entry);
#else
    (void)icID;
    (void)operation;
    (void)address;
    (void)value;
    return TMC2240_OK;
#endif
}

TMC2240Status tmc2240_initCache(uint16_t count)
{
    if (count == 0U) {
        return TMC2240_ERROR_ARGUMENT;
    }
#if TMC2240_CACHE && TMC2240_ENABLE_TMC_CACHE
    if (count > TMC2240_IC_CACHE_COUNT) {
        return TMC2240_ERROR_RANGE;
    }
    /* Include unused slots to prevent stale state after shrinking/reinitializing. */
    count = TMC2240_IC_CACHE_COUNT;
#endif
    for (uint16_t id = 0U; id < count; ++id) {
        TMC2240Status status = cacheOperation(id, TMC2240_CACHE_CLEAR, 0U, 0U);
        if (status != TMC2240_OK) {
            return status;
        }
    }
    return TMC2240_OK;
}

static TMC2240Status getBus(uint16_t icID, TMC2240BusType *bus)
{
#if TMC2240_CACHE && TMC2240_ENABLE_TMC_CACHE
    if (icID >= TMC2240_IC_CACHE_COUNT) {
        return TMC2240_ERROR_ID;
    }
#endif
    TMC2240Status status = tmc2240_getBusType(icID, bus);
    if (status != TMC2240_OK) {
        return status;
    }
    if ((*bus != IC_BUS_SPI) && (*bus != IC_BUS_UART)) {
        return TMC2240_ERROR_UNSUPPORTED;
    }
    return TMC2240_OK;
}

TMC2240Status tmc2240_invalidateCache(uint16_t icID)
{
    TMC2240BusType bus;
    TMC2240Status status = getBus(icID, &bus);
    if (status != TMC2240_OK) {
        return status;
    }
    return cacheOperation(icID, TMC2240_CACHE_RESET, 0U, 0U);
}

TMC2240Status tmc2240_getCachedRegister(uint16_t icID, uint8_t address,
                                      TMC2240CacheEntry *entry)
{
    TMC2240RegisterInfo info;
    TMC2240BusType bus;
    TMC2240Status status;
    if (entry == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_getRegisterInfo(address, &info);
    if (status != TMC2240_OK) {
        return status;
    }
    status = getBus(icID, &bus);
    if (status != TMC2240_OK) {
        return status;
    }
#if TMC2240_CACHE
    TMC2240CacheEntry cached;
    status = tmc2240_cache(icID, TMC2240_CACHE_GET, address, &cached);
    if (status == TMC2240_OK) {
        *entry = cached;
    }
    return status;
#else
    return TMC2240_ERROR_UNSUPPORTED;
#endif
}

#if TMC_API_EXTERNAL_CRC_TABLE
extern const uint8_t tmcCRCTable_Poly7Reflected[256];
#else
static const uint8_t tmcCRCTable_Poly7Reflected[256] = {
    0x00,0x91,0xE3,0x72,0x07,0x96,0xE4,0x75,0x0E,0x9F,0xED,0x7C,0x09,0x98,0xEA,0x7B,
    0x1C,0x8D,0xFF,0x6E,0x1B,0x8A,0xF8,0x69,0x12,0x83,0xF1,0x60,0x15,0x84,0xF6,0x67,
    0x38,0xA9,0xDB,0x4A,0x3F,0xAE,0xDC,0x4D,0x36,0xA7,0xD5,0x44,0x31,0xA0,0xD2,0x43,
    0x24,0xB5,0xC7,0x56,0x23,0xB2,0xC0,0x51,0x2A,0xBB,0xC9,0x58,0x2D,0xBC,0xCE,0x5F,
    0x70,0xE1,0x93,0x02,0x77,0xE6,0x94,0x05,0x7E,0xEF,0x9D,0x0C,0x79,0xE8,0x9A,0x0B,
    0x6C,0xFD,0x8F,0x1E,0x6B,0xFA,0x88,0x19,0x62,0xF3,0x81,0x10,0x65,0xF4,0x86,0x17,
    0x48,0xD9,0xAB,0x3A,0x4F,0xDE,0xAC,0x3D,0x46,0xD7,0xA5,0x34,0x41,0xD0,0xA2,0x33,
    0x54,0xC5,0xB7,0x26,0x53,0xC2,0xB0,0x21,0x5A,0xCB,0xB9,0x28,0x5D,0xCC,0xBE,0x2F,
    0xE0,0x71,0x03,0x92,0xE7,0x76,0x04,0x95,0xEE,0x7F,0x0D,0x9C,0xE9,0x78,0x0A,0x9B,
    0xFC,0x6D,0x1F,0x8E,0xFB,0x6A,0x18,0x89,0xF2,0x63,0x11,0x80,0xF5,0x64,0x16,0x87,
    0xD8,0x49,0x3B,0xAA,0xDF,0x4E,0x3C,0xAD,0xD6,0x47,0x35,0xA4,0xD1,0x40,0x32,0xA3,
    0xC4,0x55,0x27,0xB6,0xC3,0x52,0x20,0xB1,0xCA,0x5B,0x29,0xB8,0xCD,0x5C,0x2E,0xBF,
    0x90,0x01,0x73,0xE2,0x97,0x06,0x74,0xE5,0x9E,0x0F,0x7D,0xEC,0x99,0x08,0x7A,0xEB,
    0x8C,0x1D,0x6F,0xFE,0x8B,0x1A,0x68,0xF9,0x82,0x13,0x61,0xF0,0x85,0x14,0x66,0xF7,
    0xA8,0x39,0x4B,0xDA,0xAF,0x3E,0x4C,0xDD,0xA6,0x37,0x45,0xD4,0xA1,0x30,0x42,0xD3,
    0xB4,0x25,0x57,0xC6,0xB3,0x22,0x50,0xC1,0xBA,0x2B,0x59,0xC8,0xBD,0x2C,0x5E,0xCF
};
#endif

static uint8_t crc8(const uint8_t *data, size_t size)
{
    uint8_t crc = 0U;
    for (size_t i = 0U; i < size; ++i) {
        crc = tmcCRCTable_Poly7Reflected[crc ^ data[i]];
    }
    crc = (uint8_t)(((crc >> 1U) & 0x55U) | ((crc & 0x55U) << 1U));
    crc = (uint8_t)(((crc >> 2U) & 0x33U) | ((crc & 0x33U) << 2U));
    return (uint8_t)((crc >> 4U) | (crc << 4U));
}

static uint32_t unpack(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24U) | ((uint32_t)data[1] << 16U) |
           ((uint32_t)data[2] << 8U) | data[3];
}

static void pack(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24U);
    data[1] = (uint8_t)(value >> 16U);
    data[2] = (uint8_t)(value >> 8U);
    data[3] = (uint8_t)value;
}

static TMC2240Status spiFrame(uint16_t icID, uint8_t *data)
{
    TMC2240Status status = tmc2240_readWriteSPI(icID, data, 5U);
    if ((status == TMC2240_OK) && ((data[0] & 1U) != 0U)) {
        status = cacheOperation(icID, TMC2240_CACHE_RESET, 0U, 0U);
    }
    return status;
}

static TMC2240Status readTransport(uint16_t icID, TMC2240BusType bus,
                                   uint8_t address, uint32_t *value)
{
    uint8_t data[8] = { 0U };
    TMC2240Status status;
    if (bus == IC_BUS_SPI) {
        data[0] = address;
        status = spiFrame(icID, data);
        if (status != TMC2240_OK) {
            return status;
        }
        memset(data, 0, sizeof(data));
        data[0] = address;
        status = spiFrame(icID, data);
        if (status != TMC2240_OK) {
            return status;
        }
        *value = unpack(&data[1]);
        return TMC2240_OK;
    }
    data[0] = 0x05U;
    status = tmc2240_getNodeAddress(icID, &data[1]);
    if (status != TMC2240_OK) {
        return status;
    }
    if (data[1] == 255U) {
        return TMC2240_ERROR_RANGE;
    }
    data[2] = address;
    data[3] = crc8(data, 3U);
    status = tmc2240_readWriteUART(icID, data, 4U, 8U);
    if (status != TMC2240_OK) {
        return status;
    }
    if ((data[0] != 0x05U) || (data[1] != 0xFFU) || (data[2] != address)) {
        return TMC2240_ERROR_PROTOCOL;
    }
    if (data[7] != crc8(data, 7U)) {
        return TMC2240_ERROR_CRC;
    }
    *value = unpack(&data[3]);
    return TMC2240_OK;
}

TMC2240Status tmc2240_readRegister(uint16_t icID, uint8_t address, uint32_t *value)
{
    TMC2240RegisterInfo info;
    TMC2240BusType bus;
    uint32_t received = 0U;
    TMC2240Status status;
    if (value == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_getRegisterInfo(address, &info);
    if (status != TMC2240_OK) {
        return status;
    }
    status = getBus(icID, &bus);
    if (status != TMC2240_OK) {
        return status;
    }
    status = readTransport(icID, bus, address, &received);
    if (status != TMC2240_OK) {
        (void)cacheOperation(icID, TMC2240_CACHE_INVALIDATE, address, 0U);
        return status;
    }
    if ((address == TMC2240_GSTAT) &&
        ((received & (TMC2240_RESET_MASK | TMC2240_REGISTER_RESET_MASK)) != 0U)) {
        status = cacheOperation(icID, TMC2240_CACHE_RESET, 0U, 0U);
        if (status != TMC2240_OK) {
            return status;
        }
    }
    status = cacheOperation(icID, TMC2240_CACHE_OBSERVE, address, received);
    if (status == TMC2240_OK) {
        *value = received; /* Preserve the full data word, including 0xFFFFFFFF. */
    } else {
        (void)cacheOperation(icID, TMC2240_CACHE_INVALIDATE, address, 0U);
    }
    return status;
}

TMC2240Status tmc2240_writeRegister(uint16_t icID, uint8_t address, uint32_t value)
{
    TMC2240BusType bus;
    uint8_t data[8] = { 0U };
    TMC2240Status status = tmc2240_validateRegisterValue(address, value);
    if (status != TMC2240_OK) {
        return status;
    }
    status = getBus(icID, &bus);
    if (status != TMC2240_OK) {
        return status;
    }
    if (bus == IC_BUS_UART) {
        status = tmc2240_getNodeAddress(icID, &data[1]);
        if (status != TMC2240_OK) {
            return status;
        }
        if (data[1] == 255U) {
            return TMC2240_ERROR_RANGE;
        }
    }
    status = cacheOperation(icID, registers[address].write_one_to_clear
        ? TMC2240_CACHE_INVALIDATE : TMC2240_CACHE_DESIRE, address, value);
    if (status != TMC2240_OK) {
        return status;
    }
    if (bus == IC_BUS_SPI) {
        data[0] = address | TMC2240_WRITE_BIT;
        pack(&data[1], value);
        return spiFrame(icID, data);
    }
    data[0] = 0x05U;
    data[2] = address | TMC2240_WRITE_BIT;
    pack(&data[3], value);
    data[7] = crc8(data, 7U);
    return tmc2240_readWriteUART(icID, data, 8U, 0U);
}

TMC2240Status tmc2240_writeRegisterVerified(uint16_t icID, uint8_t address,
                                          uint32_t value)
{
    uint32_t actual;
    TMC2240Status status = tmc2240_validateRegisterValue(address, value);
    if (status != TMC2240_OK) {
        return status;
    }
    if (registers[address].write_one_to_clear) {
        return TMC2240_ERROR_ACCESS;
    }
    status = tmc2240_writeRegister(icID, address, value);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_readRegister(icID, address, &actual);
    if (status != TMC2240_OK) {
        return status;
    }
    return ((actual & registers[address].write_mask) == value)
        ? TMC2240_OK : TMC2240_ERROR_VERIFY;
}

TMC2240Status tmc2240_updateRegister(uint16_t icID, uint8_t address,
                                   uint32_t mask, uint32_t value)
{
    TMC2240RegisterInfo info;
    uint32_t previous;
    TMC2240Status status = tmc2240_getRegisterInfo(address, &info);
    if (status != TMC2240_OK) {
        return status;
    }
    if ((mask == 0U) || ((mask & ~info.write_mask) != 0U)) {
        return TMC2240_ERROR_ACCESS;
    }
    if ((value & ~mask) != 0U) {
        return TMC2240_ERROR_RANGE;
    }
    if ((address == TMC2240_CHOPCONF) &&
        ((mask & TMC2240_MRES_MASK) == TMC2240_MRES_MASK) &&
        (((value & TMC2240_MRES_MASK) >> TMC2240_MRES_SHIFT) > 8U)) {
        return TMC2240_ERROR_RANGE;
    }
    if (info.write_one_to_clear || (mask == info.write_mask)) {
        return tmc2240_writeRegister(icID, address, value);
    }
    status = tmc2240_readRegister(icID, address, &previous);
    if (status != TMC2240_OK) {
        return status;
    }
    return tmc2240_writeRegister(icID, address,
                                 (previous & info.write_mask & ~mask) | value);
}

static TMC2240Status checkField(RegisterField field)
{
    TMC2240RegisterInfo info;
    uint32_t base;
    TMC2240Status status = tmc2240_getRegisterInfo(field.address, &info);
    if (status != TMC2240_OK) {
        return status;
    }
    if ((field.mask == 0U) || (field.shift >= 32U) ||
        ((field.mask & ~info.read_mask) != 0U)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    base = field.mask >> field.shift;
    if (((base & 1U) == 0U) || ((base & (base + 1U)) != 0U) ||
        ((base << field.shift) != field.mask)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    return TMC2240_OK;
}

TMC2240Status tmc2240_fieldExtract(uint32_t data, RegisterField field, int64_t *value)
{
    uint32_t base;
    int64_t result;
    TMC2240Status status;
    if (value == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = checkField(field);
    if (status != TMC2240_OK) {
        return status;
    }
    base = field.mask >> field.shift;
    result = (data & field.mask) >> field.shift;
    if (field.isSigned && (((uint32_t)result & ((base >> 1U) + 1U)) != 0U)) {
        result -= (int64_t)base + 1;
    }
    *value = result;
    return TMC2240_OK;
}

TMC2240Status tmc2240_fieldUpdate(uint32_t data, RegisterField field,
                                int64_t value, uint32_t *result)
{
    uint32_t base;
    int64_t minimum;
    int64_t maximum;
    TMC2240Status status;
    if (result == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = checkField(field);
    if (status != TMC2240_OK) {
        return status;
    }
    base = field.mask >> field.shift;
    minimum = field.isSigned ? -((int64_t)(base >> 1U) + 1) : 0;
    maximum = field.isSigned ? (int64_t)(base >> 1U) : (int64_t)base;
    if ((value < minimum) || (value > maximum)) {
        return TMC2240_ERROR_RANGE;
    }
    *result = (data & ~field.mask) | (((uint32_t)value << field.shift) & field.mask);
    return TMC2240_OK;
}

TMC2240Status tmc2240_fieldRead(uint16_t icID, RegisterField field, int64_t *value)
{
    uint32_t data;
    TMC2240Status status;
    if (value == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = checkField(field);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_readRegister(icID, field.address, &data);
    return (status == TMC2240_OK) ? tmc2240_fieldExtract(data, field, value) : status;
}

TMC2240Status tmc2240_fieldWrite(uint16_t icID, RegisterField field, int64_t value)
{
    uint32_t bits;
    TMC2240Status status = tmc2240_fieldUpdate(0U, field, value, &bits);
    if (status != TMC2240_OK) {
        return status;
    }
    return tmc2240_updateRegister(icID, field.address, field.mask, bits);
}
