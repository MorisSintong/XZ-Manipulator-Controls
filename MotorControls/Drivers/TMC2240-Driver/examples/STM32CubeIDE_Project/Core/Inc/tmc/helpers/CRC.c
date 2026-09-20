/*******************************************************************************
 * Copyright © 2017 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 *
 * Copyright © 2023 Analog Devices, Inc.
 *
 * Generic CRC8 implementation with run-time initialized lookup tables.
 * Supports multiple polynomials via table indices.
 *******************************************************************************/

#include "CRC.h"
#include <stddef.h>

typedef struct {
    uint8_t  table[256];
    uint8_t  polynomial;
    bool     isReflected;
    bool     initialized;
} CRCTypeDef;

static CRCTypeDef CRCTables[CRC_TABLE_COUNT];

static uint8_t  flipByte(uint8_t value);
static uint32_t flipBitsInBytes(uint32_t value);

TMC2240Status tmc_fillCRC8Table(uint8_t polynomial, bool isReflected, uint8_t index)
{
    uint32_t crcData;
    uint8_t *table;
    uint32_t i;
    int32_t  j;

    if (index >= CRC_TABLE_COUNT) {
        return TMC2240_ERROR_RANGE;
    }

    CRCTables[index].polynomial  = polynomial;
    CRCTables[index].isReflected = isReflected;
    table = &CRCTables[index].table[0];

    /* Extend polynomial to handle MSB propagation (bit 8 set) */
    uint32_t poly = ((uint32_t)polynomial) | 0x0100U;

    /*
     * Iterate over all 256 byte values compressed into a uint32_t.
     * The byte-wise +4 increment pattern processes 4 values per outer loop.
     * Loop ends when byte overflow is detected (0x04030200).
     */
    for (i = 0x03020100U; i != 0x04030200U; i += 0x04040404U) {
        crcData = isReflected ? flipBitsInBytes(i) : i;

        for (j = 0; j < 8; j++) {
            uint8_t isMSBSet = ((crcData & 0x80000000U) != 0U) ? 1U : 0U;

            crcData <<= 1U;

            if ((crcData & 0x00000100U) != 0U) { crcData ^= poly; }
            if ((crcData & 0x00010000U) != 0U) { crcData ^= (poly << 8U); }
            if ((crcData & 0x01000000U) != 0U) { crcData ^= (poly << 16U); }
            if (isMSBSet != 0U)                 { crcData ^= (poly << 24U); }
        }

        crcData = isReflected ? flipBitsInBytes(crcData) : crcData;

        *table++ = (uint8_t)crcData;  crcData >>= 8U;
        *table++ = (uint8_t)crcData;  crcData >>= 8U;
        *table++ = (uint8_t)crcData;  crcData >>= 8U;
        *table++ = (uint8_t)crcData;
    }

    CRCTables[index].initialized = true;
    return TMC2240_OK;
}

TMC2240Status tmc_CRC8(const uint8_t *data, uint32_t bytes, uint8_t index, uint8_t *crc)
{
    uint8_t result = 0U;
    uint8_t *table;

    if ((crc == NULL) || ((data == NULL) && (bytes != 0U))) {
        return TMC2240_ERROR_ARGUMENT;
    }
    if (index >= CRC_TABLE_COUNT) {
        return TMC2240_ERROR_RANGE;
    }
    if (!CRCTables[index].initialized) {
        return TMC2240_ERROR_NOT_INITIALIZED;
    }

    table = &CRCTables[index].table[0];

    while (bytes > 0U) {
        result = table[result ^ (*data)];
        data++;
        bytes--;
    }

    if (CRCTables[index].isReflected) {
        result = flipByte(result);
    }

    *crc = result;
    return TMC2240_OK;
}

TMC2240Status tmc_tableGetPolynomial(uint8_t index, uint8_t *polynomial)
{
    if (polynomial == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    if (index >= CRC_TABLE_COUNT) {
        return TMC2240_ERROR_RANGE;
    }
    if (!CRCTables[index].initialized) {
        return TMC2240_ERROR_NOT_INITIALIZED;
    }
    *polynomial = CRCTables[index].polynomial;
    return TMC2240_OK;
}

TMC2240Status tmc_tableIsReflected(uint8_t index, bool *reflected)
{
    if (reflected == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    if (index >= CRC_TABLE_COUNT) {
        return TMC2240_ERROR_RANGE;
    }
    if (!CRCTables[index].initialized) {
        return TMC2240_ERROR_NOT_INITIALIZED;
    }
    *reflected = CRCTables[index].isReflected;
    return TMC2240_OK;
}

/* Swap bits within a byte: [b7..b0] -> [b0..b7] */
static uint8_t flipByte(uint8_t value)
{
    value = (uint8_t)(((value >> 1U) & 0x55U) | ((value & 0x55U) << 1U));
    value = (uint8_t)(((value >> 2U) & 0x33U) | ((value & 0x33U) << 2U));
    value = (uint8_t)(((value >> 4U) & 0x0FU) | ((value & 0x0FU) << 4U));

    return value;
}

/* Reverse bits within each byte, preserving byte order */
static uint32_t flipBitsInBytes(uint32_t value)
{
    value = ((value >> 1) & 0x55555555U) | ((value & 0x55555555U) << 1);
    value = ((value >> 2) & 0x33333333U) | ((value & 0x33333333U) << 2);
    value = ((value >> 4) & 0x0F0F0F0FU) | ((value & 0x0F0F0F0FU) << 4);

    return value;
}
