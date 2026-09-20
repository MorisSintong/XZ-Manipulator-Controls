/*******************************************************************************
 * Copyright © 2017 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 *
 * Copyright © 2023 Analog Devices, Inc.
 *******************************************************************************/

#ifndef TMC_HELPERS_CRC_H_
#define TMC_HELPERS_CRC_H_

#include "Types.h"
#include "tmc2240_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of CRC tables available (~260 bytes each) */
#define CRC_TABLE_COUNT 2U

TMC2240Status tmc_fillCRC8Table(uint8_t polynomial, bool isReflected, uint8_t index);
/* A NULL data pointer is allowed only for an empty message. Outputs remain
 * unchanged on failure; an uninitialized table is not a valid CRC calculator.
 */
TMC2240Status tmc_CRC8(const uint8_t *data, uint32_t bytes, uint8_t index, uint8_t *crc);

TMC2240Status tmc_tableGetPolynomial(uint8_t index, uint8_t *polynomial);
TMC2240Status tmc_tableIsReflected(uint8_t index, bool *reflected);

#ifdef __cplusplus
}
#endif
#endif /* TMC_HELPERS_CRC_H_ */
