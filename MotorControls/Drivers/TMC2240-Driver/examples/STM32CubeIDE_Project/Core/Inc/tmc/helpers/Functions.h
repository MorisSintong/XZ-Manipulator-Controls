/*******************************************************************************
 * Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 *
 * Copyright © 2023 Analog Devices, Inc.
 *******************************************************************************/

#ifndef TMC_FUNCTIONS_H_
#define TMC_FUNCTIONS_H_

#include <stdint.h>
#include "tmc2240_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bounds must satisfy min <= max. */
int32_t tmc_limitInt(int32_t value, int32_t min, int32_t max);
int64_t tmc_limitS64(int64_t value, int64_t min, int64_t max);
/* Floor(sqrt(x)); negative x returns -1. */
int32_t tmc_sqrti(int32_t x);
/* Checked fixed-point PT1 step, 0 <= actualFilter <= maxFilter <= 31.
 * Rounds negative fractional output down, matching an arithmetic right shift.
 * Accumulator and output are unchanged on null, range, or overflow errors.
 */
TMC2240Status tmc_filterPT1(int64_t *akku, int32_t newValue, int32_t lastValue,
                           uint8_t actualFilter, uint8_t maxFilter, int32_t *out);

#ifdef __cplusplus
}
#endif
#endif /* TMC_FUNCTIONS_H_ */
