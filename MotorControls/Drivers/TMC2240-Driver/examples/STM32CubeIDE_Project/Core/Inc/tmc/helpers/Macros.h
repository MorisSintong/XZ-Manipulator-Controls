/*******************************************************************************
 * Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 *
 * Copyright © 2023 Analog Devices, Inc.
 *******************************************************************************/

#ifndef TMC_MACROS_H_
#define TMC_MACROS_H_

#include <stdint.h>

/* Sign-extend the low n bits, 1 <= n <= 32. n must have no side effects. */
#define CAST_Sn_TO_S32(value, n) \
    ((int32_t)((int64_t)(((uint32_t)(value) & (UINT32_MAX >> (32U - (uint32_t)(n)))) \
        ^ (UINT32_C(1) << ((uint32_t)(n) - 1U))) \
        - (INT64_C(1) << ((uint32_t)(n) - 1U))))

/* Arguments to MIN/MAX may be evaluated twice; do not pass side effects. */
#ifndef MIN
    #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
    #define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* Static array length */
#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* Raw mask/shift macros require 0 <= shift < 32. Prefer checked field APIs
 * when descriptors/values are not compile-time trusted constants. */
#define FIELD_GET(data, mask, shift) \
    (((uint32_t)(data) & (uint32_t)(mask)) >> (uint32_t)(shift))

#define FIELD_SET(data, mask, shift, value) \
    (((uint32_t)(data) & (~((uint32_t)(mask)))) \
    | (((uint32_t)(value) << (uint32_t)(shift)) & (uint32_t)(mask)))

/* Macro to suppress unused parameter warnings */
#ifndef UNUSED
    #define UNUSED(x) ((void)(x))
#endif

/* Macro to remove write bit for shadow register array access */
#ifndef TMC_ADDRESS
    #define TMC_ADDRESS(x) ((uint8_t)((x) & 0x7FU))
#endif

#endif /* TMC_MACROS_H_ */
