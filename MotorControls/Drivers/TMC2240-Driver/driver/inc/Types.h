/*******************************************************************************
 * Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 *
 * Copyright © 2023 Analog Devices, Inc.
 * Copyright © 2024 Adapted for STM32 HAL integration
 *
 * MISRA C:2012 Compliance Notes:
 *   - Uses standard ISO C99 stdint.h types (Dir 4.6, Rule 21.6)
 *   - No custom type definitions that shadow standard types
 *   - bool from stdbool.h (Rule 21.6)
 *******************************************************************************/

#ifndef TMC_TYPES_H_
#define TMC_TYPES_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef TMC_TYPES_INTEGERS
#define TMC_TYPES_INTEGERS

typedef float  float32_t;
typedef double float64_t;

#ifndef TMC_TYPES_INTEGERS_UNSIGNED
#define TMC_TYPES_INTEGERS_UNSIGNED

#define U8_MAX   ((uint8_t)  255U)
#define U10_MAX  ((uint16_t) 1023U)
#define U12_MAX  ((uint16_t) 4095U)
#define U15_MAX  ((uint16_t) 32767U)
#define U16_MAX  ((uint16_t) 65535U)
#define U18_MAX  ((uint32_t) 262143UL)
#define U20_MAX  ((uint32_t) 1048575UL)
#define U22_MAX  ((uint32_t) 4194303UL)
#define U24_MAX  ((uint32_t) 16777215UL)
#define U32_MAX  ((uint32_t) 4294967295UL)

#endif /* TMC_TYPES_INTEGERS_UNSIGNED */

#ifndef TMC_TYPES_INTEGERS_SIGNED
#define TMC_TYPES_INTEGERS_SIGNED

#define S8_MAX   ((int8_t)   127)
#define S8_MIN   ((int8_t)  -128)
#define S16_MAX  ((int16_t)  32767)
#define S16_MIN  ((int16_t) -32768)
#define S24_MAX  ((int32_t)  8388607L)
#define S24_MIN  ((int32_t) -8388608L)
#define S32_MAX  ((int32_t)  2147483647L)
#define S32_MIN  ((int32_t) -2147483648L)

#endif /* TMC_TYPES_INTEGERS_SIGNED */

#endif /* TMC_TYPES_INTEGERS */

#ifndef TMC_TYPES_NULL
#define TMC_TYPES_NULL

#ifndef NULL
#define NULL ((void *) 0)
#endif /* NULL */

#ifndef FALSE
#define FALSE (0U)
#endif /* FALSE */

#ifndef TRUE
#define TRUE  (1U)
#endif /* TRUE */

#endif /* TMC_TYPES_NULL */

#endif /* TMC_TYPES_H_ */
