/*******************************************************************************
 * Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 *
 * Copyright © 2023 Analog Devices, Inc.
 *******************************************************************************/

#ifndef TMC_BITS_H_
#define TMC_BITS_H_

#include <stdint.h>

#define BIT0   0x00000001U
#define BIT1   0x00000002U
#define BIT2   0x00000004U
#define BIT3   0x00000008U
#define BIT4   0x00000010U
#define BIT5   0x00000020U
#define BIT6   0x00000040U
#define BIT7   0x00000080U
#define BIT8   0x00000100U
#define BIT9   0x00000200U
#define BIT10  0x00000400U
#define BIT11  0x00000800U
#define BIT12  0x00001000U
#define BIT13  0x00002000U
#define BIT14  0x00004000U
#define BIT15  0x00008000U
#define BIT16  0x00010000U
#define BIT17  0x00020000U
#define BIT18  0x00040000U
#define BIT19  0x00080000U
#define BIT20  0x00100000U
#define BIT21  0x00200000U
#define BIT22  0x00400000U
#define BIT23  0x00800000U
#define BIT24  0x01000000U
#define BIT25  0x02000000U
#define BIT26  0x04000000U
#define BIT27  0x08000000U
#define BIT28  0x10000000U
#define BIT29  0x20000000U
#define BIT30  0x40000000U
#define BIT31  0x80000000U

#define BYTE0_MASK   0x00000000000000FFULL
#define BYTE0_SHIFT  0U
#define BYTE1_MASK   0x000000000000FF00ULL
#define BYTE1_SHIFT  8U
#define BYTE2_MASK   0x0000000000FF0000ULL
#define BYTE2_SHIFT  16U
#define BYTE3_MASK   0x00000000FF000000ULL
#define BYTE3_SHIFT  24U
#define BYTE4_MASK   0x000000FF00000000ULL
#define BYTE4_SHIFT  32U
#define BYTE5_MASK   0x0000FF0000000000ULL
#define BYTE5_SHIFT  40U
#define BYTE6_MASK   0x00FF000000000000ULL
#define BYTE6_SHIFT  48U
#define BYTE7_MASK   0xFF00000000000000ULL
#define BYTE7_SHIFT  56U

#define SHORT0_MASK   (BYTE0_MASK | BYTE1_MASK)
#define SHORT0_SHIFT  BYTE0_SHIFT
#define SHORT1_MASK   (BYTE2_MASK | BYTE3_MASK)
#define SHORT1_SHIFT  BYTE2_SHIFT
#define SHORT2_MASK   (BYTE4_MASK | BYTE5_MASK)
#define SHORT2_SHIFT  BYTE4_SHIFT
#define SHORT3_MASK   (BYTE6_MASK | BYTE7_MASK)
#define SHORT3_SHIFT  BYTE6_SHIFT

#define WORD0_MASK    (SHORT0_MASK | SHORT1_MASK)
#define WORD0_SHIFT   SHORT0_SHIFT
#define WORD1_MASK    (SHORT2_MASK | SHORT3_MASK)
#define WORD1_SHIFT   SHORT2_SHIFT

/* value must be unsigned and wide enough for the requested index; indices
 * must keep the shift strictly below its width. Use uint64_t for all 8 bytes. */
#define NIBBLE(value, n)  (((value) >> ((n) << 2U)) & 0x0FU)
#define BYTE(value, n)    (((value) >> ((n) << 3U)) & 0xFFU)
#define SHORT(value, n)   (((value) >> ((n) << 4U)) & 0xFFFFU)
#define WORD(value, n)    (((value) >> ((n) << 5U)) & 0xFFFFFFFFUL)

#define COMBINE_8_16(__1, __0) \
    ((((uint16_t)(__1)) << BYTE1_SHIFT) | (((uint16_t)(__0)) << BYTE0_SHIFT))

#define COMBINE_8_32(__3, __2, __1, __0) \
    ((((uint32_t)(__3)) << BYTE3_SHIFT) | \
     (((uint32_t)(__2)) << BYTE2_SHIFT) | \
     (((uint32_t)(__1)) << BYTE1_SHIFT) | \
     (((uint32_t)(__0)) << BYTE0_SHIFT))

#define COMBINE_16_32(__1, __0) \
    ((((uint32_t)(__1)) << SHORT1_SHIFT) | (((uint32_t)(__0)) << SHORT0_SHIFT))

#endif /* TMC_BITS_H_ */
