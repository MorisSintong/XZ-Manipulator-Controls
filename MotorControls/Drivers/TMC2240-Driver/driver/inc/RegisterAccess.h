/*******************************************************************************
 * Copyright © 2017 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 *
 * Copyright © 2023 Analog Devices, Inc.
 *******************************************************************************/

#ifndef TMC_HELPERS_REGISTERACCESS_H_
#define TMC_HELPERS_REGISTERACCESS_H_

#include <stdint.h>

/* Register access permission bits.
 * Lower nibble: read/write. Upper nibble: special cases. */
#define TMC_ACCESS_NONE         0x00U
#define TMC_ACCESS_READ         0x01U
#define TMC_ACCESS_WRITE        0x02U
                                 /* 0x04 reserved */
#define TMC_ACCESS_DIRTY        0x08U  /* Written since reset -> shadow valid */

/* Special register bits */
#define TMC_ACCESS_RW_SPECIAL   0x10U  /* Read and write are independent */
#define TMC_ACCESS_FLAGS        0x20U  /* Read/write-to-clear flags */
#define TMC_ACCESS_HW_PRESET    0x40U  /* Hardware presets - skip in reset */

/* Permission combinations */
#define TMC_ACCESS_RW           (TMC_ACCESS_READ | TMC_ACCESS_WRITE)
#define TMC_ACCESS_RW_SEPARATE  (TMC_ACCESS_RW | TMC_ACCESS_RW_SPECIAL)
#define TMC_ACCESS_R_FLAGS      (TMC_ACCESS_READ | TMC_ACCESS_FLAGS)
#define TMC_ACCESS_RW_FLAGS     (TMC_ACCESS_RW | TMC_ACCESS_FLAGS)
#define TMC_ACCESS_W_PRESET     (TMC_ACCESS_WRITE | TMC_ACCESS_HW_PRESET)
#define TMC_ACCESS_RW_PRESET    (TMC_ACCESS_RW | TMC_ACCESS_HW_PRESET)

/* Helper macros */
#define TMC_IS_READABLE(x)       (((x) & TMC_ACCESS_READ) != 0U)
#define TMC_IS_WRITABLE(x)       (((x) & TMC_ACCESS_WRITE) != 0U)
#define TMC_IS_DIRTY(x)          (((x) & TMC_ACCESS_DIRTY) != 0U)
#define TMC_IS_PRESET(x)         (((x) & TMC_ACCESS_HW_PRESET) != 0U)
#define TMC_IS_RESETTABLE(x)     (((x) & (TMC_ACCESS_WRITE | TMC_ACCESS_HW_PRESET)) == TMC_ACCESS_WRITE)
#define TMC_IS_RESTORABLE(x)     ((((x) & TMC_ACCESS_WRITE) != 0U) && \
                                  ((!TMC_IS_PRESET(x)) || TMC_IS_DIRTY(x)))

/* Struct for listing register constants (write-only, hardware-preset) */
typedef struct {
    uint8_t  address;
    uint32_t value;
} TMCRegisterConstant;

/* Visual helpers for register permission arrays */
#define ____ 0x00U
#define N_A  0

#endif /* TMC_HELPERS_REGISTERACCESS_H_ */
