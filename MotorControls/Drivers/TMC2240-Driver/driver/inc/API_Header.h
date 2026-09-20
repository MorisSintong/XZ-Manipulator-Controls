/*******************************************************************************
 * Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 *
 * Copyright © 2023 Analog Devices, Inc.
 *******************************************************************************/

#ifndef TMC_API_HEADER_H_
#define TMC_API_HEADER_H_

#include "Config.h"
#include "Macros.h"
#include "Constants.h"
#include "Bits.h"
#include "CRC.h"
#include "RegisterAccess.h"
#include <stdlib.h>
#include "Types.h"

typedef enum {
    TMC_ERROR_NONE     = 0x00,
    TMC_ERROR_GENERIC  = 0x01,
    TMC_ERROR_FUNCTION = 0x02,
    TMC_ERROR_MOTOR    = 0x08,
    TMC_ERROR_VALUE    = 0x10,
    TMC_ERROR_CHIP     = 0x40
} TMCError;

typedef enum {
    TMC_COMM_DEFAULT = 0,
    TMC_COMM_SPI     = 1,
    TMC_COMM_UART    = 2
} TMC_Comm_Mode;

#endif /* TMC_API_HEADER_H_ */
