/*******************************************************************************
 * Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 *
 * Copyright © 2023 Analog Devices, Inc.
 *******************************************************************************/

#ifndef TMC_HELPERS_CONFIG_H_
#define TMC_HELPERS_CONFIG_H_

#include "Constants.h"
#include "Types.h"

/* Callback function pointer type.
 * MISRA Note: Callers must cast to the IC-specific function pointer type.
 * This is necessary because the callback signatures are IC-dependent. */
typedef void (*tmc_callback_config)(void);

/* Configuration state machine states */
typedef enum {
    CONFIG_READY   = 0,
    CONFIG_RESET   = 1,
    CONFIG_RESTORE = 2
} ConfigState;

/* Configuration descriptor structure */
typedef struct {
    ConfigState         state;
    uint8_t             configIndex;
    int32_t             shadowRegister[TMC_REGISTER_COUNT];
    uint8_t           (*reset)(void);
    uint8_t           (*restore)(void);
    tmc_callback_config callback;
    uint8_t             channel;
} ConfigurationTypeDef;

#endif /* TMC_HELPERS_CONFIG_H_ */
