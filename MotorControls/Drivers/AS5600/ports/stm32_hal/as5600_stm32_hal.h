#ifndef AS5600_STM32_HAL_H
#define AS5600_STM32_HAL_H

#include "as5600.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    I2C_HandleTypeDef *i2c;
    as5600_sync_t synchronization;
    bool synchronized;
} as5600_stm32_hal_t;

/*
 * Call after HAL/I2C initialization, before sharing these objects.
 * synchronization == NULL is restricted to single-caller bare-metal use.
 * I2C must use 7-bit addressing, <=400 kHz, and a running 1 ms HAL timebase.
 */
as5600_result_t as5600_stm32_hal_init(
    as5600_t *device, as5600_stm32_hal_t *port, I2C_HandleTypeDef *i2c,
    const as5600_sync_t *synchronization);

#ifdef __cplusplus
}
#endif

#endif
