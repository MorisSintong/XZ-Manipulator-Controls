#ifndef STM32F4XX_HAL_CONF_H
#define STM32F4XX_HAL_CONF_H

/* Compile fixture only; use CubeMX's generated configuration on the board. */
#define HAL_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define USE_RTOS 0U
#define USE_HAL_I2C_REGISTER_CALLBACKS 0U

#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_i2c.h"

#define assert_param(expression) ((void)0U)

#endif
