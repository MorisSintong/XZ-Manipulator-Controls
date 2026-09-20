#ifndef TEST_STM32F4XX_HAL_H
#define TEST_STM32F4XX_HAL_H

#include <stdint.h>

typedef struct
{
    uint32_t unused;
} I2C_TypeDef;

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef enum
{
    HAL_I2C_STATE_RESET = 0,
    HAL_I2C_STATE_READY = 0x20,
    HAL_I2C_STATE_BUSY = 0x24
} HAL_I2C_StateTypeDef;

typedef enum
{
    HAL_TICK_FREQ_10HZ = 100,
    HAL_TICK_FREQ_100HZ = 10,
    HAL_TICK_FREQ_1KHZ = 1
} HAL_TickFreqTypeDef;

typedef struct
{
    uint32_t ClockSpeed;
    uint32_t AddressingMode;
} I2C_InitTypeDef;

typedef struct
{
    I2C_TypeDef *Instance;
    I2C_InitTypeDef Init;
    volatile HAL_I2C_StateTypeDef State;
    volatile uint32_t ErrorCode;
} I2C_HandleTypeDef;

#define I2C_ADDRESSINGMODE_7BIT  UINT32_C(0x4000)
#define I2C_ADDRESSINGMODE_10BIT UINT32_C(0xC000)
#define I2C_MEMADD_SIZE_8BIT     UINT16_C(1)
#define HAL_I2C_ERROR_NONE      UINT32_C(0)
#define HAL_I2C_ERROR_BERR      UINT32_C(1)
#define HAL_I2C_ERROR_ARLO      UINT32_C(2)
#define HAL_I2C_ERROR_AF        UINT32_C(4)
#define HAL_I2C_ERROR_OVR       UINT32_C(8)
#define HAL_I2C_ERROR_DMA       UINT32_C(16)
#define HAL_I2C_ERROR_TIMEOUT   UINT32_C(32)

HAL_StatusTypeDef HAL_I2C_Mem_Read(
    I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
    uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(
    I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
    uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout);
uint32_t HAL_GetTick(void);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_Delay(uint32_t delay);
uint32_t __get_IPSR(void);
uint32_t __get_PRIMASK(void);
uint32_t __get_BASEPRI(void);
uint32_t __get_FAULTMASK(void);
uint32_t __get_CONTROL(void);

#endif
