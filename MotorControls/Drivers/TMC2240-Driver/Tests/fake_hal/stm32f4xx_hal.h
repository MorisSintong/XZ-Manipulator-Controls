#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct { unsigned id; } SPI_TypeDef;
typedef struct { unsigned id; } GPIO_TypeDef;
extern SPI_TypeDef fake_spi_instances[4];
extern GPIO_TypeDef fake_gpio_ports[3];
#define SPI1 (&fake_spi_instances[0])
#define SPI2 (&fake_spi_instances[1])
#define SPI3 (&fake_spi_instances[2])
#define SPI4 (&fake_spi_instances[3])
#define GPIOA (&fake_gpio_ports[0])
#define GPIOB (&fake_gpio_ports[1])
#define GPIOC (&fake_gpio_ports[2])
#define IS_SPI_ALL_INSTANCE(x) ((x) == SPI1 || (x) == SPI2 || (x) == SPI3 || (x) == SPI4)
#define IS_GPIO_ALL_INSTANCE(x) ((x) == GPIOA || (x) == GPIOB || (x) == GPIOC)
#define SPI_MODE_MASTER 0x104U
#define SPI_DIRECTION_2LINES 0U
#define SPI_DATASIZE_8BIT 0U
#define SPI_POLARITY_HIGH 2U
#define SPI_PHASE_2EDGE 1U
#define SPI_NSS_SOFT 0x200U
#define SPI_FIRSTBIT_MSB 0U
#define SPI_TIMODE_DISABLE 0U
#define SPI_CRCCALCULATION_DISABLE 0U
#define SPI_BAUDRATEPRESCALER_2 0U
#define SPI_BAUDRATEPRESCALER_4 8U
#define SPI_BAUDRATEPRESCALER_8 16U
#define SPI_BAUDRATEPRESCALER_16 24U
#define SPI_BAUDRATEPRESCALER_32 32U
#define SPI_BAUDRATEPRESCALER_64 40U
#define SPI_BAUDRATEPRESCALER_128 48U
#define SPI_BAUDRATEPRESCALER_256 56U
#define IS_SPI_BAUDRATE_PRESCALER(x) (((x) <= 56U) && (((x) % 8U) == 0U))

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_SPI_STATE_RESET, HAL_SPI_STATE_READY, HAL_SPI_STATE_BUSY } HAL_SPI_StateTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct {
    uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS;
    uint32_t BaudRatePrescaler, FirstBit, TIMode, CRCCalculation;
} SPI_InitTypeDef;
typedef struct {
    SPI_TypeDef *Instance;
    SPI_InitTypeDef Init;
    HAL_SPI_StateTypeDef State;
} SPI_HandleTypeDef;
extern uint32_t SystemCoreClock;
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
    uint8_t *rx, uint16_t length, uint32_t timeout);
void model_nop(void);
#define __NOP() model_nop()
#endif
