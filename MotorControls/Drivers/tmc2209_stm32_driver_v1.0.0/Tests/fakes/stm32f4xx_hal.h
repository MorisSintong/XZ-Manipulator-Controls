#ifndef FAKE_STM32F4XX_HAL_H
#define FAKE_STM32F4XX_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { volatile uint32_t SR, DR, BRR, CR1, CR2, CR3; } USART_TypeDef;
typedef struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling; } UART_InitTypeDef;
typedef struct __UART_HandleTypeDef {
    USART_TypeDef *Instance;
    UART_InitTypeDef Init;
    uint32_t gState, RxState;
} UART_HandleTypeDef;
typedef struct { volatile uint32_t CTRL, CYCCNT; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
extern DWT_Type fake_dwt;
extern CoreDebug_Type fake_debug;
extern uint32_t SystemCoreClock;
#define DWT (&fake_dwt)
#define CoreDebug (&fake_debug)
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24u)
#define USART_SR_PE (1u << 0u)
#define USART_SR_FE (1u << 1u)
#define USART_SR_NE (1u << 2u)
#define USART_SR_ORE (1u << 3u)
#define USART_SR_RXNE (1u << 5u)
#define USART_SR_TC (1u << 6u)
#define USART_SR_TXE (1u << 7u)
#define USART_CR1_RE (1u << 2u)
#define USART_CR1_TE (1u << 3u)
#define USART_CR1_UE (1u << 13u)
#define USART_CR3_HDSEL (1u << 3u)
#define UART_WORDLENGTH_8B 0u
#define UART_STOPBITS_1 0u
#define UART_PARITY_NONE 0u
#define UART_HWCONTROL_NONE 0u
#define UART_MODE_TX_RX (USART_CR1_TE | USART_CR1_RE)
#define UART_OVERSAMPLING_16 0u
#define HAL_UART_STATE_READY 0x20u
#define CLEAR_BIT(reg, mask) ((reg) &= ~(uint32_t)(mask))
#define __HAL_UART_CLEAR_OREFLAG(h) fake_hal_clear_errors(h)
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *h);
void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);
void fake_hal_clear_errors(UART_HandleTypeDef *h);
#endif
