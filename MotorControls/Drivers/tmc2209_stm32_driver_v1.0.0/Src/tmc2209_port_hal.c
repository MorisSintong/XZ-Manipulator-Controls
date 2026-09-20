/**
 * STM32F446 polling adapter. No HAL blocking wait (which would hang if SysTick
 * stopped): each polling loop uses CYCCNT plus a finite iteration backstop.
 * The UART and DWT clock configuration must not be changed during a call.
 */
#include "tmc2209_port.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>

#if defined(TMC2209_TEST_MMIO)
#include "tmc2209_test_mmio.h"
#else
#define TMC2209_CYCLES() (DWT->CYCCNT)
#define TMC2209_STATUS(h) ((h)->Instance->SR)
#define TMC2209_READ_BYTE(h) ((uint8_t)((h)->Instance->DR & 0xFFu))
#define TMC2209_WRITE_BYTE(h, v) ((h)->Instance->DR = (v))
#endif

#define RX_ERRORS (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)
typedef struct {
    uint32_t start;
    uint32_t cycles;
    uint32_t remaining;
} deadline_t;

static bool clock_ready(void)
{
    return SystemCoreClock >= 1000000u && SystemCoreClock <= 180000000u &&
           (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u &&
           (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0u;
}

static deadline_t deadline(uint32_t us)
{
    deadline_t d = {TMC2209_CYCLES(),
        (uint32_t)(((uint64_t)us * SystemCoreClock + 999999u) / 1000000u),
        TMC2209_POLL_LIMIT};
    return d;
}

static bool expired(deadline_t *d)
{
    if (d->remaining == 0u) return true;
    --d->remaining;
    return (uint32_t)(TMC2209_CYCLES() - d->start) >= d->cycles;
}

tmc2209_status_t TMC2209_Port_DelayUs(uint32_t us)
{
    if (us > 1000000u) return TMC2209_ERR_PARAM;
    if (us == 0u) return TMC2209_OK;
    if (!clock_ready()) return TMC2209_ERR_PORT;
    deadline_t d = deadline(us);
    for (;;) {
        if ((uint32_t)(TMC2209_CYCLES() - d.start) >= d.cycles) return TMC2209_OK;
        if (d.remaining == 0u) return TMC2209_ERR_TIMEOUT;
        --d.remaining;
    }
}

static bool handle_valid(const tmc2209_port_t *port)
{
    return port != NULL && port->huart != NULL && port->huart->Instance != NULL;
}

static bool port_ready(const tmc2209_port_t *port)
{
    return handle_valid(port) && port->initialized && clock_ready();
}

static tmc2209_status_t hal_result(HAL_StatusTypeDef result)
{
    switch (result) {
    case HAL_OK: return TMC2209_OK;
    case HAL_TIMEOUT: return TMC2209_ERR_TIMEOUT;
    case HAL_BUSY: return TMC2209_ERR_BUSY;
    default: return TMC2209_ERR_PORT;
    }
}

tmc2209_status_t TMC2209_Port_FlushRx(tmc2209_port_t *port)
{
    if (!handle_valid(port)) return TMC2209_ERR_PARAM;
    uint32_t remaining = TMC2209_RX_DRAIN_LIMIT;
    while ((TMC2209_STATUS(port->huart) & USART_SR_RXNE) != 0u) {
        if (remaining == 0u) {
            __HAL_UART_CLEAR_OREFLAG(port->huart);
            return TMC2209_ERR_TIMEOUT;
        }
        --remaining;
        (void)TMC2209_READ_BYTE(port->huart);
    }
    /* F446 SR then DR clears ORE/NE/FE/PE together; avoid four destructive reads. */
    __HAL_UART_CLEAR_OREFLAG(port->huart);
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_Port_Init(tmc2209_port_t *port)
{
    if (!handle_valid(port)) return TMC2209_ERR_PARAM;
    if (port->initialized) return TMC2209_ERR_STATE;
    UART_HandleTypeDef *h = port->huart;
    if (port->baud < 9000u || port->baud > 500000u ||
        h->Init.BaudRate != port->baud ||
        h->Init.WordLength != UART_WORDLENGTH_8B || h->Init.StopBits != UART_STOPBITS_1 ||
        h->Init.Parity != UART_PARITY_NONE || h->Init.HwFlowCtl != UART_HWCONTROL_NONE ||
        h->Init.Mode != UART_MODE_TX_RX || h->Init.OverSampling != UART_OVERSAMPLING_16 ||
        (h->Instance->CR3 & USART_CR3_HDSEL) == 0u ||
        (h->Instance->CR1 & USART_CR1_UE) == 0u ||
        h->gState != HAL_UART_STATE_READY || h->RxState != HAL_UART_STATE_READY ||
        port->tx_timeout_ms > 1000u || port->rx_timeout_ms > 1000u ||
        port->tx_timeout_ms < ((80000u + port->baud - 1u) / port->baud) + 1u ||
        port->rx_timeout_ms < ((200000u + port->baud - 1u) / port->baud) + 1u ||
        port->bus_idle_us > 1000000u)
        return TMC2209_ERR_PARAM;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    tmc2209_status_t st = TMC2209_Port_DelayUs(1u);
    if (st == TMC2209_OK) st = hal_result(HAL_HalfDuplex_EnableReceiver(h));
    if (st == TMC2209_OK) st = TMC2209_Port_FlushRx(port);
    if (st != TMC2209_OK) return st;
    port->awaiting_reply = false;
    port->initialized = true;
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_Port_Deinit(tmc2209_port_t *port)
{
    if (port == NULL) return TMC2209_OK;
    if (handle_valid(port)) CLEAR_BIT(port->huart->Instance->CR1, USART_CR1_TE | USART_CR1_RE);
    port->initialized = false;
    port->awaiting_reply = false;
    port->huart = NULL;
    return TMC2209_OK;
}

static tmc2209_status_t wait_flag(UART_HandleTypeDef *h, uint32_t flag, deadline_t *d)
{
    for (;;) {
        uint32_t status = TMC2209_STATUS(h);
        if ((status & RX_ERRORS) != 0u) return TMC2209_ERR_PORT;
        if (expired(d)) return TMC2209_ERR_TIMEOUT;
        if ((status & flag) != 0u) return TMC2209_OK;
    }
}

static tmc2209_status_t recover(tmc2209_port_t *port, tmc2209_status_t first)
{
    port->awaiting_reply = false;
    (void)HAL_HalfDuplex_EnableReceiver(port->huart);
    (void)TMC2209_Port_FlushRx(port);
    return first;
}

tmc2209_status_t TMC2209_Port_Transmit(tmc2209_port_t *port,
                                     const uint8_t *data, uint16_t len)
{
    if (!port_ready(port) || data == NULL || (len != 4u && len != 8u))
        return TMC2209_ERR_PARAM;
    if (port->awaiting_reply) return TMC2209_ERR_STATE;
    uint32_t idle_us = (80000000u + port->baud - 1u) / port->baud;
    if (port->bus_idle_us > idle_us) idle_us = port->bus_idle_us;
    tmc2209_status_t st = TMC2209_Port_DelayUs(idle_us);
    if (st == TMC2209_OK)
        st = hal_result(HAL_HalfDuplex_EnableTransmitter(port->huart));
    if (st == TMC2209_OK) st = TMC2209_Port_FlushRx(port);
    if (st != TMC2209_OK) return recover(port, st);
    deadline_t d = deadline(port->tx_timeout_ms * 1000u);
    for (uint16_t i = 0u; i < len; ++i) {
        st = wait_flag(port->huart, USART_SR_TXE, &d);
        if (st != TMC2209_OK) return recover(port, st);
        TMC2209_WRITE_BYTE(port->huart, data[i]);
    }
    st = wait_flag(port->huart, USART_SR_TC, &d);
    if (st != TMC2209_OK) return recover(port, st);
    /* RX was disabled while sending. Drain stale echo BEFORE enabling RX.
     * Do not delay here: SENDDELAY may be as short as eight bit times.
     */
    st = TMC2209_Port_FlushRx(port);
    if (st == TMC2209_OK)
        st = hal_result(HAL_HalfDuplex_EnableReceiver(port->huart));
    if (st != TMC2209_OK) return recover(port, st);
    port->awaiting_reply = len == 4u;
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_Port_Receive(tmc2209_port_t *port, uint8_t *data, uint16_t len)
{
    if (!port_ready(port) || data == NULL || len != 8u) return TMC2209_ERR_PARAM;
    if (!port->awaiting_reply) return TMC2209_ERR_STATE;
    port->awaiting_reply = false;
    deadline_t d = deadline(port->rx_timeout_ms * 1000u);
    for (uint16_t i = 0u; i < len; ++i) {
        tmc2209_status_t st = wait_flag(port->huart, USART_SR_RXNE, &d);
        if (st != TMC2209_OK) return recover(port, st);
        data[i] = TMC2209_READ_BYTE(port->huart);
    }
    if ((TMC2209_STATUS(port->huart) & RX_ERRORS) != 0u)
        return recover(port, TMC2209_ERR_PORT);
    /* Leave RE on and TE off. The next Transmit observes the slave release time. */
    return TMC2209_OK;
}
