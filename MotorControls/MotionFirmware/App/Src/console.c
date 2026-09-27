/**
 * @file    console.c
 * @brief   Non-blocking UART/RTT console (see console.h).
 */
#include "console.h"

#include "SEGGER_RTT.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TX_SIZE   8192U
#define RX_SIZE   256U
#define LINE_MAX  256U

static UART_HandleTypeDef *g_uart;
static uint8_t             g_tx[TX_SIZE];
static volatile uint32_t   g_tx_head;  /* written by the main loop */
static volatile uint32_t   g_tx_tail;  /* advanced by the TX-complete ISR */
static volatile uint32_t   g_tx_len;   /* bytes in flight, 0 = idle */
static uint8_t             g_rx[RX_SIZE];
static uint32_t            g_rx_rd;
static volatile bool       g_rx_restart;
static uint32_t            g_dropped;

static inline uint32_t lock_irq(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static inline void unlock_irq(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* Start the next DMA chunk (contiguous part of the ring). Must be called with
 * interrupts locked or from the UART TX-complete interrupt. */
static void tx_kick(void)
{
    uint32_t len;

    if ((g_uart == NULL) || (g_tx_len != 0U) || (g_tx_head == g_tx_tail)) {
        return;
    }
    len = (g_tx_head > g_tx_tail) ? (g_tx_head - g_tx_tail) : (TX_SIZE - g_tx_tail);
    if (HAL_UART_Transmit_DMA(g_uart, &g_tx[g_tx_tail], (uint16_t)len) == HAL_OK) {
        g_tx_len = len;
    }
}

static void rx_start(void)
{
    g_rx_rd = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(g_uart, g_rx, RX_SIZE) == HAL_OK) {
        /* Data is consumed by polling the DMA counter; no half-transfer IRQs. */
        __HAL_DMA_DISABLE_IT(g_uart->hdmarx, DMA_IT_HT);
    }
}

void console_init(UART_HandleTypeDef *huart)
{
    SEGGER_RTT_Init();
    g_uart = huart;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_len = 0U;
    g_dropped = 0U;
    g_rx_restart = false;
    if (g_uart != NULL) {
        rx_start();
    }
}

void console_write(const char *s, size_t n)
{
    uint32_t p;
    uint32_t head;
    uint32_t free_bytes;
    size_t first;

    if ((s == NULL) || (n == 0U)) {
        return;
    }
    (void)SEGGER_RTT_Write(0U, s, (unsigned)n);
    if (g_uart == NULL) {
        return;
    }
    /* Single producer (main loop): the ISR only frees space, so the copy can
     * run with interrupts enabled; only the index update is locked. */
    p = lock_irq();
    head = g_tx_head;
    free_bytes = (g_tx_tail + TX_SIZE - head - 1U) % TX_SIZE;
    unlock_irq(p);
    if (n > free_bytes) {
        g_dropped += (uint32_t)n;
        return;
    }
    first = TX_SIZE - head;
    if (first > n) {
        first = n;
    }
    memcpy(&g_tx[head], s, first);
    memcpy(&g_tx[0], s + first, n - first);
    p = lock_irq();
    g_tx_head = (head + (uint32_t)n) % TX_SIZE;
    tx_kick();
    unlock_irq(p);
}

void console_vprintf(const char *fmt, va_list args)
{
    char line[LINE_MAX];
    int len = vsnprintf(line, sizeof(line), fmt, args);

    if (len <= 0) {
        return;
    }
    if ((size_t)len >= sizeof(line)) {
        len = (int)sizeof(line) - 1;
    }
    console_write(line, (size_t)len);
}

void console_printf(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    console_vprintf(fmt, args);
    va_end(args);
}

int console_getc(void)
{
    uint32_t wr;
    int c;

    if (SEGGER_RTT_HasKey() != 0) {
        return SEGGER_RTT_GetKey();
    }
    if ((g_uart == NULL) || (g_uart->hdmarx == NULL) || g_rx_restart) {
        return -1;
    }
    wr = RX_SIZE - __HAL_DMA_GET_COUNTER(g_uart->hdmarx);
    if (wr >= RX_SIZE) {
        wr = 0U;
    }
    if (wr == g_rx_rd) {
        return -1;
    }
    c = g_rx[g_rx_rd];
    g_rx_rd = (g_rx_rd + 1U) % RX_SIZE;
    return c;
}

void console_poll(void)
{
    uint32_t p;

    if (g_uart == NULL) {
        return;
    }
    if (g_rx_restart) {
        g_rx_restart = false;
        (void)HAL_UART_AbortReceive(g_uart);
        rx_start();
    }
    p = lock_irq();
    tx_kick(); /* in case a transfer could not be started earlier */
    unlock_irq(p);
}

uint32_t console_dropped_bytes(void)
{
    return g_dropped;
}

/* HAL callbacks (weak in the HAL) ----------------------------------------- */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_uart) {
        g_tx_tail = (g_tx_tail + g_tx_len) % TX_SIZE;
        g_tx_len = 0U;
        tx_kick();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_uart) {
        /* Overrun/framing/noise abort the DMA reception in the HAL; restart it
         * from the main loop. An aborted transmission is simply resumed. */
        g_rx_restart = true;
        if (huart->gState == HAL_UART_STATE_READY) {
            g_tx_len = 0U;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)huart;
    (void)size; /* reception is consumed by polling the DMA write position */
}
