/**
 * @file    console.h
 * @brief   Non-blocking text console on USART2 (VCOM of the on-board J-Link, 115200)
 *          mirrored to SEGGER RTT terminal 0.
 *
 * TX: ring buffer drained by DMA (DMA1 Stream6) - printing never blocks the
 * main loop; messages that do not fit are dropped and counted.
 * RX: circular DMA (DMA1 Stream5) with idle-line detection, consumed by
 * polling; RTT down-channel characters are merged into the same stream.
 * This byte stream is where the PC/vision positioning protocol will be
 * parsed later. Use only from the main loop (not from interrupts).
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void     console_init(UART_HandleTypeDef *huart);
void     console_write(const char *s, size_t n);
void     console_printf(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;
void     console_vprintf(const char *fmt, va_list args);
/** Next received character (UART or RTT), or -1. */
int      console_getc(void);
/** Housekeeping: restart reception after UART errors. */
void     console_poll(void);
uint32_t console_dropped_bytes(void);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H */
