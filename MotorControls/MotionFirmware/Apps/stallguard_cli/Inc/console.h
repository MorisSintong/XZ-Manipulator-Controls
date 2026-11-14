/**
 * @file    console.h
 * @brief   Non-blocking text console on USB-C CDC (XZ-MotionControls),
 *          USART2 (115200), and SEGGER RTT terminal 0.
 *
 * TX: ring buffer drained by DMA (DMA1 Stream6) + TinyUSB CDC + RTT.
 * RX: circular DMA (DMA1 Stream5) + TinyUSB CDC RX FIFO + RTT down-channel,
 * merged into one line stream. Use only from the main loop (not from ISRs).
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void console_init(UART_HandleTypeDef *huart);
void console_write(const char *s, size_t n);
void console_printf(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;
void console_vprintf(const char *fmt, va_list args);
/** Next received character (RTT, USB-C CDC, or UART), or -1. */
int console_getc(void);
/** Housekeeping: pump TinyUSB CDC events and restart reception after UART
 * errors. */
void console_poll(void);
uint32_t console_dropped_bytes(void);
bool console_usb_mounted(void);
bool console_usb_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H */
