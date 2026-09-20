/** Copy as tmc2209_conf.h onto the application include path.
 * STM32F446RE is the supported software target; other families are unverified.
 * No setting here selects motor current or activates the chopper.
 */
#ifndef TMC2209_CONF_H
#define TMC2209_CONF_H
#ifndef TMC2209_OS_FREERTOS
#define TMC2209_OS_FREERTOS 0
#endif
#ifndef TMC2209_OS_NONE
#define TMC2209_OS_NONE (!TMC2209_OS_FREERTOS)
#endif
#ifndef TMC2209_DEFAULT_BAUD
#define TMC2209_DEFAULT_BAUD 115200u
#endif
#ifndef TMC2209_BUS_LOCK_TIMEOUT_MS
#define TMC2209_BUS_LOCK_TIMEOUT_MS 100u
#endif
/* UART timeouts cover the whole frame, not each byte; 1..1000 ms.
 * RX must cover 200 bit times (maximum SENDDELAY + reply) plus a 1 ms margin.
 * Increase these for low baud rates; invalid combinations fail Port_Init.
 */
#ifndef TMC2209_UART_TX_TIMEOUT_MS
#define TMC2209_UART_TX_TIMEOUT_MS 20u
#endif
#ifndef TMC2209_UART_RX_TIMEOUT_MS
#define TMC2209_UART_RX_TIMEOUT_MS 20u
#endif
/* Always observe at least 80 bit times before a new transaction to recover
 * partial frames (63 + 12) and allow the slave to release TX (4 bit times).
 * Optional additional minimum idle, NOT a delay before receiving a reply.
 */
#ifndef TMC2209_BUS_IDLE_US
#define TMC2209_BUS_IDLE_US 0u
#endif
/* Secondary finite bounds if DWT stops or RXNE is continuously asserted. */
#ifndef TMC2209_POLL_LIMIT
#define TMC2209_POLL_LIMIT 10000000u
#endif
#ifndef TMC2209_RX_DRAIN_LIMIT
#define TMC2209_RX_DRAIN_LIMIT 32u
#endif
/* 0: call BusInitWithLock with an application-owned native FreeRTOS mutex.
 * Static-only FreeRTOS builds must supply a static mutex this way.
 * Bare metal supplies no concurrency: the external handle must be NULL.
 */
#ifndef TMC2209_OS_CREATE_MUTEX
#define TMC2209_OS_CREATE_MUTEX 1
#endif
#endif
