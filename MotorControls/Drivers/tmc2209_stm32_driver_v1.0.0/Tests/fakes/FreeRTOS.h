#ifndef FAKE_FREERTOS_H
#define FAKE_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
#ifndef FAKE_TICK_BITS
#define FAKE_TICK_BITS 32
#endif
#ifndef FAKE_TICK_RATE
#define FAKE_TICK_RATE 1000
#endif
#if FAKE_TICK_BITS == 16
typedef uint16_t TickType_t;
#else
typedef uint32_t TickType_t;
#endif
typedef int32_t BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY ((TickType_t)~(TickType_t)0)
#define configTICK_RATE_HZ ((TickType_t)FAKE_TICK_RATE)
#define configUSE_MUTEXES 1
#ifndef configSUPPORT_DYNAMIC_ALLOCATION
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#endif
void fake_rtos_yield(BaseType_t woken);
#define portYIELD_FROM_ISR(woken) fake_rtos_yield(woken)
#endif
