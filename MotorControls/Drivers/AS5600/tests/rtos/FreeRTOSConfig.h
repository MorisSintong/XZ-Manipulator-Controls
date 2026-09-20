#ifndef TEST_FREERTOS_CONFIG_H
#define TEST_FREERTOS_CONFIG_H

#include "mps2_device.h"

void test_assert_failed(const char *expression, const char *file, unsigned line)
    __attribute__((noreturn));
void test_task_switched_in(void);

#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      1000U
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES                    56
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_TICKLESS_IDLE                 0
#define configIDLE_SHOULD_YIELD                 1
#define configMINIMAL_STACK_SIZE                256U
#define configMAX_TASK_NAME_LEN                 16
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        0
#define configKERNEL_PROVIDED_STATIC_MEMORY      1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_TIMERS                        0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            0
#define configQUEUE_REGISTRY_SIZE               0
#define configKERNEL_INTERRUPT_PRIORITY         255U
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     128U
#define configUSE_OS2_THREAD_SUSPEND_RESUME      1
#define configUSE_OS2_THREAD_ENUMERATE           0
#define configUSE_OS2_EVENTFLAGS_FROM_ISR        0
#define configUSE_OS2_THREAD_FLAGS              1
#define configUSE_OS2_TIMER                     0
#define configUSE_OS2_MUTEX                     1
#define configUSE_OS2_CPU_AFFINITY               0

#define INCLUDE_xSemaphoreGetMutexHolder         1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_uxTaskGetStackHighWaterMark       1
#define INCLUDE_uxTaskPriorityGet                 1
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xTaskAbortDelay                   1
#define INCLUDE_xTimerPendFunctionCall           0

#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler
#define configASSERT(x) \
    do { if (!(x)) { test_assert_failed(#x, __FILE__, __LINE__); } } while (0)
#define traceTASK_SWITCHED_IN() test_task_switched_in()

#endif
