#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Header-compatibility fixture, not a complete board's kernel configuration. */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configCPU_CLOCK_HZ                      180000000U
#define configTICK_RATE_HZ                      1000U
#define configMAX_PRIORITIES                    56
#define configMINIMAL_STACK_SIZE                128U
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        0
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               2U
#define configTIMER_QUEUE_LENGTH                10U
#define configTIMER_TASK_STACK_DEPTH             256U
#define configUSE_TRACE_FACILITY                1
#define configQUEUE_REGISTRY_SIZE               8U
#define configPRIO_BITS                         4U
#define configKERNEL_INTERRUPT_PRIORITY         (15U << 4U)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (5U << 4U)

#endif
