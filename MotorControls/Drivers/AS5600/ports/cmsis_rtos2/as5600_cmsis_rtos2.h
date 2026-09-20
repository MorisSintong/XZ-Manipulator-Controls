#ifndef AS5600_CMSIS_RTOS2_H
#define AS5600_CMSIS_RTOS2_H

#include "as5600.h"
#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    osMutexId_t mutex;
    uint32_t max_wait_ticks;
} as5600_cmsis_rtos2_t;

/*
 * The application owns a non-recursive, priority-inheriting mutex.
 * max_wait_ticks must fit the backend's finite tick range and be <=INT32_MAX.
 * For 32-bit FreeRTOS ticks use 0x7FFFFFFF; for 16-bit ticks use 0xFFFE.
 * This function allocates nothing and may run before osKernelStart().
 */
as5600_result_t as5600_cmsis_rtos2_init(
    as5600_cmsis_rtos2_t *adapter, osMutexId_t mutex,
    uint32_t max_wait_ticks, as5600_sync_t *synchronization);

#ifdef __cplusplus
}
#endif

#endif
