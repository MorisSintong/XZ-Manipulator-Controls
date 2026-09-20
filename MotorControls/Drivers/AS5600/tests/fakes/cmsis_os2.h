#ifndef TEST_CMSIS_OS2_H
#define TEST_CMSIS_OS2_H

#include <stdint.h>

typedef void *osMutexId_t;
typedef void *osThreadId_t;

typedef enum
{
    osOK = 0,
    osError = -1,
    osErrorTimeout = -2,
    osErrorResource = -3,
    osErrorParameter = -4,
    osErrorNoMemory = -5,
    osErrorISR = -6
} osStatus_t;

typedef enum
{
    osKernelInactive = 0,
    osKernelReady,
    osKernelRunning,
    osKernelLocked,
    osKernelSuspended,
    osKernelError = -1
} osKernelState_t;

#define osWaitForever UINT32_C(0xFFFFFFFF)

osKernelState_t osKernelGetState(void);
uint32_t osKernelGetTickFreq(void);
osThreadId_t osThreadGetId(void);
osThreadId_t osMutexGetOwner(osMutexId_t mutex);
osStatus_t osMutexAcquire(osMutexId_t mutex, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex);
osStatus_t osDelay(uint32_t ticks);

#endif
