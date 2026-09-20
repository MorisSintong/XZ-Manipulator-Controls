#include "tmc2209_os.h"
#if TMC2209_OS_FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stddef.h>
#include <limits.h>

#if configUSE_MUTEXES != 1
#error "TMC2209 requires FreeRTOS mutexes"
#endif
_Static_assert(configTICK_RATE_HZ > 0, "FreeRTOS tick rate must be positive");
_Static_assert(sizeof(TickType_t) == 2u || sizeof(TickType_t) == 4u,
               "Supported FreeRTOS tick widths are 16 and 32 bits");

static TickType_t finite_ticks(uint32_t ms)
{
    uint64_t ticks = ((uint64_t)ms * (uint64_t)configTICK_RATE_HZ + 999u) / 1000u;
    uint64_t max_finite = (uint64_t)portMAX_DELAY - 1u;
    return (TickType_t)(ticks > max_finite ? max_finite : ticks);
}

tmc2209_status_t TMC2209_Os_MutexCreate(tmc2209_mutex_t *m)
{
    if (m == NULL) return TMC2209_ERR_PARAM;
    *m = NULL;
#if configSUPPORT_DYNAMIC_ALLOCATION == 1
    *m = (tmc2209_mutex_t)xSemaphoreCreateMutex();
#endif
    return *m != NULL ? TMC2209_OK : TMC2209_ERR_OS;
}

void TMC2209_Os_MutexDelete(tmc2209_mutex_t m)
{
    if (m != NULL) vSemaphoreDelete((SemaphoreHandle_t)m);
}

tmc2209_status_t TMC2209_Os_MutexLock(tmc2209_mutex_t m, uint32_t timeout_ms)
{
    if (m == NULL) return TMC2209_ERR_PARAM;
    return xSemaphoreTake((SemaphoreHandle_t)m, finite_ticks(timeout_ms)) == pdTRUE
        ? TMC2209_OK : TMC2209_ERR_TIMEOUT;
}

tmc2209_status_t TMC2209_Os_MutexUnlock(tmc2209_mutex_t m)
{
    if (m == NULL) return TMC2209_ERR_PARAM;
    return xSemaphoreGive((SemaphoreHandle_t)m) == pdTRUE ? TMC2209_OK : TMC2209_ERR_OS;
}

tmc2209_status_t TMC2209_Os_SemCreate(tmc2209_sem_t *s)
{
    if (s == NULL) return TMC2209_ERR_PARAM;
    *s = NULL;
#if configSUPPORT_DYNAMIC_ALLOCATION == 1
    *s = (tmc2209_sem_t)xSemaphoreCreateBinary();
#endif
    return *s != NULL ? TMC2209_OK : TMC2209_ERR_OS;
}

void TMC2209_Os_SemDelete(tmc2209_sem_t s)
{
    if (s != NULL) vSemaphoreDelete((SemaphoreHandle_t)s);
}

tmc2209_status_t TMC2209_Os_SemTake(tmc2209_sem_t s, uint32_t timeout_ms)
{
    if (s == NULL) return TMC2209_ERR_PARAM;
    return xSemaphoreTake((SemaphoreHandle_t)s, finite_ticks(timeout_ms)) == pdTRUE
        ? TMC2209_OK : TMC2209_ERR_TIMEOUT;
}

tmc2209_status_t TMC2209_Os_SemGive(tmc2209_sem_t s)
{
    if (s == NULL) return TMC2209_ERR_PARAM;
    return xSemaphoreGive((SemaphoreHandle_t)s) == pdTRUE ? TMC2209_OK : TMC2209_ERR_OS;
}

tmc2209_status_t TMC2209_Os_SemGiveFromISR(tmc2209_sem_t s)
{
    if (s == NULL) return TMC2209_ERR_PARAM;
    BaseType_t woken = pdFALSE;
    BaseType_t result = xSemaphoreGiveFromISR((SemaphoreHandle_t)s, &woken);
    portYIELD_FROM_ISR(woken);
    return result == pdTRUE ? TMC2209_OK : TMC2209_ERR_OS;
}

tmc2209_status_t TMC2209_Os_DelayMs(uint32_t ms)
{
    if (ms != 0u) vTaskDelay(finite_ticks(ms));
    return TMC2209_OK;
}

uint32_t TMC2209_Os_GetTickMs(void)
{
    TimeOut_t snapshot;
    vTaskSetTimeOutState(&snapshot);
    uint64_t ticks = ((uint64_t)(uint32_t)snapshot.xOverflowCount <<
                      (sizeof(TickType_t) * CHAR_BIT)) | snapshot.xTimeOnEntering;
    /* Split quotient/remainder avoids overflow even with 32-bit tick wraps.
     * Casting before multiplication intentionally returns milliseconds mod 2^32.
     */
    return (uint32_t)(ticks / (uint64_t)configTICK_RATE_HZ) * 1000u +
           (uint32_t)((ticks % (uint64_t)configTICK_RATE_HZ) * 1000u /
                      (uint64_t)configTICK_RATE_HZ);
}
#endif
