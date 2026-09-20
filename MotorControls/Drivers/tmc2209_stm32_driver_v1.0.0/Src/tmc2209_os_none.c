#include "tmc2209_os.h"
#if TMC2209_OS_NONE
#include "stm32f4xx_hal.h"
#include <stddef.h>

tmc2209_status_t TMC2209_Os_MutexCreate(tmc2209_mutex_t *m)
{
    if (m == NULL) return TMC2209_ERR_PARAM;
    *m = NULL;
    return TMC2209_OK;
}
void TMC2209_Os_MutexDelete(tmc2209_mutex_t m) { (void)m; }
tmc2209_status_t TMC2209_Os_MutexLock(tmc2209_mutex_t m, uint32_t timeout_ms)
{
    (void)timeout_ms;
    return m == NULL ? TMC2209_OK : TMC2209_ERR_PARAM;
}
tmc2209_status_t TMC2209_Os_MutexUnlock(tmc2209_mutex_t m)
{
    return m == NULL ? TMC2209_OK : TMC2209_ERR_PARAM;
}
tmc2209_status_t TMC2209_Os_SemCreate(tmc2209_sem_t *s)
{
    if (s == NULL) return TMC2209_ERR_PARAM;
    *s = NULL;
    return TMC2209_ERR_OS;
}
void TMC2209_Os_SemDelete(tmc2209_sem_t s) { (void)s; }
tmc2209_status_t TMC2209_Os_SemTake(tmc2209_sem_t s, uint32_t timeout_ms)
{
    (void)s; (void)timeout_ms;
    return TMC2209_ERR_OS;
}
tmc2209_status_t TMC2209_Os_SemGive(tmc2209_sem_t s)
{
    (void)s;
    return TMC2209_ERR_OS;
}
tmc2209_status_t TMC2209_Os_SemGiveFromISR(tmc2209_sem_t s)
{
    (void)s;
    return TMC2209_ERR_OS;
}
tmc2209_status_t TMC2209_Os_DelayMs(uint32_t ms)
{
    if (ms == UINT32_MAX) return TMC2209_ERR_PARAM;
    if (ms != 0u) HAL_Delay(ms);
    return TMC2209_OK;
}
uint32_t TMC2209_Os_GetTickMs(void) { return HAL_GetTick(); }
#endif
