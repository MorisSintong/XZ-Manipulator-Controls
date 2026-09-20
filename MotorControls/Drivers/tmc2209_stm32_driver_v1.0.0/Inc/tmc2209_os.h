/** Native FreeRTOS or single-context bare-metal adapter; not CMSIS handles. */
#ifndef TMC2209_OS_H
#define TMC2209_OS_H
#include "tmc2209_types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef void *tmc2209_mutex_t;
typedef void *tmc2209_sem_t;

/** Creation clears the output on failure. Static-only RTOS builds return ERR_OS. */
tmc2209_status_t TMC2209_Os_MutexCreate(tmc2209_mutex_t *m);
void TMC2209_Os_MutexDelete(tmc2209_mutex_t m);
/**
 * RTOS waits ceil(ms*tick_rate/1000), capped at portMAX_DELAY-1 (never infinite).
 * Zero means nonblocking. Kernel must be running; no call from an ISR.
 * Bare metal accepts only NULL handles and supplies no mutual exclusion.
 */
tmc2209_status_t TMC2209_Os_MutexLock(tmc2209_mutex_t m, uint32_t timeout_ms);
tmc2209_status_t TMC2209_Os_MutexUnlock(tmc2209_mutex_t m);

/** Binary semaphores are unsupported (ERR_OS) by the bare-metal backend. */
tmc2209_status_t TMC2209_Os_SemCreate(tmc2209_sem_t *s);
void TMC2209_Os_SemDelete(tmc2209_sem_t s);
tmc2209_status_t TMC2209_Os_SemTake(tmc2209_sem_t s, uint32_t timeout_ms);
tmc2209_status_t TMC2209_Os_SemGive(tmc2209_sem_t s);
/** ISR-safe only in FreeRTOS, at a syscall-safe interrupt priority. */
tmc2209_status_t TMC2209_Os_SemGiveFromISR(tmc2209_sem_t s);

/** Zero does not delay. RTOS clamps as above; bare metal rejects UINT32_MAX.
 * HAL_Delay requires an advancing HAL tick; RTOS waits require a running
 * scheduler/tick. UART polling timeouts are independent of these delay helpers.
 */
tmc2209_status_t TMC2209_Os_DelayMs(uint32_t ms);
/** Milliseconds modulo 2^32. RTOS extends native ticks with kernel overflow count. */
uint32_t TMC2209_Os_GetTickMs(void);
#ifdef __cplusplus
}
#endif
#endif
