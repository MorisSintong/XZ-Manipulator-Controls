#ifndef APP_TMC2209_RTOS_EXAMPLE_H
#define APP_TMC2209_RTOS_EXAMPLE_H

#include "tmc2209.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * One shared unit, one initialization attempt per boot. Call from task context
 * after osKernelStart, with the scheduler unlocked. Requires CMSIS-RTOS2 over
 * FreeRTOS, dynamic mutex/thread allocation, TMC2209_OS_FREERTOS=1,
 * TMC2209_OS_NONE=0 and TMC2209_OS_CREATE_MUTEX=1. Link the real FreeRTOS adapter.
 * UART must already be configured for F446 half-duplex, 8N1, no flow control.
 * The UART storage must outlive this service; do not share it with an
 * uncoordinated HAL user or another driver bus.
 *
 * Init only attaches and probes communication; it neither acknowledges faults,
 * configures current/chopper registers nor activates the motor. IsReady means
 * the communication service is ready, NOT that a motor is safe to operate.
 *
 * Externally assert ENN and stop STEP generation before Init, ClearGSTAT,
 * Configure, Activate or Deactivate. Reset defaults can energize a motor;
 * successful communication alone does not prove disabled outputs.
 * Keep ENN asserted until the checked operation succeeds.
 * Supply all motor settings explicitly, including CHOPCONF.TOFF=0 to Configure.
 * ClearGSTAT disarms any previous profile; Configure must precede Activate.
 * Nonzero UART velocity is unsupported by the driver.
 *
 * A native CMSIS priority-inheritance mutex covers every shared-unit operation,
 * outside the driver's separate physical-bus mutex. Mutex waits are bounded to
 * 2 s, startup retries to five, and worker startup waiting to 1 s. Diagnostics
 * run at 500 ms / 2 s intervals; these are not motion/safety watchdogs.
 *
 * Driver/OS failures latch the first error and prohibit further unit operations.
 * An already-running driver call cannot be cancelled. Synchronous callers must
 * check returned status and disable hardware on failure. Asynchronous failure
 * calls application-supplied Error_Handler once after the unlock attempt, then
 * permanently parks the worker even if the handler returns. Failed release can
 * leave a mutex owned, so the worker is not deleted. Error_Handler MUST assert ENN
 * and stop/disable external STEP generation, including timer/DMA sources:
 * disabling CPU interrupts alone does NOT stop an external STEP timer. A broken
 * RTOS delay can make the fault park loop spin; a fatal handler should reset/halt
 * after disabling hardware, and must not try to resume this service.
 *
 * Failed startup attempts clean up what can be safely reclaimed. Cleanup
 * failures are separately observable. Runtime faults retain service storage
 * until application reset; no automatic reconfiguration, restart, or retry of
 * motor actions is provided. Do not access/copy the private live bus/unit.
 */
typedef struct {
    uint32_t ifcnt;
    uint32_t gstat;
    uint32_t drv_status;
} tmc2209_rtos_diagnostics_t;

tmc2209_status_t TMC2209_RtosInit(UART_HandleTypeDef *uart, uint32_t baud,
                                 uint8_t address);
tmc2209_status_t TMC2209_RtosConfigure(const tmc2209_motor_config_t *config);
tmc2209_status_t TMC2209_RtosActivate(uint8_t toff);
tmc2209_status_t TMC2209_RtosDeactivate(void);
tmc2209_status_t TMC2209_RtosClearGSTAT(uint8_t flags);
/* Output is unchanged on failure. Before configuration, fault bits are only
 * observed for deliberate diagnosis/acknowledgement. After Configure succeeds,
 * reset/driver fault observations also latch ERR_FAULT and stop the service. */
tmc2209_status_t TMC2209_RtosReadDiagnostics(tmc2209_rtos_diagnostics_t *out);
bool TMC2209_RtosIsReady(void);
/* Both getters are lock-free; before the first init attempt they return STATE.
 * GetStatus preserves the first operational error. GetCleanupStatus preserves
 * the first resource-release, startup-cleanup or fault-park OS error. */
tmc2209_status_t TMC2209_RtosGetStatus(void);
tmc2209_status_t TMC2209_RtosGetCleanupStatus(void);

#ifdef __cplusplus
}
#endif
#endif
