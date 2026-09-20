#ifndef APP_TMC2209_EXAMPLE_H
#define APP_TMC2209_EXAMPLE_H
#include "tmc2209.h"
#ifdef __cplusplus
extern "C" {
#endif

/* One caller context; UART1 must already be initialized. Hold external ENN
 * inactive throughout setup and faults. Init probes identity without writes.
 * Configure explicit application values, then separately activate and enable ENN.
 * Check every return value; Deinit does not disable physical outputs.
 */
tmc2209_status_t TMC2209_AppInit(void);
tmc2209_status_t TMC2209_AppPoll(void);
tmc2209_status_t TMC2209_AppConfigureMotor(const tmc2209_motor_config_t *config);
tmc2209_status_t TMC2209_AppActivate(uint8_t toff);
tmc2209_status_t TMC2209_AppDeactivate(void);
tmc2209_status_t TMC2209_AppClearGSTAT(uint8_t flags);
tmc2209_status_t TMC2209_AppDeinit(void);

#ifdef __cplusplus
}
#endif
#endif
