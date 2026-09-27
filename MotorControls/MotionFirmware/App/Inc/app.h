/**
 * @file    app.h
 * @brief   MotionFirmware application: driver bring-up, homing sequence,
 *          continuous cycling, button/LED, telemetry and the command console.
 */
#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

/** Call once after all MX_*_Init() functions. */
void app_init(void);
/** Call continuously from the main while(1) loop. */
void app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
