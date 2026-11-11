/**
 * @file    app_config.h
 * @brief   Application configuration: axis mapping, behaviour switches and
 *          telemetry. Per-axis mechanics/driver/homing values: app_config.c.
 *
 * Hardware mapping (wiring.md):
 *   Axis 0 = Z : TMC2240 #0  CS PC0  STEP PC1  DIR PC2  ENN PC6  TIM2  AS5600 on I2C3 (PA8/PC9)
 *   Axis 1 = X : TMC2240 #1  CS PC3  STEP PC4  DIR PC5  ENN PC7  TIM5  AS5600 on I2C1 (PB8/PB9)
 * If your wiring follows the thesis table instead (motor 0 = X), swap the two
 * entries of g_app_axis_cfg[] and APP_AXIS*_ENCODER_I2C.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "axis_cfg.h"
#include "axis_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_AXIS_COUNT          2U
#define APP_AXIS_Z              0U
#define APP_AXIS_X              1U

/* AS5600 bus per axis (HAL handle names from main.c). */
#define APP_AXIS0_ENCODER_I2C   hi2c3
#define APP_AXIS1_ENCODER_I2C   hi2c1

/* Homing order. Z first lifts the X arm away from the conveyor. */
#define APP_HOME_ORDER          { APP_AXIS_Z, APP_AXIS_X }

/* Cycle mode used by "start" / the button: AX_CYCLE_SOFT counts steps between
 * the StallGuard-measured ends, AX_CYCLE_BOUNCE hits the stops every stroke. */
#define APP_DEFAULT_CYCLE_MODE  AX_CYCLE_SOFT

/* 1 = home and start cycling automatically once the drivers are detected. */
#define APP_AUTOSTART           0

/* Telemetry: 0 = events only, 1 = periodic status, 2 = + SG4 trace. */
#define APP_LOG_LEVEL_DEFAULT   1
#define APP_STATUS_PERIOD_MS    250U   /* while moving */
#define APP_IDLE_STATUS_MS      5000U  /* while idle */
#define APP_SG_TRACE_PERIOD_MS  20U

/* Scheduling of the main loop services. */
#define APP_ENCODER_PERIOD_MS   2U
#define APP_HEALTH_PERIOD_MS    50U
#define APP_ADC_PERIOD_MS       1000U
#define APP_DRIVER_PROBE_MS     500U
#define APP_BUTTON_LONG_MS      1500U

/* Driver temperature warning / shutdown thresholds (TMC2240 ADC, 0.1 degC). */
#define APP_TEMP_WARN_C10       1000
#define APP_VM_MIN_MV           9000U

extern const axis_cfg_t g_app_axis_cfg[APP_AXIS_COUNT];

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
