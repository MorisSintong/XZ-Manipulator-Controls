/**
 * @file    stepgen.h
 * @brief   Timer-interrupt STEP/DIR pulse generation for the TMC2240 drivers.
 *
 * One 32-bit timer per axis (TIM2, TIM5) runs at the timer clock with the
 * auto-reload preload disabled; every update interrupt emits one microstep and
 * programs the next interval from the trapezoidal planner (motion_profile.c).
 * The TMC2240 is configured with CHOPCONF.dedge = 1, so every STEP *edge* is a
 * step: the ISR toggles the pin once and no pulse-width timing is required.
 *
 * All functions except stepgen_irq_handler() are for thread (main loop) use.
 */
#ifndef STEPGEN_H
#define STEPGEN_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STEPGEN_MAX_AXES 2U

typedef struct {
    TIM_TypeDef  *tim;
    IRQn_Type     irq;
    GPIO_TypeDef *step_port;
    uint16_t      step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
} stepgen_hw_t;

void     stepgen_init(uint8_t ax, const stepgen_hw_t *hw);
void     stepgen_set_vmin(uint8_t ax, float vmin);
void     stepgen_move_to(uint8_t ax, int32_t target, float vmax, float accel);
void     stepgen_run(uint8_t ax, int8_t dir, float vmax, float accel);
void     stepgen_stop(uint8_t ax);
void     stepgen_halt(uint8_t ax);
bool     stepgen_busy(uint8_t ax);
int32_t  stepgen_position(uint8_t ax);
bool     stepgen_set_position(uint8_t ax, int32_t pos);
float    stepgen_velocity(uint8_t ax);
bool     stepgen_at_cruise(uint8_t ax);
void     stepgen_set_limits(uint8_t ax, int32_t lo, int32_t hi);
bool     stepgen_take_limit_hit(uint8_t ax);
uint32_t stepgen_timer_hz(uint8_t ax);

/** Call from TIMx_IRQHandler (update interrupt). */
void     stepgen_irq_handler(uint8_t ax);

#ifdef __cplusplus
}
#endif

#endif /* STEPGEN_H */
