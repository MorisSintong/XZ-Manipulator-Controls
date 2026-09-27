/**
 * @file    axis_hw.c
 * @brief   Target implementation of the axis_hw.h seam: step generation
 *          (stepgen), TMC2240 (tmc_axis), AS5600 (encoder), ENN pins, log.
 */
#include "axis_hw.h"

#include "console.h"
#include "encoder.h"
#include "main.h"
#include "stepgen.h"
#include "tmc_axis.h"

#include <stdarg.h>
#include <stdio.h>

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} pin_t;

/* ENN is active low: HIGH = power stage off (also the reset state thanks to
 * the pull-up set up in MX_GPIO_Init). */
static const pin_t k_enn[2] = {
    { TMC_ENN0_GPIO_Port, TMC_ENN0_Pin },
    { TMC_ENN1_GPIO_Port, TMC_ENN1_Pin },
};

uint32_t axis_hw_now_ms(void)
{
    return HAL_GetTick();
}

void axis_hw_log(const char *fmt, ...)
{
    va_list args;
    char stamp[20];
    const uint32_t t = HAL_GetTick();
    const int n = snprintf(stamp, sizeof(stamp), "[%5lu.%03lu] ", (unsigned long)(t / 1000U),
                           (unsigned long)(t % 1000U));

    if (n > 0) {
        console_write(stamp, (size_t)n);
    }
    va_start(args, fmt);
    console_vprintf(fmt, args);
    va_end(args);
    console_write("\r\n", 2U);
}

void axis_hw_driver_enable(uint8_t ax, bool enable)
{
    if (ax < 2U) {
        HAL_GPIO_WritePin(k_enn[ax].port, k_enn[ax].pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

axis_hw_status_t axis_hw_apply_profile(uint8_t ax, axis_profile_t profile, const axis_cfg_t *cfg)
{
    return tmc_axis_apply_profile(ax, profile, cfg);
}

axis_hw_status_t axis_hw_set_sg_threshold(uint8_t ax, uint8_t thrs)
{
    return tmc_axis_set_sg_threshold(ax, thrs);
}

axis_hw_status_t axis_hw_read_sg(uint8_t ax, uint16_t *sg4, bool *stall_flag)
{
    return tmc_axis_read_sg(ax, sg4, stall_flag);
}

void axis_hw_set_vmin(uint8_t ax, float vmin)
{
    stepgen_set_vmin(ax, vmin);
}

void axis_hw_move_to(uint8_t ax, int32_t target, float vmax, float accel)
{
    stepgen_move_to(ax, target, vmax, accel);
}

void axis_hw_run(uint8_t ax, int8_t dir, float vmax, float accel)
{
    stepgen_run(ax, dir, vmax, accel);
}

void axis_hw_stop(uint8_t ax)
{
    stepgen_stop(ax);
}

void axis_hw_halt(uint8_t ax)
{
    stepgen_halt(ax);
}

bool axis_hw_busy(uint8_t ax)
{
    return stepgen_busy(ax);
}

int32_t axis_hw_position(uint8_t ax)
{
    return stepgen_position(ax);
}

bool axis_hw_set_position(uint8_t ax, int32_t pos)
{
    return stepgen_set_position(ax, pos);
}

float axis_hw_velocity(uint8_t ax)
{
    return stepgen_velocity(ax);
}

bool axis_hw_at_cruise(uint8_t ax)
{
    return stepgen_at_cruise(ax);
}

void axis_hw_set_limits(uint8_t ax, int32_t lo, int32_t hi)
{
    stepgen_set_limits(ax, lo, hi);
}

bool axis_hw_take_limit_hit(uint8_t ax)
{
    return stepgen_take_limit_hit(ax);
}

bool axis_hw_encoder(uint8_t ax, axis_enc_sample_t *sample)
{
    return encoder_sample(ax, sample);
}
