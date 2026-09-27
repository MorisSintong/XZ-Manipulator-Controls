/**
 * @file    axis_hw.h
 * @brief   Hardware seam used by axis_ctrl.c.
 *
 * On the target these functions are implemented in axis_hw.c on top of
 * stepgen (timer ISR step generation), tmc_axis (TMC2240 over SPI) and
 * encoder (AS5600 over I2C). The host unit tests link a physics simulator
 * instead, so the complete homing/cycle logic can be verified off-target.
 */
#ifndef AXIS_HW_H
#define AXIS_HW_H

#include <stdbool.h>
#include <stdint.h>

#include "axis_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AXIS_PROFILE_HOMING = 0, /* StealthChop2 only, homing current, SG4 armed */
    AXIS_PROFILE_RUN         /* run current, hybrid chopper, optional CoolStep */
} axis_profile_t;

typedef enum {
    AXIS_HW_OK = 0,
    AXIS_HW_ERR_SPI,    /* transfer failed / chip not responding */
    AXIS_HW_ERR_DRIVER  /* chip reported reset or driver error */
} axis_hw_status_t;

typedef struct {
    int32_t  counts; /* multi-turn encoder counts */
    int32_t  pos;    /* commanded position sampled just before the reading */
    uint32_t epoch;  /* increments whenever multi-turn continuity was lost */
    uint32_t seq;    /* increments with every successful reading */
} axis_enc_sample_t;

uint32_t axis_hw_now_ms(void);
void     axis_hw_log(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

/* Driver ------------------------------------------------------------- */
void             axis_hw_driver_enable(uint8_t ax, bool enable); /* ENN pin */
axis_hw_status_t axis_hw_apply_profile(uint8_t ax, axis_profile_t profile,
                                       const axis_cfg_t *cfg);
axis_hw_status_t axis_hw_set_sg_threshold(uint8_t ax, uint8_t thrs);
axis_hw_status_t axis_hw_read_sg(uint8_t ax, uint16_t *sg4, bool *stall_flag);

/* Motion (step generator) -------------------------------------------- */
void    axis_hw_set_vmin(uint8_t ax, float vmin);
void    axis_hw_move_to(uint8_t ax, int32_t target, float vmax, float accel);
void    axis_hw_run(uint8_t ax, int8_t dir, float vmax, float accel);
void    axis_hw_stop(uint8_t ax);  /* decelerate to standstill */
void    axis_hw_halt(uint8_t ax);  /* stop immediately */
bool    axis_hw_busy(uint8_t ax);
int32_t axis_hw_position(uint8_t ax);
bool    axis_hw_set_position(uint8_t ax, int32_t pos); /* only while idle */
float   axis_hw_velocity(uint8_t ax);                  /* signed, steps/s */
bool    axis_hw_at_cruise(uint8_t ax);
void    axis_hw_set_limits(uint8_t ax, int32_t lo, int32_t hi);
bool    axis_hw_take_limit_hit(uint8_t ax);            /* read and clear */

/* Encoder ------------------------------------------------------------- */
/** Latest healthy, fresh sample; false if the encoder is unavailable. */
bool    axis_hw_encoder(uint8_t ax, axis_enc_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* AXIS_HW_H */
