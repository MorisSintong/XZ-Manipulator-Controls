/**
 * @file    app_config.c
 * @brief   Default per-axis configuration (pure data, also used by the host
 *          simulation tests). Values marked "tune" need bench verification.
 *
 * Mechanics (thesis ch. 3.5.2):
 *   Z: T8x8 lead screw (8 mm/rev), 200 fullsteps x 16 = 3200 usteps/rev -> 400 usteps/mm
 *   X: GT2 belt, 20T pulley (40 mm/rev)                          ->  80 usteps/mm
 * Encoder: AS5600 on the motor shaft, 4096 counts/rev -> 4096/3200 = 1.28 counts/ustep.
 */
#include "app_config.h"

#define USTEPS          16U
#define MOTOR_FULLSTEPS 200.0f
#define USTEPS_PER_REV  (MOTOR_FULLSTEPS * (float)USTEPS)
#define ENC_PER_USTEP   (4096.0f / USTEPS_PER_REV)

const axis_cfg_t g_app_axis_cfg[APP_AXIS_COUNT] = {
    [APP_AXIS_Z] = {
        .name              = 'Z',
        .steps_per_mm      = USTEPS_PER_REV / 8.0f,
        .vmin_steps_s      = 200.0f,
        .max_travel_mm     = 350.0f,   /* seek safety limit, > real Z stroke */
        .min_travel_mm     = 10.0f,
        .home_min_first    = false,    /* go UP (MAX) first, then DOWN (MIN) */
        .home_speed_mm_s   = 12.0f,    /* 1.5 rev/s - SG4 needs back-EMF (tune) */
        .home_accel_mm_s2  = 100.0f,
        .home_backoff_mm   = 5.0f,
        .home_retract_mm   = 3.0f,
        .home_prep_ms      = 250U,
        .home_settle_ms    = 150U,
        .seek_timeout_ms   = 45000U,
        .sg_poll_ms        = 2U,
        .stall = {
            .blank_ms         = 100U,
            .baseline_samples = 16U,
            .min_baseline     = 40U,
            .no_baseline_ms   = 400U,
            .abs_threshold    = 20U,   /* = 2 x sg4_thrs_home */
            .drop_ratio       = 0.45f, /* stall below 45 % of free-run SG4 (tune) */
            .confirm          = 4U,
            .ema_shift        = 5U,
        },
        .cycle_speed_mm_s  = 20.0f,
        .cycle_accel_mm_s2 = 200.0f,
        .cycle_margin_mm   = 5.0f,
        .cycle_dwell_ms    = 250U,
        .cycle_sg_guard    = false,
        .enc_enable           = true,
        .enc_counts_per_step  = ENC_PER_USTEP,
        .enc_follow_err_steps = 2.0f * (float)USTEPS, /* 2 fullsteps (a lost step is 4) */
        .enc_stall_assist     = true,
        .tmc = {
            .rref_ohm          = 12000U,
            .run_current_ma    = 1000U,  /* 73 N.cm NEMA17, lifting (tune to motor) */
            .home_current_ma   = 600U,
            .hold_current_pct  = 50U,
            .hold_delay        = 6U,
            .power_down_delay  = 10U,
            .microsteps        = USTEPS,
            .interpolate       = true,
            .invert_dir        = false,  /* + = DIR high = up; flip if reversed */
            .slope_control     = 2U,
            .stealth_max_mm_s  = 0.0f,   /* StealthChop2 at all speeds */
            .coolstep_enable   = false,
            .coolstep_min_mm_s = 5.0f,
            .semin = 4U, .semax = 2U, .seup = 1U, .sedn = 0U, .seimin = false,
            .sg4_filter        = true,
            .sg4_thrs_home     = 10U,
        },
    },
    [APP_AXIS_X] = {
        .name              = 'X',
        .steps_per_mm      = USTEPS_PER_REV / 40.0f,
        .vmin_steps_s      = 160.0f,
        .max_travel_mm     = 400.0f,   /* seek safety limit, > 300 mm X span */
        .min_travel_mm     = 10.0f,
        .home_min_first    = true,
        .home_speed_mm_s   = 60.0f,    /* 1.5 rev/s (tune) */
        .home_accel_mm_s2  = 500.0f,
        .home_backoff_mm   = 10.0f,
        .home_retract_mm   = 5.0f,
        .home_prep_ms      = 250U,
        .home_settle_ms    = 150U,
        .seek_timeout_ms   = 12000U,
        .sg_poll_ms        = 2U,
        .stall = {
            .blank_ms         = 100U,
            .baseline_samples = 16U,
            .min_baseline     = 40U,
            .no_baseline_ms   = 400U,
            .abs_threshold    = 20U,
            .drop_ratio       = 0.45f,
            .confirm          = 4U,
            .ema_shift        = 5U,
        },
        .cycle_speed_mm_s  = 150.0f,
        .cycle_accel_mm_s2 = 1500.0f,
        .cycle_margin_mm   = 10.0f,
        .cycle_dwell_ms    = 250U,
        .cycle_sg_guard    = false,
        .enc_enable           = true,
        .enc_counts_per_step  = ENC_PER_USTEP,
        .enc_follow_err_steps = 2.0f * (float)USTEPS,
        .enc_stall_assist     = true,
        .tmc = {
            .rref_ohm          = 12000U,
            .run_current_ma    = 800U,   /* 52 N.cm NEMA17 (tune to motor) */
            .home_current_ma   = 500U,
            .hold_current_pct  = 50U,
            .hold_delay        = 6U,
            .power_down_delay  = 10U,
            .microsteps        = USTEPS,
            .interpolate       = true,
            .invert_dir        = false,
            .slope_control     = 2U,
            .stealth_max_mm_s  = 0.0f,
            .coolstep_enable   = false,
            .coolstep_min_mm_s = 30.0f,
            .semin = 4U, .semax = 2U, .seup = 1U, .sedn = 0U, .seimin = false,
            .sg4_filter        = true,
            .sg4_thrs_home     = 10U,
        },
    },
};
