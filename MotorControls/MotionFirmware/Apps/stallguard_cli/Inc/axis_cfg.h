/**
 * @file    axis_cfg.h
 * @brief   Per-axis configuration types (mechanics, homing, cycle, encoder,
 *          TMC2240 driver). The default values live in app_config.c.
 */
#ifndef AXIS_CFG_H
#define AXIS_CFG_H

#include <stdbool.h>
#include <stdint.h>

#include "stall_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

/** TMC2240 driver settings (see tmc_axis.c for how they map to registers). */
typedef struct {
    uint16_t rref_ohm;          /* reference resistor on the module (MKS: 12k) */
    uint16_t run_current_ma;    /* RMS run current (IRUN=31 via GLOBAL_SCALER) */
    uint16_t home_current_ma;   /* RMS current while homing (gentler stall) */
    uint8_t  hold_current_pct;  /* standstill current in % of run current */
    uint8_t  hold_delay;        /* IHOLDDELAY 0..15 (x 2^18 clocks per step) */
    uint8_t  power_down_delay;  /* TPOWERDOWN 0..255 (x 2^18 clocks, >= 2) */
    uint16_t microsteps;        /* MRES: 1..256, power of two */
    bool     interpolate;       /* CHOPCONF.intpol (MicroPlyer to 1/256) */
    bool     invert_dir;        /* GCONF.shaft: flip positive direction */
    uint8_t  slope_control;     /* DRV_CONF.SLOPE_CONTROL 0..3 */
    float    stealth_max_mm_s;  /* SpreadCycle above this speed; 0 = StealthChop2 always */
    bool     coolstep_enable;   /* CoolStep load-adaptive current in RUN profile */
    float    coolstep_min_mm_s; /* TCOOLTHRS in RUN profile (also SG4 guard) */
    uint8_t  semin, semax, seup, sedn;
    bool     seimin;
    bool     sg4_filter;        /* SG4_THRS.sg4_filt_en: average over 4 fullsteps */
    uint8_t  sg4_thrs_home;     /* static SG4_THRS at homing start */
} tmc_axis_cfg_t;

typedef struct {
    char        name;                 /* 'X', 'Z', ... */
    float       steps_per_mm;         /* microsteps per mm of carriage travel */
    float       vmin_steps_s;         /* start/stop speed of the ramp */

    /* Travel / homing ------------------------------------------------- */
    float       max_travel_mm;        /* seek safety limit (> real axis length) */
    float       min_travel_mm;        /* measured travel must exceed this */
    bool        home_min_first;       /* true: MIN end first, then MAX */
    float       home_speed_mm_s;      /* constant StallGuard4 seek speed */
    float       home_accel_mm_s2;
    float       home_backoff_mm;      /* initial move away from the first end */
    float       home_retract_mm;      /* back-off after each end-stop stall */
    uint16_t    home_prep_ms;         /* standstill after enable (StealthChop AT#1) */
    uint16_t    home_settle_ms;       /* pause after a stall before measuring */
    uint32_t    seek_timeout_ms;      /* per seek */
    uint16_t    sg_poll_ms;           /* SG4 polling period while seeking */
    stall_cfg_t stall;

    /* Continuous cycle ------------------------------------------------ */
    float       cycle_speed_mm_s;
    float       cycle_accel_mm_s2;
    float       cycle_margin_mm;      /* soft limits = [margin, travel - margin] */
    uint16_t    cycle_dwell_ms;       /* pause at each end */
    bool        cycle_sg_guard;       /* StallGuard4 crash detection while cycling */

    /* AS5600 encoder -------------------------------------------------- */
    bool        enc_enable;
    float       enc_counts_per_step;  /* nominal |counts per microstep| */
    float       enc_follow_err_steps; /* step-loss (following error) limit */
    bool        enc_stall_assist;     /* encoder "no motion" as backup stall detector */

    tmc_axis_cfg_t tmc;
} axis_cfg_t;

#ifdef __cplusplus
}
#endif

#endif /* AXIS_CFG_H */
