/**
 * @file    tmc_axis.h
 * @brief   TMC2240 per-axis setup, motion profiles and supervision, built on
 *          the checked driver in MotorControls/Drivers/TMC2240-Driver.
 *
 * TMC2240 features used:
 *  - SPI configuration with read-back verification of every register
 *  - current: RREF/KIFS current range + GLOBAL_SCALER so IRUN = 31 (finest
 *    microstep current resolution), standstill reduction IHOLD/IHOLDDELAY/
 *    TPOWERDOWN, separate (lower) homing current
 *  - StealthChop2 with automatic tuning (PWM_AUTOSCALE/AUTOGRAD), optional
 *    SpreadCycle above TPWMTHRS (hybrid), MicroPlyer 1/256 interpolation,
 *    double-edge STEP (DEDGE)
 *  - StallGuard4: SG4_THRS comparator (+ SG4 filter, angle offset
 *    compensation), SG4_RESULT load value, velocity gate via TCOOLTHRS,
 *    stallGuard flag in every SPI status byte, routed to DIAG0 as well
 *  - optional CoolStep (SEMIN/SEMAX/SEUP/SEDN/SEIMIN) in the RUN profile
 *  - diagnostics: GSTAT reset/driver error/UV, DRV_STATUS over-temperature,
 *    pre-warning, short to GND/supply, open load, CS_ACTUAL, and the ADC
 *    readings of chip temperature and motor supply voltage.
 */
#ifndef TMC_AXIS_H
#define TMC_AXIS_H

#include <stdbool.h>
#include <stdint.h>

#include "axis_cfg.h"
#include "axis_hw.h"
#include "tmc2240_hal.h"
#include "tmc_calc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TMC_AXIS_MAX 2U

typedef enum {
    TMC_HEALTH_OK = 0,
    TMC_HEALTH_WARN,  /* over-temperature pre-warning / open load */
    TMC_HEALTH_SPI,   /* repeated SPI failures */
    TMC_HEALTH_RESET, /* chip reset (e.g. VM brown-out): configuration lost */
    TMC_HEALTH_FAULT  /* over-temperature / short circuit / driver error */
} tmc_health_t;

typedef struct {
    bool           configured;
    axis_profile_t profile;
    tmc_current_t  current;
    uint8_t        gstat_at_init;
    uint8_t        spi_status;   /* last SPI status byte */
    uint32_t       drv_status;
    uint16_t       sg_result;    /* DRV_STATUS.SG_RESULT */
    uint16_t       sg4;          /* last SG4_RESULT */
    uint8_t        cs_actual;
    bool           stealth;
    bool           standstill;
    bool           otpw;
    bool           open_load;
    bool           adc_valid;
    int16_t        temp_c10;
    uint32_t       vm_mv;
    uint32_t       spi_errors;
    uint8_t        spi_error_run;
    TMC2240Status  last_error;
} tmc_axis_info_t;

/** Initialise the shared SPI transport (CS lines inactive, no IC writes). */
bool             tmc_axis_bus_init(const TMC2240_SPIConfig_t *configs, uint8_t count);
/** Detect the chip, acknowledge power-up flags, write and verify the complete
 *  configuration (RUN profile) and activate the chopper. ENN stays untouched. */
TMC2240Status    tmc_axis_configure(uint8_t ax, const axis_cfg_t *cfg);
void             tmc_axis_mark_unconfigured(uint8_t ax);
axis_hw_status_t tmc_axis_apply_profile(uint8_t ax, axis_profile_t profile, const axis_cfg_t *cfg);
axis_hw_status_t tmc_axis_set_sg_threshold(uint8_t ax, uint8_t thrs);
axis_hw_status_t tmc_axis_read_sg(uint8_t ax, uint16_t *sg4, bool *stall_flag);
tmc_health_t     tmc_axis_poll_health(uint8_t ax);
void             tmc_axis_poll_adc(uint8_t ax);
const tmc_axis_info_t *tmc_axis_info(uint8_t ax);
const char      *tmc_status_name(TMC2240Status st);
/** Print a decoded register dump through 'print'. */
void             tmc_axis_dump(uint8_t ax, void (*print)(const char *fmt, ...));

#ifdef __cplusplus
}
#endif

#endif /* TMC_AXIS_H */
