/**
 * @file    tmc_calc.h
 * @brief   Pure TMC2240 register math (host testable, no HAL access).
 *
 * References: Analog Devices TMC2240 datasheet Rev. 2 - current scaling
 * (RREF/KIFS/GLOBAL_SCALER/CS), TSTEP definition (time per 1/256 microstep in
 * units of the 12.5 MHz internal clock) and register bit layouts.
 */
#ifndef TMC_CALC_H
#define TMC_CALC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TMC_FCLK_HZ        12500000.0f /* internal clock, nominal */
#define TMC_TSTEP_MAX      0x000FFFFFUL
#define TMC_RREF_MIN_OHM   12000U
#define TMC_RREF_MAX_OHM   60000U

typedef struct {
    uint8_t  range;    /* DRV_CONF.CURRENT_RANGE 0..2 */
    uint16_t gs;       /* global scaler 32..256 (256 is written as 0) */
    uint8_t  irun;     /* CS for normal motion */
    uint8_t  ihold;    /* CS at standstill */
    uint8_t  ihome;    /* CS used as IRUN (and IHOLD) while homing */
    uint16_t irun_ma;  /* resulting RMS currents, for reporting */
    uint16_t ihold_ma;
    uint16_t ihome_ma;
} tmc_current_t;

/** Full scale RMS current in mA for a current range (0..3) and RREF. */
float    tmc_calc_ifs_rms_ma(uint8_t range, uint16_t rref_ohm);
/** Pick CURRENT_RANGE/GLOBAL_SCALER so that IRUN=31 equals run_ma (maximum
 *  microstep resolution), then derive hold and homing CS values. */
bool     tmc_calc_currents(uint16_t rref_ohm, uint16_t run_ma, uint16_t home_ma,
                           uint8_t hold_pct, tmc_current_t *out);
/** RMS current in mA for the given range, scaler and CS. */
uint16_t tmc_calc_current_ma(uint16_t rref_ohm, uint8_t range, uint16_t gs, uint8_t cs);
/** TSTEP value for a STEP rate in microsteps/s at the given MRES resolution.
 *  Returns TMC_TSTEP_MAX for rates <= 0. */
uint32_t tmc_calc_tstep(float usteps_per_s, uint16_t microsteps);
/** MRES field for 1..256 microsteps (powers of two), 0xFF if invalid. */
uint8_t  tmc_calc_mres(uint16_t microsteps);

uint32_t tmc_calc_gconf(bool shaft, bool stealthchop, bool diag0_stall);
uint32_t tmc_calc_drv_conf(uint8_t range, uint8_t slope_control);
uint32_t tmc_calc_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t iholddelay,
                             uint8_t irundelay);
/** CHOPCONF with TOFF=0 (the checked driver enables TOFF on activation),
 *  SpreadCycle HSTRT=5/HEND=2/TBL=2 (datasheet defaults), TPFD=4. */
uint32_t tmc_calc_chopconf(uint8_t mres, bool intpol, bool dedge);
uint32_t tmc_calc_coolconf(uint8_t semin, uint8_t seup, uint8_t semax,
                           uint8_t sedn, bool seimin);
uint32_t tmc_calc_sg4_thrs(uint8_t thrs, bool filt_en, bool angle_offset);

#ifdef __cplusplus
}
#endif

#endif /* TMC_CALC_H */
