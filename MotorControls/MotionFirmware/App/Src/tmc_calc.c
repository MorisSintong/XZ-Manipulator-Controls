/**
 * @file    tmc_calc.c
 * @brief   Pure TMC2240 register math.
 */
#include "tmc_calc.h"

#include <stddef.h>

/* KIFS per CURRENT_RANGE in mA*kOhm (datasheet: 11.75 / 24 / 36 / 36 A*kOhm). */
static const float kifs_ma_kohm[4] = { 11750.0f, 24000.0f, 36000.0f, 36000.0f };

static long round_to_long(float x)
{
    return (x >= 0.0f) ? (long)(x + 0.5f) : (long)(x - 0.5f);
}

float tmc_calc_ifs_rms_ma(uint8_t range, uint16_t rref_ohm)
{
    if ((range > 3U) || (rref_ohm == 0U)) {
        return 0.0f;
    }
    return kifs_ma_kohm[range] * 1000.0f / (float)rref_ohm / 1.41421356f;
}

static uint8_t cs_for(float ma, uint16_t gs, float ifs_ma)
{
    /* I = GS/256 * (CS+1)/32 * IFS  ->  CS = I*8192/(GS*IFS) - 1 */
    const long cs = round_to_long(ma * 8192.0f / ((float)gs * ifs_ma) - 1.0f);

    if (cs < 0) {
        return 0U;
    }
    if (cs > 31) {
        return 31U;
    }
    return (uint8_t)cs;
}

uint16_t tmc_calc_current_ma(uint16_t rref_ohm, uint8_t range, uint16_t gs, uint8_t cs)
{
    const float ifs = tmc_calc_ifs_rms_ma(range, rref_ohm);
    const float i = (float)gs / 256.0f * (float)(cs + 1U) / 32.0f * ifs;

    return (uint16_t)(i + 0.5f);
}

bool tmc_calc_currents(uint16_t rref_ohm, uint16_t run_ma, uint16_t home_ma,
                       uint8_t hold_pct, tmc_current_t *out)
{
    uint8_t range = 0U;
    float ifs;
    long gs;
    uint32_t hold_ma;

    if ((out == NULL) || (rref_ohm < TMC_RREF_MIN_OHM) || (rref_ohm > TMC_RREF_MAX_OHM) ||
        (run_ma == 0U) || (hold_pct > 100U)) {
        return false;
    }
    while ((range < 2U) && ((float)run_ma > tmc_calc_ifs_rms_ma(range, rref_ohm))) {
        range++;
    }
    ifs = tmc_calc_ifs_rms_ma(range, rref_ohm);
    if ((float)run_ma > ifs) {
        return false; /* above the 3 A peak full scale of CURRENT_RANGE 2 */
    }
    gs = round_to_long((float)run_ma * 256.0f / ifs);
    if (gs < 32) {
        gs = 32; /* GLOBAL_SCALER 1..31 is forbidden; CS then does the rest */
    }
    if (gs > 256) {
        gs = 256;
    }
    out->range = range;
    out->gs = (uint16_t)gs;
    out->irun = cs_for((float)run_ma, out->gs, ifs);

    hold_ma = ((uint32_t)run_ma * hold_pct) / 100U;
    out->ihold = cs_for((float)hold_ma, out->gs, ifs);
    if (out->ihold > out->irun) {
        out->ihold = out->irun;
    }
    if ((home_ma == 0U) || (home_ma >= run_ma)) {
        out->ihome = out->irun;
    } else {
        out->ihome = cs_for((float)home_ma, out->gs, ifs);
    }
    out->irun_ma = tmc_calc_current_ma(rref_ohm, range, out->gs, out->irun);
    out->ihold_ma = tmc_calc_current_ma(rref_ohm, range, out->gs, out->ihold);
    out->ihome_ma = tmc_calc_current_ma(rref_ohm, range, out->gs, out->ihome);
    return true;
}

uint32_t tmc_calc_tstep(float usteps_per_s, uint16_t microsteps)
{
    float t;

    if (!(usteps_per_s > 0.0f) || (microsteps == 0U)) {
        return TMC_TSTEP_MAX;
    }
    /* TSTEP counts fCLK periods per 1/256 microstep, independent of MRES. */
    t = TMC_FCLK_HZ * (float)microsteps / (256.0f * usteps_per_s);
    if (t >= (float)TMC_TSTEP_MAX) {
        return TMC_TSTEP_MAX;
    }
    return (uint32_t)(t + 0.5f);
}

uint8_t tmc_calc_mres(uint16_t microsteps)
{
    uint8_t mres = 0U;
    uint16_t m = 256U;

    if ((microsteps == 0U) || (microsteps > 256U) ||
        ((microsteps & (uint16_t)(microsteps - 1U)) != 0U)) {
        return 0xFFU;
    }
    while (m > microsteps) {
        m >>= 1U;
        mres++;
    }
    return mres;
}

uint32_t tmc_calc_gconf(bool shaft, bool stealthchop, bool diag0_stall)
{
    uint32_t v = 1UL << 3; /* multistep_filt (reset default) */

    if (stealthchop) {
        v |= 1UL << 2; /* en_pwm_mode: StealthChop2 below TPWMTHRS */
    }
    if (shaft) {
        v |= 1UL << 4;
    }
    if (diag0_stall) {
        v |= 1UL << 7;
    }
    return v;
}

uint32_t tmc_calc_drv_conf(uint8_t range, uint8_t slope_control)
{
    return ((uint32_t)range & 3UL) | (((uint32_t)slope_control & 3UL) << 4);
}

uint32_t tmc_calc_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t iholddelay,
                             uint8_t irundelay)
{
    return ((uint32_t)ihold & 31UL) | (((uint32_t)irun & 31UL) << 8) |
           (((uint32_t)iholddelay & 15UL) << 16) | (((uint32_t)irundelay & 15UL) << 24);
}

uint32_t tmc_calc_chopconf(uint8_t mres, bool intpol, bool dedge)
{
    uint32_t v = (5UL << 4)    /* HSTRT (encoded) */
               | (2UL << 7)    /* HEND (encoded)  */
               | (2UL << 15)   /* TBL = 36 clocks */
               | (4UL << 20)   /* TPFD passive fast decay */
               | (((uint32_t)mres & 15UL) << 24);

    if (intpol) {
        v |= 1UL << 28; /* MicroPlyer: interpolate to 256 microsteps */
    }
    if (dedge) {
        v |= 1UL << 29; /* step on both STEP edges */
    }
    return v;
}

uint32_t tmc_calc_coolconf(uint8_t semin, uint8_t seup, uint8_t semax,
                           uint8_t sedn, bool seimin)
{
    uint32_t v = ((uint32_t)semin & 15UL) | (((uint32_t)seup & 3UL) << 5) |
                 (((uint32_t)semax & 15UL) << 8) | (((uint32_t)sedn & 3UL) << 13);

    if (seimin) {
        v |= 1UL << 15;
    }
    return v;
}

uint32_t tmc_calc_sg4_thrs(uint8_t thrs, bool filt_en, bool angle_offset)
{
    uint32_t v = thrs;

    if (filt_en) {
        v |= 1UL << 8;
    }
    if (angle_offset) {
        v |= 1UL << 9;
    }
    return v;
}
