/**
 * @file    tmc_axis.c
 * @brief   TMC2240 per-axis setup, profiles and supervision (see tmc_axis.h).
 *
 * All calls are made from the main loop only: the TMC2240 driver has no
 * internal locking and both chips share SPI2.
 */
#include "tmc_axis.h"

#include "tmc2240_core.h"

#include <string.h>

#define TMC_PWMCONF_DEFAULT 0xC40C001DUL /* reset value: PWM_OFS 29, autoscale+autograd, REG 4, LIM 12 */
#define TMC_TOFF_ACTIVE     3U
#define TMC_IRUNDELAY       4U
#define TMC_SPI_ERR_LIMIT   3U

#define SPI_ST_RESET        0x01U /* GSTAT.reset */
#define SPI_ST_DRV_ERR      0x02U /* GSTAT.drv_err */
#define SPI_ST_STALL        0x04U /* DRV_STATUS.stallGuard */

typedef struct {
    uint32_t ihold_irun;
    uint32_t tpwmthrs;
    uint32_t tcoolthrs;
    uint32_t thigh;
    uint32_t coolconf;
    uint32_t sg4_thrs;
} profile_regs_t;

static tmc_axis_info_t g_tmc[TMC_AXIS_MAX];
static uint8_t         g_count;

static bool valid(uint8_t ax)
{
    return ax < g_count;
}

bool tmc_axis_bus_init(const TMC2240_SPIConfig_t *configs, uint8_t count)
{
    if ((configs == NULL) || (count == 0U) || (count > TMC_AXIS_MAX)) {
        return false;
    }
    memset(g_tmc, 0, sizeof(g_tmc));
    if (tmc2240_hal_init(configs, count) != TMC2240_OK) {
        g_count = 0U;
        return false;
    }
    g_count = count;
    return true;
}

static void build_profile(const tmc_axis_info_t *t, axis_profile_t profile,
                          const axis_cfg_t *cfg, profile_regs_t *r)
{
    const tmc_axis_cfg_t *c = &cfg->tmc;
    const float spm = cfg->steps_per_mm;

    r->thigh = 0U; /* no upper velocity limit for StallGuard/CoolStep */
    r->sg4_thrs = tmc_calc_sg4_thrs(c->sg4_thrs_home, c->sg4_filter, true);
    if (profile == AXIS_PROFILE_HOMING) {
        /* StallGuard4 only works in StealthChop2 -> TPWMTHRS = 0 (StealthChop at
         * every speed). IHOLD = IRUN keeps the StealthChop tuning valid and the
         * stall force constant. SG4 is enabled above half the homing speed
         * (TCOOLTHRS >= TSTEP). CoolStep off for a predictable load signal. */
        r->ihold_irun = tmc_calc_ihold_irun(t->current.ihome, t->current.ihome,
                                            c->hold_delay, TMC_IRUNDELAY);
        r->tpwmthrs = 0U;
        r->tcoolthrs = tmc_calc_tstep(0.5f * cfg->home_speed_mm_s * spm, c->microsteps);
        r->coolconf = 0U;
    } else {
        r->ihold_irun = tmc_calc_ihold_irun(t->current.ihold, t->current.irun,
                                            c->hold_delay, TMC_IRUNDELAY);
        r->tpwmthrs = (c->stealth_max_mm_s > 0.0f)
                    ? tmc_calc_tstep(c->stealth_max_mm_s * spm, c->microsteps) : 0U;
        r->tcoolthrs = (c->coolstep_enable || cfg->cycle_sg_guard)
                     ? tmc_calc_tstep(c->coolstep_min_mm_s * spm, c->microsteps) : 0U;
        r->coolconf = c->coolstep_enable
                    ? tmc_calc_coolconf(c->semin, c->seup, c->semax, c->sedn, c->seimin) : 0U;
    }
}

static axis_hw_status_t classify(uint8_t ax, TMC2240Status st)
{
    tmc_axis_info_t *t = &g_tmc[ax];
    uint8_t spi = 0U;

    if (st != TMC2240_OK) {
        t->spi_errors++;
        t->last_error = st;
        return AXIS_HW_ERR_SPI;
    }
    if (tmc2240_hal_getSPIStatus(ax, &spi) == TMC2240_OK) {
        t->spi_status = spi;
        if ((spi & (SPI_ST_RESET | SPI_ST_DRV_ERR)) != 0U) {
            t->configured = false;
            return AXIS_HW_ERR_DRIVER;
        }
    }
    return AXIS_HW_OK;
}

static TMC2240Status write_profile(uint8_t ax, const profile_regs_t *r, bool with_current)
{
    TMC2240Status st = TMC2240_OK;

    if (with_current) {
        st = tmc2240_writeRegisterVerified(ax, TMC2240_IHOLD_IRUN, r->ihold_irun);
    }
    if (st == TMC2240_OK) {
        st = tmc2240_writeRegisterVerified(ax, TMC2240_TPWMTHRS, r->tpwmthrs);
    }
    if (st == TMC2240_OK) {
        st = tmc2240_writeRegisterVerified(ax, TMC2240_TCOOLTHRS, r->tcoolthrs);
    }
    if (st == TMC2240_OK) {
        st = tmc2240_writeRegisterVerified(ax, TMC2240_THIGH, r->thigh);
    }
    if (st == TMC2240_OK) {
        st = tmc2240_writeRegisterVerified(ax, TMC2240_COOLCONF, r->coolconf);
    }
    if (st == TMC2240_OK) {
        st = tmc2240_writeRegisterVerified(ax, TMC2240_SG4_THRS, r->sg4_thrs);
    }
    return st;
}

TMC2240Status tmc_axis_configure(uint8_t ax, const axis_cfg_t *cfg)
{
    tmc_axis_info_t *t;
    const tmc_axis_cfg_t *c;
    TMC2240_MotorConfig_t mc;
    profile_regs_t r;
    uint8_t mres;
    uint8_t gstat = 0U;
    TMC2240Status st;

    if (!valid(ax) || (cfg == NULL)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    t = &g_tmc[ax];
    c = &cfg->tmc;
    t->configured = false;
    mres = tmc_calc_mres(c->microsteps);
    if ((mres == 0xFFU) ||
        !tmc_calc_currents(c->rref_ohm, c->run_current_ma, c->home_current_ma,
                           c->hold_current_pct, &t->current)) {
        t->last_error = TMC2240_ERROR_RANGE;
        return TMC2240_ERROR_RANGE;
    }

    /* 1. The chip must answer with IOIN.VERSION = 0x40 (needs VM powered). */
    st = tmc2240_hal_testConnection(ax);
    /* 2. Acknowledge the power-up flags (reset, register reset, UV). The
     *    helper verifies they cleared: a persisting UV means VM is too low. */
    if (st == TMC2240_OK) {
        st = tmc2240_check_faults(ax, &gstat);
    }
    if (st == TMC2240_OK) {
        t->gstat_at_init = gstat;
        if (gstat != 0U) {
            st = tmc2240_clear_faults(ax, gstat);
        }
    }
    /* 3. Base configuration, written and read back by the checked driver. */
    if (st == TMC2240_OK) {
        mc.gconf = tmc_calc_gconf(c->invert_dir, true, true);
        mc.drv_conf = tmc_calc_drv_conf(t->current.range, c->slope_control);
        mc.global_scaler = (t->current.gs >= 256U) ? 0U : t->current.gs;
        mc.ihold_irun = tmc_calc_ihold_irun(t->current.ihold, t->current.irun,
                                            c->hold_delay, TMC_IRUNDELAY);
        mc.chopconf = tmc_calc_chopconf(mres, c->interpolate, true);
        mc.pwmconf = TMC_PWMCONF_DEFAULT;
        st = tmc2240_hal_configureMotor(ax, &mc);
    }
    if (st == TMC2240_OK) {
        st = tmc2240_writeRegisterVerified(ax, TMC2240_TPOWERDOWN,
                                           (c->power_down_delay < 2U) ? 2U : c->power_down_delay);
    }
    if (st == TMC2240_OK) {
        build_profile(t, AXIS_PROFILE_RUN, cfg, &r);
        st = write_profile(ax, &r, false); /* IHOLD_IRUN already part of mc */
    }
    /* 4. Enable the chopper (TOFF) after a fresh read-back of everything. */
    if (st == TMC2240_OK) {
        st = tmc2240_hal_activateMotor(ax, TMC_TOFF_ACTIVE);
    }
    t->last_error = st;
    if (st != TMC2240_OK) {
        return st;
    }
    t->profile = AXIS_PROFILE_RUN;
    t->spi_error_run = 0U;
    t->configured = true;
    return TMC2240_OK;
}

void tmc_axis_mark_unconfigured(uint8_t ax)
{
    if (valid(ax)) {
        g_tmc[ax].configured = false;
        (void)tmc2240_hal_disableMotor(ax); /* best effort TOFF = 0 */
    }
}

axis_hw_status_t tmc_axis_apply_profile(uint8_t ax, axis_profile_t profile, const axis_cfg_t *cfg)
{
    tmc_axis_info_t *t;
    profile_regs_t r;
    axis_hw_status_t hs;

    if (!valid(ax) || (cfg == NULL) || !g_tmc[ax].configured) {
        return AXIS_HW_ERR_DRIVER;
    }
    t = &g_tmc[ax];
    build_profile(t, profile, cfg, &r);
    hs = classify(ax, write_profile(ax, &r, true));
    if (hs == AXIS_HW_OK) {
        t->profile = profile;
    }
    return hs;
}

axis_hw_status_t tmc_axis_set_sg_threshold(uint8_t ax, uint8_t thrs)
{
    if (!valid(ax) || !g_tmc[ax].configured) {
        return AXIS_HW_ERR_DRIVER;
    }
    return classify(ax, tmc2240_stallguard_set_threshold(ax, thrs));
}

axis_hw_status_t tmc_axis_read_sg(uint8_t ax, uint16_t *sg4, bool *stall_flag)
{
    tmc_axis_info_t *t;
    uint32_t value = 0U;
    axis_hw_status_t hs;

    if (!valid(ax) || !g_tmc[ax].configured) {
        return AXIS_HW_ERR_DRIVER;
    }
    t = &g_tmc[ax];
    hs = classify(ax, tmc2240_readRegister(ax, TMC2240_SG4_RESULT, &value));
    if (hs != AXIS_HW_OK) {
        return hs;
    }
    t->sg4 = (uint16_t)(value & 0x3FFU);
    *sg4 = t->sg4;
    *stall_flag = (t->spi_status & SPI_ST_STALL) != 0U;
    return AXIS_HW_OK;
}

tmc_health_t tmc_axis_poll_health(uint8_t ax)
{
    tmc_axis_info_t *t;
    uint32_t drv = 0U;
    uint8_t spi = 0U;
    TMC2240Status st;

    if (!valid(ax) || !g_tmc[ax].configured) {
        return TMC_HEALTH_SPI;
    }
    t = &g_tmc[ax];
    st = tmc2240_driver_status(ax, &drv);
    if (st != TMC2240_OK) {
        t->spi_errors++;
        t->last_error = st;
        t->spi_error_run++;
        return (t->spi_error_run >= TMC_SPI_ERR_LIMIT) ? TMC_HEALTH_SPI : TMC_HEALTH_OK;
    }
    t->spi_error_run = 0U;
    (void)tmc2240_hal_getSPIStatus(ax, &spi);
    t->spi_status = spi;
    t->drv_status = drv;
    t->sg_result = (uint16_t)(drv & TMC2240_SG_RESULT_MASK);
    t->cs_actual = (uint8_t)((drv & TMC2240_CS_ACTUAL_MASK) >> TMC2240_CS_ACTUAL_SHIFT);
    t->stealth = (drv & TMC2240_STEALTH_MASK) != 0U;
    t->standstill = (drv & TMC2240_STST_MASK) != 0U;
    t->otpw = (drv & TMC2240_OTPW_MASK) != 0U;
    /* Open-load detection is only meaningful in SpreadCycle while moving. */
    t->open_load = !t->stealth && !t->standstill &&
                   ((drv & (TMC2240_OLA_MASK | TMC2240_OLB_MASK)) != 0U);

    if ((spi & SPI_ST_RESET) != 0U) {
        t->configured = false;
        return TMC_HEALTH_RESET;
    }
    if (((spi & SPI_ST_DRV_ERR) != 0U) ||
        ((drv & (TMC2240_OT_MASK | TMC2240_S2GA_MASK | TMC2240_S2GB_MASK |
                 TMC2240_S2VSA_MASK | TMC2240_S2VSB_MASK)) != 0U)) {
        return TMC_HEALTH_FAULT;
    }
    return (t->otpw || t->open_load) ? TMC_HEALTH_WARN : TMC_HEALTH_OK;
}

void tmc_axis_poll_adc(uint8_t ax)
{
    tmc_axis_info_t *t;
    int16_t temp = 0;
    uint32_t mv = 0U;

    if (!valid(ax)) {
        return;
    }
    t = &g_tmc[ax];
    if (t->configured &&
        (tmc2240_get_temperature_c10(ax, &temp) == TMC2240_OK) &&
        (tmc2240_get_vsupply_mV(ax, &mv) == TMC2240_OK)) {
        t->temp_c10 = temp;
        t->vm_mv = mv;
        t->adc_valid = true;
    } else {
        t->adc_valid = false;
    }
}

const tmc_axis_info_t *tmc_axis_info(uint8_t ax)
{
    return valid(ax) ? &g_tmc[ax] : NULL;
}

const char *tmc_status_name(TMC2240Status st)
{
    switch (st) {
    case TMC2240_OK:                    return "ok";
    case TMC2240_ERROR_ARGUMENT:        return "argument";
    case TMC2240_ERROR_ID:              return "id";
    case TMC2240_ERROR_ADDRESS:         return "address";
    case TMC2240_ERROR_ACCESS:          return "access";
    case TMC2240_ERROR_RANGE:           return "range";
    case TMC2240_ERROR_NOT_INITIALIZED: return "not initialised";
    case TMC2240_ERROR_UNSUPPORTED:     return "unsupported";
    case TMC2240_ERROR_IO:              return "SPI I/O";
    case TMC2240_ERROR_TIMEOUT:         return "SPI timeout";
    case TMC2240_ERROR_BUSY:            return "SPI busy";
    case TMC2240_ERROR_PROTOCOL:        return "protocol";
    case TMC2240_ERROR_CRC:             return "CRC";
    case TMC2240_ERROR_VERIFY:          return "no answer / read-back mismatch";
    case TMC2240_ERROR_STATE:           return "state";
    case TMC2240_ERROR_FAULT:           return "GSTAT flag persists";
    default:                            return "?";
    }
}

void tmc_axis_dump(uint8_t ax, void (*print)(const char *fmt, ...))
{
    static const struct {
        uint8_t     reg;
        const char *name;
    } regs[] = {
        { TMC2240_GCONF, "GCONF" },          { TMC2240_GSTAT, "GSTAT" },
        { TMC2240_IOIN, "IOIN" },            { TMC2240_DRV_CONF, "DRV_CONF" },
        { TMC2240_GLOBAL_SCALER, "GLOBALSCALER" }, { TMC2240_IHOLD_IRUN, "IHOLD_IRUN" },
        { TMC2240_TPOWERDOWN, "TPOWERDOWN" }, { TMC2240_TSTEP, "TSTEP" },
        { TMC2240_TPWMTHRS, "TPWMTHRS" },    { TMC2240_TCOOLTHRS, "TCOOLTHRS" },
        { TMC2240_THIGH, "THIGH" },          { TMC2240_MSCNT, "MSCNT" },
        { TMC2240_CHOPCONF, "CHOPCONF" },    { TMC2240_COOLCONF, "COOLCONF" },
        { TMC2240_DRVSTATUS, "DRV_STATUS" }, { TMC2240_PWMCONF, "PWMCONF" },
        { TMC2240_PWM_SCALE, "PWM_SCALE" },  { TMC2240_PWM_AUTO, "PWM_AUTO" },
        { TMC2240_SG4_THRS, "SG4_THRS" },    { TMC2240_SG4_RESULT, "SG4_RESULT" },
        { TMC2240_SG4_IND, "SG4_IND" },
    };
    const tmc_axis_info_t *t;
    size_t i;
    uint32_t drv = 0U;

    if (!valid(ax) || (print == NULL)) {
        return;
    }
    t = &g_tmc[ax];
    print("  TMC2240 #%u: %s, profile %s, last error: %s\r\n", (unsigned)ax,
          t->configured ? "configured" : "NOT configured",
          (t->profile == AXIS_PROFILE_HOMING) ? "HOMING" : "RUN", tmc_status_name(t->last_error));
    for (i = 0U; i < sizeof(regs) / sizeof(regs[0]); i++) {
        uint32_t v = 0U;
        const TMC2240Status st = tmc2240_readRegister(ax, regs[i].reg, &v);
        if (st == TMC2240_OK) {
            print("    %-12s (0x%02X) = 0x%08lX\r\n", regs[i].name, (unsigned)regs[i].reg,
                  (unsigned long)v);
            if (regs[i].reg == TMC2240_DRVSTATUS) {
                drv = v;
            }
        } else {
            print("    %-12s (0x%02X) : %s\r\n", regs[i].name, (unsigned)regs[i].reg,
                  tmc_status_name(st));
        }
    }
    print("    current: range %u, GS %u, IRUN %u (%u mA), IHOLD %u (%u mA), home CS %u (%u mA)\r\n",
          (unsigned)t->current.range, (unsigned)t->current.gs, (unsigned)t->current.irun,
          (unsigned)t->current.irun_ma, (unsigned)t->current.ihold, (unsigned)t->current.ihold_ma,
          (unsigned)t->current.ihome, (unsigned)t->current.ihome_ma);
    print("    DRV_STATUS: SG %u, CS_ACTUAL %lu, %s%s%s%s%s%s%s\r\n",
          (unsigned)(drv & TMC2240_SG_RESULT_MASK),
          (unsigned long)((drv & TMC2240_CS_ACTUAL_MASK) >> TMC2240_CS_ACTUAL_SHIFT),
          (drv & TMC2240_STEALTH_MASK) ? "StealthChop " : "SpreadCycle ",
          (drv & TMC2240_STST_MASK) ? "standstill " : "",
          (drv & TMC2240_STALLGUARD_MASK) ? "STALL " : "",
          (drv & TMC2240_OTPW_MASK) ? "OT-prewarn " : "",
          (drv & TMC2240_OT_MASK) ? "OVERTEMP " : "",
          (drv & (TMC2240_S2GA_MASK | TMC2240_S2GB_MASK | TMC2240_S2VSA_MASK | TMC2240_S2VSB_MASK))
              ? "SHORT " : "",
          (drv & (TMC2240_OLA_MASK | TMC2240_OLB_MASK)) ? "open-load" : "");
    if (t->adc_valid) {
        print("    chip %d.%d C, VM %lu mV, GSTAT at init 0x%02X\r\n", t->temp_c10 / 10,
              (t->temp_c10 < 0 ? -t->temp_c10 : t->temp_c10) % 10, (unsigned long)t->vm_mv,
              (unsigned)t->gstat_at_init);
    }
}
