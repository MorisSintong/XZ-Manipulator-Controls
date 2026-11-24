/**
 * @file    mc_bench.c
 * @brief   STM32H723 hardware binding for MotionFirmware_Characterization.
 *
 * Implements the five fw_app.h hooks and binds mc_core_t to:
 *  - Native USB-C CDC (usb_cdc_port) exclusively (USART2 untouched)
 *  - TIM2 free-running 1 MHz timestamp + 1 ms CC1 interlock IRQ
 *  - TIM5 stepgen slot 1 dynamically rebound to the single active axis
 *  - AS5600 HAL ports on I2C4 (bus 0 / Z) and I2C1 (bus 1 / X or theta_fixture)
 *  - TMC2240 SPI1 with explicit PLL1Q kernel clock derivation and prescaler sweep
 */
#include "mc_core.h"

#include "app_config.h"
#include "as5600.h"
#include "as5600_stm32_hal.h"
#include "fw_app.h"
#include "main.h"
#include "stepgen.h"
#include "tmc2240_core.h"
#include "tmc2240_hal.h"
#include "tmc_axis.h"
#include "tmc_calc.h"
#include "usb_cdc_port.h"

#include <string.h>

#define MC_STEPGEN_SLOT        1U   /* Reuse TIM5 slot for single active axis */
#define MC_I2C_RAW_TIMEOUT_MS  2U
#define MC_I2C_FULL_TIMEOUT_MS 10U  /* > 6 transfers for as5600_read_sample */
#define MC_I2C_FAST_TIMINGR    0x30A31A23UL /* Proposed 400 kHz bench-only candidate at 100 MHz */
#define MC_ROT_USTEPS_PER_DEG  (3200.0f / 360.0f)
#define MC_ROT_STEALTH_DEG_S   120.0f

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c4;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

static mc_core_t g_mc;
static as5600_t g_enc[2];
static as5600_stm32_hal_t g_enc_port[2];
static bool g_enc_bound[2];
static uint32_t g_saved_i2c_timingr[2];
static uint32_t g_saved_i2c_df[2];
static bool g_spi_ready;
static uint32_t g_spi_div_active = 32U;

static uint32_t hw_now_us_raw(void) { return TIM2->CNT; }
static uint32_t hw_now_us(void *ctx) { (void)ctx; return TIM2->CNT; }
static uint32_t hw_now_ms(void *ctx) { (void)ctx; return HAL_GetTick(); }

static bool hw_usb_mounted(void *ctx) {
    (void)ctx;
    return usb_cdc_port_is_mounted();
}

static uint32_t hw_usb_write(void *ctx, const uint8_t *data, uint32_t len) {
    (void)ctx;
    return usb_cdc_port_write(data, len);
}

static bool hw_b1_asserted(void *ctx) {
    (void)ctx;
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;
}

static void hw_disable_all_drivers(void *ctx) {
    (void)ctx;
    HAL_GPIO_WritePin(TMC_ENN0_GPIO_Port, TMC_ENN0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_ENN1_GPIO_Port, TMC_ENN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_ENN2_GPIO_Port, TMC_ENN2_Pin, GPIO_PIN_SET);
}

static void hw_set_driver_enable(void *ctx, uint8_t axis, bool enable) {
    (void)ctx;
    hw_disable_all_drivers(NULL);
    if (!enable) {
        return;
    }
    if (axis == MC_AXIS_Z) {
        HAL_GPIO_WritePin(TMC_ENN0_GPIO_Port, TMC_ENN0_Pin, GPIO_PIN_RESET);
    } else if (axis == MC_AXIS_X) {
        HAL_GPIO_WritePin(TMC_ENN1_GPIO_Port, TMC_ENN1_Pin, GPIO_PIN_RESET);
    } else if (axis == MC_AXIS_THETA) {
        HAL_GPIO_WritePin(TMC_ENN2_GPIO_Port, TMC_ENN2_Pin, GPIO_PIN_RESET);
    }
}

static uint32_t spi123_kernel_hz(void) {
    return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI1);
}

static uint32_t i2c_kernel_hz(uint8_t bus) {
    return (bus == 0U)
               ? HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C4)
               : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C1);
}

static I2C_HandleTypeDef *bus_to_hi2c(uint8_t bus) {
    return (bus == 0U) ? &hi2c4 : &hi2c1;
}

static uint32_t spi_div_to_hal(uint32_t div) {
    switch (div) {
        case 256U: return SPI_BAUDRATEPRESCALER_256;
        case 128U: return SPI_BAUDRATEPRESCALER_128;
        case 64U:  return SPI_BAUDRATEPRESCALER_64;
        case 32U:  return SPI_BAUDRATEPRESCALER_32;
        case 16U:  return SPI_BAUDRATEPRESCALER_16;
        default:   return 0U;
    }
}

static void spi_cs_all_high(void) {
    HAL_GPIO_WritePin(TMC_CS0_GPIO_Port, TMC_CS0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_CS1_GPIO_Port, TMC_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_CS2_GPIO_Port, TMC_CS2_Pin, GPIO_PIN_SET);
}

static bool spi_rebind_tmc_hal(void) {
    static const TMC2240_SPIConfig_t k_spi_cfgs[MC_MAX_AXES] = {
        { .hspi = &hspi1, .cs_port = TMC_CS0_GPIO_Port, .cs_pin = TMC_CS0_Pin, .spi_timeout_ms = 5U },
        { .hspi = &hspi1, .cs_port = TMC_CS1_GPIO_Port, .cs_pin = TMC_CS1_Pin, .spi_timeout_ms = 5U },
        { .hspi = &hspi1, .cs_port = TMC_CS2_GPIO_Port, .cs_pin = TMC_CS2_Pin, .spi_timeout_ms = 5U },
    };
    if (tmc2240_hal_isInitialized()) {
        (void)tmc2240_hal_deinit();
    }
    g_spi_ready = tmc_axis_bus_init(k_spi_cfgs, MC_MAX_AXES);
    return g_spi_ready;
}

static void hw_get_board_info(void *ctx, mc_board_info_t *out) {
    (void)ctx;
    memset(out, 0, sizeof(*out));
    out->sysclk_hz = HAL_RCC_GetSysClockFreq();
    out->tim2_hz = 1000000U;
    out->tim5_hz = stepgen_timer_hz(MC_STEPGEN_SLOT);
    out->spi_kernel_hz = spi123_kernel_hz();
    out->spi_prescaler = g_spi_div_active;
    out->i2c1_timingr = hi2c1.Instance->TIMINGR;
    out->i2c4_timingr = hi2c4.Instance->TIMINGR;

    if (g_spi_ready) {
        for (uint8_t i = 0U; i < MC_MAX_AXES; ++i) {
            uint32_t ioin = 0U;
            uint8_t gstat = 0U;
            if (tmc2240_readRegister(i, TMC2240_IOIN, &ioin) == TMC2240_OK) {
                out->drv_version[i] = (uint8_t)((ioin >> 24U) & 0xFFU);
            }
            if (tmc2240_check_faults(i, &gstat) == TMC2240_OK) {
                out->drv_gstat[i] = gstat;
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Stepgen binding (reuses TIM5 slot 1 for Z, X, or Theta)                   */
/* ------------------------------------------------------------------------- */

static bool hw_step_bind_axis(void *ctx, uint8_t axis) {
    (void)ctx;
    stepgen_hw_t hw;
    memset(&hw, 0, sizeof(hw));
    hw.tim = TIM5;
    hw.irq = TIM5_IRQn;
    if (axis == MC_AXIS_Z) {
        hw.step_port = TMC_STEP0_GPIO_Port;
        hw.step_pin  = TMC_STEP0_Pin;
        hw.dir_port  = TMC_DIR0_GPIO_Port;
        hw.dir_pin   = TMC_DIR0_Pin;
    } else if (axis == MC_AXIS_X) {
        hw.step_port = TMC_STEP1_GPIO_Port;
        hw.step_pin  = TMC_STEP1_Pin;
        hw.dir_port  = TMC_DIR1_GPIO_Port;
        hw.dir_pin   = TMC_DIR1_Pin;
    } else if (axis == MC_AXIS_THETA) {
        hw.step_port = TMC_STEP2_GPIO_Port;
        hw.step_pin  = TMC_STEP2_Pin;
        hw.dir_port  = TMC_DIR2_GPIO_Port;
        hw.dir_pin   = TMC_DIR2_Pin;
    } else {
        return false;
    }
    stepgen_init(MC_STEPGEN_SLOT, &hw);
    stepgen_char_set_inhibit(MC_STEPGEN_SLOT, true);
    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    return true;
}

static void hw_step_set_inhibit(void *ctx, bool inhibit) {
    (void)ctx;
    stepgen_char_set_inhibit(MC_STEPGEN_SLOT, inhibit);
}

static void hw_step_set_limits(void *ctx, int32_t lo, int32_t hi) {
    (void)ctx;
    stepgen_set_limits(MC_STEPGEN_SLOT, lo, hi);
}

static void hw_step_set_vmin(void *ctx, float vmin) {
    (void)ctx;
    stepgen_set_vmin(MC_STEPGEN_SLOT, vmin);
}

static bool hw_step_set_position(void *ctx, int32_t pos) {
    (void)ctx;
    return stepgen_set_position(MC_STEPGEN_SLOT, pos);
}

static int32_t hw_step_position(void *ctx) {
    (void)ctx;
    return stepgen_position(MC_STEPGEN_SLOT);
}

static bool hw_step_busy(void *ctx) {
    (void)ctx;
    return stepgen_busy(MC_STEPGEN_SLOT);
}

static bool hw_step_validate_rate(void *ctx, float vmax, float vmin) {
    (void)ctx;
    return stepgen_char_validate_rate(MC_STEPGEN_SLOT, vmax, vmin);
}

static void hw_step_reset_stats(void *ctx) {
    (void)ctx;
    stepgen_char_reset_stats(MC_STEPGEN_SLOT);
}

static void hw_step_get_stats(void *ctx, mc_step_stats_t *out) {
    (void)ctx;
    stepgen_char_stats_t s;
    memset(&s, 0, sizeof(s));
    stepgen_char_get_stats(MC_STEPGEN_SLOT, &s);
    out->edges_emitted     = s.edges_emitted;
    out->edges_inhibited   = s.edges_inhibited;
    out->first_edge_us     = s.first_edge_us;
    out->last_edge_us      = s.last_edge_us;
    out->min_dt_us         = s.min_dt_us;
    out->max_dt_us         = s.max_dt_us;
    out->max_lateness_us   = s.max_lateness_us;
    out->late_edges        = s.late_edges;
    out->ramp_up_end_us    = s.ramp_up_end_us;
    out->ramp_up_end_edges = s.ramp_up_end_edges;
    out->cruise_end_us     = s.cruise_end_us;
    out->cruise_end_edges  = s.cruise_end_edges;
    out->lateness_fault    = s.lateness_fault;
}

static void hw_step_move_to(void *ctx, int32_t target, float vmax, float accel) {
    (void)ctx;
    stepgen_move_to(MC_STEPGEN_SLOT, target, vmax, accel);
}

static void hw_step_halt(void *ctx) {
    (void)ctx;
    stepgen_halt(MC_STEPGEN_SLOT);
}

static bool hw_step_take_limit_hit(void *ctx) {
    (void)ctx;
    return stepgen_take_limit_hit(MC_STEPGEN_SLOT);
}

/* ------------------------------------------------------------------------- */
/* AS5600 Encoder HAL binding                                                */
/* ------------------------------------------------------------------------- */

static bool enc_ensure_bound(uint8_t bus) {
    if (bus > 1U) {
        return false;
    }
    if (!g_enc_bound[bus]) {
        memset(&g_enc[bus], 0, sizeof(g_enc[bus]));
        memset(&g_enc_port[bus], 0, sizeof(g_enc_port[bus]));
        if (as5600_stm32_hal_init(&g_enc[bus], &g_enc_port[bus],
                                  bus_to_hi2c(bus), NULL) != AS5600_OK) {
            return false;
        }
        g_enc_bound[bus] = true;
    }
    return true;
}

static bool hw_enc_configure(void *ctx, uint8_t bus, uint16_t *conf_rb,
                             uint8_t *status, uint8_t *agc, uint16_t *mag,
                             uint16_t *raw) {
    (void)ctx;
    if (!enc_ensure_bound(bus)) {
        return false;
    }
    /* Apply only volatile CONF = 0x0300 (normal power, 2x slow filter,
     * fast filter/hysteresis/watchdog off). Never touch OTP/range/zero. */
    as5600_config_t cfg;
    if (as5600_default_config(&cfg) != AS5600_OK) {
        return false;
    }
    cfg.power = AS5600_POWER_NORMAL;
    cfg.hysteresis = AS5600_HYSTERESIS_OFF;
    cfg.output = AS5600_OUTPUT_ANALOG_FULL;
    cfg.pwm_frequency = AS5600_PWM_115_HZ;
    cfg.slow_filter = AS5600_SLOW_FILTER_2X;
    cfg.fast_filter = AS5600_FAST_FILTER_OFF;
    cfg.watchdog = false;
    if (as5600_write_config(&g_enc[bus], &cfg, 5U) != AS5600_OK) {
        return false;
    }
    HAL_Delay(AS5600_POWER_UP_MS);

    /* Read back raw CONF register (0x07, 2 bytes big-endian). */
    uint8_t bytes[2] = {0U, 0U};
    if (HAL_I2C_Mem_Read(bus_to_hi2c(bus), (uint16_t)(AS5600_I2C_ADDRESS << 1U),
                         0x07U, I2C_MEMADD_SIZE_8BIT, bytes, 2U, 5U) != HAL_OK) {
        return false;
    }
    *conf_rb = (uint16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);

    as5600_diagnostics_t diag;
    if (as5600_read_diagnostics(&g_enc[bus], &diag, 5U) != AS5600_OK) {
        return false;
    }
    if (as5600_read_raw_angle(&g_enc[bus], raw, 5U) != AS5600_OK) {
        return false;
    }
    *status = diag.status;
    *agc = diag.agc;
    *mag = diag.magnitude;
    return true;
}

static uint8_t hw_enc_read_raw(void *ctx, uint8_t bus, uint16_t *raw) {
    (void)ctx;
    if (!enc_ensure_bound(bus)) {
        return (uint8_t)AS5600_ERROR_NOT_INITIALIZED;
    }
    return (uint8_t)as5600_read_raw_angle(&g_enc[bus], raw, MC_I2C_RAW_TIMEOUT_MS);
}

static uint8_t hw_enc_read_health(void *ctx, uint8_t bus, uint8_t *status,
                                  uint8_t *agc, uint16_t *mag) {
    (void)ctx;
    if (!enc_ensure_bound(bus)) {
        return (uint8_t)AS5600_ERROR_NOT_INITIALIZED;
    }
    as5600_diagnostics_t d;
    const as5600_result_t r = as5600_read_diagnostics(&g_enc[bus], &d, MC_I2C_RAW_TIMEOUT_MS);
    if (r == AS5600_OK) {
        *status = d.status;
        *agc = d.agc;
        *mag = d.magnitude;
    }
    return (uint8_t)r;
}

static uint8_t hw_enc_read_sample(void *ctx, uint8_t bus, mc_enc_full_t *out) {
    (void)ctx;
    if (!enc_ensure_bound(bus)) {
        return (uint8_t)AS5600_ERROR_NOT_INITIALIZED;
    }
    as5600_sample_t s;
    const as5600_result_t r = as5600_read_sample(&g_enc[bus], &s, MC_I2C_FULL_TIMEOUT_MS);
    out->err_code = (uint8_t)r;
    if (r == AS5600_OK) {
        out->raw_angle = s.raw_angle;
        out->angle = s.angle;
        out->status = s.diagnostics.status;
        out->agc = s.diagnostics.agc;
        out->magnitude = s.diagnostics.magnitude;
    }
    return (uint8_t)r;
}

static bool hw_enc_set_fast_mode(void *ctx, uint8_t bus, bool fast,
                                 uint32_t *timingr_out) {
    (void)ctx;
    if (bus > 1U) {
        return false;
    }
    I2C_HandleTypeDef *hi2c = bus_to_hi2c(bus);
    if (fast) {
        /* Bench-only candidate 0x30A31A23 requires verified 100 MHz I2C kernel. */
        const uint32_t clk = i2c_kernel_hz(bus);
        if (clk < 99000000U || clk > 101000000U) {
            return false;
        }
        hi2c->Init.Timing = MC_I2C_FAST_TIMINGR;
    } else {
        hi2c->Init.Timing = g_saved_i2c_timingr[bus];
    }
    if (HAL_I2C_Init(hi2c) != HAL_OK) {
        return false;
    }
    if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        return false;
    }
    if (HAL_I2CEx_ConfigDigitalFilter(hi2c, g_saved_i2c_df[bus]) != HAL_OK) {
        return false;
    }
    if (timingr_out != NULL) {
        *timingr_out = hi2c->Instance->TIMINGR;
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* TMC2240 SPI & configuration binding                                       */
/* ------------------------------------------------------------------------- */

static bool hw_tmc_configure(void *ctx, uint8_t axis, mc_axis_policy_t *policy) {
    (void)ctx;
    if (!g_spi_ready || axis >= MC_MAX_AXES || policy == NULL) {
        return false;
    }
    /* Capture GSTAT before tmc_axis_configure acknowledges power-up flags. */
    uint8_t gstat_pre = 0U;
    (void)tmc2240_check_faults(axis, &gstat_pre);
    policy->gstat_as_found = gstat_pre;

    axis_cfg_t cfg;
    if (axis == MC_AXIS_Z) {
        cfg = g_app_axis_cfg[APP_AXIS_Z];
    } else if (axis == MC_AXIS_X) {
        cfg = g_app_axis_cfg[APP_AXIS_X];
    } else {
        /* Copy X config and apply rot_build_cfg conversion; clamp <= 800 mA. */
        cfg = g_app_axis_cfg[APP_AXIS_X];
        cfg.name = 'R';
        cfg.steps_per_mm = MC_ROT_USTEPS_PER_DEG;
        cfg.tmc.run_current_ma = MC_THETA_MAX_MA;
        cfg.tmc.home_current_ma = 600U;
        cfg.tmc.hold_current_pct = 50U;
        cfg.tmc.stealth_max_mm_s = MC_ROT_STEALTH_DEG_S;
    }

    /* Clamp configured current DOWN to the lower of confirmed and commissioned values. */
    if (policy->run_current_ma > 0U && policy->run_current_ma < cfg.tmc.run_current_ma) {
        cfg.tmc.run_current_ma = policy->run_current_ma;
    }
    if (axis == MC_AXIS_THETA && cfg.tmc.run_current_ma > MC_THETA_MAX_MA) {
        cfg.tmc.run_current_ma = MC_THETA_MAX_MA;
    }
    if (policy->hold_pct > 0U) {
        cfg.tmc.hold_current_pct = policy->hold_pct;
    }
    cfg.tmc.microsteps = 16U; /* Verified 1/16 microsteps + DEDGE=1 */

    if (tmc_axis_configure(axis, &cfg) != TMC2240_OK) {
        return false;
    }
    const tmc_axis_info_t *info = tmc_axis_info(axis);
    if (info == NULL || !info->configured) {
        return false;
    }
    policy->configured = true;
    policy->applied_current_ma = info->current.irun_ma;
    policy->applied_range = info->current.range;
    policy->applied_gs = info->current.gs;
    policy->applied_irun = info->current.irun;
    policy->applied_ihold = info->current.ihold;
    return true;
}

static bool hw_tmc_poll_status(void *ctx, uint8_t axis, mc_driver_snap_t *out) {
    (void)ctx;
    if (!g_spi_ready || axis >= MC_MAX_AXES || out == NULL) {
        return false;
    }
    uint32_t drv = 0U, tstep = 0U, sg4 = 0U, mscnt = 0U;
    uint8_t spi = 0U;
    /* Check explicit transfer results directly; do not swallow SPI errors. */
    if (tmc2240_driver_status(axis, &drv) != TMC2240_OK) {
        return false;
    }
    (void)tmc2240_hal_getSPIStatus(axis, &spi);
    (void)tmc2240_readRegister(axis, TMC2240_TSTEP, &tstep);
    (void)tmc2240_readRegister(axis, TMC2240_SG4_RESULT, &sg4);
    (void)tmc2240_readRegister(axis, TMC2240_MSCNT, &mscnt);

    out->drv_status  = drv;
    out->tstep       = tstep & 0xFFFFFU;
    out->sg4         = (uint16_t)(sg4 & 0x3FFU);
    out->mscnt       = (uint16_t)(mscnt & 0x3FFU);
    out->spi_status  = spi;
    out->cs_actual   = (uint8_t)((drv & TMC2240_CS_ACTUAL_MASK) >> TMC2240_CS_ACTUAL_SHIFT);
    out->stealth     = (drv & TMC2240_STEALTH_MASK) != 0U;
    out->standstill  = (drv & TMC2240_STST_MASK) != 0U;
    out->otpw        = (drv & TMC2240_OTPW_MASK) != 0U;
    out->ot          = (drv & TMC2240_OT_MASK) != 0U;
    out->short_fault = (drv & (TMC2240_S2GA_MASK | TMC2240_S2GB_MASK |
                               TMC2240_S2VSA_MASK | TMC2240_S2VSB_MASK)) != 0U;
    out->open_load   = !out->stealth && !out->standstill &&
                       ((drv & (TMC2240_OLA_MASK | TMC2240_OLB_MASK)) != 0U);
    out->reset_flag  = (spi & 0x01U) != 0U;
    out->drv_err     = (spi & 0x02U) != 0U;
    out->ok          = true;
    return true;
}

static bool hw_tmc_poll_adc(void *ctx, uint8_t axis, mc_driver_adc_t *out) {
    (void)ctx;
    if (!g_spi_ready || axis >= MC_MAX_AXES || out == NULL) {
        return false;
    }
    int16_t t_c10 = 0;
    uint32_t vm_mv = 0U;
    if (tmc2240_get_temperature_c10(axis, &t_c10) != TMC2240_OK ||
        tmc2240_get_vsupply_mV(axis, &vm_mv) != TMC2240_OK) {
        return false;
    }
    out->temp_c10 = t_c10;
    out->vm_mv = vm_mv;
    out->ok = true;
    return true;
}

static bool hw_tmc_spi_set_prescaler(void *ctx, uint32_t div, uint32_t *sck_hz) {
    (void)ctx;
    const uint32_t kernel = spi123_kernel_hz();
    const uint32_t hal_div = spi_div_to_hal(div);
    if (kernel == 0U || hal_div == 0U) {
        return false;
    }
    const uint32_t sck = kernel / div;
    if (sck == 0U || sck > 10000000U) {
        return false;
    }
    /* Deselect all CS lines before changing SPI prescaler. */
    spi_cs_all_high();
    if (tmc2240_hal_isInitialized()) {
        (void)tmc2240_hal_deinit();
    }
    hspi1.Init.BaudRatePrescaler = hal_div;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        return false;
    }
    if (!spi_rebind_tmc_hal()) {
        return false;
    }
    g_spi_div_active = div;
    if (sck_hz != NULL) {
        *sck_hz = sck;
    }
    return true;
}

static bool hw_tmc_spi_read_reg(void *ctx, uint8_t axis, uint8_t reg, uint32_t *val) {
    (void)ctx;
    if (!g_spi_ready || axis >= MC_MAX_AXES || val == NULL) {
        return false;
    }
    return tmc2240_readRegister(axis, reg, val) == TMC2240_OK;
}

/* ------------------------------------------------------------------------- */
/* TIM2 1 MHz timestamp + independent 1 ms interlock compare                 */
/* ------------------------------------------------------------------------- */

static void tim2_start_interlock(void) {
    /* CubeMX sets TIM2 PSC=199 (1 MHz at 200 MHz APB1 timer clock), ARR=0xFFFFFFFF. */
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CNT = 0U;
    TIM2->CCR1 = 1000U;
    TIM2->SR = ~TIM_SR_CC1IF;
    TIM2->DIER |= TIM_DIER_CC1IE;
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* ------------------------------------------------------------------------- */
/* fw_app.h hooks                                                            */
/* ------------------------------------------------------------------------- */

void fw_app_init(void) {
    /* Passive boot: keep all ENNs HIGH immediately. */
    hw_disable_all_drivers(NULL);
    spi_cs_all_high();

    g_saved_i2c_timingr[0] = hi2c4.Instance->TIMINGR;
    g_saved_i2c_timingr[1] = hi2c1.Instance->TIMINGR;
    g_saved_i2c_df[0] = (hi2c4.Instance->CR1 & I2C_CR1_DNF) >> I2C_CR1_DNF_Pos;
    g_saved_i2c_df[1] = (hi2c1.Instance->CR1 & I2C_CR1_DNF) >> I2C_CR1_DNF_Pos;

    tim2_start_interlock();
    stepgen_char_set_time_fn(hw_now_us_raw);
    (void)hw_step_bind_axis(NULL, MC_AXIS_X);
    (void)spi_rebind_tmc_hal();
    (void)usb_cdc_port_init();

    mc_hal_t hal;
    memset(&hal, 0, sizeof(hal));
    hal.now_us                = hw_now_us;
    hal.now_ms                = hw_now_ms;
    hal.usb_mounted           = hw_usb_mounted;
    hal.usb_write             = hw_usb_write;
    hal.b1_asserted           = hw_b1_asserted;
    hal.get_board_info        = hw_get_board_info;
    hal.set_driver_enable     = hw_set_driver_enable;
    hal.disable_all_drivers   = hw_disable_all_drivers;
    hal.step_bind_axis        = hw_step_bind_axis;
    hal.step_set_inhibit      = hw_step_set_inhibit;
    hal.step_set_limits       = hw_step_set_limits;
    hal.step_set_vmin         = hw_step_set_vmin;
    hal.step_set_position     = hw_step_set_position;
    hal.step_position         = hw_step_position;
    hal.step_busy             = hw_step_busy;
    hal.step_validate_rate    = hw_step_validate_rate;
    hal.step_reset_stats      = hw_step_reset_stats;
    hal.step_get_stats        = hw_step_get_stats;
    hal.step_move_to          = hw_step_move_to;
    hal.step_halt             = hw_step_halt;
    hal.step_take_limit_hit   = hw_step_take_limit_hit;
    hal.enc_configure         = hw_enc_configure;
    hal.enc_read_raw          = hw_enc_read_raw;
    hal.enc_read_health       = hw_enc_read_health;
    hal.enc_read_sample       = hw_enc_read_sample;
    hal.enc_set_fast_mode     = hw_enc_set_fast_mode;
    hal.tmc_configure         = hw_tmc_configure;
    hal.tmc_poll_status       = hw_tmc_poll_status;
    hal.tmc_poll_adc          = hw_tmc_poll_adc;
    hal.tmc_spi_set_prescaler = hw_tmc_spi_set_prescaler;
    hal.tmc_spi_read_reg      = hw_tmc_spi_read_reg;

    mc_core_init(&g_mc, &hal);
}

void fw_app_poll(void) {
    usb_cdc_port_poll();
    uint8_t rx[64];
    const uint32_t n = usb_cdc_port_read(rx, sizeof(rx));
    if (n > 0U) {
        mc_core_rx_bytes(&g_mc, rx, n);
    }
    mc_core_poll(&g_mc);
}

void fw_app_tim_irq(uint8_t motor) {
    if (motor == 0U) {
        if ((TIM2->SR & TIM_SR_CC1IF) != 0U) {
            TIM2->SR = ~TIM_SR_CC1IF;
            TIM2->CCR1 = TIM2->CNT + 1000U;
            mc_core_interlock_tick_1ms(&g_mc);
        }
    } else if (motor == 1U) {
        stepgen_irq_handler(MC_STEPGEN_SLOT);
    }
}

bool fw_app_usart2_irq(void) {
    /* Leave USART2 untouched by the characterization profile. */
    return false;
}

void fw_app_error_handler(void) {
    stepgen_char_set_inhibit(MC_STEPGEN_SLOT, true);
    stepgen_halt(MC_STEPGEN_SLOT);
    hw_disable_all_drivers(NULL);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin) {
        mc_core_b1_irq(&g_mc);
    }
}
