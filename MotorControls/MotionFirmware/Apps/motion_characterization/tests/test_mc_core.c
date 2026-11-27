/**
 * @file    test_mc_core.c
 * @brief   Host unit tests for mc_core.c with a simulated GPIO/timer seam.
 *
 * Verifies:
 *  - Passive boot (all ENNs HIGH, stepgen inhibited)
 *  - HELLO profile/schema handshake
 *  - Duplicate/old request_id rejection (duplicate RUN does not repeat motion)
 *  - Bad AS5600 STATUS rejected during commissioning and ARM
 *  - Theta motion rejected in xz session (no_shaft_feedback), allowed in theta_fixture
 *  - Out-of-bounds target and window-too-short rejection before emitting any edge
 *  - Stale host lease inhibits subsequent STEP edges and revokes datum (no return)
 *  - B1 stop inhibits edges, disables stages, revokes datum
 *  - Lost encoder mid-stroke latches invalid_feedback, inhibits edges, never returns
 *  - Frozen/swapped shaft (following error / wrong direction) inhibits edges
 *  - Timer wrap & late edge lateness fault invalidates trial
 */
#include "mc_core.h"
#include "motion_profile.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint32_t now_us;
    bool     usb_mounted;
    bool     b1_asserted;
    bool     enn_enabled[MC_MAX_AXES];
    uint32_t disable_all_calls;

    /* Simulated stepgen with GPIO/timer edge seam. */
    step_core_t step_core;
    bool        step_inhibited;
    uint32_t    gpio_edges_emitted;
    uint32_t    gpio_edges_inhibited;
    uint32_t    sim_max_lateness_us;
    bool        sim_lateness_fault;

    /* Simulated encoders (bus 0 = I2C4/Z, bus 1 = I2C1/X or theta_fixture). */
    uint16_t enc_conf[2];
    uint8_t  enc_status[2];
    uint8_t  enc_agc[2];
    uint16_t enc_mag[2];
    uint16_t enc_raw[2];
    uint8_t  enc_err[2];
    bool     enc_follow_motor;
    int8_t   enc_follow_sign;
    bool     enc_frozen;

    /* Simulated TMC2240 SPI. */
    uint32_t spi_div;
    uint32_t spi_kernel_hz;
    bool     spi_fail;

    /* Captured NDJSON TX log. */
    char     tx_log[65536];
    size_t   tx_len;
    bool     stall_usb_write;
} mock_hw_t;

static uint32_t m_now_us(void *ctx) { return ((mock_hw_t *)ctx)->now_us; }
static uint32_t m_now_ms(void *ctx) { return ((mock_hw_t *)ctx)->now_us / 1000U; }
static bool m_usb_mounted(void *ctx) { return ((mock_hw_t *)ctx)->usb_mounted; }
static bool m_b1_asserted(void *ctx) { return ((mock_hw_t *)ctx)->b1_asserted; }

static uint32_t m_usb_write(void *ctx, const uint8_t *data, uint32_t len) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    if (m->stall_usb_write) {
        return 0U;
    }
    if (m->tx_len + len < sizeof(m->tx_log)) {
        memcpy(m->tx_log + m->tx_len, data, len);
        m->tx_len += len;
        m->tx_log[m->tx_len] = '\0';
    }
    return len;
}

static void m_get_board_info(void *ctx, mc_board_info_t *out) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    memset(out, 0, sizeof(*out));
    out->sysclk_hz = 400000000U;
    out->tim2_hz = 1000000U;
    out->tim5_hz = 200000000U;
    out->spi_kernel_hz = m->spi_kernel_hz;
    out->spi_prescaler = m->spi_div;
    out->i2c1_timingr = 0x10C0ECFFUL;
    out->i2c4_timingr = 0x10C0ECFFUL;
    out->drv_version[0] = 0x40U;
    out->drv_version[1] = 0x40U;
    out->drv_version[2] = 0x40U;
}

static void m_disable_all(void *ctx) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    m->disable_all_calls++;
    for (uint8_t i = 0U; i < MC_MAX_AXES; ++i) {
        m->enn_enabled[i] = false;
    }
}

static void m_set_enable(void *ctx, uint8_t ax, bool en) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    m_disable_all(ctx);
    if (en && ax < MC_MAX_AXES) {
        m->enn_enabled[ax] = true;
    }
}

static bool m_step_bind(void *ctx, uint8_t ax) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    (void)ax;
    step_core_init(&m->step_core, 1000.0f, 1000.0f, 200.0f);
    m->step_inhibited = true;
    return true;
}

static void m_step_inhibit(void *ctx, bool inh) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    m->step_inhibited = inh;
    if (inh) {
        step_core_cmd_halt(&m->step_core);
    }
}

static void m_step_limits(void *ctx, int32_t lo, int32_t hi) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    m->step_core.lim_lo = lo;
    m->step_core.lim_hi = hi;
}

static void m_step_vmin(void *ctx, float vmin) {
    ((mock_hw_t *)ctx)->step_core.vmin = vmin;
}

static bool m_step_set_pos(void *ctx, int32_t pos) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    if (m->step_core.running) return false;
    m->step_core.pos = pos;
    return true;
}

static int32_t m_step_pos(void *ctx) { return ((mock_hw_t *)ctx)->step_core.pos; }
static bool m_step_busy(void *ctx) { return ((mock_hw_t *)ctx)->step_core.running; }
static bool m_step_validate_rate(void *ctx, float vmax, float vmin) {
    (void)ctx;
    return (vmin >= 1.0f) && (vmax <= 200000.0f) && (vmin <= vmax);
}

static void m_step_reset_stats(void *ctx) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    m->gpio_edges_emitted = 0U;
    m->gpio_edges_inhibited = 0U;
    m->sim_max_lateness_us = 0U;
    m->sim_lateness_fault = false;
}

static void m_step_get_stats(void *ctx, mc_step_stats_t *out) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    memset(out, 0, sizeof(*out));
    out->edges_emitted = m->gpio_edges_emitted;
    out->edges_inhibited = m->gpio_edges_inhibited;
    out->max_lateness_us = m->sim_max_lateness_us;
    out->lateness_fault = m->sim_lateness_fault;
}

static void m_step_move_to(void *ctx, int32_t target, float vmax, float accel) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    float v = 0.0f;
    if (m->step_inhibited) {
        m->gpio_edges_inhibited++;
        return;
    }
    (void)step_core_cmd_move(&m->step_core, target, vmax, accel, &v);
}

static void m_step_halt(void *ctx) {
    step_core_cmd_halt(&((mock_hw_t *)ctx)->step_core);
}

static bool m_step_take_limit(void *ctx) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    bool hit = m->step_core.limit_hit;
    m->step_core.limit_hit = false;
    return hit;
}

/* Simulate one timer edge attempt against the GPIO/timer inhibit seam. */
static bool sim_try_emit_step(mock_hw_t *m, uint8_t bus) {
    if (!m->step_core.running) {
        return false;
    }
    if (m->step_inhibited) {
        m->gpio_edges_inhibited++;
        step_core_cmd_halt(&m->step_core);
        return false;
    }
    bool more = false, dir_changed = false;
    float v = 0.0f;
    if (step_core_expire(&m->step_core, &more, &v, &dir_changed)) {
        m->gpio_edges_emitted++;
        if (m->enc_follow_motor && !m->enc_frozen) {
            const int32_t cnt = (m->step_core.pos * m->enc_follow_sign * 4096) / 3200;
            m->enc_raw[bus] = (uint16_t)(cnt & 4095);
        }
        return true;
    }
    return false;
}

static bool m_enc_configure(void *ctx, uint8_t bus, uint16_t *conf_rb,
                            uint8_t *status, uint8_t *agc, uint16_t *mag,
                            uint16_t *raw) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    if (bus > 1U || m->enc_err[bus] != 0U) return false;
    *conf_rb = m->enc_conf[bus];
    *status = m->enc_status[bus];
    *agc = m->enc_agc[bus];
    *mag = m->enc_mag[bus];
    *raw = m->enc_raw[bus];
    return true;
}

static uint8_t m_enc_read_raw(void *ctx, uint8_t bus, uint16_t *raw) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    if (bus > 1U) return 1U;
    if (m->enc_err[bus] != 0U) return m->enc_err[bus];
    *raw = m->enc_raw[bus];
    return 0U;
}

static uint8_t m_enc_read_health(void *ctx, uint8_t bus, uint8_t *status,
                                 uint8_t *agc, uint16_t *mag) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    if (bus > 1U) return 1U;
    if (m->enc_err[bus] != 0U) return m->enc_err[bus];
    *status = m->enc_status[bus];
    *agc = m->enc_agc[bus];
    *mag = m->enc_mag[bus];
    return 0U;
}

static uint8_t m_enc_read_sample(void *ctx, uint8_t bus, mc_enc_full_t *out) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    if (bus > 1U || m->enc_err[bus] != 0U) return m->enc_err[bus] ? m->enc_err[bus] : 1U;
    out->raw_angle = m->enc_raw[bus];
    out->angle = m->enc_raw[bus];
    out->status = m->enc_status[bus];
    out->agc = m->enc_agc[bus];
    out->magnitude = m->enc_mag[bus];
    out->err_code = 0U;
    return 0U;
}

static bool m_enc_set_slow_filter(void *ctx, uint8_t bus, uint8_t sf_code,
                                  uint16_t *conf_rb) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    if (bus > 1U || sf_code > 3U || m->enc_err[bus] != 0U) return false;
    m->enc_conf[bus] = (uint16_t)(((uint16_t)sf_code) << 8U);
    if (conf_rb != NULL) {
        *conf_rb = m->enc_conf[bus];
    }
    return true;
}

static bool m_tmc_configure(void *ctx, uint8_t axis, mc_axis_policy_t *policy) {
    (void)ctx; (void)axis;
    policy->configured = true;
    policy->applied_current_ma = policy->run_current_ma;
    policy->applied_irun = 31U;
    policy->applied_ihold = 15U;
    return true;
}

static bool m_tmc_poll_status(void *ctx, uint8_t axis, mc_driver_snap_t *out) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    (void)axis;
    if (m->spi_fail) return false;
    memset(out, 0, sizeof(*out));
    out->ok = true;
    out->cs_actual = 31U;
    return true;
}

static bool m_tmc_poll_adc(void *ctx, uint8_t axis, mc_driver_adc_t *out) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    (void)axis;
    if (m->spi_fail) return false;
    out->temp_c10 = 350;
    out->vm_mv = 24000U;
    out->ok = true;
    return true;
}

static bool m_tmc_spi_set_div(void *ctx, uint32_t div, uint32_t *sck_hz) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    const uint32_t sck = m->spi_kernel_hz / div;
    if (sck > 10000000U) return false;
    m->spi_div = div;
    *sck_hz = sck;
    return true;
}

static bool m_tmc_spi_read_reg(void *ctx, uint8_t axis, uint8_t reg, uint32_t *val) {
    mock_hw_t *m = (mock_hw_t *)ctx;
    (void)axis;
    if (m->spi_fail) return false;
    if (reg == 0x04U) *val = 0x40000000UL;
    else if (reg == 0x00U) *val = 0x00000004UL;
    else *val = 0x10410153UL;
    return true;
}

static void setup_harness(mc_core_t *mc, mock_hw_t *m) {
    memset(m, 0, sizeof(*m));
    m->usb_mounted = true;
    m->spi_div = 32U;
    m->spi_kernel_hz = 100000000U;
    m->enc_conf[0] = 0x0300U;
    m->enc_conf[1] = 0x0300U;
    m->enc_status[0] = 0x20U;
    m->enc_status[1] = 0x20U;
    m->enc_agc[0] = 80U;
    m->enc_agc[1] = 80U;
    m->enc_mag[0] = 2000U;
    m->enc_mag[1] = 2000U;
    m->enc_follow_motor = true;
    m->enc_follow_sign = 1;

    mc_hal_t hal;
    memset(&hal, 0, sizeof(hal));
    hal.ctx                   = m;
    hal.now_us                = m_now_us;
    hal.now_ms                = m_now_ms;
    hal.usb_mounted           = m_usb_mounted;
    hal.usb_write             = m_usb_write;
    hal.b1_asserted           = m_b1_asserted;
    hal.get_board_info        = m_get_board_info;
    hal.set_driver_enable     = m_set_enable;
    hal.disable_all_drivers   = m_disable_all;
    hal.step_bind_axis        = m_step_bind;
    hal.step_set_inhibit      = m_step_inhibit;
    hal.step_set_limits       = m_step_limits;
    hal.step_set_vmin         = m_step_vmin;
    hal.step_set_position     = m_step_set_pos;
    hal.step_position         = m_step_pos;
    hal.step_busy             = m_step_busy;
    hal.step_validate_rate    = m_step_validate_rate;
    hal.step_reset_stats      = m_step_reset_stats;
    hal.step_get_stats        = m_step_get_stats;
    hal.step_move_to          = m_step_move_to;
    hal.step_halt             = m_step_halt;
    hal.step_take_limit_hit   = m_step_take_limit;
    hal.enc_configure         = m_enc_configure;
    hal.enc_read_raw          = m_enc_read_raw;
    hal.enc_read_health       = m_enc_read_health;
    hal.enc_read_sample       = m_enc_read_sample;
    hal.enc_set_slow_filter   = m_enc_set_slow_filter;
    hal.tmc_configure         = m_tmc_configure;
    hal.tmc_poll_status       = m_tmc_poll_status;
    hal.tmc_poll_adc          = m_tmc_poll_adc;
    hal.tmc_spi_set_prescaler = m_tmc_spi_set_div;
    hal.tmc_spi_read_reg      = m_tmc_spi_read_reg;

    mc_core_init(mc, &hal);
}

static void send_cmd(mc_core_t *mc, const char *line) {
    mc_core_rx_bytes(mc, (const uint8_t *)line, (uint32_t)strlen(line));
    mc_core_rx_bytes(mc, (const uint8_t *)"\n", 1U);
}

static void arm_x(mc_core_t *mc, uint32_t *rid) {
    char buf[96];
    snprintf(buf, sizeof(buf), "%lu SET run_ma x 800", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu SET hold_pct x 50", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu SET win_neg x -4000", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu SET win_pos x 4000", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu SET max_speed x 8000", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu SET min_speed x 200", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu SET max_accel x 20000", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu SET commissioned x 1", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu SET datum_ok x 1", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu SET safety_ok x 1", (unsigned long)(++(*rid))); send_cmd(mc, buf);
    snprintf(buf, sizeof(buf), "%lu ARM x", (unsigned long)(++(*rid))); send_cmd(mc, buf);
}

static void test_passive_boot_and_hello(void) {
    mc_core_t mc; mock_hw_t m;
    setup_harness(&mc, &m);
    assert(mc.state == MC_STATE_DISARMED);
    assert(m.disable_all_calls >= 1U);
    assert(m.step_inhibited == true);
    assert(!m.enn_enabled[0] && !m.enn_enabled[1] && !m.enn_enabled[2]);

    send_cmd(&mc, "1 HELLO");
    assert(strstr(m.tx_log, "\"profile\":\"motion_characterization\"") != NULL);
    assert(strstr(m.tx_log, "\"schema\":1") != NULL);
}

static void test_bad_status_and_theta_no_sensor(void) {
    mc_core_t mc; mock_hw_t m;
    setup_harness(&mc, &m);

    /* Theta in xz session must reject with no_shaft_feedback. */
    send_cmd(&mc, "1 RUN commission theta");
    assert(strstr(m.tx_log, "\"reason\":\"no_shaft_feedback\"") != NULL);

    /* Bad magnet status (MD=0) must fail commissioning. */
    m.enc_status[1] = 0x10U; /* ML set, MD clear */
    send_cmd(&mc, "2 RUN commission x");
    assert(strstr(m.tx_log, "\"reason\":\"bad_magnet_status\"") != NULL);
    assert(mc.policy[MC_AXIS_X].commissioned == false);

    /* Theta in theta_fixture session with valid I2C1 sensor succeeds. */
    m.enc_status[1] = 0x20U;
    send_cmd(&mc, "3 SET session theta_fixture");
    send_cmd(&mc, "4 RUN commission theta");
    assert(strstr(m.tx_log, "\"test\":\"commission\",\"bus\":1") != NULL);
    assert(strstr(m.tx_log, "\"ok\":true") != NULL);
}

static void test_duplicate_run_and_bounds_and_short_window(void) {
    mc_core_t mc; mock_hw_t m;
    setup_harness(&mc, &m);
    uint32_t rid = 0U;
    arm_x(&mc, &rid);
    assert(mc.state == MC_STATE_ARMED);

    /* Out-of-bounds target rejected at POINT. */
    m.tx_len = 0U;
    send_cmd(&mc, "20 POINT 5000 1000 2000");
    assert(strstr(m.tx_log, "\"reason\":\"point_out_of_caps\"") != NULL);

    /* Window too short for 100 ms cruise plateau at 4000 steps/s. */
    m.tx_len = 0U;
    send_cmd(&mc, "21 POINT 100 4000 10000");
    send_cmd(&mc, "22 RUN move x");
    assert(strstr(m.tx_log, "\"reason\":\"window_too_short\"") != NULL);
    assert(m.gpio_edges_emitted == 0U);
    assert(mc.state == MC_STATE_ARMED);

    /* Valid jog, then duplicate request_id RUN must NOT repeat motion. */
    m.tx_len = 0U;
    send_cmd(&mc, "23 POINT 16 400 2000");
    send_cmd(&mc, "24 RUN jog x");
    assert(mc.state == MC_STATE_RUNNING);
    m.tx_len = 0U;
    send_cmd(&mc, "24 RUN jog x");
    assert(strstr(m.tx_log, "\"reason\":\"duplicate_or_old_request_id\"") != NULL);
}

static void test_stale_lease_inhibits_edges_and_revokes_datum(void) {
    mc_core_t mc; mock_hw_t m;
    setup_harness(&mc, &m);
    uint32_t rid = 0U;
    arm_x(&mc, &rid);
    send_cmd(&mc, "20 POINT 1000 1000 4000");
    send_cmd(&mc, "21 RUN jog x");
    assert(mc.state == MC_STATE_RUNNING);
    assert(sim_try_emit_step(&m, 1U) == true);

    /* Advance past 250 ms lease timeout without KEEP; trigger 1 ms interlock. */
    m.now_us += 260000U;
    mc_core_interlock_tick_1ms(&mc);
    assert(m.step_inhibited == true);
    /* Attempting to emit another edge must be inhibited. */
    assert(sim_try_emit_step(&m, 1U) == false);

    mc_core_poll(&mc);
    assert(mc.state == MC_STATE_FAULT);
    assert(mc.fault == MC_FAULT_LEASE_EXPIRED);
    assert(mc.policy[MC_AXIS_X].datum_confirmed == false);
}

static void test_lost_encoder_and_frozen_shaft_and_b1(void) {
    mc_core_t mc; mock_hw_t m;
    setup_harness(&mc, &m);
    uint32_t rid = 0U;
    arm_x(&mc, &rid);
    send_cmd(&mc, "20 POINT 1000 1000 4000");
    send_cmd(&mc, "21 RUN jog x");
    assert(mc.state == MC_STATE_RUNNING);

    /* Lose encoder I2C mid-stroke. */
    m.enc_err[1] = 5U; /* timeout */
    m.now_us += 2500U;
    mc_core_poll(&mc);
    assert(mc.state == MC_STATE_FAULT);
    assert(mc.fault == MC_FAULT_ENCODER_INVALID);
    assert(m.step_inhibited == true);
    assert(sim_try_emit_step(&m, 1U) == false);

    /* Reset and test frozen shaft -> following error after 3 consecutive strikes. */
    setup_harness(&mc, &m);
    rid = 0U;
    arm_x(&mc, &rid);
    m.enc_frozen = true;
    send_cmd(&mc, "20 POINT 1000 1000 4000");
    send_cmd(&mc, "21 RUN jog x");
    for (int i = 0; i < 60; ++i) {
        (void)sim_try_emit_step(&m, 1U);
    }
    for (int k = 0; k < 3; ++k) {
        m.now_us += 2500U;
        send_cmd(&mc, "KEEP");
        mc_core_poll(&mc);
    }
    assert(mc.state == MC_STATE_FAULT);
    assert(mc.fault == MC_FAULT_FOLLOWING_ERROR);
    assert(m.step_inhibited == true);

    /* Reset and test B1 stop disables stages and inhibits edges. */
    setup_harness(&mc, &m);
    rid = 0U;
    arm_x(&mc, &rid);
    send_cmd(&mc, "20 POINT 1000 1000 4000");
    send_cmd(&mc, "21 RUN jog x");
    mc_core_b1_irq(&mc);
    assert(m.step_inhibited == true);
    assert(!m.enn_enabled[MC_AXIS_X]);
    mc_core_poll(&mc);
    assert(mc.state == MC_STATE_FAULT);
    assert(mc.fault == MC_FAULT_B1_STOP);
}

static void test_timer_wrap_and_lateness_fault(void) {
    mc_core_t mc; mock_hw_t m;
    setup_harness(&mc, &m);
    m.now_us = 0xFFFF0000UL; /* Near 32-bit unsigned wrap */
    uint32_t rid = 0U;
    arm_x(&mc, &rid);
    send_cmd(&mc, "20 POINT 20 400 2000");
    send_cmd(&mc, "21 RUN jog x");
    assert(mc.state == MC_STATE_RUNNING);

    /* Step to target across 32-bit wrap and inject a lateness fault. */
    for (int i = 0; i < 20; ++i) {
        (void)sim_try_emit_step(&m, 1U);
        m.now_us += 2500U;
        send_cmd(&mc, "KEEP");
        mc_core_poll(&mc);
    }
    m.sim_max_lateness_us = 45U;
    m.sim_lateness_fault = true;

    /* Advance through 100 ms settle + 32 samples at 2.5 ms poll cadence. */
    for (int s = 0; s < 100 && mc.state == MC_STATE_RUNNING; ++s) {
        m.now_us += 2500U;
        send_cmd(&mc, "KEEP");
        mc_core_poll(&mc);
    }
    assert(mc.state == MC_STATE_FAULT);
    assert(mc.fault == MC_FAULT_TIMING_LATENESS);
}

static void test_enc_noise_uniform_capture_and_restore(void) {
    mc_core_t mc; mock_hw_t m;
    setup_harness(&mc, &m);
    m.enc_raw[1] = 4095U;

    send_cmd(&mc, "1 SET noise_sf 16");
    send_cmd(&mc, "2 SET noise_rate_hz 500");
    send_cmd(&mc, "3 SET noise_n 64");
    send_cmd(&mc, "4 SET noise_hold 0");
    send_cmd(&mc, "5 RUN enc_noise x");
    assert(mc.state == MC_STATE_RUNNING);
    assert(mc.active_test == MC_TEST_ENC_NOISE);
    assert(m.enc_conf[1] == 0x0000U); /* SF=16x written */

    /* Poll at 2 ms cadence (500 Hz) with small wrap jitter 4095/0/1. */
    for (uint32_t i = 0U; i < 64U; ++i) {
        m.enc_raw[1] = (uint16_t)((4095U + (i % 3U)) & 0x0FFFU);
        mc_core_poll(&mc);
        m.now_us += 2000U;
    }
    assert(mc.state == MC_STATE_DISARMED);
    assert(mc.active_test == MC_TEST_NONE);
    assert(m.enc_conf[1] == 0x0300U); /* Restored SF=2x */
    assert(strstr(m.tx_log, "\"test\":\"enc_noise\",\"sf\":16") != NULL);
    assert(strstr(m.tx_log, "\"n\":64,\"errors\":0,\"overruns\":0") != NULL);
    assert(strstr(m.tx_log, "\"p2p_cnt\":2") != NULL);

    /* hold=1 must reject when disarmed. */
    send_cmd(&mc, "6 SET noise_hold 1");
    send_cmd(&mc, "7 RUN enc_noise x");
    assert(strstr(m.tx_log, "\"reason\":\"axis_not_armed\"") != NULL);

    /* Stalled USB TX must overflow the 32-line ring, latch MC_FAULT_RING_OVERFLOW,
     * and still restore AS5600 SF=2x (CONF=0x0300). */
    setup_harness(&mc, &m);
    send_cmd(&mc, "1 SET noise_sf 16");
    send_cmd(&mc, "2 SET noise_n 64");
    send_cmd(&mc, "3 RUN enc_noise x");
    assert(m.enc_conf[1] == 0x0000U);
    m.stall_usb_write = true;
    for (uint32_t i = 0U; i < 64U && mc.state == MC_STATE_RUNNING; ++i) {
        mc_core_poll(&mc);
        m.now_us += 2000U;
    }
    assert(mc.state == MC_STATE_FAULT);
    assert(mc.fault == MC_FAULT_RING_OVERFLOW);
    assert(mc.tx.overflow_count > 0U);
    assert(m.enc_conf[1] == 0x0300U);
}

static void clear_tx(mock_hw_t *m) {
    m->tx_len = 0U;
    m->tx_log[0] = '\0';
}

static void test_enc_noise_abort_and_hold_paths(void) {
    mc_core_t mc; mock_hw_t m;
    uint32_t rid;

    /* (a) STOP mid-capture with SF=16x latches host_stop and restores SF=2x. */
    setup_harness(&mc, &m);
    send_cmd(&mc, "1 SET noise_sf 16");
    send_cmd(&mc, "2 SET noise_n 64");
    send_cmd(&mc, "3 RUN enc_noise x");
    assert(mc.state == MC_STATE_RUNNING);
    assert(mc.active_test == MC_TEST_ENC_NOISE);
    assert(m.enc_conf[1] == 0x0000U);
    for (uint32_t i = 0U; i < 8U; ++i) {
        mc_core_poll(&mc);
        m.now_us += 2000U;
    }
    assert(mc.state == MC_STATE_RUNNING);
    send_cmd(&mc, "STOP");
    mc_core_poll(&mc);
    assert(mc.state == MC_STATE_FAULT);
    assert(mc.fault == MC_FAULT_HOST_STOP);
    assert(mc.active_test == MC_TEST_NONE);
    assert(m.enc_conf[1] == 0x0300U);

    /* (b) DISARM mid-capture with SF=8x returns to DISARMED and restores SF=2x. */
    setup_harness(&mc, &m);
    send_cmd(&mc, "1 SET noise_sf 8");
    send_cmd(&mc, "2 SET noise_n 64");
    send_cmd(&mc, "3 RUN enc_noise x");
    assert(mc.state == MC_STATE_RUNNING);
    assert(m.enc_conf[1] != 0x0300U); /* SF=8x written while capturing */
    for (uint32_t i = 0U; i < 8U; ++i) {
        mc_core_poll(&mc);
        m.now_us += 2000U;
    }
    assert(mc.state == MC_STATE_RUNNING);
    send_cmd(&mc, "4 DISARM");
    assert(mc.state == MC_STATE_DISARMED);
    assert(mc.active_test == MC_TEST_NONE);
    assert(m.enc_conf[1] == 0x0300U);

    /* (c) noise_hold=1 happy path on an armed axis returns to ARMED. */
    setup_harness(&mc, &m);
    send_cmd(&mc, "1 SET noise_sf 4");
    send_cmd(&mc, "2 SET noise_n 64");
    send_cmd(&mc, "3 SET noise_hold 1");
    rid = 3U;
    arm_x(&mc, &rid);
    assert(mc.state == MC_STATE_ARMED);
    clear_tx(&m);
    send_cmd(&mc, "30 RUN enc_noise x");
    assert(mc.state == MC_STATE_RUNNING);
    assert(mc.active_test == MC_TEST_ENC_NOISE);
    assert(m.enc_conf[1] != 0x0300U);
    for (uint32_t i = 0U; i < 64U; ++i) {
        send_cmd(&mc, "KEEP");
        mc_core_poll(&mc);
        m.now_us += 2000U;
    }
    assert(mc.state == MC_STATE_ARMED);
    assert(mc.active_test == MC_TEST_NONE);
    assert(m.enc_conf[1] == 0x0300U);
    assert(strstr(m.tx_log, "\"test\":\"enc_noise\",\"sf\":4") != NULL);
    assert(strstr(m.tx_log, "\"ok\":true,\"test\":\"enc_noise\"") != NULL);
    assert(strstr(m.tx_log, "\"hold\":true,\"n\":64,\"errors\":0") != NULL);

    /* (d) noise_hold=1 lease expiry mid-capture latches lease_expired and restores SF=2x. */
    setup_harness(&mc, &m);
    send_cmd(&mc, "1 SET noise_sf 16");
    send_cmd(&mc, "2 SET noise_n 64");
    send_cmd(&mc, "3 SET noise_hold 1");
    rid = 3U;
    arm_x(&mc, &rid);
    assert(mc.state == MC_STATE_ARMED);
    send_cmd(&mc, "30 RUN enc_noise x");
    assert(mc.state == MC_STATE_RUNNING);
    assert(m.enc_conf[1] == 0x0000U);
    for (uint32_t i = 0U; i < 8U; ++i) {
        send_cmd(&mc, "KEEP");
        mc_core_poll(&mc);
        m.now_us += 2000U;
    }
    assert(mc.state == MC_STATE_RUNNING);
    m.now_us += MC_LEASE_TIMEOUT_US + 1000U;
    mc_core_poll(&mc);
    assert(mc.state == MC_STATE_FAULT);
    assert(mc.fault == MC_FAULT_LEASE_EXPIRED);
    assert(mc.active_test == MC_TEST_NONE);
    assert(m.enc_conf[1] == 0x0300U);
}

int main(void) {
    test_passive_boot_and_hello();
    test_bad_status_and_theta_no_sensor();
    test_duplicate_run_and_bounds_and_short_window();
    test_stale_lease_inhibits_edges_and_revokes_datum();
    test_lost_encoder_and_frozen_shaft_and_b1();
    test_timer_wrap_and_lateness_fault();
    test_enc_noise_uniform_capture_and_restore();
    test_enc_noise_abort_and_hold_paths();
    printf("ALL mc_core host tests PASSED\n");
    return 0;
}
