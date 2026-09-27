/* Closed-loop simulation tests of axis_ctrl.c (homing, cycling, faults)
 * against sim_axis.c, using the real firmware defaults from app_config.c. */
#include "test_common.h"

#include "app_config.h"
#include "axis_ctrl.h"
#include "axis_hw.h"
#include "sim_axis.h"

#include <stdlib.h>
#include <string.h>

#define Z APP_AXIS_Z
#define X APP_AXIS_X

static const float Z_TRAVEL_MM = 150.0f;
static const float X_TRAVEL_MM = 300.0f;

static uint8_t g_pred_ax;

static bool pred_ready_or_fault(void)
{
    const axis_state_t s = axis_ctrl_status(g_pred_ax)->state;
    return (s == AX_ST_READY) || (s == AX_ST_FAULT);
}

static bool pred_idle(void)
{
    return !axis_ctrl_is_busy(g_pred_ax);
}

static float spm(uint8_t ax)
{
    return g_app_axis_cfg[ax].steps_per_mm;
}

static sim_axis_cfg_t sim_cfg(uint8_t ax, float travel_mm, float start_mm)
{
    sim_axis_cfg_t c;

    memset(&c, 0, sizeof(c));
    c.stop_lo = 0;
    c.stop_hi = (int32_t)(travel_mm * spm(ax));
    c.start = (int32_t)(start_mm * spm(ax));
    c.sg_free = 260.0f;
    c.sg_noise = 30.0f;
    c.sg_stall = 12.0f;
    c.enc_present = true;
    c.enc_ratio = (ax == Z) ? -1.28f : 1.28f; /* both encoder orientations */
    c.enc_offset = (ax == Z) ? 1234 : -777;
    c.enc_source = ax;
    return c;
}

static void setup_axis(uint8_t ax, const sim_axis_cfg_t *sc, const axis_cfg_t *cfg)
{
    sim_setup(ax, sc);
    axis_ctrl_init(ax, (cfg != NULL) ? cfg : &g_app_axis_cfg[ax]);
    axis_ctrl_set_driver_ready(ax, true);
}

static bool home_axis(uint8_t ax, uint32_t timeout_ms)
{
    if (!axis_ctrl_home(ax)) {
        return false;
    }
    g_pred_ax = ax;
    (void)sim_run_until(pred_ready_or_fault, timeout_ms);
    return axis_ctrl_status(ax)->state == AX_ST_READY;
}

static float travel_mm(uint8_t ax)
{
    return (float)axis_ctrl_status(ax)->travel_steps / spm(ax);
}

/* ------------------------------------------------------------------------ */

static void test_home_z_with_encoder(void)
{
    sim_axis_cfg_t sc = sim_cfg(Z, Z_TRAVEL_MM, 40.0f);
    const axis_status_t *st;

    sim_reset(1U);
    setup_axis(Z, &sc, NULL);
    CHECK(home_axis(Z, 120000U));
    st = axis_ctrl_status(Z);
    CHECK(st->homed);
    CHECK(st->enc_ok);
    CHECK_NEAR(fabs((double)st->enc_ratio), 1.28, 0.02);
    CHECK_NEAR(travel_mm(Z), Z_TRAVEL_MM, 0.05);
    /* Encoder re-sync: commanded position equals the true rotor position. */
    CHECK_NEAR((float)st->pos_steps, sim_rotor(Z), 3.0);
    CHECK_NEAR((float)st->soft_min / spm(Z), g_app_axis_cfg[Z].cycle_margin_mm, 0.01);
    CHECK(sim_log_count("[HOME] Z: stall at MAX end") == 1U);
    CHECK(sim_log_count("[HOME] Z: stall at MIN end") == 1U);
}

static void test_soft_cycle_z(void)
{
    sim_axis_cfg_t sc = sim_cfg(Z, Z_TRAVEL_MM, 70.0f);
    const axis_status_t *st;
    const float margin = g_app_axis_cfg[Z].cycle_margin_mm;
    float lo;
    float hi;

    sim_reset(2U);
    setup_axis(Z, &sc, NULL);
    CHECK(home_axis(Z, 120000U));
    CHECK(axis_ctrl_cycle(Z, AX_CYCLE_SOFT));
    sim_run_ms(12000U); /* first leg: from the post-homing position to MAX */
    sim_clear_blocked(Z);
    sim_track_range(Z, NULL, NULL, true);
    sim_run_ms(60000U);
    st = axis_ctrl_status(Z);
    CHECK_MSG(st->state == AX_ST_CYCLE_MOVE || st->state == AX_ST_CYCLE_DWELL,
              "state %s", axis_ctrl_state_name(st->state));
    CHECK_MSG(st->cycles >= 3U, "cycles %u", (unsigned)st->cycles);
    CHECK(sim_blocked_ms(Z) == 0U); /* never touched the hard stops */
    sim_track_range(Z, &lo, &hi, false);
    CHECK_NEAR(lo / spm(Z), margin, 0.05);
    CHECK_NEAR(hi / spm(Z), Z_TRAVEL_MM - margin, 0.05);
    CHECK_MSG(abs(st->follow_err_peak) < 24, "peak following error %d", (int)st->follow_err_peak);
}

static void test_home_and_cycle_both_axes(void)
{
    sim_axis_cfg_t sz = sim_cfg(Z, Z_TRAVEL_MM, 20.0f);
    sim_axis_cfg_t sx = sim_cfg(X, X_TRAVEL_MM, 250.0f);

    sim_reset(3U);
    setup_axis(Z, &sz, NULL);
    setup_axis(X, &sx, NULL);
    CHECK(home_axis(Z, 120000U));
    CHECK(home_axis(X, 60000U));
    CHECK_NEAR(travel_mm(X), X_TRAVEL_MM, 0.1);
    CHECK(axis_ctrl_cycle(Z, AX_CYCLE_SOFT));
    CHECK(axis_ctrl_cycle(X, AX_CYCLE_SOFT));
    sim_clear_blocked(Z);
    sim_clear_blocked(X);
    sim_run_ms(40000U);
    CHECK(axis_ctrl_status(Z)->fault == AX_FAULT_NONE);
    CHECK(axis_ctrl_status(X)->fault == AX_FAULT_NONE);
    CHECK_MSG(axis_ctrl_status(X)->cycles >= 7U, "X cycles %u", (unsigned)axis_ctrl_status(X)->cycles);
    CHECK(sim_blocked_ms(Z) == 0U);
    CHECK(sim_blocked_ms(X) == 0U);
}

static void test_home_without_encoder(void)
{
    sim_axis_cfg_t sc = sim_cfg(X, X_TRAVEL_MM, 120.0f);
    axis_cfg_t cfg = g_app_axis_cfg[X];
    const axis_status_t *st;

    sc.enc_present = false;
    sim_reset(4U);
    setup_axis(X, &sc, &cfg);
    CHECK(home_axis(X, 60000U));
    st = axis_ctrl_status(X);
    CHECK(!st->enc_ok);
    CHECK_NEAR(travel_mm(X), X_TRAVEL_MM, 1.0);
    CHECK_NEAR((float)st->pos_steps / spm(X), sim_rotor(X) / spm(X), 1.0);
}

static void test_home_starting_against_stops(void)
{
    sim_axis_cfg_t sc = sim_cfg(Z, Z_TRAVEL_MM, 0.0f); /* resting on the MIN stop */

    sim_reset(5U);
    setup_axis(Z, &sc, NULL);
    CHECK(home_axis(Z, 120000U));
    CHECK_NEAR(travel_mm(Z), Z_TRAVEL_MM, 0.05);

    sc = sim_cfg(Z, Z_TRAVEL_MM, Z_TRAVEL_MM); /* resting on the MAX stop */
    sim_reset(6U);
    setup_axis(Z, &sc, NULL);
    CHECK(home_axis(Z, 120000U));
    CHECK_NEAR(travel_mm(Z), Z_TRAVEL_MM, 0.05);

    /* Back-off pushes straight into a stop: X homes MIN first, so its
     * back-off goes towards MAX. */
    sc = sim_cfg(X, X_TRAVEL_MM, X_TRAVEL_MM);
    sim_reset(7U);
    setup_axis(X, &sc, NULL);
    CHECK(home_axis(X, 60000U));
    CHECK(sim_log_count("stall during back-off") == 1U);
    CHECK_NEAR(travel_mm(X), X_TRAVEL_MM, 0.1);
}

static void test_encoder_backup_when_sg_fails(void)
{
    sim_axis_cfg_t sc = sim_cfg(X, X_TRAVEL_MM, 150.0f);

    sc.sg_broken = true;
    sim_reset(8U);
    setup_axis(X, &sc, NULL);
    CHECK(home_axis(X, 60000U));
    CHECK(sim_log_count("(encoder") >= 2U);
    CHECK_NEAR(travel_mm(X), X_TRAVEL_MM, 0.5);
}

static void test_no_stall_faults(void)
{
    sim_axis_cfg_t sc = sim_cfg(X, X_TRAVEL_MM, 150.0f);

    sc.sg_broken = true;
    sc.enc_present = false;
    sim_reset(9U);
    setup_axis(X, &sc, NULL);
    CHECK(!home_axis(X, 60000U));
    CHECK(axis_ctrl_status(X)->state == AX_ST_FAULT);
    CHECK(axis_ctrl_status(X)->fault == AX_FAULT_NO_STALL);
    CHECK(axis_ctrl_clear_fault(X));
    CHECK(axis_ctrl_status(X)->state == AX_ST_IDLE);
}

static void test_bounce_mode(void)
{
    sim_axis_cfg_t sc = sim_cfg(X, X_TRAVEL_MM, 150.0f);
    const axis_status_t *st;

    sim_reset(10U);
    setup_axis(X, &sc, NULL);
    CHECK(home_axis(X, 60000U));
    sim_clear_blocked(X);
    CHECK(axis_ctrl_cycle(X, AX_CYCLE_BOUNCE));
    sim_run_ms(60000U);
    st = axis_ctrl_status(X);
    CHECK(st->fault == AX_FAULT_NONE);
    CHECK_MSG(st->strokes >= 6U, "strokes %u", (unsigned)st->strokes);
    CHECK(st->cycles >= 3U);
    CHECK_MSG(abs(st->max_dev_steps) <= (int32_t)(0.5f * spm(X)),
              "max deviation %d steps", (int)st->max_dev_steps);
    CHECK(sim_blocked_ms(X) > 0U); /* really touched the stops */
    /* Leave bounce mode and do a soft cycle again. */
    axis_ctrl_stop(X);
    g_pred_ax = X;
    CHECK(sim_run_until(pred_idle, 10000U));
    CHECK(axis_ctrl_status(X)->state == AX_ST_READY);
    CHECK(axis_ctrl_cycle(X, AX_CYCLE_SOFT));
    sim_clear_blocked(X);
    sim_run_ms(10000U);
    CHECK(axis_ctrl_status(X)->fault == AX_FAULT_NONE);
    CHECK(sim_blocked_ms(X) == 0U);
}

static void test_step_loss_detected(void)
{
    sim_axis_cfg_t sc = sim_cfg(X, X_TRAVEL_MM, 150.0f);

    sim_reset(11U);
    setup_axis(X, &sc, NULL);
    CHECK(home_axis(X, 60000U));
    CHECK(axis_ctrl_cycle(X, AX_CYCLE_SOFT));
    sim_run_ms(3000U);
    CHECK(axis_ctrl_status(X)->fault == AX_FAULT_NONE);
    sim_inject_slip(X, 4 * 16); /* one electrical cycle = 4 fullsteps */
    sim_run_ms(200U);
    CHECK_MSG(axis_ctrl_status(X)->state == AX_ST_FAULT, "state %s err %d peak %d enc_ok %d",
              axis_ctrl_state_name(axis_ctrl_status(X)->state), (int)axis_ctrl_status(X)->follow_err,
              (int)axis_ctrl_status(X)->follow_err_peak, (int)axis_ctrl_status(X)->enc_ok);
    CHECK(axis_ctrl_status(X)->fault == AX_FAULT_STEP_LOSS);
    CHECK(!axis_ctrl_status(X)->homed);
}

static void test_stop_resume_and_move(void)
{
    sim_axis_cfg_t sc = sim_cfg(Z, Z_TRAVEL_MM, 60.0f);
    const axis_status_t *st;

    sim_reset(12U);
    setup_axis(Z, &sc, NULL);
    CHECK(home_axis(Z, 120000U));
    CHECK(axis_ctrl_cycle(Z, AX_CYCLE_SOFT));
    sim_run_ms(4000U);
    axis_ctrl_stop(Z);
    g_pred_ax = Z;
    CHECK(sim_run_until(pred_idle, 5000U));
    st = axis_ctrl_status(Z);
    CHECK(st->state == AX_ST_READY);
    CHECK(st->homed);
    CHECK(axis_ctrl_move_mm(Z, 75.0f));
    CHECK(sim_run_until(pred_idle, 20000U));
    CHECK_NEAR((float)st->pos_steps / spm(Z), 75.0, 0.003);
    CHECK_NEAR(sim_rotor(Z) / spm(Z), 75.0, 0.01);
    /* Targets are clamped to the soft limits. */
    CHECK(axis_ctrl_move_mm(Z, 500.0f));
    CHECK(sim_run_until(pred_idle, 30000U));
    CHECK(st->pos_steps == st->soft_max);
    CHECK(axis_ctrl_cycle(Z, AX_CYCLE_SOFT));
    sim_run_ms(5000U);
    CHECK(axis_ctrl_status(Z)->fault == AX_FAULT_NONE);
}

static void test_swapped_encoders(void)
{
    sim_axis_cfg_t sz = sim_cfg(Z, Z_TRAVEL_MM, 50.0f);
    sim_axis_cfg_t sx = sim_cfg(X, X_TRAVEL_MM, 100.0f);

    sz.enc_source = X;
    sx.enc_source = Z;
    sim_reset(13U);
    setup_axis(Z, &sz, NULL);
    setup_axis(X, &sx, NULL);
    CHECK(home_axis(Z, 120000U));
    CHECK(!axis_ctrl_status(Z)->enc_ok);
    CHECK(sim_log_count("swap the encoder buses") >= 1U);
    CHECK_NEAR(travel_mm(Z), Z_TRAVEL_MM, 1.0); /* still homes via StallGuard4 */
}

static void test_spi_failure(void)
{
    sim_axis_cfg_t sc = sim_cfg(X, X_TRAVEL_MM, 150.0f);

    sim_reset(14U);
    setup_axis(X, &sc, NULL);
    CHECK(axis_ctrl_home(X));
    sim_run_ms(700U);
    sim_set_spi_fail(X, true);
    sim_run_ms(50U);
    CHECK(axis_ctrl_status(X)->state == AX_ST_FAULT);
    CHECK(axis_ctrl_status(X)->fault == AX_FAULT_SPI);
}

static void test_short_travel_fault(void)
{
    sim_axis_cfg_t sc = sim_cfg(X, 6.0f, 3.0f); /* 6 mm axis < 10 mm minimum */

    sim_reset(15U);
    setup_axis(X, &sc, NULL);
    CHECK(!home_axis(X, 60000U));
    CHECK(axis_ctrl_status(X)->fault == AX_FAULT_SHORT_TRAVEL);
}

static void test_driver_lost_while_cycling(void)
{
    sim_axis_cfg_t sc = sim_cfg(X, X_TRAVEL_MM, 150.0f);

    sim_reset(16U);
    setup_axis(X, &sc, NULL);
    CHECK(home_axis(X, 60000U));
    CHECK(axis_ctrl_cycle(X, AX_CYCLE_SOFT));
    sim_run_ms(1000U);
    axis_ctrl_set_driver_ready(X, false);
    CHECK(axis_ctrl_status(X)->state == AX_ST_FAULT);
    CHECK(axis_ctrl_status(X)->fault == AX_FAULT_DRIVER);
    CHECK(!axis_hw_busy(X));
    axis_ctrl_set_driver_ready(X, true);
    CHECK(axis_ctrl_clear_fault(X));
    CHECK(home_axis(X, 60000U));
}

int main(void)
{
    sim_set_verbose(getenv("SIM_VERBOSE") != NULL);
    RUN_TEST(test_home_z_with_encoder);
    RUN_TEST(test_soft_cycle_z);
    RUN_TEST(test_home_and_cycle_both_axes);
    RUN_TEST(test_home_without_encoder);
    RUN_TEST(test_home_starting_against_stops);
    RUN_TEST(test_encoder_backup_when_sg_fails);
    RUN_TEST(test_no_stall_faults);
    RUN_TEST(test_bounce_mode);
    RUN_TEST(test_step_loss_detected);
    RUN_TEST(test_stop_resume_and_move);
    RUN_TEST(test_swapped_encoders);
    RUN_TEST(test_spi_failure);
    RUN_TEST(test_short_travel_fault);
    RUN_TEST(test_driver_lost_while_cycling);
    return TEST_SUMMARY();
}
