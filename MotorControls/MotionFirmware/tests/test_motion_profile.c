/* Unit tests of the trapezoidal planner and the step sequencer. */
#include "test_common.h"

#include "motion_profile.h"

#include <stdint.h>
#include <stdlib.h>

typedef struct {
    uint32_t steps;
    double   time_s;
    float    vpeak;
    double   worst_dv2;   /* largest |v'^2 - v^2| between consecutive intervals */
    int32_t  pos_min;
    int32_t  pos_max;
    uint32_t reversals;
} run_stats_t;

/* Drive the sequencer like the timer ISR would, optionally calling 'hook'
 * after every step (to issue commands mid-move). */
static run_stats_t run_core(step_core_t *c, float first_v, uint32_t max_steps,
                            void (*hook)(step_core_t *, uint32_t))
{
    run_stats_t s = { 0U, 0.0, 0.0f, 0.0, c->pos, c->pos, 0U };
    float v = first_v;
    uint64_t prev_v2 = c->mp.v2;
    int8_t prev_dir = c->step_dir;

    while (c->running && (s.steps < max_steps)) {
        bool more = false;
        bool dirchg = false;
        float nv = 0.0f;

        s.time_s += 1.0 / (double)v;
        if (step_core_expire(c, &more, &nv, &dirchg)) {
            s.steps++;
            if (c->pos < s.pos_min) {
                s.pos_min = c->pos;
            }
            if (c->pos > s.pos_max) {
                s.pos_max = c->pos;
            }
        }
        if (!more) {
            break;
        }
        if (c->step_dir != prev_dir) {
            s.reversals++;
            prev_dir = c->step_dir;
        } else {
            /* Exact planner acceleration: change of the integer v^2. */
            const double dv2 = fabs((double)c->mp.v2 - (double)prev_v2);
            if (dv2 > s.worst_dv2) {
                s.worst_dv2 = dv2;
            }
        }
        if (nv > s.vpeak) {
            s.vpeak = nv;
        }
        prev_v2 = c->mp.v2;
        v = nv;
        if (hook != NULL) {
            hook(c, s.steps);
        }
    }
    return s;
}

static void test_long_move(void)
{
    const float vmax = 8000.0f;
    const float acc = 80000.0f;
    const float vmin = 200.0f;
    step_core_t c;
    float v;
    run_stats_t s;
    double t_acc;
    double s_acc;
    double t_theory;

    step_core_init(&c, vmax, acc, vmin);
    CHECK(step_core_cmd_move(&c, 10000, vmax, acc, &v));
    CHECK_NEAR(v, vmin, 0.01);
    s = run_core(&c, v, 20000U, NULL);
    CHECK(!c.running);
    CHECK(c.pos == 10000);
    CHECK(s.steps == 10000U);
    CHECK(s.reversals == 0U);
    CHECK(s.pos_max == 10000);
    CHECK_NEAR(s.vpeak, vmax, 0.5);
    CHECK_MSG(s.worst_dv2 <= (double)c.mp.two_a, "worst dv2 %g", (double)s.worst_dv2);
    t_acc = (double)(vmax - vmin) / (double)acc;
    s_acc = ((double)vmax * vmax - (double)vmin * vmin) / (2.0 * (double)acc);
    t_theory = 2.0 * t_acc + (10000.0 - 2.0 * s_acc) / (double)vmax;
    CHECK_MSG(fabs(s.time_s - t_theory) / t_theory < 0.02, "t=%g theory=%g", s.time_s, t_theory);
}

static void test_short_triangle_move(void)
{
    step_core_t c;
    float v;
    run_stats_t s;

    step_core_init(&c, 20000.0f, 50000.0f, 300.0f);
    c.pos = 1000;
    CHECK(step_core_cmd_move(&c, 700, 20000.0f, 50000.0f, &v));
    s = run_core(&c, v, 2000U, NULL);
    CHECK(c.pos == 700);
    CHECK(s.steps == 300U);
    CHECK(s.pos_min == 700);
    CHECK(s.vpeak < 20000.0f);
    CHECK(s.vpeak > 2000.0f);
    CHECK(s.worst_dv2 <= (double)c.mp.two_a);
}

static void test_single_steps_and_noop(void)
{
    step_core_t c;
    float v;
    run_stats_t s;

    step_core_init(&c, 5000.0f, 10000.0f, 100.0f);
    CHECK(!step_core_cmd_move(&c, 0, 5000.0f, 10000.0f, &v)); /* already there */
    CHECK(!c.running);
    CHECK(step_core_cmd_move(&c, 1, 5000.0f, 10000.0f, &v));
    s = run_core(&c, v, 10U, NULL);
    CHECK(c.pos == 1 && s.steps == 1U);
    CHECK(step_core_cmd_move(&c, -1, 5000.0f, 10000.0f, &v));
    s = run_core(&c, v, 10U, NULL);
    CHECK(c.pos == -1 && s.steps == 2U);
}

static int32_t g_retarget;
static uint32_t g_retarget_at;

static void retarget_hook(step_core_t *c, uint32_t steps)
{
    if (steps == g_retarget_at) {
        float v;
        CHECK(!step_core_cmd_move(c, g_retarget, c->mp.vmax, c->mp.accel, &v));
    }
}

static void test_retargeting(void)
{
    /* Retarget at step 3000 while cruising at 8000 steps/s: the brake
     * distance is (8000^2 - 200^2) / (2 * 60000) = 533 steps, so targets less
     * than 533 steps ahead (or behind) need an overshoot and one reversal. */
    const int32_t targets[] = { 20000, 3600, 3500, 1000, -2000, 3001 };
    const uint32_t reversals[] = { 0U, 0U, 1U, 1U, 1U, 1U };
    unsigned i;

    for (i = 0U; i < sizeof(targets) / sizeof(targets[0]); i++) {
        step_core_t c;
        float v;
        run_stats_t s;

        step_core_init(&c, 8000.0f, 60000.0f, 200.0f);
        CHECK(step_core_cmd_move(&c, 10000, 8000.0f, 60000.0f, &v));
        g_retarget = targets[i];
        g_retarget_at = 3000U;
        s = run_core(&c, v, 100000U, retarget_hook);
        CHECK_MSG(c.pos == targets[i], "target %d reached %d", (int)targets[i], (int)c.pos);
        CHECK(!c.running);
        CHECK_MSG(s.worst_dv2 <= (double)c.mp.two_a, "target %d worst dv2 %g",
                  (int)targets[i], (double)s.worst_dv2);
        CHECK_MSG(s.reversals == reversals[i], "target %d reversals %u", (int)targets[i],
                  (unsigned)s.reversals);
    }
}

static uint32_t g_stop_at;

static void stop_hook(step_core_t *c, uint32_t steps)
{
    if (steps == g_stop_at) {
        step_core_cmd_stop(c);
    }
}

static void test_velocity_mode_stop(void)
{
    step_core_t c;
    float v;
    run_stats_t s;
    float dist;

    step_core_init(&c, 4800.0f, 40000.0f, 200.0f);
    CHECK(step_core_cmd_run(&c, -1, 4800.0f, 40000.0f, &v));
    g_stop_at = 5000U;
    dist = mp_stop_distance(&c.mp, 4800.0f);
    s = run_core(&c, v, 100000U, stop_hook);
    CHECK(!c.running);
    CHECK(c.mp.mode == MP_MODE_IDLE);
    CHECK_NEAR((float)s.steps - 5000.0f, dist, 3.0);
    CHECK(c.pos == -(int32_t)s.steps);
    CHECK_NEAR(s.vpeak, 4800.0f, 0.5);
}

static void reverse_hook(step_core_t *c, uint32_t steps)
{
    if (steps == 3000U) {
        float v;
        CHECK(!step_core_cmd_run(c, -1, c->mp.vmax, c->mp.accel, &v));
    }
    if (steps == 9000U) {
        step_core_cmd_stop(c);
    }
}

static void test_velocity_reversal(void)
{
    step_core_t c;
    float v;
    run_stats_t s;

    step_core_init(&c, 4000.0f, 40000.0f, 200.0f);
    CHECK(step_core_cmd_run(&c, 1, 4000.0f, 40000.0f, &v));
    s = run_core(&c, v, 100000U, reverse_hook);
    CHECK(s.reversals == 1U);
    CHECK(c.pos < s.pos_max);
    CHECK(s.worst_dv2 <= (double)c.mp.two_a);
    CHECK(!c.running);
}

static void test_limits_and_halt(void)
{
    step_core_t c;
    float v;
    run_stats_t s;

    step_core_init(&c, 4000.0f, 40000.0f, 200.0f);
    c.lim_hi = 500;
    CHECK(step_core_cmd_run(&c, 1, 4000.0f, 40000.0f, &v));
    s = run_core(&c, v, 100000U, NULL);
    CHECK(c.pos == 500);
    CHECK(s.steps == 500U);
    CHECK(c.limit_hit);
    CHECK(!c.running);

    /* A step towards the limit from exactly the limit is refused at once. */
    c.limit_hit = false;
    CHECK(step_core_cmd_run(&c, 1, 4000.0f, 40000.0f, &v));
    s = run_core(&c, v, 10U, NULL);
    CHECK(s.steps == 0U && c.limit_hit && c.pos == 500);

    /* Halt stops immediately and keeps the position. */
    CHECK(step_core_cmd_move(&c, -5000, 4000.0f, 40000.0f, &v));
    (void)run_core(&c, v, 100U, NULL);
    step_core_cmd_halt(&c);
    CHECK(!c.running);
    CHECK(c.pos == 400);
    CHECK(!step_core_expire(&c, &(bool){ false }, &v, &(bool){ false }));
}

static void test_randomised_moves(void)
{
    unsigned i;

    srand(1234U);
    for (i = 0U; i < 400U; i++) {
        step_core_t c;
        float v;
        run_stats_t s;
        const float vmin = 50.0f + (float)(rand() % 400);
        const float vmax = vmin + (float)(rand() % 30000);
        const float acc = 500.0f + (float)(rand() % 300000);
        const int32_t start = (rand() % 20001) - 10000;
        const int32_t target = (rand() % 200001) - 100000;

        step_core_init(&c, vmax, acc, vmin);
        c.pos = start;
        if (!step_core_cmd_move(&c, target, vmax, acc, &v)) {
            CHECK(start == target);
            continue;
        }
        s = run_core(&c, v, 1000000U, NULL);
        CHECK_MSG(c.pos == target, "case %u: %d -> %d ended at %d", i, (int)start,
                  (int)target, (int)c.pos);
        CHECK(!c.running);
        CHECK(s.reversals == 0U);
        CHECK(s.vpeak <= vmax * 1.0001f);
        CHECK_MSG(s.worst_dv2 <= (double)c.mp.two_a, "case %u dv2 %g acc %g", i,
                  (double)s.worst_dv2, (double)acc);
        CHECK(s.steps == (uint32_t)abs(target - start));
    }
}

int main(void)
{
    RUN_TEST(test_long_move);
    RUN_TEST(test_short_triangle_move);
    RUN_TEST(test_single_steps_and_noop);
    RUN_TEST(test_retargeting);
    RUN_TEST(test_velocity_mode_stop);
    RUN_TEST(test_velocity_reversal);
    RUN_TEST(test_limits_and_halt);
    RUN_TEST(test_randomised_moves);
    return TEST_SUMMARY();
}
