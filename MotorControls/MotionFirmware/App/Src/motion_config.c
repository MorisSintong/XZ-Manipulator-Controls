#include "motion_config.h"
#include <limits.h>
#include <stddef.h>

const motion_config_t motion_default_config = {
    .axis = {
        {40000U, 2000U, 4000U, 2900, 23200, 30000000U, 400U, 160U, 1, 1, 1U, 10U},
        {8000U, 2000U, 4000U, 1400, 56000, 40000000U, 800U, 400U, -1, 1, 0U, 10U}
    },
    .safe_z_01mm = 0, .pick_depth_01mm = 0,
    .settle_us = 10000U, .dwell_us = 100000U,
    .pulse_high_us = 5U, .pulse_low_us = 5U,
    .dir_setup_us = 20U, .dir_hold_us = 20U, .lateness_us = 20U,
    .bounds_confirmed = false, .clamp_x = false, .mscnt_qualified = false, .id = 1U
};

bool motion_config_valid(const motion_config_t *c)
{
    if (c == NULL || c->id == 0U || c->safe_z_01mm < 0 ||
        c->pick_depth_01mm < c->safe_z_01mm ||
        c->pick_depth_01mm > c->axis[AXIS_Z].usable_max_01mm ||
        c->pulse_high_us < 5U || c->pulse_low_us < 5U ||
        c->dir_setup_us < 20U || c->dir_hold_us < 20U ||
        c->settle_us < 10000U || c->lateness_us > 20U) {
        return false;
    }
    for (unsigned i = 0U; i < 2U; ++i) {
        const motion_axis_config_t *a = &c->axis[i];
        if (a->lead_um == 0U || a->lead_um > 1000000U || a->max_rate < 500U || a->max_rate > 2000U ||
            a->acceleration == 0U || a->usable_max_01mm <= 0 ||
            a->home_budget_steps <= 0 || a->home_budget_steps > 100000 ||
            a->home_timeout_us == 0U || a->home_timeout_us >= 0x80000000U ||
            a->backoff_steps == 0U || a->backoff_steps > 10000U ||
            a->clearance_steps == 0U || a->clearance_steps > 10000U ||
            (a->dir_sign != 1 && a->dir_sign != -1) ||
            (a->encoder_sign != 1 && a->encoder_sign != -1)) {
            return false;
        }
    }
    return true;
}

static int64_t rounded(int64_t n, int64_t d)
{
    return n < 0 ? -((-n + d / 2) / d) : (n + d / 2) / d;
}

bool motion_target_steps(const motion_axis_config_t *a, int32_t target,
                         int32_t *steps, int32_t *residual)
{
    if (a == NULL || steps == NULL || residual == NULL ||
        a->lead_um == 0U || a->lead_um > 1000000U) {
        return false;
    }
    const int64_t s = rounded((int64_t)target * 100 * 3200, a->lead_um);
    if (s < INT32_MIN || s > INT32_MAX) {
        return false;
    }
    const int64_t nm = rounded(s * a->lead_um * 1000, 3200) -
                       (int64_t)target * 100000;
    if (nm < INT32_MIN || nm > INT32_MAX) {
        return false;
    }
    *steps = (int32_t)s;
    *residual = (int32_t)nm;
    return true;
}
