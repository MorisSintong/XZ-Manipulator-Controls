/**
 * @file    motion_profile.c
 * @brief   Per-step trapezoidal velocity planner and step sequencer.
 *
 * Every decision (accelerate / cruise / brake / arrive) is taken on the
 * squared speed held as a 64-bit integer: v^2 changes by exactly 2a per step,
 * so the braking point is exact even for ramps of tens of thousands of steps.
 * Floats are only used to hand the interval speed sqrt(v^2) to the timer.
 */
#include "motion_profile.h"

#include <math.h>

#define MP_VMAX_LIMIT  1.0e6f
#define MP_ACCEL_LIMIT 1.0e9f

static void set_v2(mp_t *mp, uint64_t v2)
{
    mp->v2 = v2;
    mp->v = sqrtf((float)v2);
}

void mp_set_limits(mp_t *mp, float vmax, float accel, float vmin)
{
    if (!(vmin >= 1.0f)) { /* also rejects NaN */
        vmin = 1.0f;
    }
    if (vmin > MP_VMAX_LIMIT) {
        vmin = MP_VMAX_LIMIT;
    }
    if (!(vmax >= vmin)) {
        vmax = vmin;
    }
    if (vmax > MP_VMAX_LIMIT) {
        vmax = MP_VMAX_LIMIT;
    }
    if (!(accel >= 1.0f)) {
        accel = 1.0f;
    }
    if (accel > MP_ACCEL_LIMIT) {
        accel = MP_ACCEL_LIMIT;
    }
    mp->vmax = vmax;
    mp->vmin = vmin;
    mp->accel = accel;
    mp->vmin2 = (uint64_t)(vmin * vmin + 0.5f);
    mp->vmax2 = (uint64_t)(vmax * vmax + 0.5f);
    if (mp->vmax2 < mp->vmin2) {
        mp->vmax2 = mp->vmin2;
    }
    mp->two_a = (uint32_t)(2.0f * accel + 0.5f);
    if (mp->two_a == 0U) {
        mp->two_a = 1U;
    }
}

float mp_stop_distance(const mp_t *mp, float v)
{
    const float d = (v * v - mp->vmin * mp->vmin) / (2.0f * mp->accel);
    return (d > 0.0f) ? d : 0.0f;
}

bool mp_at_cruise(const mp_t *mp)
{
    return (mp->mode != MP_MODE_IDLE) && !mp->stop && (mp->v2 >= mp->vmax2);
}

/* Can a ramp starting at v2 brake to vmin within 'left' steps? */
static bool fits(const mp_t *mp, uint64_t v2, int64_t left)
{
    if (v2 <= mp->vmin2) {
        return true;
    }
    if (left < 0) {
        return false;
    }
    return (v2 - mp->vmin2) <= (uint64_t)left * (uint64_t)mp->two_a;
}

/* A position move may finish at its target when it is within 1.5 braking
 * steps of vmin (one braking step is the resolution of the planner). */
static bool can_stop_now(const mp_t *mp, uint64_t v2)
{
    return v2 <= mp->vmin2 + (uint64_t)mp->two_a + (uint64_t)(mp->two_a / 2U);
}

static uint64_t speed_up(const mp_t *mp, uint64_t v2)
{
    const uint64_t n = v2 + mp->two_a;
    return (n > mp->vmax2) ? mp->vmax2 : n;
}

static uint64_t slow_down(const mp_t *mp, uint64_t v2, uint64_t floor2)
{
    if (v2 <= floor2) {
        return v2;
    }
    return (v2 > floor2 + mp->two_a) ? (v2 - mp->two_a) : floor2;
}

bool mp_begin(mp_t *mp, int32_t pos)
{
    int8_t dir;

    set_v2(mp, mp->vmin2);
    if (mp->stop) {
        mp->stop = false;
        mp->mode = MP_MODE_IDLE;
        return false;
    }
    if (mp->mode == MP_MODE_POSITION) {
        if (mp->target == pos) {
            mp->mode = MP_MODE_IDLE;
            return false;
        }
        dir = (mp->target > pos) ? 1 : -1;
    } else if (mp->mode == MP_MODE_VELOCITY) {
        dir = (mp->vel_dir < 0) ? -1 : 1;
        mp->vel_dir = dir;
    } else {
        return false;
    }
    mp->dir = dir;
    return true;
}

bool mp_next(mp_t *mp, int32_t pos)
{
    const uint64_t v2 = mp->v2;

    if (mp->mode == MP_MODE_IDLE) {
        return false;
    }

    if (mp->stop) {
        if (v2 <= mp->vmin2) {
            mp->stop = false;
            mp->mode = MP_MODE_IDLE;
            return false;
        }
        set_v2(mp, slow_down(mp, v2, mp->vmin2));
        return true;
    }

    if (mp->mode == MP_MODE_VELOCITY) {
        if (mp->vel_dir != mp->dir) {
            if (v2 <= mp->vmin2) {
                mp->dir = mp->vel_dir;
                set_v2(mp, mp->vmin2);
            } else {
                set_v2(mp, slow_down(mp, v2, mp->vmin2));
            }
        } else if (v2 < mp->vmax2) {
            set_v2(mp, speed_up(mp, v2));
        } else if (v2 > mp->vmax2) {
            set_v2(mp, slow_down(mp, v2, mp->vmax2));
        }
        return true;
    }

    /* MP_MODE_POSITION: 'rem' is the distance still to go in the current
     * direction; <= 0 means the target is here or behind us. */
    {
        const int64_t rem = ((int64_t)mp->target - (int64_t)pos) * (int64_t)mp->dir;
        const int64_t left = rem - 1; /* steps left after the next one */

        if (rem <= 0) {
            if ((rem == 0) && can_stop_now(mp, v2)) {
                mp->mode = MP_MODE_IDLE;
                return false;
            }
            if ((rem < 0) && (v2 <= mp->vmin2)) {
                mp->dir = (int8_t)-mp->dir;
                set_v2(mp, mp->vmin2);
                return true;
            }
            set_v2(mp, slow_down(mp, v2, mp->vmin2)); /* overshoot: brake, come back */
            return true;
        }
        if (v2 < mp->vmax2) {
            const uint64_t up = speed_up(mp, v2);
            if (fits(mp, up, left)) {
                set_v2(mp, up);
                return true;
            }
        }
        if (fits(mp, v2, left)) {
            if (v2 > mp->vmax2) {
                set_v2(mp, slow_down(mp, v2, mp->vmax2));
            }
            return true; /* cruise */
        }
        set_v2(mp, slow_down(mp, v2, mp->vmin2));
        return true;
    }
}

void step_core_init(step_core_t *c, float vmax, float accel, float vmin)
{
    c->mp.mode = MP_MODE_IDLE;
    c->mp.target = 0;
    c->mp.vel_dir = 1;
    c->mp.stop = false;
    c->mp.dir = 1;
    mp_set_limits(&c->mp, vmax, accel, vmin);
    set_v2(&c->mp, c->mp.vmin2);
    c->vmin = c->mp.vmin;
    c->pos = 0;
    c->lim_lo = INT32_MIN;
    c->lim_hi = INT32_MAX;
    c->step_dir = 1;
    c->running = false;
    c->limit_hit = false;
}

bool step_core_start(step_core_t *c, float *v)
{
    if (c->running) {
        return false;
    }
    if (!mp_begin(&c->mp, c->pos)) {
        return false;
    }
    c->step_dir = c->mp.dir;
    *v = c->mp.v;
    c->running = true;
    return true;
}

bool step_core_expire(step_core_t *c, bool *more, float *v, bool *dir_changed)
{
    int64_t next;

    *more = false;
    *dir_changed = false;
    if (!c->running) {
        return false;
    }
    next = (int64_t)c->pos + (int64_t)c->step_dir;
    /* Directional check: a step further out of [lim_lo, lim_hi] is refused,
     * a step back towards the range is always allowed. */
    if (((c->step_dir > 0) && (next > (int64_t)c->lim_hi)) ||
        ((c->step_dir < 0) && (next < (int64_t)c->lim_lo))) {
        c->running = false;
        c->limit_hit = true;
        c->mp.mode = MP_MODE_IDLE;
        c->mp.stop = false;
        return false;
    }
    c->pos = (int32_t)next;
    if (mp_next(&c->mp, c->pos)) {
        *dir_changed = (c->mp.dir != c->step_dir);
        c->step_dir = c->mp.dir;
        *v = c->mp.v;
        *more = true;
    } else {
        c->running = false;
    }
    return true;
}

bool step_core_cmd_move(step_core_t *c, int32_t target, float vmax, float accel, float *v)
{
    mp_set_limits(&c->mp, vmax, accel, c->vmin);
    c->mp.target = target;
    c->mp.mode = MP_MODE_POSITION;
    c->mp.stop = false;
    if (c->running) {
        return false; /* re-planned on the fly by the next mp_next() */
    }
    return step_core_start(c, v);
}

bool step_core_cmd_run(step_core_t *c, int8_t dir, float vmax, float accel, float *v)
{
    mp_set_limits(&c->mp, vmax, accel, c->vmin);
    c->mp.vel_dir = (dir < 0) ? -1 : 1;
    c->mp.mode = MP_MODE_VELOCITY;
    c->mp.stop = false;
    if (c->running) {
        return false;
    }
    return step_core_start(c, v);
}

void step_core_cmd_stop(step_core_t *c)
{
    if (c->running) {
        c->mp.stop = true;
    } else {
        c->mp.mode = MP_MODE_IDLE;
        c->mp.stop = false;
    }
}

void step_core_cmd_halt(step_core_t *c)
{
    c->running = false;
    c->mp.mode = MP_MODE_IDLE;
    c->mp.stop = false;
    set_v2(&c->mp, c->mp.vmin2);
}
