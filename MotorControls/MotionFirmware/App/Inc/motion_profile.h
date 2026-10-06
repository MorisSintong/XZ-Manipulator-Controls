/**
 * @file    motion_profile.h
 * @brief   Per-step trapezoidal velocity planner and step sequencer.
 *
 * Pure C (no HAL access), reused from the reference branch. The diagnostic
 * step engine runs this planner in foreground to fill its ISR look-ahead queue.
 *
 * The planner works in the distance domain: after every emitted step the
 * speed of the next step interval becomes v' = sqrt(v^2 +/- 2a), which gives
 * a constant acceleration 'a'. Every call is O(1) and allocation free.
 *
 * Units: microsteps, microsteps/s, microsteps/s^2.
 */
#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MP_MODE_IDLE = 0,
    MP_MODE_POSITION, /* move to 'target', braking/reversing as required */
    MP_MODE_VELOCITY  /* run in 'vel_dir' at vmax until a stop request   */
} mp_mode_t;

typedef struct {
    /* Commands: written by the owner (inside a critical section while the
     * sequencer is running), read by the planner. */
    mp_mode_t mode;
    int32_t   target;  /* MP_MODE_POSITION */
    int8_t    vel_dir; /* MP_MODE_VELOCITY: +1 / -1 */
    bool      stop;    /* brake to vmin and finish, abandoning the target */
    float     vmax;    /* requested limits (informational) */
    float     vmin;    /* start/stop speed, always <= vmax */
    float     accel;
    /* Exact integer representation used for every decision, so that long
     * ramps cannot accumulate float rounding into an overshoot. */
    uint64_t  vmax2;   /* vmax^2 */
    uint64_t  vmin2;   /* vmin^2 */
    uint32_t  two_a;   /* 2*accel = change of v^2 per step */
    /* Planner state. */
    int8_t    dir;     /* direction of the step currently being timed */
    uint64_t  v2;      /* squared speed of the interval currently being timed */
    float     v;       /* sqrt(v2): speed of that interval in steps/s */
} mp_t;

/** Set speed/acceleration limits (sanitised: 1 <= vmin <= vmax <= 1e6,
 *  1 <= accel <= 1e9). */
void  mp_set_limits(mp_t *mp, float vmax, float accel, float vmin);
/** Plan the first interval from standstill at 'pos'. False = nothing to do. */
bool  mp_begin(mp_t *mp, int32_t pos);
/** Plan the interval after a step that brought the axis to 'pos'.
 *  False = motion complete (mode becomes MP_MODE_IDLE). */
bool  mp_next(mp_t *mp, int32_t pos);
/** Steps needed to brake from speed v down to vmin. */
float mp_stop_distance(const mp_t *mp, float v);
/** True while running at the commanded speed (cruise phase). */
bool  mp_at_cruise(const mp_t *mp);

/* ------------------------------------------------------------------------ */
/* Step sequencer shared by the timer ISR (stepgen.c) and the host simulator */
/* ------------------------------------------------------------------------ */

typedef struct {
    mp_t    mp;
    float   vmin;      /* start/stop speed used for every command */
    int32_t pos;       /* position after the last emitted step */
    int32_t lim_lo;    /* hard software travel limits (inclusive) */
    int32_t lim_hi;
    int8_t  step_dir;  /* direction of the pending step */
    bool    running;
    bool    limit_hit; /* latched when a step was refused by the limits */
} step_core_t;

/** Initialise with position 0, open limits and the given speed limits. */
void step_core_init(step_core_t *c, float vmax, float accel, float vmin);

/** Start from standstill. On true, '*v' is the speed of the first interval
 *  and c->step_dir the level to present on DIR before the first edge. */
bool step_core_start(step_core_t *c, float *v);

/** Call when the current interval expires. Returns true if a step edge must
 *  be emitted now. '*more' is true if another interval follows, in which case
 *  '*v' is its speed and '*dir_changed' tells whether DIR must be updated
 *  after this edge. A step that would move further outside [lim_lo, lim_hi]
 *  is refused, the motion stops and limit_hit latches; steps back towards
 *  the range are always allowed. */
bool step_core_expire(step_core_t *c, bool *more, float *v, bool *dir_changed);

/* Commands. While running they re-plan on the fly (retarget, reverse, brake).
 * They return true only when the sequencer was idle and has to be started,
 * with '*v' the speed of the first interval (see step_core_start). The caller
 * must serialise them against step_core_expire (critical section). */
bool step_core_cmd_move(step_core_t *c, int32_t target, float vmax, float accel, float *v);
bool step_core_cmd_run(step_core_t *c, int8_t dir, float vmax, float accel, float *v);
void step_core_cmd_stop(step_core_t *c);
void step_core_cmd_halt(step_core_t *c);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_PROFILE_H */
