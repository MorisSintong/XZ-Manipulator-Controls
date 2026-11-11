#include "step_engine.h"
#include <limits.h>
#include <string.h>

static uint32_t lock(step_engine_t *e) { return e->io.lock(e->io.context); }
static void unlock(step_engine_t *e, uint32_t p) { e->io.unlock(e->io.context, p); }
static void finish(step_engine_t *e, uint8_t axis, uint32_t now)
{
    step_axis_t *s = &e->axis[axis];
    s->running = false;
    s->done_us = now;
    e->io.schedule(e->io.context, axis, 0U, false);
}

void step_engine_init(step_engine_t *e, const motion_config_t *c, step_engine_io_t io)
{
    memset(e, 0, sizeof(*e)); e->config = c; e->io = io;
}

static void fill(step_engine_t *e, uint8_t axis)
{
    step_axis_t *s = &e->axis[axis];
    while (s->planner.running && s->head - s->tail < STEP_LOOKAHEAD) {
        const float speed = s->planner.mp.v;
        const uint32_t period = (uint32_t)(1000000.0f / speed + 0.5f);
        bool more, changed;
        float next;
        if (!step_core_expire(&s->planner, &more, &next, &changed)) { break; }
        /* Commands never retarget an active profile or reverse mid-phase. */
        if (changed || period < 500U) { s->timing_fault = true; break; }
        const uint32_t p = lock(e);
        s->period[s->head % STEP_LOOKAHEAD] = period;
        ++s->head;
        unlock(e, p);
    }
    const uint32_t p = lock(e);
    s->plan_done = !s->planner.running;
    unlock(e, p);
}

bool step_engine_move(step_engine_t *e, uint8_t axis, int32_t target, uint32_t now)
{
    if (axis >= 2U) { return false; }
    step_axis_t *s = &e->axis[axis];
    const int64_t displacement = (int64_t)target - s->position;
    if (displacement < INT32_MIN || displacement > INT32_MAX) { return false; }
    if (s->running || s->aborted || s->timing_fault ||
        (s->edges != 0U && now - s->done_us < e->config->dir_hold_us)) { return false; }
    s->emitted = 0; s->edges = 0U; s->head = 0U; s->tail = 0U;
    s->stop_requested = false; s->plan_done = false; s->start_us = now; s->done_us = now;
    step_core_init(&s->planner, (float)e->config->axis[axis].max_rate,
                   (float)e->config->axis[axis].acceleration, 500.0f);
    s->planner.pos = s->position;
    float v;
    if (target == s->position) { return true; }
    if (!step_core_cmd_move(&s->planner, target,
                           (float)e->config->axis[axis].max_rate,
                           (float)e->config->axis[axis].acceleration, &v)) { return false; }
    s->direction = s->planner.step_dir;
    fill(e, axis);
    if (s->timing_fault || s->head == 0U) { return false; }
    const uint32_t p = lock(e);
    e->io.direction(e->io.context, axis,
                    (int8_t)(s->direction * e->config->axis[axis].dir_sign));
    s->start_us = e->io.now_us(e->io.context);
    s->deadline = s->start_us + e->config->dir_setup_us + s->period[0];
    s->running = true;
    e->io.schedule(e->io.context, axis, s->deadline, true);
    unlock(e, p);
    return true;
}

void step_engine_poll(step_engine_t *e)
{
    for (uint8_t axis = 0U; axis < 2U; ++axis) {
        if (e->axis[axis].running && !e->axis[axis].stop_requested &&
            !e->axis[axis].aborted) { fill(e, axis); }
    }
}

void step_engine_irq(step_engine_t *e, uint8_t axis, uint32_t now)
{
    step_axis_t *s = &e->axis[axis];
    if (!s->running) { return; }
    const uint32_t late = now - s->deadline;
    if (late > e->config->lateness_us) { s->timing_fault = true; }
    if (s->high) {
        e->io.step(e->io.context, axis, false);
        s->high = false;
        if (s->aborted || s->stop_requested || s->timing_fault ||
            (s->head == s->tail && s->plan_done)) { finish(e, axis, now); return; }
        if (s->head == s->tail) { s->timing_fault = true; finish(e, axis, now); return; }
        s->deadline = s->rise_deadline + s->period[s->tail % STEP_LOOKAHEAD];
        if ((int32_t)(s->deadline - now) < (int32_t)e->config->pulse_low_us) {
            s->timing_fault = true; finish(e, axis, now); return;
        }
    } else {
        if (s->aborted || s->stop_requested || s->timing_fault) { finish(e, axis, now); return; }
        if (s->head == s->tail) { s->timing_fault = true; finish(e, axis, now); return; }
        ++s->tail;
        e->io.step(e->io.context, axis, true);
        s->high = true;
        s->position += s->direction;
        s->emitted += s->direction;
        ++s->edges;
        s->rise_deadline = s->deadline;
        /* High time is measured from the actual edge, never catch up after latency. */
        s->deadline = now + e->config->pulse_high_us;
    }
    e->io.schedule(e->io.context, axis, s->deadline, true);
}

void step_engine_stop(step_engine_t *e, uint8_t axis)
{
    const uint32_t p = lock(e);
    e->axis[axis].stop_requested = true;
    unlock(e, p);
}

void step_engine_abort(step_engine_t *e)
{
    const uint32_t p = lock(e);
    for (unsigned i = 0U; i < 2U; ++i) { e->axis[i].aborted = true; }
    unlock(e, p);
}

bool step_engine_clear(step_engine_t *e)
{
    const uint32_t p = lock(e);
    const bool idle = !e->axis[0].running && !e->axis[1].running;
    if (idle) {
        for (unsigned i = 0U; i < 2U; ++i) {
            e->axis[i].aborted = false; e->axis[i].timing_fault = false;
        }
    }
    unlock(e, p);
    return idle;
}

void step_engine_zero(step_engine_t *e, uint8_t axis)
{
    const uint32_t p = lock(e);
    if (!e->axis[axis].running) { e->axis[axis].position = 0; }
    unlock(e, p);
}

step_snapshot_t step_engine_snapshot(step_engine_t *e, uint8_t axis)
{
    const uint32_t p = lock(e);
    const step_axis_t *s = &e->axis[axis];
    const step_snapshot_t v = {s->position, s->emitted, s->edges,
        s->start_us, s->done_us, s->running, s->high, s->aborted, s->timing_fault};
    unlock(e, p);
    return v;
}
