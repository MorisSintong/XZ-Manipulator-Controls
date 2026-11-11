#include "homing.h"
#include <limits.h>
#include <string.h>

void homing_init(homing_t *h, const motion_config_t *c, step_engine_t *s,
                 encoder_sampler_t *e, driver_evidence_t *d)
{
    memset(h, 0, sizeof(*h)); h->config = c; h->steps = s; h->encoders = e; h->drivers = d;
}
static void enter(homing_t *h, homing_state_t state, uint32_t now)
{
    h->state = state; h->state_us = now;
}
static void failed(homing_t *h, uint8_t code, uint32_t now)
{
    const step_snapshot_t s = step_engine_snapshot(h->steps, h->axis);
    if (h->state == HOME_SEEK || h->state == HOME_STOP) {
        h->result[h->axis].seek_emitted_steps = s.emitted;
    } else if (h->state == HOME_LATCH || h->state == HOME_LATCH_STOP) {
        h->result[h->axis].latch_emitted_steps = s.emitted;
    } else if (h->state == HOME_BACKOFF) { h->result[h->axis].backoff_steps = s.edges; }
    h->result[h->axis].axis_flags |= h->encoders->axis[h->axis].faults;
    h->result[h->axis].result = code;
    h->result[h->axis].home_duration_us = now - h->started_us;
    step_engine_stop(h->steps, h->axis);
    enter(h, HOME_DONE, now);
}
void homing_stop(homing_t *h, uint8_t result, uint32_t now)
{
    if (h->state != HOME_IDLE && h->state != HOME_DONE) { failed(h, result, now); }
}
static bool relative_move(homing_t *h, int32_t delta, uint32_t now)
{
    const step_snapshot_t s = step_engine_snapshot(h->steps, h->axis);
    const int64_t target = (int64_t)s.position + delta;
    return target >= INT32_MIN && target <= INT32_MAX &&
        step_engine_move(h->steps, h->axis, (int32_t)target, now);
}
static bool axis_begin(homing_t *h, uint8_t axis, uint32_t now)
{
    h->axis = axis; h->started_us = now; h->confirmations = 0U; h->repeat = false;
    h->result[axis].axis = axis;
    h->result[axis].sg_threshold = h->config->axis[axis].sg_threshold;
    if (!encoder_sampler_reseed(h->encoders, axis) ||
        !driver_evidence_poll(h->drivers, axis)) { failed(h, 5U, now); return false; }
    enter(h, HOME_PREP, now);
    return true;
}
bool homing_begin(homing_t *h, uint32_t now)
{
    if (h->state != HOME_IDLE && h->state != HOME_DONE) { return false; }
    if (!motion_config_valid(h->config) || !step_engine_clear(h->steps)) { return false; }
    memset(h->result, 0, sizeof(h->result));
    h->result[0].axis = AXIS_X; h->result[1].axis = AXIS_Z;
    (void)axis_begin(h, AXIS_Z, now);
    return true;
}
bool homing_success(const homing_t *h)
{
    return h->state == HOME_DONE && h->result[0].result == 1U && h->result[1].result == 1U;
}
void homing_poll(homing_t *h, uint32_t now)
{
    if (h->state == HOME_IDLE || h->state == HOME_DONE) { return; }
    const uint8_t axis = h->axis;
    const motion_axis_config_t *c = &h->config->axis[axis];
    diag_home_t *r = &h->result[axis];
    const step_snapshot_t s = step_engine_snapshot(h->steps, axis);
    if (s.aborted) { failed(h, 4U, now); return; }
    if (s.timing_fault || !encoder_sampler_fresh(&h->encoders->axis[axis], now)) {
        failed(h, 5U, now); return;
    }
    if (now - h->started_us > c->home_timeout_us) { failed(h, 2U, now); return; }
    if (now - h->sg_poll_us >= 2000U) {
        h->sg_poll_us = now;
        if (!driver_evidence_poll(h->drivers, axis)) { failed(h, 5U, now); return; }
    }
    const driver_sample_t *d = &h->drivers->axis[axis];
    switch (h->state) {
    case HOME_PREP:
        if (now - h->state_us < 250000U) { break; }
        h->seek_start = s.position;
        if (!relative_move(h, -c->home_budget_steps, now)) { failed(h, 5U, now); break; }
        r->sg_baseline = d->sg;
        enter(h, HOME_SEEK, now);
        break;
    case HOME_SEEK:
    case HOME_LATCH:
        /* Blank acceleration, then require four distinct low SG4 observations.
         * Both seeks use the same capped operating speed, not a slow latch. */
        if (now - h->state_us >= 400000U && now == h->sg_poll_us) {
            if (d->sg <= (uint16_t)(2U * c->sg_threshold)) { ++h->confirmations; }
            else { h->confirmations = 0U; }
            if (h->confirmations >= 4U) {
                r->sg_trigger = d->sg;
                step_engine_stop(h->steps, axis);
                enter(h, h->state == HOME_SEEK ? HOME_STOP : HOME_LATCH_STOP, now);
                break;
            }
        }
        if (!s.running) { failed(h, 3U, now); }
        break;
    case HOME_STOP:
    case HOME_LATCH_STOP:
        if (s.running || now - s.done_us < 10000U) { break; }
        if (h->state == HOME_STOP) {
            r->seek_emitted_steps = s.emitted;
            if (!relative_move(h, (int32_t)c->backoff_steps, now)) { failed(h, 5U, now); break; }
            enter(h, HOME_BACKOFF, now);
        } else {
            r->latch_emitted_steps = s.emitted;
            if (!relative_move(h, (int32_t)c->clearance_steps, now)) { failed(h, 5U, now); break; }
            enter(h, HOME_CLEARANCE, now);
        }
        break;
    case HOME_BACKOFF:
        if (s.running || now - s.done_us < 10000U) { break; }
        r->backoff_steps = s.edges;
        h->confirmations = 0U;
        if (!relative_move(h, -(int32_t)(c->backoff_steps * 2U), now)) {
            failed(h, 5U, now); break;
        }
        enter(h, HOME_LATCH, now);
        break;
    case HOME_CLEARANCE:
        if (!s.running) { enter(h, HOME_SETTLE, now); }
        break;
    case HOME_SETTLE:
        if (now - h->state_us < h->config->settle_us) { break; }
        encoder_sampler_poll(h->encoders, true);
        if (!encoder_sampler_fresh(&h->encoders->axis[axis],
                                   h->encoders->io.now_us(h->encoders->io.context)) ||
            !driver_evidence_poll(h->drivers, axis)) { failed(h, 5U, now); break; }
        {
            encoder_sample_t *e = &h->encoders->axis[axis];
            r->home_offset_counts = e->unwrap.total_counts;
            e->home_offset = r->home_offset_counts;
            r->raw_zero = e->raw; r->mscnt_zero = h->drivers->axis[axis].mscnt;
            r->as_status = e->status; r->agc = e->agc;
            r->home_emitted_origin_steps = s.position;
            r->home_duration_us = now - h->started_us;
            r->axis_flags = PH_HOME | PH_CONTINUOUS | PH_END_ENC | PH_END_MSCNT;
            if (c->encoder_sign < 0) { r->axis_flags |= PH_ENC_NEG; }
            if (c->dir_sign < 0) { r->axis_flags |= PH_MSCNT_NEG; }
            r->result = 1U;
            step_engine_zero(h->steps, axis);
        }
        if (axis == AXIS_Z) { (void)axis_begin(h, AXIS_X, now); }
        else { enter(h, HOME_DONE, now); }
        break;
    default: break;
    }
}
