#include "motion_executor.h"
#include "motion_units.h"
#include <limits.h>
#include <string.h>

static void header(motion_executor_t *e, diag_record_t *r, uint8_t type,
                   const vision_cmd_t *c, uint32_t seq, uint32_t now)
{
    memset(r, 0, sizeof(*r)); r->type = type; r->config_id = e->config->id;
    (void)now;
    r->command_seq = seq; r->home_epoch = e->home_epoch;
    r->timestamp_ms = e->encoders->io.now_ms(e->encoders->io.context);
    if (c != NULL) { r->command = *c; }
}
static uint32_t now_us(motion_executor_t *e)
{
    return e->encoders->io.now_us(e->encoders->io.context);
}
static uint32_t sensor_faults(motion_executor_t *e, uint32_t now)
{
    uint32_t flags = 0U;
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (!encoder_sampler_fresh(&e->encoders->axis[i], now)) { flags |= DIAG_ENCODER_INVALID; }
        if ((e->encoders->axis[i].faults & PH_OVERRUN) != 0U) { flags |= DIAG_SAMPLE_OVERRUN; }
        if (e->encoders->axis[i].conf_fault) { flags |= DIAG_CONFIG_INVALID; }
        if (e->drivers->axis[i].fault) { flags |= DIAG_DRIVER_FAULT; }
        const step_snapshot_t step = step_engine_snapshot(e->steps, i);
        if (step.timing_fault) { flags |= DIAG_STEP_TIMING_FAULT; }
        if (step.aborted && e->faults == 0U) { flags |= DIAG_ABORTED; }
    }
    return flags;
}

void motion_executor_init(motion_executor_t *e, const motion_config_t *c, step_engine_t *s,
                          encoder_sampler_t *a, driver_evidence_t *d, uart_transport_t *u)
{
    memset(e, 0, sizeof(*e)); e->config = c; e->steps = s; e->encoders = a;
    e->drivers = d; e->uart = u;
    homing_init(&e->home, c, s, a, d);
    if (!motion_config_valid(c)) { e->faults = DIAG_CONFIG_INVALID; e->state = EXEC_FAULT; }
}
static bool begin_home(motion_executor_t *e, const vision_cmd_t *c, uint32_t seq, uint32_t now)
{
    if (!motion_config_valid(e->config) || !uart_transport_reserve(e->uart)) { return false; }
    header(e, &e->home_record, DIAG_HOME_RESULT, c, seq, now);
    e->homed_mask = 0U; e->home_pending = true; e->state = EXEC_HOMING;
    if (e->safe_inputs != NULL && !e->safe_inputs(e->safe_context)) {
        memset(e->home.result, 0, sizeof(e->home.result));
        e->home.result[AXIS_X].axis = AXIS_X;
        e->home.result[AXIS_Z].axis = AXIS_Z;
        e->home.result[AXIS_Z].result = 4U;
        e->home.state = HOME_DONE;
        e->faults |= DIAG_ABORTED | DIAG_POSITION_UNCERTAIN;
        step_engine_abort(e->steps);
        return true;
    }
    if (!homing_begin(&e->home, now)) {
        e->home.state = HOME_DONE; e->home.result[AXIS_Z].result = 5U;
    }
    return true;
}
void motion_executor_boot(motion_executor_t *e, uint32_t now)
{
    if (e->state == EXEC_BOOT) { (void)begin_home(e, NULL, 0U, now); }
}

void motion_executor_status(motion_executor_t *e, const vision_cmd_t *c,
                            uint32_t seq, bool periodic, uint32_t now)
{
    diag_record_t r;
    header(e, &r, DIAG_STATUS, c, seq, now);
    r.status_bits = e->faults | (e->homed_mask != 3U ? DIAG_NOT_HOMED : 0U);
    const bool rx_event = e->uart->rx_overflows != e->status_rx_overflows ||
        e->uart->parser.crc_errors != e->status_rx_crc_errors ||
        e->uart->parser.format_errors != e->status_rx_format_errors;
    if (rx_event) { r.status_bits |= DIAG_RX_OVERFLOW; }
    if (uart_transport_backpressure(e->uart, now)) { r.status_bits |= DIAG_TX_BACKPRESSURE; }
    diag_status_t *s = &r.data.status;
    s->state = (uint8_t)e->state; s->command_queue_depth = (uint8_t)(e->qhead - e->qtail);
    s->tx_queue_depth = uart_transport_depth(e->uart); s->homed_mask = e->homed_mask;
    s->rx_crc_errors = e->uart->parser.crc_errors; s->rx_format_errors = e->uart->parser.format_errors;
    s->rx_overflows = e->uart->rx_overflows; s->rejected_commands = e->rejected;
    s->tx_record_drops = e->uart->tx_drops;
    s->x_drv_status = e->drivers->axis[AXIS_X].status; s->z_drv_status = e->drivers->axis[AXIS_Z].status;
    s->x_position_steps = step_engine_snapshot(e->steps, AXIS_X).position;
    s->z_position_steps = step_engine_snapshot(e->steps, AXIS_Z).position;
    const encoder_sample_t *x = &e->encoders->axis[AXIS_X], *z = &e->encoders->axis[AXIS_Z];
    s->x_unwrap_counts = x->unwrap.total_counts; s->z_unwrap_counts = z->unwrap.total_counts;
    s->x_max_sample_gap_us = x->status_gap_us; s->z_max_sample_gap_us = z->status_gap_us;
    s->x_as_status = x->status; s->z_as_status = z->status; s->x_agc = x->agc; s->z_agc = z->agc;
    s->x_home_offset_counts = x->home_offset; s->z_home_offset_counts = z->home_offset;
    const uint32_t xa = x->health_seen ? (now - x->health_us) / 1000U : UINT32_MAX;
    const uint32_t za = z->health_seen ? (now - z->health_us) / 1000U : UINT32_MAX;
    s->x_health_age_ms = xa > UINT16_MAX ? UINT16_MAX : (uint16_t)xa;
    s->z_health_age_ms = za > UINT16_MAX ? UINT16_MAX : (uint16_t)za;
    /* Event-bearing status is immutable until transmitted, not coalescible.
     * Advance event cursors only when a record was successfully committed. */
    if (uart_transport_record(e->uart, &r, false, periodic && !rx_event)) {
        e->status_rx_overflows = s->rx_overflows;
        e->status_rx_crc_errors = s->rx_crc_errors;
        e->status_rx_format_errors = s->rx_format_errors;
        if (periodic) {
            e->encoders->axis[0].status_gap_us = 0U; e->encoders->axis[1].status_gap_us = 0U;
        }
    }
}
static bool plan(motion_executor_t *e, diag_record_t *r, int32_t applied_x, bool clamped)
{
    const int32_t targets[3] = {applied_x, e->config->pick_depth_01mm, e->config->safe_z_01mm};
    for (uint8_t i = 0U; i < 3U; ++i) {
        diag_phase_t *p = &r->data.phases[i];
        p->axis = (uint8_t)(i == 0U ? AXIS_X : AXIS_Z); p->phase = (uint8_t)(i + 1U);
        p->requested_target_01mm = i == 0U ? r->command.x_01mm : targets[i];
        p->applied_target_01mm = targets[i]; p->step_angle_udeg = 112500U;
        if (!motion_target_steps(&e->config->axis[p->axis], targets[i],
                                 &p->target_position_steps, &p->quant_residual_nm)) { return false; }
        if (i == 0U && clamped) { p->axis_flags |= PH_CLAMPED; }
    }
    return true;
}
static void terminal_pick(motion_executor_t *e, const queued_command_t *q,
                          uint32_t flags, bool reserved, uint32_t now)
{
    diag_record_t r;
    header(e, &r, DIAG_COMMAND_RESULT, &q->command, q->seq, now);
    r.status_bits = flags;
    if (!plan(e, &r, q->applied_x, q->clamped)) { r.status_bits |= DIAG_CONFIG_INVALID; }
    (void)uart_transport_record(e->uart, &r, reserved, false);
}
static void cancel_queue(motion_executor_t *e, uint32_t now)
{
    while (e->qtail != e->qhead) {
        const queued_command_t *q = &e->queue[e->qtail % MOTION_QUEUE_SIZE];
        terminal_pick(e, q, DIAG_CANCELLED | e->faults | DIAG_NOT_HOMED, true, now);
        ++e->qtail;
    }
}
static void latch_fault(motion_executor_t *e, uint32_t flags, uint32_t now)
{
    e->faults |= flags | DIAG_POSITION_UNCERTAIN;
    e->homed_mask = 0U;
    step_engine_abort(e->steps);
    if (e->home_pending) { homing_stop(&e->home, (flags & DIAG_ABORTED) != 0U ? 4U : 5U, now); }
    cancel_queue(e, now);
    if (e->state != EXEC_HOMING && !e->active) {
        e->state = (flags & DIAG_ABORTED) != 0U ? EXEC_ABORTED : EXEC_FAULT;
    }
}
void motion_executor_submit(motion_executor_t *e, const vision_cmd_t *c, uint32_t now)
{
    const uint32_t seq = ++e->command_seq;
    if (c->type == VISION_TYPE_STATUS) {
        motion_executor_status(e, c, seq, false, now); return;
    }
    if (c->type == VISION_TYPE_ABORT) {
        latch_fault(e, DIAG_ABORTED, now);
        motion_executor_status(e, c, seq, false, now); return;
    }
    if (c->type == VISION_TYPE_HOME) {
        const bool idle = !e->active && e->qhead == e->qtail && !e->home_pending &&
            !step_engine_snapshot(e->steps, 0U).running &&
            !step_engine_snapshot(e->steps, 1U).running;
        /* Explicit re-home may recover continuity only after fresh health checks.
         * Driver reset/fault recovery needs board reconfiguration, never hidden W1C. */
        bool healthy = idle && motion_config_valid(e->config) &&
            (e->safe_inputs == NULL || e->safe_inputs(e->safe_context));
        for (uint8_t i = 0U; i < 2U && healthy; ++i) {
            healthy = driver_evidence_poll(e->drivers, i) && encoder_sampler_reseed(e->encoders, i);
        }
        now = now_us(e);
        if (!healthy || !begin_home(e, c, seq, now)) {
            ++e->rejected;
            const uint32_t previous = e->faults;
            e->faults |= DIAG_REJECTED;
            motion_executor_status(e, c, seq, false, now);
            e->faults = previous;
        } else { e->faults = 0U; }
        return;
    }
    if (c->type != VISION_TYPE_PICK) { ++e->rejected; return; }
    queued_command_t q = {.command = *c, .seq = seq, .applied_x = c->x_01mm};
    uint32_t rejection = 0U;
    if (c->class_id > 2U || c->angle_01deg >= 3600U ||
        c->corr_01deg < -1800 || c->corr_01deg > 1800) { rejection |= DIAG_CONFIG_INVALID; }
    if (!motion_config_valid(e->config) || !e->config->bounds_confirmed) {
        rejection |= DIAG_CONFIG_INVALID;
    }
    if (q.applied_x < 0 || q.applied_x > e->config->axis[AXIS_X].usable_max_01mm) {
        if (e->config->clamp_x) {
            q.applied_x = q.applied_x < 0 ? 0 : e->config->axis[AXIS_X].usable_max_01mm;
            q.clamped = true;
        } else { rejection |= DIAG_LIMIT_FAULT; }
    }
    if (e->homed_mask != 3U || e->faults != 0U ||
        (e->state != EXEC_READY && e->state != EXEC_MOVING)) {
        rejection |= DIAG_NOT_HOMED | e->faults;
    }
    if (e->qhead - e->qtail >= MOTION_QUEUE_SIZE || uart_transport_backpressure(e->uart, now)) {
        rejection |= DIAG_TX_BACKPRESSURE;
    }
    rejection |= sensor_faults(e, now);
    if (rejection == 0U && uart_transport_reserve(e->uart)) {
        e->queue[e->qhead % MOTION_QUEUE_SIZE] = q; ++e->qhead; return;
    }
    ++e->rejected;
    terminal_pick(e, &q, DIAG_REJECTED | rejection, false, now);
}

static void start_sample(motion_executor_t *e, diag_phase_t *p, uint32_t now)
{
    const uint8_t axis = p->axis;
    const encoder_sample_t *s = &e->encoders->axis[axis];
    const driver_sample_t *d = &e->drivers->axis[axis];
    p->axis_flags |= PH_BEGUN;
    if (e->homed_mask == 3U) { p->axis_flags |= PH_HOME; }
    if (e->config->axis[axis].encoder_sign < 0) { p->axis_flags |= PH_ENC_NEG; }
    if (e->config->axis[axis].dir_sign < 0) { p->axis_flags |= PH_MSCNT_NEG; }
    if (encoder_sampler_fresh(s, now)) {
        p->axis_flags |= PH_START_ENC | PH_CONTINUOUS;
        p->raw_start = s->raw; p->unwrap_start = s->unwrap.total_counts;
        p->as_status_start = s->status; p->agc_start = s->agc;
        p->as_conf = s->conf; p->raw_start_timestamp_us = s->timestamp_us;
        p->health_start_age_ms = s->health_age_ms; p->home_offset_counts = s->home_offset;
    } else { p->health_start_age_ms = UINT16_MAX; }
    if (d->valid && !d->fault) { p->axis_flags |= PH_START_MSCNT; p->mscnt_start = d->mscnt; }
    p->driver_mode = d->mode;
    e->encoders->axis[axis].max_gap_us = 0U;
}
static int64_t round_ratio(int64_t n, int64_t divisor)
{
    return n < 0 ? -((-n + divisor / 2) / divisor) : (n + divisor / 2) / divisor;
}
static void end_sample(motion_executor_t *e, diag_phase_t *p, bool complete, uint32_t now)
{
    const uint8_t axis = p->axis;
    const step_snapshot_t step = step_engine_snapshot(e->steps, axis);
    const encoder_sample_t *s = &e->encoders->axis[axis];
    const driver_sample_t *d = &e->drivers->axis[axis];
    p->emitted_delta_steps = step.emitted; p->emitted_edge_count = step.edges;
    p->move_duration_us = step.done_us - step.start_us;
    p->max_sample_gap_us = s->max_gap_us;
    if (s->conf_seen) { p->as_conf = s->conf; }
    p->axis_flags |= s->faults;
    if (complete) { p->axis_flags |= PH_COMPLETE; }
    if (encoder_sampler_fresh(s, now)) {
        p->axis_flags |= PH_END_ENC;
        p->raw_end = s->raw; p->unwrap_end = s->unwrap.total_counts;
        p->as_status_end = s->status; p->agc_end = s->agc;
        p->raw_end_timestamp_us = s->timestamp_us; p->health_end_age_ms = s->health_age_ms;
        const int64_t delta = (int64_t)p->unwrap_end - p->unwrap_start;
        if ((p->axis_flags & PH_START_ENC) != 0U && delta >= INT32_MIN && delta <= INT32_MAX) {
            p->unwrap_delta = (int32_t)delta;
            const int64_t signed_delta = delta * e->config->axis[axis].encoder_sign;
            const int64_t angle = round_ratio(signed_delta * 36000, 4096);
            const int64_t um = round_ratio(signed_delta * e->config->axis[axis].lead_um, 4096);
            if (signed_delta >= INT32_MIN && signed_delta <= INT32_MAX &&
                angle >= INT32_MIN && angle <= INT32_MAX && um >= INT32_MIN && um <= INT32_MAX) {
                p->shaft_delta_001deg = (int32_t)angle;
                p->displacement_um = enc_counts_to_um((int32_t)signed_delta, e->config->axis[axis].lead_um);
            } else { p->axis_flags |= PH_CONVERSION; }
        } else { p->axis_flags |= PH_CONVERSION; }
    } else {
        p->health_end_age_ms = UINT16_MAX;
        p->axis_flags &= (uint16_t)~PH_CONTINUOUS;
    }
    if (d->valid && !d->fault) { p->axis_flags |= PH_END_MSCNT; p->mscnt_end = d->mscnt; }
    if ((p->axis_flags & (PH_START_MSCNT | PH_END_MSCNT)) == (PH_START_MSCNT | PH_END_MSCNT)) {
        p->mscnt_observed = (uint16_t)((p->mscnt_end - p->mscnt_start) & 1023);
        p->mscnt_expected = (uint16_t)((uint64_t)((int64_t)step.emitted *
            e->config->axis[axis].dir_sign * 16) & 1023U);
        p->mscnt_check = e->config->mscnt_qualified ?
            (p->mscnt_observed == p->mscnt_expected ? 1U : 2U) : 3U;
        if (p->mscnt_check == 2U) {
            e->result.status_bits |= DIAG_MSCNT_MISMATCH;
            latch_fault(e, DIAG_MSCNT_MISMATCH, now);
        }
        if (p->mscnt_check == 3U) { e->result.status_bits |= DIAG_MSCNT_UNQUALIFIED; }
    }
    if ((p->axis_flags & PH_CONVERSION) != 0U) {
        e->result.status_bits |= DIAG_CONFIG_INVALID;
        latch_fault(e, DIAG_CONFIG_INVALID, now);
    }
}
static void commit(motion_executor_t *e, uint32_t now)
{
    (void)now;
    e->result.timestamp_ms = e->encoders->io.now_ms(e->encoders->io.context);
    e->result.status_bits |= e->faults;
    if (e->faults == 0U && (e->result.status_bits &
        (DIAG_MSCNT_MISMATCH | DIAG_CONFIG_INVALID)) == 0U) { e->result.status_bits |= DIAG_SUCCESS; }
    (void)uart_transport_record(e->uart, &e->result, true, false);
    e->active = false;
    e->state = e->faults == 0U ? EXEC_READY :
        (e->faults & DIAG_ABORTED) != 0U ? EXEC_ABORTED : EXEC_FAULT;
}
void motion_executor_poll(motion_executor_t *e, uint32_t now)
{
    step_engine_poll(e->steps);
    if (e->state == EXEC_HOMING || e->active) {
        const uint32_t f = sensor_faults(e, now);
        if (f != 0U && e->faults == 0U) { latch_fault(e, f, now); }
    }
    if (e->home_pending) {
        homing_poll(&e->home, now);
        if (e->home.state == HOME_DONE &&
            !step_engine_snapshot(e->steps, 0U).running &&
            !step_engine_snapshot(e->steps, 1U).running) {
            e->home_record.timestamp_ms = e->encoders->io.now_ms(e->encoders->io.context);
            memcpy(e->home_record.data.home, e->home.result, sizeof(e->home.result));
            if (homing_success(&e->home) && e->faults == 0U) {
                ++e->home_epoch; e->homed_mask = 3U; e->state = EXEC_READY;
                e->home_record.status_bits = DIAG_SUCCESS;
            } else {
                e->faults |= DIAG_HOME_FAILED | DIAG_POSITION_UNCERTAIN;
                e->state = (e->faults & DIAG_ABORTED) != 0U ? EXEC_ABORTED : EXEC_FAULT;
                e->home_record.status_bits = e->faults | DIAG_NOT_HOMED;
            }
            e->home_record.home_epoch = e->home_epoch;
            (void)uart_transport_record(e->uart, &e->home_record, true, false);
            e->home_pending = false;
        }
    } else if (!e->active && e->state == EXEC_READY && e->qhead != e->qtail &&
               !uart_transport_backpressure(e->uart, now)) {
        const queued_command_t q = e->queue[e->qtail % MOTION_QUEUE_SIZE]; ++e->qtail;
        header(e, &e->result, DIAG_COMMAND_RESULT, &q.command, q.seq, now);
        if (q.clamped) { e->result.status_bits = DIAG_CLAMPED; }
        if (!plan(e, &e->result, q.applied_x, q.clamped)) {
            latch_fault(e, DIAG_CONFIG_INVALID, now); e->active = true; commit(e, now); return;
        }
        e->active = true; e->phase = 0U; e->phase_state = PHASE_START; e->state = EXEC_MOVING;
    }
    if (e->active) {
        diag_phase_t *p = &e->result.data.phases[e->phase];
        step_snapshot_t s = step_engine_snapshot(e->steps, p->axis);
        if (e->faults != 0U && !s.running) {
            if ((p->axis_flags & PH_BEGUN) != 0U && (p->axis_flags & PH_COMPLETE) == 0U) {
                encoder_sampler_poll(e->encoders, true);
                (void)driver_evidence_poll(e->drivers, p->axis);
                end_sample(e, p, false, now_us(e));
            }
            commit(e, now_us(e)); return;
        }
        switch (e->phase_state) {
        case PHASE_START:
            encoder_sampler_poll(e->encoders, true);
            (void)driver_evidence_poll(e->drivers, p->axis); now = now_us(e);
            {
                const uint32_t f = sensor_faults(e, now);
                if (f != 0U) { latch_fault(e, f, now); break; }
            }
            if (p->axis == AXIS_X) {
                int32_t safe, residual;
                if (!motion_target_steps(&e->config->axis[AXIS_Z], e->config->safe_z_01mm,
                                         &safe, &residual) ||
                    step_engine_snapshot(e->steps, AXIS_Z).position != safe) {
                    latch_fault(e, DIAG_LIMIT_FAULT, now); break;
                }
            }
            s = step_engine_snapshot(e->steps, p->axis);
            p->start_position_steps = s.position;
            {
                const int64_t delta = (int64_t)p->target_position_steps - s.position;
                if (delta < INT32_MIN || delta > INT32_MAX) {
                    latch_fault(e, DIAG_CONFIG_INVALID, now); break;
                }
                p->commanded_delta_steps = (int32_t)delta;
            }
            start_sample(e, p, now); ++e->result.phase_count;
            if (!step_engine_move(e->steps, p->axis, p->target_position_steps, now)) {
                latch_fault(e, DIAG_STEP_TIMING_FAULT, now); break;
            }
            e->phase_state = PHASE_RUN;
            break;
        case PHASE_RUN:
            if (!s.running) { e->settle_us = s.done_us; e->phase_state = PHASE_SETTLE; }
            break;
        case PHASE_SETTLE:
            if (now - e->settle_us < e->config->settle_us) { break; }
            encoder_sampler_poll(e->encoders, true);
            (void)driver_evidence_poll(e->drivers, p->axis); now = now_us(e);
            end_sample(e, p, true, now);
            {
                const uint32_t f = sensor_faults(e, now);
                if (f != 0U) { latch_fault(e, f, now); break; }
            }
            if (e->phase == 2U) { commit(e, now); }
            else if (e->phase == 1U) { e->settle_us = now; e->phase_state = PHASE_DWELL; }
            else { ++e->phase; e->phase_state = PHASE_START; }
            break;
        case PHASE_DWELL:
            if (now - e->settle_us >= e->config->dwell_us) { ++e->phase; e->phase_state = PHASE_START; }
            break;
        }
    }
    if (!e->active && e->state == EXEC_READY) {
        const uint32_t f = sensor_faults(e, now);
        if (f != 0U) { latch_fault(e, f, now); }
    }
    if (now - e->status_us >= 1000000U) {
        e->status_us = now; motion_executor_status(e, NULL, 0U, true, now);
    }
}
