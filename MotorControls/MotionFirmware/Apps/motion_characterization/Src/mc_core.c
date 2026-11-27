/**
 * @file    mc_core.c
 * @brief   Hardware-independent motion characterization controller,
 *          command parser, NDJSON emitter, safety interlocks, and test runner.
 */
#include "mc_core.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MC_USTEPS_PER_REV      3200
#define MC_ENC_COUNTS_PER_REV  4096
#define MC_FOLLOW_MIN_USTEPS   32U
#define MC_FILTER_ALLOW_US     2000U /* 2 ms conservative allowance at CONF=0x0300 */
#define MC_SETTLE_WAIT_US      100000U /* 100 ms standstill settle */
#define MC_SETTLE_COUNT        32U
#define MC_CRUISE_MIN_STEPS    16U
#define MC_CRUISE_MIN_US       100000U /* 100 ms cruise plateau */

const char *mc_fault_name(mc_fault_t f) {
    switch (f) {
        case MC_FAULT_NONE:            return "none";
        case MC_FAULT_HOST_STOP:       return "host_stop";
        case MC_FAULT_B1_STOP:         return "b1_stop";
        case MC_FAULT_USB_DISCONNECT:  return "usb_disconnect";
        case MC_FAULT_LEASE_EXPIRED:   return "lease_expired";
        case MC_FAULT_SOFT_LIMIT:      return "soft_limit";
        case MC_FAULT_ENCODER_INVALID: return "invalid_feedback";
        case MC_FAULT_ENCODER_STALE:   return "encoder_stale";
        case MC_FAULT_ENCODER_GAP:     return "encoder_gap";
        case MC_FAULT_FOLLOWING_ERROR: return "tracking_loss_suspected";
        case MC_FAULT_WRONG_DIRECTION: return "wrong_direction";
        case MC_FAULT_NO_PROGRESS:     return "no_progress";
        case MC_FAULT_DRIVER_FAULT:    return "driver_fault";
        case MC_FAULT_DRIVER_RESET:    return "driver_reset";
        case MC_FAULT_DRIVER_SPI:      return "driver_spi";
        case MC_FAULT_TEMP_LIMIT:      return "temp_limit";
        case MC_FAULT_VM_OUT_OF_RANGE: return "vm_out_of_range";
        case MC_FAULT_TIMING_LATENESS: return "timing_lateness";
        case MC_FAULT_RING_OVERFLOW:   return "ring_overflow";
        case MC_FAULT_RESTORE_FAILED:  return "restore_failed";
        default:                       return "unknown";
    }
}

const char *mc_axis_name(mc_axis_t a) {
    switch (a) {
        case MC_AXIS_Z:     return "z";
        case MC_AXIS_X:     return "x";
        case MC_AXIS_THETA: return "theta";
        default:            return "none";
    }
}

static bool parse_axis(const char *s, mc_axis_t *out) {
    if (s == NULL || out == NULL) {
        return false;
    }
    if (strcmp(s, "z") == 0) {
        *out = MC_AXIS_Z;
        return true;
    }
    if (strcmp(s, "x") == 0) {
        *out = MC_AXIS_X;
        return true;
    }
    if (strcmp(s, "theta") == 0 || strcmp(s, "t") == 0) {
        *out = MC_AXIS_THETA;
        return true;
    }
    return false;
}

static bool parse_i32(const char *s, int32_t *out) {
    if (s == NULL || out == NULL || *s == '\0') {
        return false;
    }
    bool neg = false;
    if (*s == '-') {
        neg = true;
        s++;
    } else if (*s == '+') {
        s++;
    }
    uint32_t mag = 0U;
    if (!cmd_parse_u32(s, &mag)) {
        return false;
    }
    if (neg) {
        if (mag > 2147483648U) {
            return false;
        }
        *out = (mag == 2147483648U) ? INT32_MIN : -(int32_t)mag;
    } else {
        if (mag > 2147483647U) {
            return false;
        }
        *out = (int32_t)mag;
    }
    return true;
}

uint8_t mc_axis_to_enc_bus(mc_session_mode_t mode, mc_axis_t axis) {
    if (axis == MC_AXIS_Z) {
        return (mode == MC_SESSION_XZ) ? 0U : 0xFFU;
    }
    if (axis == MC_AXIS_X) {
        return (mode == MC_SESSION_XZ) ? 1U : 0xFFU;
    }
    if (axis == MC_AXIS_THETA) {
        return (mode == MC_SESSION_THETA_FIXTURE) ? 1U : 0xFFU;
    }
    return 0xFFU;
}

bool mc_axis_motion_allowed(mc_session_mode_t mode, mc_axis_t axis) {
    if (mode == MC_SESSION_XZ) {
        return (axis == MC_AXIS_X) || (axis == MC_AXIS_Z);
    }
    return axis == MC_AXIS_THETA;
}

bool mc_validate_stroke_geometry(int32_t start_steps, int32_t target_steps,
                                 uint32_t v_target, uint32_t v_start,
                                 uint32_t accel, bool require_ramp) {
    const int64_t diff = (int64_t)target_steps - (int64_t)start_steps;
    const uint64_t dist = (diff >= 0) ? (uint64_t)diff : (uint64_t)(-diff);
    if (v_target == 0U || accel == 0U || dist == 0U) {
        return false;
    }
    const uint64_t vs = (v_start < v_target) ? (uint64_t)v_start : (uint64_t)v_target;
    if (require_ramp && vs >= (uint64_t)v_target) {
        return false;
    }
    uint64_t ramp_steps = 0U;
    if ((uint64_t)v_target > vs) {
        const uint64_t num = (uint64_t)v_target * (uint64_t)v_target - vs * vs;
        const uint64_t den = 2ULL * (uint64_t)accel;
        ramp_steps = (num + den - 1ULL) / den;
    }
    /* Cruise must be >= 100 ms AND >= 16 steps. */
    uint64_t cruise_steps = ((uint64_t)v_target * (uint64_t)MC_CRUISE_MIN_US + 999999ULL) / 1000000ULL;
    if (cruise_steps < MC_CRUISE_MIN_STEPS) {
        cruise_steps = MC_CRUISE_MIN_STEPS;
    }
    const uint64_t min_total = 2ULL * ramp_steps + cruise_steps;
    return dist >= min_total;
}

static uint32_t now_us(const mc_core_t *mc) {
    return (mc->hal.now_us != NULL) ? mc->hal.now_us(mc->hal.ctx) : 0U;
}

static void tx_drain(mc_core_t *mc) {
    if (mc->hal.usb_write == NULL) {
        return;
    }
    if (mc->hal.usb_mounted != NULL && !mc->hal.usb_mounted(mc->hal.ctx)) {
        return;
    }
    while (mc->tx.count > 0U) {
        const uint16_t idx = mc->tx.tail;
        const uint16_t rem = (uint16_t)(mc->tx.lens[idx] - mc->tx.cursor);
        const uint32_t wrote = mc->hal.usb_write(
            mc->hal.ctx,
            (const uint8_t *)&mc->tx.lines[idx][mc->tx.cursor],
            rem);
        if (wrote == 0U) {
            break;
        }
        mc->tx.cursor = (uint16_t)(mc->tx.cursor + wrote);
        if (mc->tx.cursor >= mc->tx.lens[idx]) {
            mc->tx.tail = (uint16_t)((mc->tx.tail + 1U) % MC_TX_RING_LINES);
            mc->tx.count--;
            mc->tx.cursor = 0U;
        } else {
            break;
        }
    }
}

static bool tx_enqueue_raw(mc_core_t *mc, const char *line, uint16_t len) {
    tx_drain(mc);
    if (mc->tx.count >= MC_TX_RING_LINES) {
        mc->tx.overflow_count++;
        if (mc->state == MC_STATE_RUNNING) {
            mc_core_latch_fault(mc, MC_FAULT_RING_OVERFLOW, false);
        }
        return false;
    }
    if (len >= MC_TX_LINE_MAX) {
        len = MC_TX_LINE_MAX - 1U;
    }
    const uint16_t idx = mc->tx.head;
    memcpy(mc->tx.lines[idx], line, len);
    mc->tx.lines[idx][len] = '\0';
    mc->tx.lens[idx] = len;
    mc->tx.head = (uint16_t)((mc->tx.head + 1U) % MC_TX_RING_LINES);
    mc->tx.count++;
    tx_drain(mc);
    return true;
}

static void emit_json(mc_core_t *mc, const char *kind, uint32_t req_id,
                      uint32_t trial_id, const char *axis_str,
                      const char *body_fmt, ...) {
    char buf[MC_TX_LINE_MAX];
    const uint32_t t = now_us(mc);
    mc->seq++;
    int n = snprintf(buf, sizeof(buf),
                     "{\"schema_version\":%u,\"seq\":%lu,\"request_id\":%lu,"
                     "\"trial_id\":%lu,\"kind\":\"%s\",\"axis\":\"%s\","
                     "\"t_us\":%lu",
                     (unsigned)MC_SCHEMA_VERSION,
                     (unsigned long)mc->seq,
                     (unsigned long)req_id,
                     (unsigned long)trial_id,
                     kind,
                     axis_str ? axis_str : "none",
                     (unsigned long)t);
    if (n < 0 || (size_t)n >= sizeof(buf) - 4U) {
        return;
    }
    if (body_fmt != NULL && body_fmt[0] != '\0') {
        buf[n++] = ',';
        va_list ap;
        va_start(ap, body_fmt);
        int m = vsnprintf(buf + n, sizeof(buf) - (size_t)n - 2U, body_fmt, ap);
        va_end(ap);
        if (m > 0) {
            n += m;
            if ((size_t)n >= sizeof(buf) - 2U) {
                n = (int)(sizeof(buf) - 3U);
            }
        }
    }
    buf[n++] = '}';
    buf[n++] = '\n';
    buf[n] = '\0';
    (void)tx_enqueue_raw(mc, buf, (uint16_t)n);
}

static void emit_reject(mc_core_t *mc, uint32_t req_id, const char *axis_str,
                        const char *reason) {
    emit_json(mc, "result", req_id, mc->trial_id, axis_str,
              "\"ok\":false,\"status\":\"rejected\",\"reason\":\"%s\"", reason);
}

static void emit_ack(mc_core_t *mc, uint32_t req_id, const char *axis_str,
                     const char *cmd) {
    emit_json(mc, "result", req_id, mc->trial_id, axis_str,
              "\"ok\":true,\"status\":\"accepted\",\"cmd\":\"%s\"", cmd);
}

static int32_t counts_to_usteps_signed(int32_t counts, int8_t sign) {
    const int64_t scaled = (int64_t)counts * (int64_t)sign * (int64_t)MC_USTEPS_PER_REV;
    if (scaled >= 0) {
        return (int32_t)((scaled + (MC_ENC_COUNTS_PER_REV / 2)) / MC_ENC_COUNTS_PER_REV);
    }
    return (int32_t)((scaled - (MC_ENC_COUNTS_PER_REV / 2)) / MC_ENC_COUNTS_PER_REV);
}

static void enc_noise_abort_restore(mc_core_t *mc) {
    if (mc->active_test != MC_TEST_ENC_NOISE || mc->noise_sf_code == 3U ||
        mc->hal.enc_set_slow_filter == NULL) {
        return;
    }
    uint16_t rb = 0U;
    (void)mc->hal.enc_set_slow_filter(mc->hal.ctx, mc->noise_bus, 3U, &rb);
}

void mc_core_latch_fault(mc_core_t *mc, mc_fault_t fault, bool disable_stage) {
    if (mc == NULL || fault == MC_FAULT_NONE) {
        return;
    }
    /* Cancel future edges immediately from the interlock. */
    if (mc->hal.step_set_inhibit != NULL) {
        mc->hal.step_set_inhibit(mc->hal.ctx, true);
    }
    if (mc->hal.step_halt != NULL) {
        mc->hal.step_halt(mc->hal.ctx);
    }
    if (disable_stage && mc->hal.disable_all_drivers != NULL) {
        mc->hal.disable_all_drivers(mc->hal.ctx);
    }
    const bool first = (mc->fault == MC_FAULT_NONE);
    if (first) {
        mc->fault = fault;
    }
    /* Revoke datum & arming: no resumption without operator inspection + new datum. */
    if (mc->armed_axis < MC_MAX_AXES) {
        mc->policy[mc->armed_axis].datum_confirmed = false;
    }
    const mc_axis_t prev_axis = mc->armed_axis;
    mc->state = MC_STATE_FAULT;
    enc_noise_abort_restore(mc);
    mc->active_test = MC_TEST_NONE;
    mc->lease_active = false;
    mc->armed_axis = MC_AXIS_NONE;

    if (first) {
        mc_step_stats_t st;
        memset(&st, 0, sizeof(st));
        if (mc->hal.step_get_stats != NULL) {
            mc->hal.step_get_stats(mc->hal.ctx, &st);
        }
        emit_json(mc, "event", mc->active_request_id, mc->trial_id,
                  mc_axis_name(prev_axis),
                  "\"event\":\"fault_latched\",\"reason\":\"%s\","
                  "\"stage_disabled\":%s,\"edges_emitted\":%lu,"
                  "\"edges_inhibited\":%lu,\"max_lateness_us\":%lu",
                  mc_fault_name(fault),
                  disable_stage ? "true" : "false",
                  (unsigned long)st.edges_emitted,
                  (unsigned long)st.edges_inhibited,
                  (unsigned long)st.max_lateness_us);
    }
}

void mc_core_b1_irq(mc_core_t *mc) {
    if (mc == NULL) {
        return;
    }
    mc->b1_latched = true;
    if (mc->hal.step_set_inhibit != NULL) {
        mc->hal.step_set_inhibit(mc->hal.ctx, true);
    }
    if (mc->hal.step_halt != NULL) {
        mc->hal.step_halt(mc->hal.ctx);
    }
    if (mc->hal.disable_all_drivers != NULL) {
        mc->hal.disable_all_drivers(mc->hal.ctx);
    }
}

void mc_core_interlock_tick_1ms(mc_core_t *mc) {
    if (mc == NULL) {
        return;
    }
    const bool active = (mc->state == MC_STATE_ARMED || mc->state == MC_STATE_RUNNING);
    if (!active) {
        return;
    }
    if (mc->b1_latched ||
        (mc->hal.b1_asserted != NULL && mc->hal.b1_asserted(mc->hal.ctx))) {
        mc->b1_latched = true;
        if (mc->hal.step_set_inhibit != NULL) {
            mc->hal.step_set_inhibit(mc->hal.ctx, true);
        }
        if (mc->hal.step_halt != NULL) {
            mc->hal.step_halt(mc->hal.ctx);
        }
        if (mc->hal.disable_all_drivers != NULL) {
            mc->hal.disable_all_drivers(mc->hal.ctx);
        }
        return;
    }
    if (mc->hal.usb_mounted != NULL && !mc->hal.usb_mounted(mc->hal.ctx)) {
        if (mc->hal.step_set_inhibit != NULL) {
            mc->hal.step_set_inhibit(mc->hal.ctx, true);
        }
        if (mc->hal.step_halt != NULL) {
            mc->hal.step_halt(mc->hal.ctx);
        }
        if (mc->hal.disable_all_drivers != NULL) {
            mc->hal.disable_all_drivers(mc->hal.ctx);
        }
        return;
    }
    if (mc->lease_active) {
        const uint32_t elapsed = now_us(mc) - mc->last_keepalive_us;
        if (elapsed > MC_LEASE_TIMEOUT_US) {
            if (mc->hal.step_set_inhibit != NULL) {
                mc->hal.step_set_inhibit(mc->hal.ctx, true);
            }
            if (mc->hal.step_halt != NULL) {
                mc->hal.step_halt(mc->hal.ctx);
            }
        }
    }
}

void mc_core_init(mc_core_t *mc, const mc_hal_t *hal) {
    memset(mc, 0, sizeof(*mc));
    if (hal != NULL) {
        mc->hal = *hal;
    }
    mc->state = MC_STATE_DISARMED;
    mc->session_mode = MC_SESSION_XZ;
    mc->armed_axis = MC_AXIS_NONE;
    mc->active_test = MC_TEST_NONE;
    mc->fault = MC_FAULT_NONE;
    mc->opt_stress_s = 60U;
    mc->opt_stress_spi_div = 32U;
    mc->opt_noise_sf = 3U;        /* SF=2x default (CONF=0x0300) */
    mc->opt_noise_rate_hz = 500U; /* 2 ms uniform cadence */
    mc->opt_noise_n = 2048U;
    mc->opt_noise_hold = false;

    for (uint8_t i = 0U; i < MC_MAX_AXES; ++i) {
        mc->policy[i].encoder_sign = 1;
        mc->policy[i].static_noise_cnt = 2U;
        mc->policy[i].follow_band_usteps = MC_FOLLOW_MIN_USTEPS;
        mc->policy[i].vm_min_mv = 9000U;
        mc->policy[i].vm_max_mv = 28000U;
        mc->policy[i].temp_cutoff_c10 = 1000;
        mc->enc_state[i].health_age_ms = UINT16_MAX;
    }
    cmd_linebuf_init(&mc->rx_line);

    /* Passive-by-default boot: all ENNs HIGH, stepgen inhibited. */
    if (mc->hal.disable_all_drivers != NULL) {
        mc->hal.disable_all_drivers(mc->hal.ctx);
    }
    if (mc->hal.step_set_inhibit != NULL) {
        mc->hal.step_set_inhibit(mc->hal.ctx, true);
    }
}

/* ------------------------------------------------------------------------- */
/* Passive tests: commission, encoder_rate, static, reference, spi_rate      */
/* ------------------------------------------------------------------------- */

static void run_commission(mc_core_t *mc, uint32_t req_id, mc_axis_t axis) {
    const uint8_t bus = mc_axis_to_enc_bus(mc->session_mode, axis);
    if (bus == 0xFFU) {
        emit_reject(mc, req_id, mc_axis_name(axis), "no_shaft_feedback");
        return;
    }
    if (mc->hal.disable_all_drivers != NULL) {
        mc->hal.disable_all_drivers(mc->hal.ctx);
    }
    uint16_t conf_rb = 0U, mag = 0U, raw = 0U;
    uint8_t status = 0U, agc = 0U;
    bool ok = false;
    if (mc->hal.enc_configure != NULL) {
        ok = mc->hal.enc_configure(mc->hal.ctx, bus, &conf_rb, &status, &agc, &mag, &raw);
    }
    mc->trial_id++;
    if (!ok || (conf_rb & ENCODER_CONF_MASK) != ENCODER_CONF_EXPECTED ||
        (status & 0x38U) != 0x20U) {
        mc->policy[axis].commissioned = false;
        emit_json(mc, "result", req_id, mc->trial_id, mc_axis_name(axis),
                  "\"ok\":false,\"test\":\"commission\",\"bus\":%u,"
                  "\"conf\":%u,\"status\":%u,\"agc\":%u,\"mag\":%u,\"raw\":%u,"
                  "\"reason\":\"%s\"",
                  (unsigned)bus, (unsigned)conf_rb, (unsigned)status,
                  (unsigned)agc, (unsigned)mag, (unsigned)raw,
                  !ok ? "i2c_error"
                      : (((conf_rb & ENCODER_CONF_MASK) != ENCODER_CONF_EXPECTED)
                             ? "conf_mismatch"
                             : "bad_magnet_status"));
        return;
    }

    /* Seed encoder_sampler with validated config + health + first raw. */
    encoder_sample_t *es = &mc->enc_state[axis];
    memset(es, 0, sizeof(*es));
    es->health_age_ms = UINT16_MAX;
    const uint32_t t0 = now_us(mc);
    encoder_sampler_config(es, true, conf_rb);
    encoder_sampler_health(es, true, status, agc, t0);
    const bool accepted = encoder_sampler_accept(es, raw, t0, 1U);

    emit_json(mc, "result", req_id, mc->trial_id, mc_axis_name(axis),
              "\"ok\":%s,\"test\":\"commission\",\"bus\":%u,\"conf\":%u,"
              "\"status\":%u,\"agc\":%u,\"mag\":%u,\"raw\":%u",
              accepted ? "true" : "false",
              (unsigned)bus, (unsigned)conf_rb, (unsigned)status,
              (unsigned)agc, (unsigned)mag, (unsigned)raw);
}

static void run_encoder_rate(mc_core_t *mc, uint32_t req_id, const char *target) {
    const bool both = (strcmp(target, "both") == 0);
    mc_axis_t single_ax = MC_AXIS_NONE;
    uint8_t buses[2] = {0U, 1U};
    uint8_t n_buses = 1U;

    if (both) {
        if (mc->session_mode != MC_SESSION_XZ) {
            emit_reject(mc, req_id, "both", "session_mode_forbids_both");
            return;
        }
        n_buses = 2U;
    } else {
        if (!parse_axis(target, &single_ax)) {
            emit_reject(mc, req_id, target, "unknown_axis");
            return;
        }
        const uint8_t b = mc_axis_to_enc_bus(mc->session_mode, single_ax);
        if (b == 0xFFU) {
            emit_reject(mc, req_id, target, "no_shaft_feedback");
            return;
        }
        buses[0] = b;
    }

    if (mc->opt_enc_fast && !mc->opt_enc_fast_ok) {
        emit_reject(mc, req_id, target, "fast_mode_unqualified");
        return;
    }

    uint32_t timingr[2] = {0U, 0U};
    if (mc->opt_enc_fast && mc->hal.enc_set_fast_mode != NULL) {
        for (uint8_t i = 0U; i < n_buses; ++i) {
            if (!mc->hal.enc_set_fast_mode(mc->hal.ctx, buses[i], true, &timingr[i])) {
                emit_reject(mc, req_id, target, "fast_mode_apply_failed");
                return;
            }
        }
    }

    mc->trial_id++;
    const bool full_sample = (mc->opt_enc_mode == 1U);
    for (uint8_t w = 0U; w < MC_RATE_WINDOWS; ++w) {
        uint32_t attempts = 0U, ok_cnt = 0U, nack_cnt = 0U, to_cnt = 0U, other_err = 0U;
        uint32_t dup_cnt = 0U;
        uint32_t min_lat = UINT32_MAX, max_lat = 0U, max_gap = 0U;
        uint16_t prev_raw[2] = {0xFFFFU, 0xFFFFU};
        uint32_t prev_ok_us[2] = {0U, 0U};
        bool have_ok[2] = {false, false};
        uint8_t pre_st = 0U, post_st = 0U, agc = 0U;
        uint16_t mag = 0U;

        if (!full_sample && mc->hal.enc_read_health != NULL) {
            (void)mc->hal.enc_read_health(mc->hal.ctx, buses[0], &pre_st, &agc, &mag);
        }

        const uint32_t win_start = now_us(mc);
        const uint32_t total_ops = MC_RATE_WINDOW_OPS * (uint32_t)n_buses;
        for (uint32_t op = 0U; op < total_ops; ++op) {
            const uint8_t bi = (uint8_t)(op % n_buses);
            const uint8_t bus = buses[bi];
            const uint32_t t0 = now_us(mc);
            uint8_t err = 1U;
            uint16_t raw = 0U;
            if (full_sample) {
                mc_enc_full_t fs;
                memset(&fs, 0, sizeof(fs));
                if (mc->hal.enc_read_sample != NULL) {
                    err = mc->hal.enc_read_sample(mc->hal.ctx, bus, &fs);
                    raw = fs.raw_angle;
                }
            } else {
                if (mc->hal.enc_read_raw != NULL) {
                    err = mc->hal.enc_read_raw(mc->hal.ctx, bus, &raw);
                }
            }
            const uint32_t t1 = now_us(mc);
            const uint32_t lat = t1 - t0;
            attempts++;
            if (lat < min_lat) {
                min_lat = lat;
            }
            if (lat > max_lat) {
                max_lat = lat;
            }
            if (err == 0U) {
                ok_cnt++;
                if (have_ok[bi]) {
                    const uint32_t gap = t1 - prev_ok_us[bi];
                    if (gap > max_gap) {
                        max_gap = gap;
                    }
                    if (raw == prev_raw[bi]) {
                        dup_cnt++;
                    }
                }
                prev_raw[bi] = raw;
                prev_ok_us[bi] = t1;
                have_ok[bi] = true;
            } else if (err == 6U) { /* AS5600_ERROR_NACK */
                nack_cnt++;
            } else if (err == 5U) { /* AS5600_ERROR_TIMEOUT */
                to_cnt++;
            } else {
                other_err++;
            }
        }
        const uint32_t elapsed_us = now_us(mc) - win_start;
        if (!full_sample && mc->hal.enc_read_health != NULL) {
            (void)mc->hal.enc_read_health(mc->hal.ctx, buses[0], &post_st, &agc, &mag);
        }
        emit_json(mc, "sample", req_id, mc->trial_id, target,
                  "\"test\":\"encoder_rate\",\"mode\":\"%s\",\"fast\":%s,"
                  "\"window\":%u,\"axes\":%u,\"attempts\":%lu,\"ok\":%lu,"
                  "\"nack\":%lu,\"timeout\":%lu,\"other_err\":%lu,"
                  "\"duplicates\":%lu,\"elapsed_us\":%lu,\"min_lat_us\":%lu,"
                  "\"max_lat_us\":%lu,\"max_gap_us\":%lu,"
                  "\"pre_status\":%u,\"post_status\":%u,\"timingr\":%lu",
                  full_sample ? "sample" : "raw",
                  mc->opt_enc_fast ? "true" : "false",
                  (unsigned)w, (unsigned)n_buses,
                  (unsigned long)attempts, (unsigned long)ok_cnt,
                  (unsigned long)nack_cnt, (unsigned long)to_cnt,
                  (unsigned long)other_err, (unsigned long)dup_cnt,
                  (unsigned long)elapsed_us,
                  (unsigned long)(min_lat == UINT32_MAX ? 0U : min_lat),
                  (unsigned long)max_lat, (unsigned long)max_gap,
                  (unsigned)pre_st, (unsigned)post_st,
                  (unsigned long)timingr[0]);
    }

    /* Always restore installed TIMINGR after Fast-mode trial. */
    if (mc->opt_enc_fast && mc->hal.enc_set_fast_mode != NULL) {
        for (uint8_t i = 0U; i < n_buses; ++i) {
            uint32_t rb = 0U;
            if (!mc->hal.enc_set_fast_mode(mc->hal.ctx, buses[i], false, &rb)) {
                mc_core_latch_fault(mc, MC_FAULT_RESTORE_FAILED, true);
                return;
            }
        }
        mc->opt_enc_fast = false;
    }

    emit_json(mc, "result", req_id, mc->trial_id, target,
              "\"ok\":true,\"test\":\"encoder_rate\",\"windows\":%u",
              (unsigned)MC_RATE_WINDOWS);
}

static void run_static_or_reference(mc_core_t *mc, uint32_t req_id,
                                    mc_axis_t axis, bool is_ref) {
    const uint8_t bus = mc_axis_to_enc_bus(mc->session_mode, axis);
    if (bus == 0xFFU) {
        emit_reject(mc, req_id, mc_axis_name(axis), "no_shaft_feedback");
        return;
    }
    mc->trial_id++;
    const uint32_t target_n = is_ref ? MC_REF_SAMPLES : MC_STATIC_SAMPLES;
    enc_unwrap_t uw;
    memset(&uw, 0, sizeof(uw));
    int32_t min_c = INT32_MAX, max_c = INT32_MIN;
    int64_t sum_c = 0;
    uint32_t ok_n = 0U, err_n = 0U;
    uint8_t status = 0U, agc = 0U;
    uint16_t mag = 0U;

    if (mc->hal.enc_read_health != NULL) {
        (void)mc->hal.enc_read_health(mc->hal.ctx, bus, &status, &agc, &mag);
    }
    if ((status & 0x38U) != 0x20U) {
        emit_json(mc, "result", req_id, mc->trial_id, mc_axis_name(axis),
                  "\"ok\":false,\"test\":\"%s\",\"reason\":\"bad_magnet_status\","
                  "\"status\":%u",
                  is_ref ? "reference" : "static", (unsigned)status);
        return;
    }

    for (uint32_t i = 0U; i < target_n; ++i) {
        uint16_t raw = 0U;
        uint8_t err = 1U;
        if (mc->hal.enc_read_raw != NULL) {
            err = mc->hal.enc_read_raw(mc->hal.ctx, bus, &raw);
        }
        if (err != 0U || raw > 4095U) {
            err_n++;
            continue;
        }
        if (!uw.initialized) {
            enc_unwrap_init(&uw, raw);
        } else {
            (void)enc_unwrap_update(&uw, raw);
        }
        const int32_t c = uw.total_counts;
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
        ok_n++;
        /* Emit sparse/reference raw samples for host replay. */
        if (is_ref || (i < 16U) || ((i & 63U) == 0U)) {
            emit_json(mc, "sample", req_id, mc->trial_id, mc_axis_name(axis),
                      "\"test\":\"%s\",\"idx\":%lu,\"raw\":%u,\"unwrapped\":%ld",
                      is_ref ? "reference" : "static",
                      (unsigned long)i, (unsigned)raw, (long)c);
        }
    }

    const int32_t p2p = (ok_n > 0U) ? (max_c - min_c) : 0;
    const int32_t mean_x100 = (ok_n > 0U) ? (int32_t)((sum_c * 100LL) / (int64_t)ok_n) : 0;
    emit_json(mc, "result", req_id, mc->trial_id, mc_axis_name(axis),
              "\"ok\":%s,\"test\":\"%s\",\"n\":%lu,\"errors\":%lu,"
              "\"first_raw\":%u,\"mean_rel_x100\":%ld,\"p2p_cnt\":%ld,"
              "\"status\":%u,\"agc\":%u,\"mag\":%u",
              (ok_n == target_n && err_n == 0U && !uw.suspect_alias) ? "true" : "false",
              is_ref ? "reference" : "static",
              (unsigned long)ok_n, (unsigned long)err_n,
              (unsigned)uw.last_raw, (long)mean_x100, (long)p2p,
              (unsigned)status, (unsigned)agc, (unsigned)mag);
}

static uint8_t sf_code_to_factor(uint8_t sf_code) {
    switch (sf_code & 3U) {
        case 0U: return 16U;
        case 1U: return 8U;
        case 2U: return 4U;
        default: return 2U;
    }
}

static bool start_enc_noise(mc_core_t *mc, uint32_t req_id, mc_axis_t axis) {
    const uint8_t bus = mc_axis_to_enc_bus(mc->session_mode, axis);
    if (bus == 0xFFU) {
        emit_reject(mc, req_id, mc_axis_name(axis), "no_shaft_feedback");
        return false;
    }
    if (mc->opt_noise_hold) {
        if (mc->state != MC_STATE_ARMED || axis != mc->armed_axis) {
            emit_reject(mc, req_id, mc_axis_name(axis), "axis_not_armed");
            return false;
        }
    } else {
        if (mc->state != MC_STATE_DISARMED) {
            emit_reject(mc, req_id, mc_axis_name(axis), "must_be_disarmed");
            return false;
        }
    }
    uint16_t conf_rb = ENCODER_CONF_EXPECTED;
    if (mc->hal.enc_set_slow_filter != NULL) {
        if (!mc->hal.enc_set_slow_filter(mc->hal.ctx, bus, mc->opt_noise_sf, &conf_rb)) {
            if (mc->hal.enc_set_slow_filter != NULL && mc->opt_noise_sf != 3U) {
                uint16_t restore_rb = 0U;
                (void)mc->hal.enc_set_slow_filter(mc->hal.ctx, bus, 3U, &restore_rb);
            }
            emit_reject(mc, req_id, mc_axis_name(axis), "conf_write_failed");
            return false;
        }
    }
    uint8_t status = 0U, agc = 0U;
    uint16_t mag = 0U;
    if (mc->hal.enc_read_health != NULL) {
        (void)mc->hal.enc_read_health(mc->hal.ctx, bus, &status, &agc, &mag);
    }
    if ((status & 0x38U) != 0x20U) {
        if (mc->hal.enc_set_slow_filter != NULL && mc->opt_noise_sf != 3U) {
            uint16_t dummy = 0U;
            (void)mc->hal.enc_set_slow_filter(mc->hal.ctx, bus, 3U, &dummy);
        }
        emit_json(mc, "result", req_id, mc->trial_id, mc_axis_name(axis),
                  "\"ok\":false,\"test\":\"enc_noise\",\"reason\":\"bad_magnet_status\","
                  "\"status\":%u,\"agc\":%u,\"mag\":%u",
                  (unsigned)status, (unsigned)agc, (unsigned)mag);
        return false;
    }

    mc->trial_id++;
    mc->active_request_id = req_id;
    mc->active_test = MC_TEST_ENC_NOISE;
    mc->noise_return_state = mc->state;
    mc->state = MC_STATE_RUNNING;
    mc->noise_axis = axis;
    mc->noise_bus = bus;
    mc->noise_sf_code = mc->opt_noise_sf & 3U;
    mc->noise_conf_rb = conf_rb;
    mc->noise_rate_hz = mc->opt_noise_rate_hz;
    mc->noise_period_us = 1000000U / mc->opt_noise_rate_hz;
    mc->noise_next_us = now_us(mc);
    mc->noise_target_n = mc->opt_noise_n;
    mc->noise_idx = 0U;
    mc->noise_ok_n = 0U;
    mc->noise_err_n = 0U;
    mc->noise_overruns = 0U;
    memset(&mc->noise_uw, 0, sizeof(mc->noise_uw));
    mc->noise_min_c = INT32_MAX;
    mc->noise_max_c = INT32_MIN;
    mc->noise_sum_c = 0;
    mc->noise_status = status;
    mc->noise_agc = agc;
    mc->noise_mag = mag;
    return true;
}

static void poll_active_enc_noise(mc_core_t *mc) {
    const uint32_t t_now = now_us(mc);
    if ((int32_t)(t_now - mc->noise_next_us) < 0) {
        return;
    }
    if ((uint32_t)(t_now - mc->noise_next_us) > mc->noise_period_us) {
        mc->noise_overruns++;
        mc->noise_next_us = t_now + mc->noise_period_us;
    } else {
        mc->noise_next_us += mc->noise_period_us;
    }

    uint16_t raw = 0U;
    uint8_t err = 1U;
    if (mc->hal.enc_read_raw != NULL) {
        err = mc->hal.enc_read_raw(mc->hal.ctx, mc->noise_bus, &raw);
    }
    const uint32_t idx = mc->noise_idx++;
    if (err == 0U && raw <= 4095U) {
        if (!mc->noise_uw.initialized) {
            enc_unwrap_init(&mc->noise_uw, raw);
        } else {
            (void)enc_unwrap_update(&mc->noise_uw, raw);
        }
        const int32_t c = mc->noise_uw.total_counts;
        if (c < mc->noise_min_c) mc->noise_min_c = c;
        if (c > mc->noise_max_c) mc->noise_max_c = c;
        mc->noise_sum_c += c;
        mc->noise_ok_n++;
        emit_json(mc, "sample", mc->active_request_id, mc->trial_id,
                  mc_axis_name(mc->noise_axis),
                  "\"test\":\"enc_noise\",\"idx\":%lu,\"raw\":%u,\"unwrapped\":%ld",
                  (unsigned long)idx, (unsigned)raw, (long)c);
        if (mc->state != MC_STATE_RUNNING) {
            /* TX ring overflow latched MC_FAULT_RING_OVERFLOW; restore SF=2x without clearing fault. */
            if (mc->hal.enc_set_slow_filter != NULL && mc->noise_sf_code != 3U) {
                uint16_t rb = 0U;
                (void)mc->hal.enc_set_slow_filter(mc->hal.ctx, mc->noise_bus, 3U, &rb);
            }
            return;
        }
    } else {
        mc->noise_err_n++;
    }

    if (mc->noise_idx >= mc->noise_target_n) {
        /* Restore default SF=2x (CONF=0x0300) if modified. */
        bool restored = true;
        if (mc->hal.enc_set_slow_filter != NULL && mc->noise_sf_code != 3U) {
            uint16_t rb = 0U;
            restored = mc->hal.enc_set_slow_filter(mc->hal.ctx, mc->noise_bus, 3U, &rb);
        }
        const int32_t p2p = (mc->noise_ok_n > 0U)
                                ? (mc->noise_max_c - mc->noise_min_c)
                                : 0;
        const int32_t mean_x100 = (mc->noise_ok_n > 0U)
                                      ? (int32_t)((mc->noise_sum_c * 100LL) /
                                                  (int64_t)mc->noise_ok_n)
                                      : 0;
        const bool ok = (mc->noise_ok_n == mc->noise_target_n) &&
                        (mc->noise_err_n == 0U) &&
                        !mc->noise_uw.suspect_alias && restored;
        emit_json(mc, "result", mc->active_request_id, mc->trial_id,
                  mc_axis_name(mc->noise_axis),
                  "\"ok\":%s,\"test\":\"enc_noise\",\"sf\":%u,\"conf\":%u,"
                  "\"rate_hz\":%lu,\"hold\":%s,\"n\":%lu,\"errors\":%lu,"
                  "\"overruns\":%lu,\"mean_rel_x100\":%ld,\"p2p_cnt\":%ld,"
                  "\"status\":%u,\"agc\":%u,\"mag\":%u",
                  ok ? "true" : "false",
                  (unsigned)sf_code_to_factor(mc->noise_sf_code),
                  (unsigned)mc->noise_conf_rb,
                  (unsigned long)mc->noise_rate_hz,
                  mc->opt_noise_hold ? "true" : "false",
                  (unsigned long)mc->noise_ok_n,
                  (unsigned long)mc->noise_err_n,
                  (unsigned long)mc->noise_overruns,
                  (long)mean_x100, (long)p2p,
                  (unsigned)mc->noise_status, (unsigned)mc->noise_agc,
                  (unsigned)mc->noise_mag);
        if (mc->state == MC_STATE_RUNNING) {
            mc->state = mc->noise_return_state;
            mc->active_test = MC_TEST_NONE;
        }
    }
}

static void run_spi_rate(mc_core_t *mc, uint32_t req_id, const char *target) {
    static const uint32_t k_divs[] = {256U, 128U, 64U, 32U, 16U};
    const bool all = (strcmp(target, "all") == 0);
    uint8_t chips[MC_MAX_AXES] = {0U, 1U, 2U};
    uint8_t n_chips = MC_MAX_AXES;
    if (!all) {
        mc_axis_t ax = MC_AXIS_NONE;
        if (!parse_axis(target, &ax)) {
            emit_reject(mc, req_id, target, "unknown_axis");
            return;
        }
        chips[0] = (uint8_t)ax;
        n_chips = 1U;
    }

    /* ENNs stay HIGH during read-only electrical tests. */
    if (mc->hal.disable_all_drivers != NULL) {
        mc->hal.disable_all_drivers(mc->hal.ctx);
    }

    /* Establish baseline IOIN.VERSION == 0x40 and stable GCONF/CHOPCONF at /32. */
    uint32_t sck_hz = 0U;
    if (mc->hal.tmc_spi_set_prescaler == NULL || mc->hal.tmc_spi_read_reg == NULL ||
        !mc->hal.tmc_spi_set_prescaler(mc->hal.ctx, 32U, &sck_hz)) {
        emit_reject(mc, req_id, target, "spi_init_failed");
        return;
    }
    uint32_t ref_gconf[MC_MAX_AXES] = {0U, 0U, 0U};
    uint32_t ref_chop[MC_MAX_AXES] = {0U, 0U, 0U};
    for (uint8_t i = 0U; i < n_chips; ++i) {
        uint32_t ioin = 0U;
        const uint8_t id = chips[i];
        if (!mc->hal.tmc_spi_read_reg(mc->hal.ctx, id, 0x04U, &ioin) ||
            ((ioin >> 24U) & 0xFFU) != 0x40U ||
            !mc->hal.tmc_spi_read_reg(mc->hal.ctx, id, 0x00U, &ref_gconf[id]) ||
            !mc->hal.tmc_spi_read_reg(mc->hal.ctx, id, 0x6CU, &ref_chop[id])) {
            emit_reject(mc, req_id, target, "baseline_ioin_failed");
            return;
        }
    }

    mc->trial_id++;
    for (size_t di = 0U; di < sizeof(k_divs) / sizeof(k_divs[0]); ++di) {
        const uint32_t div = k_divs[di];
        if (!mc->hal.tmc_spi_set_prescaler(mc->hal.ctx, div, &sck_hz) ||
            sck_hz == 0U || sck_hz > 10000000U) {
            emit_json(mc, "sample", req_id, mc->trial_id, target,
                      "\"test\":\"spi_rate\",\"div\":%lu,\"sck_hz\":%lu,"
                      "\"skipped\":true,\"reason\":\"clock_out_of_spec\"",
                      (unsigned long)div, (unsigned long)sck_hz);
            continue;
        }
        for (uint8_t w = 0U; w < MC_RATE_WINDOWS; ++w) {
            uint32_t ok_cnt = 0U, mismatch_cnt = 0U, err_cnt = 0U;
            uint32_t min_lat = UINT32_MAX, max_lat = 0U;
            const uint32_t t_win = now_us(mc);
            for (uint32_t op = 0U; op < MC_RATE_WINDOW_OPS; ++op) {
                const uint8_t id = chips[op % n_chips];
                const uint8_t sel = (uint8_t)(op % 3U);
                const uint8_t reg = (sel == 0U) ? 0x04U : ((sel == 1U) ? 0x00U : 0x6CU);
                uint32_t val = 0U;
                const uint32_t t0 = now_us(mc);
                const bool rd_ok = mc->hal.tmc_spi_read_reg(mc->hal.ctx, id, reg, &val);
                const uint32_t lat = now_us(mc) - t0;
                if (lat < min_lat) min_lat = lat;
                if (lat > max_lat) max_lat = lat;
                if (!rd_ok) {
                    err_cnt++;
                } else {
                    bool match = false;
                    if (sel == 0U) {
                        match = (((val >> 24U) & 0xFFU) == 0x40U);
                    } else if (sel == 1U) {
                        match = (val == ref_gconf[id]);
                    } else {
                        match = (val == ref_chop[id]);
                    }
                    if (match) {
                        ok_cnt++;
                    } else {
                        mismatch_cnt++;
                    }
                }
            }
            const uint32_t elapsed_us = now_us(mc) - t_win;
            emit_json(mc, "sample", req_id, mc->trial_id, target,
                      "\"test\":\"spi_rate\",\"div\":%lu,\"sck_hz\":%lu,"
                      "\"window\":%u,\"chips\":%u,\"attempts\":%lu,\"ok\":%lu,"
                      "\"mismatches\":%lu,\"errors\":%lu,\"elapsed_us\":%lu,"
                      "\"min_lat_us\":%lu,\"max_lat_us\":%lu",
                      (unsigned long)div, (unsigned long)sck_hz,
                      (unsigned)w, (unsigned)n_chips,
                      (unsigned long)MC_RATE_WINDOW_OPS, (unsigned long)ok_cnt,
                      (unsigned long)mismatch_cnt, (unsigned long)err_cnt,
                      (unsigned long)elapsed_us,
                      (unsigned long)(min_lat == UINT32_MAX ? 0U : min_lat),
                      (unsigned long)max_lat);
        }
    }

    /* Always restore /32 and verify readbacks before exiting. */
    bool restored = mc->hal.tmc_spi_set_prescaler(mc->hal.ctx, 32U, &sck_hz);
    if (restored) {
        for (uint8_t i = 0U; i < n_chips; ++i) {
            uint32_t ioin = 0U;
            if (!mc->hal.tmc_spi_read_reg(mc->hal.ctx, chips[i], 0x04U, &ioin) ||
                ((ioin >> 24U) & 0xFFU) != 0x40U) {
                restored = false;
                break;
            }
        }
    }
    if (!restored) {
        mc_core_latch_fault(mc, MC_FAULT_RESTORE_FAILED, true);
        return;
    }

    emit_json(mc, "result", req_id, mc->trial_id, target,
              "\"ok\":true,\"test\":\"spi_rate\",\"restored_div\":32");
}

/* ------------------------------------------------------------------------- */
/* Supervised motion start & active polling                                  */
/* ------------------------------------------------------------------------- */

static bool start_supervised_leg(mc_core_t *mc, uint32_t req_id,
                                 mc_test_kind_t kind, int32_t target,
                                 uint32_t speed, uint32_t accel,
                                 bool check_cruise_geom) {
    const mc_axis_t ax = mc->armed_axis;
    const mc_axis_policy_t *p = &mc->policy[ax];
    if (target < p->win_neg_steps || target > p->win_pos_steps) {
        emit_reject(mc, req_id, mc_axis_name(ax), "target_out_of_bounds");
        return false;
    }
    if (speed == 0U || speed > p->max_speed_steps_s ||
        accel == 0U || accel > p->max_accel_steps_s2) {
        emit_reject(mc, req_id, mc_axis_name(ax), "rate_exceeds_cap");
        return false;
    }
    const uint32_t vmin = (p->min_speed_steps_s > 0U && p->min_speed_steps_s < speed)
                              ? p->min_speed_steps_s
                              : ((speed < 200U) ? speed : 200U);
    if (mc->hal.step_validate_rate != NULL &&
        !mc->hal.step_validate_rate(mc->hal.ctx, (float)speed, (float)vmin)) {
        emit_reject(mc, req_id, mc_axis_name(ax), "timing_ceiling_rejected");
        return false;
    }
    const int32_t cur_pos = (mc->hal.step_position != NULL)
                                ? mc->hal.step_position(mc->hal.ctx)
                                : 0;
    if (check_cruise_geom &&
        !mc_validate_stroke_geometry(cur_pos, target, speed, vmin, accel, false)) {
        emit_reject(mc, req_id, mc_axis_name(ax), "window_too_short");
        return false;
    }

    /* Fresh encoder health + raw check before issuing any STEP edge. */
    const uint8_t bus = mc_axis_to_enc_bus(mc->session_mode, ax);
    uint8_t st = 0U, agc = 0U;
    uint16_t mag = 0U, raw = 0U;
    if (mc->hal.enc_read_health == NULL || mc->hal.enc_read_raw == NULL ||
        mc->hal.enc_read_health(mc->hal.ctx, bus, &st, &agc, &mag) != 0U ||
        mc->hal.enc_read_raw(mc->hal.ctx, bus, &raw) != 0U) {
        mc_core_latch_fault(mc, MC_FAULT_ENCODER_INVALID, false);
        emit_reject(mc, req_id, mc_axis_name(ax), "invalid_feedback");
        return false;
    }
    const uint32_t t0 = now_us(mc);
    encoder_sample_t *es = &mc->enc_state[ax];
    encoder_sampler_health(es, true, st, agc, t0);
    if (!encoder_sampler_accept(es, raw, t0, speed)) {
        mc_core_latch_fault(mc, MC_FAULT_ENCODER_INVALID, false);
        emit_reject(mc, req_id, mc_axis_name(ax), "invalid_feedback");
        return false;
    }

    mc->active_test = kind;
    mc->active_request_id = req_id;
    mc->state = MC_STATE_RUNNING;
    mc->move_start_steps = cur_pos;
    mc->move_target_steps = target;
    mc->move_start_counts = es->unwrap.total_counts;
    mc->move_max_abs_err_usteps = 0;
    mc->move_start_us = t0;
    mc->move_last_enc_us = t0;
    mc->move_last_health_us = t0;
    mc->move_last_drv_us = t0;
    mc->move_last_adc_us = t0;
    mc->move_follow_strikes = 0U;
    mc->move_settling = false;
    mc->move_settle_samples = 0U;
    mc->move_settle_min_cnt = INT32_MAX;
    mc->move_settle_max_cnt = INT32_MIN;
    mc->move_settle_sum_cnt = 0;

    if (mc->hal.step_reset_stats != NULL) {
        mc->hal.step_reset_stats(mc->hal.ctx);
    }
    if (mc->hal.step_set_vmin != NULL) {
        mc->hal.step_set_vmin(mc->hal.ctx, (float)vmin);
    }
    if (mc->hal.step_set_inhibit != NULL) {
        mc->hal.step_set_inhibit(mc->hal.ctx, false);
    }
    if (mc->hal.step_move_to != NULL) {
        mc->hal.step_move_to(mc->hal.ctx, target, (float)speed, (float)accel);
    }
    return true;
}

static void finish_motion_leg(mc_core_t *mc) {
    const mc_axis_t ax = mc->armed_axis;
    const mc_axis_policy_t *p = &mc->policy[ax];
    const int32_t end_pos = (mc->hal.step_position != NULL)
                                ? mc->hal.step_position(mc->hal.ctx)
                                : mc->move_start_steps;
    const int32_t emitted = end_pos - mc->move_start_steps;
    const int32_t settled_cnt = (mc->move_settle_samples > 0U)
                                    ? (int32_t)(mc->move_settle_sum_cnt /
                                                (int64_t)mc->move_settle_samples)
                                    : mc->enc_state[ax].unwrap.total_counts;
    const int32_t delta_cnt = settled_cnt - mc->move_start_counts;
    const int32_t enc_usteps = counts_to_usteps_signed(delta_cnt, p->encoder_sign);
    const int32_t residual_usteps = enc_usteps - emitted;
    const int32_t settle_spread = (mc->move_settle_samples > 0U)
                                      ? (mc->move_settle_max_cnt - mc->move_settle_min_cnt)
                                      : 0;

    mc_step_stats_t st;
    memset(&st, 0, sizeof(st));
    if (mc->hal.step_get_stats != NULL) {
        mc->hal.step_get_stats(mc->hal.ctx, &st);
    }

    if (st.lateness_fault) {
        mc_core_latch_fault(mc, MC_FAULT_TIMING_LATENESS, false);
        return;
    }

    const uint32_t abs_res = (residual_usteps >= 0) ? (uint32_t)residual_usteps
                                                    : (uint32_t)(-residual_usteps);
    const bool persistent_loss = (settle_spread <= (int32_t)p->static_noise_cnt) &&
                                 (abs_res > p->follow_band_usteps);

    emit_json(mc, "result", mc->active_request_id, mc->trial_id, mc_axis_name(ax),
              "\"ok\":%s,\"test\":\"%s\",\"start_steps\":%ld,\"target_steps\":%ld,"
              "\"end_steps\":%ld,\"emitted_edges\":%ld,\"delta_counts\":%ld,"
              "\"residual_usteps\":%ld,\"max_moving_err_usteps\":%ld,"
              "\"settle_spread_cnt\":%ld,\"follow_band_usteps\":%lu,"
              "\"first_edge_us\":%lu,\"last_edge_us\":%lu,\"min_dt_us\":%lu,"
              "\"max_dt_us\":%lu,\"max_lateness_us\":%lu,"
              "\"ramp_up_end_us\":%lu,\"ramp_up_end_edges\":%lu,"
              "\"cruise_end_us\":%lu,\"cruise_end_edges\":%lu,"
              "\"persistent_step_loss\":%s,\"stress_legs\":%lu,"
              "\"stress_extra_reads\":%lu,\"stress_extra_errs\":%lu",
              persistent_loss ? "false" : "true",
              (mc->active_test == MC_TEST_JOG)
                  ? "jog"
                  : ((mc->active_test == MC_TEST_STRESS) ? "stress" : "move"),
              (long)mc->move_start_steps, (long)mc->move_target_steps,
              (long)end_pos, (long)emitted, (long)delta_cnt,
              (long)residual_usteps, (long)mc->move_max_abs_err_usteps,
              (long)settle_spread, (unsigned long)p->follow_band_usteps,
              (unsigned long)st.first_edge_us, (unsigned long)st.last_edge_us,
              (unsigned long)st.min_dt_us, (unsigned long)st.max_dt_us,
              (unsigned long)st.max_lateness_us,
              (unsigned long)st.ramp_up_end_us, (unsigned long)st.ramp_up_end_edges,
              (unsigned long)st.cruise_end_us, (unsigned long)st.cruise_end_edges,
              persistent_loss ? "true" : "false",
              (unsigned long)mc->stress_legs_done,
              (unsigned long)mc->stress_extra_reads,
              (unsigned long)mc->stress_extra_errs);

    if (persistent_loss) {
        mc_core_latch_fault(mc, MC_FAULT_FOLLOWING_ERROR, false);
        return;
    }

    if (mc->active_test == MC_TEST_STRESS) {
        mc->stress_legs_done++;
        const uint32_t t_now = now_us(mc);
        if ((int32_t)(mc->stress_end_us - t_now) > 0) {
            mc->stress_to_b = !mc->stress_to_b;
            const int32_t next_tgt = mc->stress_to_b ? mc->stress_leg_b : mc->stress_leg_a;
            if (start_supervised_leg(mc, mc->active_request_id, MC_TEST_STRESS,
                                     next_tgt, mc->point_speed_steps_s,
                                     mc->point_accel_steps_s2, false)) {
                return;
            }
            return;
        }
    }

    /* Normal completed leg holds position (stays ARMED). */
    mc->state = MC_STATE_ARMED;
    mc->active_test = MC_TEST_NONE;
}

static void poll_active_motion(mc_core_t *mc) {
    const mc_axis_t ax = mc->armed_axis;
    const mc_axis_policy_t *p = &mc->policy[ax];
    const uint8_t bus = mc_axis_to_enc_bus(mc->session_mode, ax);
    const uint32_t t_now = now_us(mc);

    if (mc->hal.step_take_limit_hit != NULL && mc->hal.step_take_limit_hit(mc->hal.ctx)) {
        mc_core_latch_fault(mc, MC_FAULT_SOFT_LIMIT, false);
        return;
    }

    /* Health poll at 20 ms target. */
    if ((t_now - mc->move_last_health_us) >= MC_ENC_HEALTH_US) {
        uint8_t st = 0U, agc = 0U;
        uint16_t mag = 0U;
        const bool h_ok = (mc->hal.enc_read_health != NULL) &&
                          (mc->hal.enc_read_health(mc->hal.ctx, bus, &st, &agc, &mag) == 0U);
        encoder_sampler_health(&mc->enc_state[ax], h_ok, st, agc, now_us(mc));
        mc->move_last_health_us = t_now;
        if (!h_ok || (st & 0x38U) != 0x20U) {
            mc_core_latch_fault(mc, MC_FAULT_ENCODER_INVALID, false);
            return;
        }
    }

    /* Raw encoder poll at 2 ms target, bracketed with emitted counts. */
    if ((t_now - mc->move_last_enc_us) >= MC_ENC_POLL_US) {
        const int32_t pos_before = (mc->hal.step_position != NULL)
                                       ? mc->hal.step_position(mc->hal.ctx)
                                       : mc->move_start_steps;
        uint16_t raw = 0U;
        const uint32_t tr0 = now_us(mc);
        const bool r_ok = (mc->hal.enc_read_raw != NULL) &&
                          (mc->hal.enc_read_raw(mc->hal.ctx, bus, &raw) == 0U);
        const uint32_t tr1 = now_us(mc);
        const int32_t pos_after = (mc->hal.step_position != NULL)
                                      ? mc->hal.step_position(mc->hal.ctx)
                                      : pos_before;
        mc->move_last_enc_us = t_now;

        if (!r_ok || !encoder_sampler_accept(&mc->enc_state[ax], raw, tr1,
                                             mc->point_speed_steps_s)) {
            mc_core_latch_fault(mc, MC_FAULT_ENCODER_INVALID, false);
            return;
        }
        if (!encoder_sampler_fresh(&mc->enc_state[ax], tr1)) {
            mc_core_latch_fault(mc, MC_FAULT_ENCODER_STALE, false);
            return;
        }

        const int32_t emitted_mid = ((pos_before + pos_after) / 2) - mc->move_start_steps;
        const int32_t delta_cnt = mc->enc_state[ax].unwrap.total_counts - mc->move_start_counts;
        const int32_t enc_usteps = counts_to_usteps_signed(delta_cnt, p->encoder_sign);
        const int32_t res = enc_usteps - emitted_mid;
        const int32_t abs_res = (res >= 0) ? res : -res;
        if (abs_res > mc->move_max_abs_err_usteps) {
            mc->move_max_abs_err_usteps = abs_res;
        }

        /* Emit time-series sample for velocity/accel least-squares fits. */
        emit_json(mc, "sample", mc->active_request_id, mc->trial_id, mc_axis_name(ax),
                  "\"t0_us\":%lu,\"t1_us\":%lu,\"pos_before\":%ld,\"pos_after\":%ld,"
                  "\"raw\":%u,\"counts\":%ld,\"residual_usteps\":%ld",
                  (unsigned long)tr0, (unsigned long)tr1,
                  (long)pos_before, (long)pos_after,
                  (unsigned)raw, (long)mc->enc_state[ax].unwrap.total_counts,
                  (long)res);

        /* Moving following-error guard: base band + travel over read bracket + 2 ms filter. */
        const uint32_t bracket_steps = (uint32_t)((pos_after >= pos_before)
                                                      ? (pos_after - pos_before)
                                                      : (pos_before - pos_after));
        const uint32_t filter_steps = (uint32_t)(((uint64_t)mc->point_speed_steps_s *
                                                  (uint64_t)MC_FILTER_ALLOW_US) / 1000000ULL);
        const uint32_t moving_band = p->follow_band_usteps + bracket_steps + filter_steps;

        if (!mc->move_settling) {
            /* Wrong-sign gross motion guard. */
            const int32_t cmd_dir = (mc->move_target_steps >= mc->move_start_steps) ? 1 : -1;
            if ((cmd_dir > 0 && enc_usteps < -(int32_t)moving_band) ||
                (cmd_dir < 0 && enc_usteps > (int32_t)moving_band)) {
                mc_core_latch_fault(mc, MC_FAULT_WRONG_DIRECTION, false);
                return;
            }
            if ((uint32_t)abs_res > moving_band) {
                mc->move_follow_strikes++;
                if (mc->move_follow_strikes >= 3U) {
                    mc_core_latch_fault(mc, MC_FAULT_FOLLOWING_ERROR, false);
                    return;
                }
            } else {
                mc->move_follow_strikes = 0U;
            }
        } else if ((t_now - mc->move_settle_start_us) >= MC_SETTLE_WAIT_US) {
            const int32_t c = mc->enc_state[ax].unwrap.total_counts;
            if (c < mc->move_settle_min_cnt) mc->move_settle_min_cnt = c;
            if (c > mc->move_settle_max_cnt) mc->move_settle_max_cnt = c;
            mc->move_settle_sum_cnt += c;
            mc->move_settle_samples++;
            if (mc->move_settle_samples >= MC_SETTLE_COUNT) {
                finish_motion_leg(mc);
                return;
            }
        }
    }

    /* Driver DRV_STATUS poll every 50 ms. */
    if ((t_now - mc->move_last_drv_us) >= MC_DRV_POLL_US) {
        mc->move_last_drv_us = t_now;
        if (mc->hal.tmc_poll_status != NULL) {
            mc_driver_snap_t ds;
            memset(&ds, 0, sizeof(ds));
            if (!mc->hal.tmc_poll_status(mc->hal.ctx, (uint8_t)ax, &ds) || !ds.ok) {
                mc_core_latch_fault(mc, MC_FAULT_DRIVER_SPI, true);
                return;
            }
            emit_json(mc, "driver", mc->active_request_id, mc->trial_id, mc_axis_name(ax),
                      "\"drv_status\":%lu,\"tstep\":%lu,\"sg4\":%u,\"mscnt\":%u,"
                      "\"cs_actual\":%u,\"stealth\":%s,\"standstill\":%s,"
                      "\"otpw\":%s,\"open_load\":%s",
                      (unsigned long)ds.drv_status, (unsigned long)ds.tstep,
                      (unsigned)ds.sg4, (unsigned)ds.mscnt, (unsigned)ds.cs_actual,
                      ds.stealth ? "true" : "false",
                      ds.standstill ? "true" : "false",
                      ds.otpw ? "true" : "false",
                      ds.open_load ? "true" : "false");
            if (ds.reset_flag) {
                mc_core_latch_fault(mc, MC_FAULT_DRIVER_RESET, true);
                return;
            }
            if (ds.ot || ds.otpw || ds.short_fault || ds.drv_err || ds.open_load) {
                mc_core_latch_fault(mc, MC_FAULT_DRIVER_FAULT, true);
                return;
            }
        }
    }

    /* Driver ADC (temp + VM) poll every 250 ms. */
    if ((t_now - mc->move_last_adc_us) >= MC_ADC_POLL_US) {
        mc->move_last_adc_us = t_now;
        if (mc->hal.tmc_poll_adc != NULL) {
            mc_driver_adc_t adc;
            memset(&adc, 0, sizeof(adc));
            if (!mc->hal.tmc_poll_adc(mc->hal.ctx, (uint8_t)ax, &adc) || !adc.ok) {
                mc_core_latch_fault(mc, MC_FAULT_DRIVER_SPI, true);
                return;
            }
            emit_json(mc, "driver", mc->active_request_id, mc->trial_id, mc_axis_name(ax),
                      "\"temp_c10\":%d,\"vm_mv\":%lu",
                      (int)adc.temp_c10, (unsigned long)adc.vm_mv);
            if (adc.temp_c10 >= p->temp_cutoff_c10) {
                mc_core_latch_fault(mc, MC_FAULT_TEMP_LIMIT, true);
                return;
            }
            if (adc.vm_mv < p->vm_min_mv || adc.vm_mv > p->vm_max_mv) {
                mc_core_latch_fault(mc, MC_FAULT_VM_OUT_OF_RANGE, true);
                return;
            }
        }
    }

    /* Optional extra round-robin SPI read during stress mode (bounded: 1 per poll). */
    if (mc->active_test == MC_TEST_STRESS && mc->opt_stress_extra_spi &&
        mc->hal.tmc_spi_read_reg != NULL) {
        uint32_t ioin = 0U;
        const uint8_t chip = mc->move_spi_rr;
        mc->move_spi_rr = (uint8_t)((mc->move_spi_rr + 1U) % MC_MAX_AXES);
        mc->stress_extra_reads++;
        if (!mc->hal.tmc_spi_read_reg(mc->hal.ctx, chip, 0x04U, &ioin) ||
            ((ioin >> 24U) & 0xFFU) != 0x40U) {
            mc->stress_extra_errs++;
        }
    }

    /* Check stepgen completion -> enter 100 ms standstill settle. */
    if (!mc->move_settling && mc->hal.step_busy != NULL &&
        !mc->hal.step_busy(mc->hal.ctx)) {
        mc->move_settling = true;
        mc->move_settle_start_us = t_now;
    }
}

/* ------------------------------------------------------------------------- */
/* Command dispatcher                                                        */
/* ------------------------------------------------------------------------- */

static void handle_set(mc_core_t *mc, uint32_t req_id, int argc, char *argv[]) {
    if (mc->state != MC_STATE_DISARMED) {
        emit_reject(mc, req_id, "none", "must_be_disarmed");
        return;
    }
    if (argc < 4) {
        emit_reject(mc, req_id, "none", "missing_tokens");
        return;
    }
    const char *key = argv[2];
    const char *val = argv[3];

    if (strcmp(key, "session") == 0) {
        if (strcmp(val, "xz") == 0) {
            mc->session_mode = MC_SESSION_XZ;
        } else if (strcmp(val, "theta_fixture") == 0) {
            mc->session_mode = MC_SESSION_THETA_FIXTURE;
        } else {
            emit_reject(mc, req_id, "none", "invalid_session");
            return;
        }
        emit_ack(mc, req_id, "none", "set");
        return;
    }
    if (strcmp(key, "enc_mode") == 0) {
        if (strcmp(val, "raw") == 0) mc->opt_enc_mode = 0U;
        else if (strcmp(val, "sample") == 0) mc->opt_enc_mode = 1U;
        else { emit_reject(mc, req_id, "none", "invalid_enc_mode"); return; }
        emit_ack(mc, req_id, "none", "set");
        return;
    }
    if (strcmp(key, "enc_fast") == 0) {
        uint32_t v = 0U;
        if (!cmd_parse_u32(val, &v) || v > 1U) { emit_reject(mc, req_id, "none", "invalid_value"); return; }
        mc->opt_enc_fast = (v == 1U);
        emit_ack(mc, req_id, "none", "set");
        return;
    }
    if (strcmp(key, "enc_fast_ok") == 0) {
        uint32_t v = 0U;
        if (!cmd_parse_u32(val, &v) || v > 1U) { emit_reject(mc, req_id, "none", "invalid_value"); return; }
        mc->opt_enc_fast_ok = (v == 1U);
        emit_ack(mc, req_id, "none", "set");
        return;
    }
    if (strcmp(key, "stress_s") == 0) {
        uint32_t v = 0U;
        if (!cmd_parse_u32(val, &v) || v == 0U || v > 600U) { emit_reject(mc, req_id, "none", "invalid_value"); return; }
        mc->opt_stress_s = v;
        emit_ack(mc, req_id, "none", "set");
        return;
    }
    if (strcmp(key, "stress_extra_spi") == 0) {
        uint32_t v = 0U;
        if (!cmd_parse_u32(val, &v) || v > 1U) { emit_reject(mc, req_id, "none", "invalid_value"); return; }
        mc->opt_stress_extra_spi = (v == 1U);
        emit_ack(mc, req_id, "none", "set");
        return;
    }
    if (strcmp(key, "noise_sf") == 0) {
        uint32_t v = 0U;
        if (!cmd_parse_u32(val, &v)) { emit_reject(mc, req_id, "none", "invalid_value"); return; }
        if (v == 16U) mc->opt_noise_sf = 0U;
        else if (v == 8U) mc->opt_noise_sf = 1U;
        else if (v == 4U) mc->opt_noise_sf = 2U;
        else if (v == 2U) mc->opt_noise_sf = 3U;
        else { emit_reject(mc, req_id, "none", "invalid_noise_sf"); return; }
        emit_ack(mc, req_id, "none", "set");
        return;
    }
    if (strcmp(key, "noise_rate_hz") == 0) {
        uint32_t v = 0U;
        if (!cmd_parse_u32(val, &v) || v < 50U || v > 1000U) {
            emit_reject(mc, req_id, "none", "invalid_noise_rate");
            return;
        }
        mc->opt_noise_rate_hz = v;
        emit_ack(mc, req_id, "none", "set");
        return;
    }
    if (strcmp(key, "noise_n") == 0) {
        uint32_t v = 0U;
        if (!cmd_parse_u32(val, &v) || v < 64U || v > 4096U) {
            emit_reject(mc, req_id, "none", "invalid_noise_n");
            return;
        }
        mc->opt_noise_n = v;
        emit_ack(mc, req_id, "none", "set");
        return;
    }
    if (strcmp(key, "noise_hold") == 0) {
        uint32_t v = 0U;
        if (!cmd_parse_u32(val, &v) || v > 1U) {
            emit_reject(mc, req_id, "none", "invalid_value");
            return;
        }
        mc->opt_noise_hold = (v == 1U);
        emit_ack(mc, req_id, "none", "set");
        return;
    }

    /* Per-axis keys: SET <key> <axis> <value> */
    if (argc < 5) {
        emit_reject(mc, req_id, "none", "missing_axis_or_value");
        return;
    }
    mc_axis_t ax = MC_AXIS_NONE;
    if (!parse_axis(argv[3], &ax)) {
        emit_reject(mc, req_id, argv[3], "unknown_axis");
        return;
    }
    mc_axis_policy_t *p = &mc->policy[ax];
    const char *aval = argv[4];
    uint32_t u32 = 0U;
    int32_t i32 = 0;

    if (strcmp(key, "run_ma") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 < 100U || u32 > 2000U) {
            emit_reject(mc, req_id, mc_axis_name(ax), "invalid_current");
            return;
        }
        if (ax == MC_AXIS_THETA && u32 > MC_THETA_MAX_MA) {
            u32 = MC_THETA_MAX_MA;
        }
        p->run_current_ma = (uint16_t)u32;
    } else if (strcmp(key, "hold_pct") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 < 10U || u32 > 100U) {
            emit_reject(mc, req_id, mc_axis_name(ax), "invalid_hold_pct");
            return;
        }
        p->hold_pct = (uint8_t)u32;
    } else if (strcmp(key, "vm_min_mv") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 < 6000U) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_vm"); return; }
        p->vm_min_mv = u32;
    } else if (strcmp(key, "vm_max_mv") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 > 36000U) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_vm"); return; }
        p->vm_max_mv = u32;
    } else if (strcmp(key, "temp_cutoff_c10") == 0) {
        if (!parse_i32(aval, &i32) || i32 < 400 || i32 > 1350) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_temp"); return; }
        p->temp_cutoff_c10 = (int16_t)i32;
    } else if (strcmp(key, "win_neg") == 0) {
        if (!parse_i32(aval, &i32) || i32 > 0) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_win_neg"); return; }
        p->win_neg_steps = i32;
    } else if (strcmp(key, "win_pos") == 0) {
        if (!parse_i32(aval, &i32) || i32 < 0) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_win_pos"); return; }
        p->win_pos_steps = i32;
    } else if (strcmp(key, "max_speed") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 == 0U) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_speed"); return; }
        p->max_speed_steps_s = u32;
    } else if (strcmp(key, "min_speed") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 == 0U) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_speed"); return; }
        p->min_speed_steps_s = u32;
    } else if (strcmp(key, "max_accel") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 == 0U) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_accel"); return; }
        p->max_accel_steps_s2 = u32;
    } else if (strcmp(key, "enc_sign") == 0) {
        if (!parse_i32(aval, &i32) || (i32 != 1 && i32 != -1)) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_sign"); return; }
        p->encoder_sign = (int8_t)i32;
    } else if (strcmp(key, "static_noise") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 == 0U || u32 > 200U) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_noise"); return; }
        p->static_noise_cnt = (uint16_t)u32;
    } else if (strcmp(key, "follow_band") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 < MC_FOLLOW_MIN_USTEPS) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_band"); return; }
        p->follow_band_usteps = u32;
    } else if (strcmp(key, "commissioned") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 > 1U) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_flag"); return; }
        p->commissioned = (u32 == 1U);
    } else if (strcmp(key, "datum_ok") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 > 1U) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_flag"); return; }
        p->datum_confirmed = (u32 == 1U);
    } else if (strcmp(key, "safety_ok") == 0) {
        if (!cmd_parse_u32(aval, &u32) || u32 > 1U) { emit_reject(mc, req_id, mc_axis_name(ax), "invalid_flag"); return; }
        p->safety_confirmed = (u32 == 1U);
    } else {
        emit_reject(mc, req_id, mc_axis_name(ax), "unknown_key");
        return;
    }

    emit_ack(mc, req_id, mc_axis_name(ax), "set");
}

static void handle_arm(mc_core_t *mc, uint32_t req_id, mc_axis_t ax) {
    if (mc->state != MC_STATE_DISARMED) {
        emit_reject(mc, req_id, mc_axis_name(ax), "not_disarmed");
        return;
    }
    if (mc->b1_latched ||
        (mc->hal.b1_asserted != NULL && mc->hal.b1_asserted(mc->hal.ctx))) {
        emit_reject(mc, req_id, mc_axis_name(ax), "b1_asserted");
        return;
    }
    if (!mc_axis_motion_allowed(mc->session_mode, ax)) {
        emit_reject(mc, req_id, mc_axis_name(ax), "no_shaft_feedback");
        return;
    }
    mc_axis_policy_t *p = &mc->policy[ax];
    if (!p->commissioned || !p->datum_confirmed || !p->safety_confirmed) {
        emit_reject(mc, req_id, mc_axis_name(ax), "prerequisites_missing");
        return;
    }
    if (p->run_current_ma == 0U || p->hold_pct == 0U ||
        p->max_speed_steps_s == 0U || p->max_accel_steps_s2 == 0U ||
        p->win_neg_steps >= p->win_pos_steps) {
        emit_reject(mc, req_id, mc_axis_name(ax), "limits_incomplete");
        return;
    }
    if (p->static_noise_cnt > 41U) { /* > 32 microsteps (41 counts) fails static gate */
        emit_reject(mc, req_id, mc_axis_name(ax), "static_noise_too_high");
        return;
    }

    /* Fresh live encoder check before enabling driver stage. */
    const uint8_t bus = mc_axis_to_enc_bus(mc->session_mode, ax);
    uint8_t st = 0U, agc = 0U;
    uint16_t mag = 0U, raw = 0U;
    if (mc->hal.enc_read_health == NULL || mc->hal.enc_read_raw == NULL ||
        mc->hal.enc_read_health(mc->hal.ctx, bus, &st, &agc, &mag) != 0U ||
        (st & 0x38U) != 0x20U ||
        mc->hal.enc_read_raw(mc->hal.ctx, bus, &raw) != 0U) {
        emit_reject(mc, req_id, mc_axis_name(ax), "invalid_feedback");
        return;
    }
    const uint32_t t0 = now_us(mc);
    encoder_sampler_config(&mc->enc_state[ax], true, ENCODER_CONF_EXPECTED);
    encoder_sampler_health(&mc->enc_state[ax], true, st, agc, t0);
    if (!encoder_sampler_accept(&mc->enc_state[ax], raw, t0, 1U)) {
        emit_reject(mc, req_id, mc_axis_name(ax), "invalid_feedback");
        return;
    }

    /* Configure TMC2240 with ENNs HIGH first, then bind stepgen slot, set limits, enable stage. */
    if (mc->hal.disable_all_drivers != NULL) {
        mc->hal.disable_all_drivers(mc->hal.ctx);
    }
    if (mc->hal.tmc_configure == NULL || !mc->hal.tmc_configure(mc->hal.ctx, (uint8_t)ax, p)) {
        emit_reject(mc, req_id, mc_axis_name(ax), "driver_config_failed");
        return;
    }
    if (mc->hal.step_bind_axis == NULL || !mc->hal.step_bind_axis(mc->hal.ctx, (uint8_t)ax)) {
        emit_reject(mc, req_id, mc_axis_name(ax), "step_bind_failed");
        return;
    }
    if (mc->hal.step_set_position != NULL && !mc->hal.step_set_position(mc->hal.ctx, 0)) {
        emit_reject(mc, req_id, mc_axis_name(ax), "step_not_idle");
        return;
    }
    if (mc->hal.step_set_limits != NULL) {
        mc->hal.step_set_limits(mc->hal.ctx, p->win_neg_steps, p->win_pos_steps);
    }
    if (mc->hal.set_driver_enable != NULL) {
        mc->hal.set_driver_enable(mc->hal.ctx, (uint8_t)ax, true);
    }

    mc->armed_axis = ax;
    mc->state = MC_STATE_ARMED;
    mc->fault = MC_FAULT_NONE;
    mc->last_keepalive_us = t0;
    mc->lease_active = true;
    mc->point_valid = false;

    emit_json(mc, "config", req_id, mc->trial_id, mc_axis_name(ax),
              "\"armed\":true,\"applied_ma\":%u,\"range\":%u,\"gs\":%u,"
              "\"irun\":%u,\"ihold\":%u,\"gstat_as_found\":%u,"
              "\"win_neg\":%ld,\"win_pos\":%ld,\"max_speed\":%lu,\"max_accel\":%lu",
              (unsigned)p->applied_current_ma, (unsigned)p->applied_range,
              (unsigned)p->applied_gs, (unsigned)p->applied_irun,
              (unsigned)p->applied_ihold, (unsigned)p->gstat_as_found,
              (long)p->win_neg_steps, (long)p->win_pos_steps,
              (unsigned long)p->max_speed_steps_s,
              (unsigned long)p->max_accel_steps_s2);
    emit_ack(mc, req_id, mc_axis_name(ax), "arm");
}

static void handle_disarm(mc_core_t *mc, uint32_t req_id) {
    if (mc->hal.step_set_inhibit != NULL) {
        mc->hal.step_set_inhibit(mc->hal.ctx, true);
    }
    if (mc->hal.step_halt != NULL) {
        mc->hal.step_halt(mc->hal.ctx);
    }
    if (mc->hal.disable_all_drivers != NULL) {
        mc->hal.disable_all_drivers(mc->hal.ctx);
    }
    if (mc->armed_axis < MC_MAX_AXES) {
        mc->policy[mc->armed_axis].datum_confirmed = false;
    }
    mc->armed_axis = MC_AXIS_NONE;
    mc->state = MC_STATE_DISARMED;
    enc_noise_abort_restore(mc);
    mc->active_test = MC_TEST_NONE;
    mc->fault = MC_FAULT_NONE;
    mc->b1_latched = false;
    mc->lease_active = false;
    mc->point_valid = false;
    emit_ack(mc, req_id, "none", "disarm");
}

static void handle_line(mc_core_t *mc, char *line) {
    char *argv[CMD_ARGV_MAX];
    const int argc = cmd_tokenize(line, argv, CMD_ARGV_MAX);
    if (argc <= 0) {
        return;
    }

    /* Heartbeat and STOP are always actionable without request_id ordering. */
    if (strcmp(argv[0], "keep") == 0) {
        mc->last_keepalive_us = now_us(mc);
        return;
    }
    if (strcmp(argv[0], "stop") == 0) {
        mc_core_latch_fault(mc, MC_FAULT_HOST_STOP, true);
        emit_json(mc, "result", 0U, mc->trial_id, "none",
                  "\"ok\":true,\"status\":\"stopped\"");
        return;
    }
    /* Also allow "<req_id> KEEP" or "<req_id> STOP". */
    if (argc >= 2 && strcmp(argv[1], "keep") == 0) {
        mc->last_keepalive_us = now_us(mc);
        return;
    }
    if (argc >= 2 && strcmp(argv[1], "stop") == 0) {
        uint32_t rid = 0U;
        (void)cmd_parse_u32(argv[0], &rid);
        mc_core_latch_fault(mc, MC_FAULT_HOST_STOP, true);
        emit_json(mc, "result", rid, mc->trial_id, "none",
                  "\"ok\":true,\"status\":\"stopped\"");
        return;
    }

    /* All other commands require a strictly increasing request_id prefix. */
    if (argc < 2) {
        emit_reject(mc, 0U, "none", "missing_request_id");
        return;
    }
    uint32_t req_id = 0U;
    if (!cmd_parse_u32(argv[0], &req_id) || req_id == 0U) {
        emit_reject(mc, 0U, "none", "invalid_request_id");
        return;
    }
    if (mc->have_request_id && req_id <= mc->last_request_id) {
        emit_reject(mc, req_id, "none", "duplicate_or_old_request_id");
        return;
    }
    mc->have_request_id = true;
    mc->last_request_id = req_id;

    const char *cmd = argv[1];
    if (strcmp(cmd, "hello") == 0) {
        mc_board_info_t bi;
        memset(&bi, 0, sizeof(bi));
        if (mc->hal.get_board_info != NULL) {
            mc->hal.get_board_info(mc->hal.ctx, &bi);
        }
        emit_json(mc, "hello", req_id, mc->trial_id, "none",
                  "\"profile\":\"%s\",\"schema\":%u,\"sysclk_hz\":%lu,"
                  "\"tim2_hz\":%lu,\"tim5_hz\":%lu,\"spi_kernel_hz\":%lu,"
                  "\"spi_div\":%lu,\"i2c1_timingr\":%lu,\"i2c4_timingr\":%lu,"
                  "\"drv_ver\":[%u,%u,%u],\"drv_gstat\":[%u,%u,%u]",
                  MC_PROFILE_NAME, (unsigned)MC_SCHEMA_VERSION,
                  (unsigned long)bi.sysclk_hz, (unsigned long)bi.tim2_hz,
                  (unsigned long)bi.tim5_hz, (unsigned long)bi.spi_kernel_hz,
                  (unsigned long)bi.spi_prescaler,
                  (unsigned long)bi.i2c1_timingr, (unsigned long)bi.i2c4_timingr,
                  (unsigned)bi.drv_version[0], (unsigned)bi.drv_version[1],
                  (unsigned)bi.drv_version[2], (unsigned)bi.drv_gstat[0],
                  (unsigned)bi.drv_gstat[1], (unsigned)bi.drv_gstat[2]);
        return;
    }
    if (strcmp(cmd, "set") == 0) {
        handle_set(mc, req_id, argc, argv);
        return;
    }
    if (strcmp(cmd, "disarm") == 0) {
        handle_disarm(mc, req_id);
        return;
    }
    if (strcmp(cmd, "arm") == 0) {
        mc_axis_t ax = MC_AXIS_NONE;
        if (argc != 3 || !parse_axis(argv[2], &ax)) {
            emit_reject(mc, req_id, (argc >= 3) ? argv[2] : "none", "invalid_axis");
            return;
        }
        handle_arm(mc, req_id, ax);
        return;
    }
    if (strcmp(cmd, "point") == 0) {
        if (mc->state != MC_STATE_ARMED) {
            emit_reject(mc, req_id, mc_axis_name(mc->armed_axis), "not_armed_idle");
            return;
        }
        int32_t tgt = 0;
        uint32_t spd = 0U, acc = 0U;
        if (argc != 5 || !parse_i32(argv[2], &tgt) ||
            !cmd_parse_u32(argv[3], &spd) || !cmd_parse_u32(argv[4], &acc)) {
            emit_reject(mc, req_id, mc_axis_name(mc->armed_axis), "invalid_point_args");
            return;
        }
        const mc_axis_policy_t *p = &mc->policy[mc->armed_axis];
        if (tgt < p->win_neg_steps || tgt > p->win_pos_steps ||
            spd == 0U || spd > p->max_speed_steps_s ||
            acc == 0U || acc > p->max_accel_steps_s2) {
            emit_reject(mc, req_id, mc_axis_name(mc->armed_axis), "point_out_of_caps");
            return;
        }
        mc->point_target_steps = tgt;
        mc->point_speed_steps_s = spd;
        mc->point_accel_steps_s2 = acc;
        mc->point_valid = true;
        emit_ack(mc, req_id, mc_axis_name(mc->armed_axis), "point");
        return;
    }
    if (strcmp(cmd, "run") == 0) {
        if (argc != 4) {
            emit_reject(mc, req_id, "none", "invalid_run_args");
            return;
        }
        if (mc->state == MC_STATE_RUNNING || mc->state == MC_STATE_FAULT) {
            emit_reject(mc, req_id, argv[3], "busy_or_faulted");
            return;
        }
        const char *test = argv[2];
        const char *target = argv[3];

        /* Passive tests require DISARMED. */
        if (strcmp(test, "commission") == 0) {
            mc_axis_t ax = MC_AXIS_NONE;
            if (mc->state != MC_STATE_DISARMED || !parse_axis(target, &ax)) {
                emit_reject(mc, req_id, target, "invalid_state_or_axis");
                return;
            }
            run_commission(mc, req_id, ax);
            return;
        }
        if (strcmp(test, "encoder_rate") == 0) {
            if (mc->state != MC_STATE_DISARMED) {
                emit_reject(mc, req_id, target, "must_be_disarmed");
                return;
            }
            run_encoder_rate(mc, req_id, target);
            return;
        }
        if (strcmp(test, "static") == 0 || strcmp(test, "reference") == 0) {
            mc_axis_t ax = MC_AXIS_NONE;
            if (mc->state != MC_STATE_DISARMED || !parse_axis(target, &ax)) {
                emit_reject(mc, req_id, target, "invalid_state_or_axis");
                return;
            }
            run_static_or_reference(mc, req_id, ax, strcmp(test, "reference") == 0);
            return;
        }
        if (strcmp(test, "spi_rate") == 0) {
            if (mc->state != MC_STATE_DISARMED) {
                emit_reject(mc, req_id, target, "must_be_disarmed");
                return;
            }
            run_spi_rate(mc, req_id, target);
            return;
        }
        if (strcmp(test, "enc_noise") == 0) {
            mc_axis_t ax = MC_AXIS_NONE;
            if (!parse_axis(target, &ax)) {
                emit_reject(mc, req_id, target, "unknown_axis");
                return;
            }
            if (start_enc_noise(mc, req_id, ax)) {
                emit_ack(mc, req_id, target, "run_enc_noise");
            }
            return;
        }

        /* Motion tests require ARMED on the exact single axis (never both/all). */
        mc_axis_t ax = MC_AXIS_NONE;
        if (mc->state != MC_STATE_ARMED || !parse_axis(target, &ax) ||
            ax != mc->armed_axis) {
            emit_reject(mc, req_id, target, "axis_not_armed");
            return;
        }
        if (!mc->point_valid) {
            emit_reject(mc, req_id, target, "point_not_set");
            return;
        }
        if (strcmp(test, "jog") == 0) {
            mc->trial_id++;
            if (start_supervised_leg(mc, req_id, MC_TEST_JOG,
                                     mc->point_target_steps,
                                     mc->point_speed_steps_s,
                                     mc->point_accel_steps_s2, false)) {
                emit_ack(mc, req_id, target, "run_jog");
            }
            return;
        }
        if (strcmp(test, "move") == 0) {
            mc->trial_id++;
            if (start_supervised_leg(mc, req_id, MC_TEST_MOVE,
                                     mc->point_target_steps,
                                     mc->point_speed_steps_s,
                                     mc->point_accel_steps_s2, true)) {
                emit_ack(mc, req_id, target, "run_move");
            }
            return;
        }
        if (strcmp(test, "stress") == 0) {
            mc->trial_id++;
            const int32_t cur = (mc->hal.step_position != NULL)
                                    ? mc->hal.step_position(mc->hal.ctx)
                                    : 0;
            mc->stress_leg_a = cur;
            mc->stress_leg_b = mc->point_target_steps;
            mc->stress_to_b = true;
            mc->stress_legs_done = 0U;
            mc->stress_extra_reads = 0U;
            mc->stress_extra_errs = 0U;
            mc->stress_end_us = now_us(mc) + mc->opt_stress_s * 1000000U;
            if (start_supervised_leg(mc, req_id, MC_TEST_STRESS,
                                     mc->stress_leg_b,
                                     mc->point_speed_steps_s,
                                     mc->point_accel_steps_s2, true)) {
                emit_ack(mc, req_id, target, "run_stress");
            }
            return;
        }

        emit_reject(mc, req_id, target, "unknown_test");
        return;
    }

    emit_reject(mc, req_id, "none", "unknown_command");
}

void mc_core_rx_bytes(mc_core_t *mc, const uint8_t *buf, uint32_t len) {
    if (mc == NULL || buf == NULL) {
        return;
    }
    for (uint32_t i = 0U; i < len; ++i) {
        if (cmd_linebuf_push(&mc->rx_line, (char)buf[i])) {
            handle_line(mc, mc->rx_line.buf);
            cmd_linebuf_init(&mc->rx_line);
        }
    }
}

void mc_core_poll(mc_core_t *mc) {
    if (mc == NULL) {
        return;
    }
    tx_drain(mc);

    if (mc->state == MC_STATE_ARMED || mc->state == MC_STATE_RUNNING) {
        if (mc->b1_latched ||
            (mc->hal.b1_asserted != NULL && mc->hal.b1_asserted(mc->hal.ctx))) {
            mc_core_latch_fault(mc, MC_FAULT_B1_STOP, true);
            return;
        }
        if (mc->hal.usb_mounted != NULL && !mc->hal.usb_mounted(mc->hal.ctx)) {
            mc_core_latch_fault(mc, MC_FAULT_USB_DISCONNECT, true);
            return;
        }
        if (mc->lease_active) {
            const uint32_t elapsed = now_us(mc) - mc->last_keepalive_us;
            if (elapsed > MC_LEASE_TIMEOUT_US) {
                mc_core_latch_fault(mc, MC_FAULT_LEASE_EXPIRED, false);
                return;
            }
        }
    }

    if (mc->state == MC_STATE_RUNNING) {
        if (mc->active_test == MC_TEST_ENC_NOISE) {
            poll_active_enc_noise(mc);
        } else {
            poll_active_motion(mc);
        }
    }

    tx_drain(mc);
}
