/**
 * @file    axis_ctrl.c
 * @brief   Per-axis homing / cycling state machine. All hardware access goes
 *          through axis_hw.h so this file also runs in the host simulator.
 */
#include "axis_ctrl.h"

#include "axis_hw.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HIST_LEN          32U     /* encoder history kept during SEEK2 (640 ms) */
#define HIST_PERIOD_MS    20U
#define CAL_GUARD_MS      400U    /* calibration sample must predate the stall */
#define CAL_MIN_REVS      0.5f    /* minimum calibration travel in motor revs */
#define ENC_RATIO_TOL     0.35f   /* accepted deviation of |counts/step| */
#define ENC_TRAVEL_TOL    0.10f   /* encoder vs commanded travel agreement */
#define STUCK_WINDOW_MS   100U
#define STUCK_FRACTION    0.25f
#define STUCK_MIN_COUNTS  64.0f
#define STUCK_WINDOWS     2U
#define ENC_LAG_S         0.001f  /* nominal AS5600 filter + sampling latency */
#define ENC_LAG_TOL_S     0.001f  /* uncertainty of that latency */
#define FOLLOW_HITS       3U
#define BOUNCE_TOL_MM     5.0f

typedef enum {
    SEEK_RUN = 0,
    SEEK_STALL,
    SEEK_LIMIT,
    SEEK_TIMEOUT,
    SEEK_ERROR,
    SEEK_ENDED
} seek_result_t;

typedef struct {
    uint32_t t;
    int32_t  p;
    int32_t  e;
} sample_t;

typedef struct {
    axis_cfg_t        cfg;
    axis_status_t     st;
    bool              initialized;
    bool              driver_ready;
    bool              profile_valid;
    bool              profile_homing;
    uint32_t          t_state;
    axis_state_t      stop_to;
    /* seek / stall detection */
    int8_t            seek_dir;
    bool              cruising;
    uint32_t          t_cruise;
    uint32_t          t_seek;
    uint32_t          t_sg_poll;
    bool              thr_checked;
    bool              thr_raised;
    stall_det_t       sd;
    axis_hw_status_t  last_hw_err;
    bool              stuck_armed;
    bool              enc_tracking;  /* encoder followed the steps during this seek */
    uint32_t          stuck_t;
    int32_t           stuck_p;
    int32_t           stuck_e;
    uint8_t           stuck_hits;
    /* homing measurements */
    int8_t            end1_dir;
    int32_t           p_end[2];
    int32_t           e_end[2];
    bool              e_end_ok[2];
    sample_t          cal_start;
    bool              cal_start_ok;
    int32_t           other_e_start;
    bool              other_e_start_ok;
    sample_t          hist[HIST_LEN];
    uint8_t           hist_head;
    uint8_t           hist_count;
    uint32_t          t_hist;
    /* encoder */
    axis_enc_sample_t enc;
    bool              enc_valid;
    bool              enc_new;
    uint32_t          enc_last_seq;
    bool              enc_cal;
    uint32_t          cal_epoch;
    float             enc_ratio;
    int32_t           enc_zero;
    uint8_t           follow_hits;
    /* cycle */
    int8_t            cycle_dir;
} axis_t;

static axis_t g_axes[AXIS_CTRL_MAX_AXES];

/* ------------------------------------------------------------------------ */
/* Helpers                                                                  */
/* ------------------------------------------------------------------------ */

static axis_t *get(uint8_t ax)
{
    return ((ax < AXIS_CTRL_MAX_AXES) && g_axes[ax].initialized) ? &g_axes[ax] : NULL;
}

static int32_t round_i32(float x)
{
    if (x >= 2147483520.0f) {
        return INT32_MAX;
    }
    if (x <= -2147483520.0f) {
        return INT32_MIN;
    }
    return (x >= 0.0f) ? (int32_t)(x + 0.5f) : (int32_t)(x - 0.5f);
}

static int32_t sat_add(int32_t a, int32_t b)
{
    const int64_t r = (int64_t)a + (int64_t)b;

    if (r > INT32_MAX) {
        return INT32_MAX;
    }
    if (r < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)r;
}

static int32_t abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t mm_to_steps(const axis_t *a, float mm)
{
    return round_i32(mm * a->cfg.steps_per_mm);
}

static float mm_to_stepsf(const axis_t *a, float mm)
{
    return mm * a->cfg.steps_per_mm;
}

static float to_mm(const axis_t *a, int32_t steps)
{
    return (float)steps / a->cfg.steps_per_mm;
}

static void enter(axis_t *a, axis_state_t s)
{
    a->st.state = s;
    a->t_state = axis_hw_now_ms();
}

static void go_fault(uint8_t ax, axis_fault_t f)
{
    axis_t *a = &g_axes[ax];
    const axis_state_t from = a->st.state;

    axis_hw_halt(ax);
    axis_hw_driver_enable(ax, false);
    a->st.homed = false;
    a->st.cycling = false;
    a->enc_cal = false;
    a->st.enc_ok = false;
    a->st.fault = f;
    a->st.fault_count++;
    enter(a, AX_ST_FAULT);
    axis_hw_log("[FAULT] %c: %s (during %s at %.2f mm) - driver disabled",
                a->cfg.name, axis_ctrl_fault_name(f), axis_ctrl_state_name(from),
                (double)to_mm(a, axis_hw_position(ax)));
}

static bool ensure_profile(uint8_t ax, bool homing)
{
    axis_t *a = &g_axes[ax];

    if (a->profile_valid && (a->profile_homing == homing)) {
        return true;
    }
    if (axis_hw_apply_profile(ax, homing ? AXIS_PROFILE_HOMING : AXIS_PROFILE_RUN,
                              &a->cfg) != AXIS_HW_OK) {
        a->profile_valid = false;
        go_fault(ax, AX_FAULT_SPI);
        return false;
    }
    a->profile_valid = true;
    a->profile_homing = homing;
    a->thr_raised = false;
    a->st.sg_thr = a->cfg.tmc.sg4_thrs_home;
    return true;
}

static void set_run_limits(uint8_t ax)
{
    axis_t *a = &g_axes[ax];

    axis_hw_set_limits(ax, 0, a->st.travel_steps);
    (void)axis_hw_take_limit_hit(ax);
}

static void refresh_encoder(uint8_t ax)
{
    axis_t *a = &g_axes[ax];
    axis_enc_sample_t s;

    a->enc_new = false;
    if (a->cfg.enc_enable && axis_hw_encoder(ax, &s)) {
        if (!a->enc_valid || (s.seq != a->enc_last_seq)) {
            a->enc_new = true;
            a->enc_last_seq = s.seq;
        }
        a->enc = s;
        a->enc_valid = true;
    } else {
        a->enc_valid = false;
    }
    if (a->enc_cal && a->enc_valid && (a->enc.epoch != a->cal_epoch)) {
        a->enc_cal = false;
        a->st.enc_ok = false;
        axis_hw_log("[ENC] %c: encoder continuity lost - position check off until next homing",
                    a->cfg.name);
    }
}

static int32_t enc_position(const axis_t *a)
{
    return round_i32((float)(a->enc.counts - a->enc_zero) / a->enc_ratio);
}

/* ------------------------------------------------------------------------ */
/* Seek (constant speed run until StallGuard4 detects the end stop)         */
/* ------------------------------------------------------------------------ */

static void seek_arm(uint8_t ax, int8_t dir)
{
    axis_t *a = &g_axes[ax];
    const uint32_t now = axis_hw_now_ms();

    a->seek_dir = dir;
    a->cruising = false;
    a->t_seek = now;
    a->t_sg_poll = now;
    a->thr_checked = false;
    stall_det_reset(&a->sd, &a->cfg.stall);
    a->stuck_armed = false;
    a->enc_tracking = false;
    a->stuck_hits = 0U;
    a->hist_head = 0U;
    a->hist_count = 0U;
    a->t_hist = now;
}

static bool restore_static_threshold(uint8_t ax)
{
    axis_t *a = &g_axes[ax];

    if (!a->thr_raised) {
        return true;
    }
    if (axis_hw_set_sg_threshold(ax, a->cfg.tmc.sg4_thrs_home) != AXIS_HW_OK) {
        go_fault(ax, AX_FAULT_SPI);
        return false;
    }
    a->thr_raised = false;
    a->st.sg_thr = a->cfg.tmc.sg4_thrs_home;
    return true;
}

static bool seek_begin(uint8_t ax, int8_t dir, int32_t span)
{
    axis_t *a = &g_axes[ax];
    const int32_t pos = axis_hw_position(ax);

    if (!restore_static_threshold(ax)) {
        return false;
    }
    axis_hw_set_limits(ax, (dir < 0) ? sat_add(pos, -span) : INT32_MIN,
                       (dir > 0) ? sat_add(pos, span) : INT32_MAX);
    (void)axis_hw_take_limit_hit(ax);
    seek_arm(ax, dir);
    axis_hw_run(ax, dir, mm_to_stepsf(a, a->cfg.home_speed_mm_s),
                mm_to_stepsf(a, a->cfg.home_accel_mm_s2));
    return true;
}

/* Encoder based backup: commanded steps advance but the shaft does not. The
 * test is sign-agnostic so it also works before the encoder is calibrated.
 * It only fires after the encoder was seen following the steps during this
 * seek, so a missing magnet or a swapped bus cannot fake a stall (an axis
 * that starts against the stop is caught by the SG4 checks instead). */
static bool stuck_check(uint8_t ax, uint32_t now)
{
    axis_t *a = &g_axes[ax];
    int32_t dp;
    int32_t de;
    float expected;

    if (!a->cfg.enc_stall_assist || !a->enc_valid || !a->cruising) {
        a->stuck_armed = false;
        return false;
    }
    if (!a->stuck_armed) {
        a->stuck_armed = true;
        a->stuck_t = now;
        a->stuck_p = a->enc.pos;
        a->stuck_e = a->enc.counts;
        a->stuck_hits = 0U;
        return false;
    }
    if ((now - a->stuck_t) < STUCK_WINDOW_MS) {
        return false;
    }
    dp = a->enc.pos - a->stuck_p;
    de = a->enc.counts - a->stuck_e;
    a->stuck_t = now;
    a->stuck_p = a->enc.pos;
    a->stuck_e = a->enc.counts;
    expected = fabsf((float)dp) * a->cfg.enc_counts_per_step;
    if (expected < STUCK_MIN_COUNTS) {
        a->stuck_hits = 0U;
        return false;
    }
    if (fabsf((float)de) >= 0.5f * expected) {
        a->enc_tracking = true;
        a->stuck_hits = 0U;
        return false;
    }
    if (a->enc_tracking && (fabsf((float)de) < STUCK_FRACTION * expected)) {
        a->stuck_hits++;
        return a->stuck_hits >= STUCK_WINDOWS;
    }
    a->stuck_hits = 0U;
    return false;
}

static seek_result_t seek_poll(uint8_t ax, uint32_t now, stall_source_t *src)
{
    axis_t *a = &g_axes[ax];

    if (axis_hw_take_limit_hit(ax)) {
        return SEEK_LIMIT;
    }
    if (!axis_hw_busy(ax)) {
        return SEEK_ENDED;
    }
    if ((now - a->t_seek) > a->cfg.seek_timeout_ms) {
        return SEEK_TIMEOUT;
    }
    if (!a->cruising && axis_hw_at_cruise(ax)) {
        a->cruising = true;
        a->t_cruise = now;
    }
    if ((now - a->t_sg_poll) >= a->cfg.sg_poll_ms) {
        uint16_t sg = 0U;
        bool flag = false;
        axis_hw_status_t hs;
        stall_source_t s;

        a->t_sg_poll = now;
        hs = axis_hw_read_sg(ax, &sg, &flag);
        if (hs != AXIS_HW_OK) {
            a->last_hw_err = hs;
            return SEEK_ERROR;
        }
        a->st.sg = sg;
        a->st.sg_flag = flag;
        s = stall_det_update(&a->sd, a->cruising ? (int32_t)(now - a->t_cruise) : -1,
                             sg, flag);
        a->st.sg_baseline = a->sd.baseline;
        if (a->sd.baseline_valid && !a->thr_checked) {
            /* Let the chip comparator (SPI status / DIAG) use the calibrated
             * threshold as well. */
            const uint8_t thr = stall_det_hw_threshold(&a->sd);
            a->thr_checked = true;
            if (thr > a->cfg.tmc.sg4_thrs_home) {
                hs = axis_hw_set_sg_threshold(ax, thr);
                if (hs != AXIS_HW_OK) {
                    a->last_hw_err = hs;
                    return SEEK_ERROR;
                }
                a->thr_raised = true;
                a->st.sg_thr = thr;
            }
        }
        if (s != STALL_NONE) {
            *src = s;
            return SEEK_STALL;
        }
    }
    if (stuck_check(ax, now)) {
        *src = STALL_ENCODER;
        return SEEK_STALL;
    }
    return SEEK_RUN;
}

static void seek_fail(uint8_t ax, seek_result_t r)
{
    axis_t *a = &g_axes[ax];

    switch (r) {
    case SEEK_LIMIT:
        axis_hw_log("[SEEK] %c: no stall detected within the travel limit - check SG4 tuning",
                    a->cfg.name);
        go_fault(ax, AX_FAULT_NO_STALL);
        break;
    case SEEK_TIMEOUT:
        go_fault(ax, AX_FAULT_TIMEOUT);
        break;
    case SEEK_ERROR:
        go_fault(ax, (a->last_hw_err == AXIS_HW_ERR_DRIVER) ? AX_FAULT_DRIVER : AX_FAULT_SPI);
        break;
    default:
        go_fault(ax, AX_FAULT_INTERNAL);
        break;
    }
}

static void log_stall(uint8_t ax, const char *tag, stall_source_t src)
{
    axis_t *a = &g_axes[ax];

    axis_hw_log("[%s] %c: stall at %s end, %.2f mm (%s, SG4 %u, baseline %.0f, SG4_THRS %u)",
                tag, a->cfg.name, (a->seek_dir < 0) ? "MIN" : "MAX",
                (double)to_mm(a, axis_hw_position(ax)), stall_source_name(src),
                (unsigned)a->sd.last_sg, (double)a->sd.baseline, (unsigned)a->st.sg_thr);
}

/* ------------------------------------------------------------------------ */
/* Homing                                                                   */
/* ------------------------------------------------------------------------ */

static void hist_push(axis_t *a, uint32_t now)
{
    if (!a->enc_valid || ((now - a->t_hist) < HIST_PERIOD_MS)) {
        return;
    }
    a->t_hist = now;
    a->hist[a->hist_head].t = now;
    a->hist[a->hist_head].p = a->enc.pos;
    a->hist[a->hist_head].e = a->enc.counts;
    a->hist_head = (uint8_t)((a->hist_head + 1U) % HIST_LEN);
    if (a->hist_count < HIST_LEN) {
        a->hist_count++;
    }
}

static void try_calibrate(uint8_t ax, uint32_t now)
{
    axis_t *a = &g_axes[ax];
    const sample_t *best = NULL;
    const float nominal = a->cfg.enc_counts_per_step;
    float ratio;
    int32_t dp;
    int32_t de;
    uint8_t i;

    a->enc_cal = false;
    if (!a->cfg.enc_enable) {
        return;
    }
    if (!a->cal_start_ok) {
        axis_hw_log("[ENC] %c: no encoder reading - running open loop", a->cfg.name);
        return;
    }
    for (i = 0U; i < a->hist_count; i++) {
        const sample_t *s = &a->hist[(a->hist_head + HIST_LEN - 1U - i) % HIST_LEN];
        if ((now - s->t) >= CAL_GUARD_MS) {
            best = s;
            break;
        }
    }
    if (best == NULL) {
        axis_hw_log("[ENC] %c: traverse too short to calibrate the encoder", a->cfg.name);
        return;
    }
    dp = best->p - a->cal_start.p;
    de = best->e - a->cal_start.e;
    if (fabsf((float)dp) * nominal < CAL_MIN_REVS * 4096.0f) {
        axis_hw_log("[ENC] %c: traverse too short to calibrate the encoder", a->cfg.name);
        return;
    }
    ratio = (float)de / (float)dp;
    if ((fabsf(ratio) < nominal * (1.0f - ENC_RATIO_TOL)) ||
        (fabsf(ratio) > nominal * (1.0f + ENC_RATIO_TOL))) {
        axis_enc_sample_t other;
        const uint8_t oax = (uint8_t)(ax ^ 1U);
        axis_hw_log("[ENC] %c: measured %.3f counts/step, expected %.3f - encoder ignored "
                    "(magnet, mounting or bus mapping?)",
                    a->cfg.name, (double)ratio, (double)nominal);
        if ((oax < AXIS_CTRL_MAX_AXES) && a->other_e_start_ok && axis_hw_encoder(oax, &other)) {
            const float moved = fabsf((float)(other.counts - a->other_e_start));
            if (moved > 0.5f * fabsf((float)dp) * nominal) {
                axis_hw_log("[ENC] %c: the encoder assigned to the other axis turned instead - "
                            "swap the encoder buses in app_config.c", a->cfg.name);
            }
        }
        return;
    }
    a->enc_ratio = ratio;
    a->enc_cal = true;
    a->cal_epoch = a->enc.epoch;
    axis_hw_log("[ENC] %c: calibrated %.4f counts/step (nominal %.4f)", a->cfg.name,
                (double)ratio, (double)nominal);
}

static void finish_homing(uint8_t ax)
{
    axis_t *a = &g_axes[ax];
    const int idx_min = (a->end1_dir < 0) ? 0 : 1;
    const int idx_max = 1 - idx_min;
    const int32_t p_min = a->p_end[idx_min];
    const int32_t travel_cmd = a->p_end[idx_max] - p_min;
    int32_t travel = travel_cmd;
    int32_t new_pos = axis_hw_position(ax) - p_min;
    const int32_t margin = mm_to_steps(a, a->cfg.cycle_margin_mm);

    a->st.home_travel_cmd = travel_cmd;
    a->st.home_travel_enc = 0;
    if (a->enc_cal && a->e_end_ok[0] && a->e_end_ok[1] && a->enc_valid &&
        (a->enc.epoch == a->cal_epoch)) {
        const int32_t e_min = a->e_end[idx_min];
        const float t_enc = (float)(a->e_end[idx_max] - e_min) / a->enc_ratio;

        a->st.home_travel_enc = round_i32(t_enc);
        if ((t_enc > 0.0f) &&
            (fabsf(t_enc - (float)travel_cmd) <= ENC_TRAVEL_TOL * (float)travel_cmd)) {
            /* The encoder sees where the rotor really stopped, so the stall
             * overshoot (lost microsteps at the stop) does not bias the frame. */
            travel = round_i32(t_enc);
            a->enc_zero = e_min;
            new_pos = round_i32((float)(a->enc.counts - e_min) / a->enc_ratio);
        } else {
            axis_hw_log("[ENC] %c: encoder travel %.2f mm disagrees with steps %.2f mm - ignored",
                        a->cfg.name, (double)(t_enc / a->cfg.steps_per_mm),
                        (double)to_mm(a, travel_cmd));
            a->enc_cal = false;
        }
    } else {
        a->enc_cal = false;
    }

    a->st.travel_steps = travel;
    a->st.soft_min = margin;
    a->st.soft_max = travel - margin;
    if ((a->st.soft_max - a->st.soft_min) < mm_to_steps(a, 1.0f)) {
        axis_hw_log("[HOME] %c: travel %.2f mm leaves no room inside the %.1f mm margins",
                    a->cfg.name, (double)to_mm(a, travel), (double)a->cfg.cycle_margin_mm);
        go_fault(ax, AX_FAULT_SHORT_TRAVEL);
        return;
    }
    if (!axis_hw_set_position(ax, new_pos)) {
        go_fault(ax, AX_FAULT_INTERNAL);
        return;
    }
    if (!ensure_profile(ax, false)) {
        return;
    }
    set_run_limits(ax);
    a->follow_hits = 0U;
    a->st.follow_err = 0;
    a->st.follow_err_peak = 0;
    a->st.enc_ok = a->enc_cal;
    a->st.enc_ratio = a->enc_ratio;
    a->st.homed = true;
    a->st.home_count++;
    enter(a, AX_ST_READY);
    if (a->enc_cal) {
        axis_hw_log("[HOME] %c: done - travel %.2f mm (encoder; steps %.2f mm), soft %.2f..%.2f mm, at %.2f mm",
                    a->cfg.name, (double)to_mm(a, travel), (double)to_mm(a, travel_cmd),
                    (double)to_mm(a, a->st.soft_min), (double)to_mm(a, a->st.soft_max),
                    (double)to_mm(a, new_pos));
    } else {
        axis_hw_log("[HOME] %c: done - travel %.2f mm (steps, no encoder), soft %.2f..%.2f mm, at %.2f mm",
                    a->cfg.name, (double)to_mm(a, travel), (double)to_mm(a, a->st.soft_min),
                    (double)to_mm(a, a->st.soft_max), (double)to_mm(a, new_pos));
    }
}

/* ------------------------------------------------------------------------ */
/* Supervision while homed                                                  */
/* ------------------------------------------------------------------------ */

/* Following error between the AS5600 and the commanded position. The
 * sensor reports the shaft angle about ENC_LAG_S late, so the commanded
 * position is shifted back by that time before comparing. Returns false
 * (after faulting the axis) when steps were lost. */
static bool check_follow(uint8_t ax)
{
    axis_t *a = &g_axes[ax];
    const float vel = axis_hw_velocity(ax);
    float expected;
    float err;
    float allowed;

    if (!a->enc_cal || !a->enc_valid || !a->enc_new) {
        return true;
    }
    expected = (float)a->enc_zero + ((float)a->enc.pos - vel * ENC_LAG_S) * a->enc_ratio;
    err = ((float)a->enc.counts - expected) / a->enc_ratio;
    allowed = a->cfg.enc_follow_err_steps + fabsf(vel) * ENC_LAG_TOL_S;
    a->st.follow_err = round_i32(err);
    if (abs_i32(a->st.follow_err) > abs_i32(a->st.follow_err_peak)) {
        a->st.follow_err_peak = a->st.follow_err;
    }
    if (fabsf(err) > allowed) {
        a->follow_hits++;
        if (a->follow_hits >= FOLLOW_HITS) {
            axis_hw_log("[ENC] %c: step loss - encoder %.2f mm, commanded %.2f mm",
                        a->cfg.name, (double)to_mm(a, enc_position(a)),
                        (double)to_mm(a, a->enc.pos));
            go_fault(ax, AX_FAULT_STEP_LOSS);
            return false;
        }
    } else {
        a->follow_hits = 0U;
    }
    return true;
}

/* Optional StallGuard4 crash detection during the cruise phase of a cycle
 * move. Returns false (after faulting) when a stall was detected. */
static bool crash_guard(uint8_t ax, uint32_t now)
{
    axis_t *a = &g_axes[ax];
    uint16_t sg = 0U;
    bool flag = false;
    axis_hw_status_t hs;

    if (!axis_hw_at_cruise(ax)) {
        a->cruising = false;
        return true;
    }
    if (!a->cruising) {
        a->cruising = true;
        a->t_cruise = now;
        stall_det_reset(&a->sd, &a->cfg.stall);
    }
    if ((now - a->t_sg_poll) < a->cfg.sg_poll_ms) {
        return true;
    }
    a->t_sg_poll = now;
    hs = axis_hw_read_sg(ax, &sg, &flag);
    if (hs != AXIS_HW_OK) {
        go_fault(ax, (hs == AXIS_HW_ERR_DRIVER) ? AX_FAULT_DRIVER : AX_FAULT_SPI);
        return false;
    }
    a->st.sg = sg;
    a->st.sg_flag = flag;
    /* The chip comparator is not re-tuned per move, so only use SG4 values. */
    if (stall_det_update(&a->sd, (int32_t)(now - a->t_cruise), sg, false) != STALL_NONE) {
        a->st.last_stall = a->sd.candidate;
        axis_hw_log("[CYCLE] %c: StallGuard4 crash detection (SG4 %u, baseline %.0f)",
                    a->cfg.name, (unsigned)sg, (double)a->sd.baseline);
        go_fault(ax, AX_FAULT_CRASH);
        return false;
    }
    a->st.sg_baseline = a->sd.baseline;
    return true;
}

static void start_cycle_leg(uint8_t ax)
{
    axis_t *a = &g_axes[ax];
    const int32_t target = (a->cycle_dir > 0) ? a->st.soft_max : a->st.soft_min;

    a->cruising = false;
    a->t_sg_poll = axis_hw_now_ms();
    axis_hw_move_to(ax, target, mm_to_stepsf(a, a->cfg.cycle_speed_mm_s),
                    mm_to_stepsf(a, a->cfg.cycle_accel_mm_s2));
    enter(a, AX_ST_CYCLE_MOVE);
}

static bool bounce_seek(uint8_t ax, int8_t dir)
{
    axis_t *a = &g_axes[ax];
    const int32_t pos = axis_hw_position(ax);
    int32_t tol = mm_to_steps(a, BOUNCE_TOL_MM);
    int32_t span;

    if (a->st.travel_steps / 20 > tol) {
        tol = a->st.travel_steps / 20;
    }
    span = (dir > 0) ? (a->st.travel_steps - pos) + tol : pos + tol;
    if (span < tol) {
        span = tol;
    }
    return seek_begin(ax, dir, span);
}

static void bounce_measure(uint8_t ax)
{
    axis_t *a = &g_axes[ax];
    const int8_t dir = a->seek_dir;
    const int32_t pos = axis_hw_position(ax);
    const int32_t expected = (dir > 0) ? a->st.travel_steps : 0;
    const bool enc = a->enc_cal && a->enc_valid;
    const int32_t actual = enc ? enc_position(a) : pos;
    const int32_t dev = actual - expected;
    int32_t p;

    a->st.strokes++;
    a->st.last_dev_steps = dev;
    if (abs_i32(dev) > abs_i32(a->st.max_dev_steps)) {
        a->st.max_dev_steps = dev;
    }
    if (dir > 0) {
        a->st.last_travel_steps = actual;
    }
    axis_hw_log("[BOUNCE] %c: %s stop #%lu, deviation %+.3f mm (%s), steps %+.3f mm, SG4 %u/%.0f %s",
                a->cfg.name, (dir < 0) ? "MIN" : "MAX", (unsigned long)a->st.strokes,
                (double)to_mm(a, dev), enc ? "encoder" : "steps",
                (double)to_mm(a, pos - expected), (unsigned)a->sd.last_sg,
                (double)a->sd.baseline, stall_source_name(a->st.last_stall));
    if (dir < 0) {
        /* Re-reference at the MIN stop on every stroke. */
        a->st.cycles++;
        if (enc) {
            a->enc_zero = a->enc.counts;
        }
        (void)axis_hw_set_position(ax, 0);
    } else if (enc) {
        (void)axis_hw_set_position(ax, actual);
    }
    p = axis_hw_position(ax);
    axis_hw_set_limits(ax, INT32_MIN, INT32_MAX);
    (void)axis_hw_take_limit_hit(ax);
    axis_hw_move_to(ax, sat_add(p, -(int32_t)dir * mm_to_steps(a, a->cfg.home_retract_mm)),
                    mm_to_stepsf(a, a->cfg.home_speed_mm_s),
                    mm_to_stepsf(a, a->cfg.home_accel_mm_s2));
    enter(a, AX_ST_BOUNCE_RETRACT);
}

static void update_status(uint8_t ax)
{
    axis_t *a = &g_axes[ax];

    a->st.pos_steps = axis_hw_position(ax);
    a->st.speed_steps_s = axis_hw_velocity(ax);
    a->st.enc_valid = a->enc_valid;
    a->st.enc_counts = a->enc.counts;
    a->st.enc_ok = a->enc_cal;
    a->st.enc_ratio = a->enc_ratio;
    a->st.enc_pos_steps = (a->enc_cal && a->enc_valid) ? enc_position(a) : 0;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

void axis_ctrl_init(uint8_t ax, const axis_cfg_t *cfg)
{
    axis_t *a;

    if ((ax >= AXIS_CTRL_MAX_AXES) || (cfg == NULL)) {
        return;
    }
    a = &g_axes[ax];
    memset(a, 0, sizeof(*a));
    a->cfg = *cfg;
    a->initialized = true;
    a->st.state = AX_ST_DISABLED;
    a->st.sg_thr = cfg->tmc.sg4_thrs_home;
    a->enc_ratio = cfg->enc_counts_per_step;
    a->t_state = axis_hw_now_ms();
    axis_hw_set_vmin(ax, cfg->vmin_steps_s);
}

axis_cfg_t *axis_ctrl_cfg(uint8_t ax)
{
    axis_t *a = get(ax);

    return (a != NULL) ? &a->cfg : NULL;
}

void axis_ctrl_step(uint8_t ax)
{
    axis_t *a = get(ax);
    uint32_t now;
    seek_result_t r;
    stall_source_t src = STALL_NONE;

    if (a == NULL) {
        return;
    }
    now = axis_hw_now_ms();
    refresh_encoder(ax);

    switch (a->st.state) {
    case AX_ST_DISABLED:
    case AX_ST_IDLE:
    case AX_ST_FAULT:
        break;

    case AX_ST_HOME_PREP:
        if ((now - a->t_state) >= a->cfg.home_prep_ms) {
            const int32_t max_span = mm_to_steps(a, a->cfg.max_travel_mm);
            if (a->cfg.home_backoff_mm > 0.0f) {
                const int32_t target = sat_add(axis_hw_position(ax),
                    -(int32_t)a->end1_dir * mm_to_steps(a, a->cfg.home_backoff_mm));
                if (!restore_static_threshold(ax)) {
                    break;
                }
                axis_hw_set_limits(ax, INT32_MIN, INT32_MAX);
                (void)axis_hw_take_limit_hit(ax);
                seek_arm(ax, (int8_t)-a->end1_dir);
                axis_hw_move_to(ax, target, mm_to_stepsf(a, a->cfg.home_speed_mm_s),
                                mm_to_stepsf(a, a->cfg.home_accel_mm_s2));
                enter(a, AX_ST_HOME_BACKOFF);
            } else if (seek_begin(ax, a->end1_dir, max_span)) {
                enter(a, AX_ST_HOME_SEEK1);
            }
        }
        break;

    case AX_ST_HOME_BACKOFF:
        r = seek_poll(ax, now, &src);
        if ((r == SEEK_ENDED) || (r == SEEK_STALL)) {
            if (r == SEEK_STALL) {
                axis_hw_halt(ax);
                axis_hw_log("[HOME] %c: stall during back-off (%s) - carriage was at the far end",
                            a->cfg.name, stall_source_name(src));
            }
            if (seek_begin(ax, a->end1_dir, mm_to_steps(a, a->cfg.max_travel_mm))) {
                enter(a, AX_ST_HOME_SEEK1);
            }
        } else if (r != SEEK_RUN) {
            seek_fail(ax, r);
        }
        break;

    case AX_ST_HOME_SEEK1:
    case AX_ST_HOME_SEEK2:
        if (a->st.state == AX_ST_HOME_SEEK2) {
            hist_push(a, now);
        }
        r = seek_poll(ax, now, &src);
        if (r == SEEK_STALL) {
            const bool second = (a->st.state == AX_ST_HOME_SEEK2);
            axis_hw_halt(ax);
            a->st.last_stall = src;
            log_stall(ax, "HOME", src);
            if (second) {
                try_calibrate(ax, now);
            }
            enter(a, second ? AX_ST_HOME_SETTLE2 : AX_ST_HOME_SETTLE1);
        } else if (r != SEEK_RUN) {
            seek_fail(ax, r);
        }
        break;

    case AX_ST_HOME_SETTLE1:
    case AX_ST_HOME_SETTLE2:
        if ((now - a->t_state) >= a->cfg.home_settle_ms) {
            const int idx = (a->st.state == AX_ST_HOME_SETTLE1) ? 0 : 1;
            const int32_t pos = axis_hw_position(ax);
            const int8_t away = (idx == 0) ? (int8_t)-a->end1_dir : a->end1_dir;

            a->p_end[idx] = pos;
            a->e_end_ok[idx] = a->enc_valid;
            a->e_end[idx] = a->enc.counts;
            if (idx == 1) {
                const int32_t t = abs_i32(a->p_end[1] - a->p_end[0]);
                if (t < mm_to_steps(a, a->cfg.min_travel_mm)) {
                    axis_hw_log("[HOME] %c: travel %.2f mm below minimum %.1f mm - false stall? "
                                "tune 'sgratio'/'sg' or the homing speed",
                                a->cfg.name, (double)to_mm(a, t), (double)a->cfg.min_travel_mm);
                    go_fault(ax, AX_FAULT_SHORT_TRAVEL);
                    break;
                }
            }
            axis_hw_set_limits(ax, INT32_MIN, INT32_MAX);
            (void)axis_hw_take_limit_hit(ax);
            axis_hw_move_to(ax, sat_add(pos, (int32_t)away * mm_to_steps(a, a->cfg.home_retract_mm)),
                            mm_to_stepsf(a, a->cfg.home_speed_mm_s),
                            mm_to_stepsf(a, a->cfg.home_accel_mm_s2));
            enter(a, (idx == 0) ? AX_ST_HOME_RETRACT1 : AX_ST_HOME_RETRACT2);
        }
        break;

    case AX_ST_HOME_RETRACT1:
        if (axis_hw_take_limit_hit(ax)) {
            go_fault(ax, AX_FAULT_LIMIT);
        } else if (!axis_hw_busy(ax)) {
            axis_enc_sample_t other;
            const uint8_t oax = (uint8_t)(ax ^ 1U);
            a->cal_start_ok = a->enc_valid;
            a->cal_start.t = now;
            a->cal_start.p = a->enc.pos;
            a->cal_start.e = a->enc.counts;
            a->other_e_start_ok = (oax < AXIS_CTRL_MAX_AXES) && axis_hw_encoder(oax, &other);
            a->other_e_start = a->other_e_start_ok ? other.counts : 0;
            if (seek_begin(ax, (int8_t)-a->end1_dir, mm_to_steps(a, a->cfg.max_travel_mm))) {
                enter(a, AX_ST_HOME_SEEK2);
            }
        }
        break;

    case AX_ST_HOME_RETRACT2:
        if (axis_hw_take_limit_hit(ax)) {
            go_fault(ax, AX_FAULT_LIMIT);
        } else if (!axis_hw_busy(ax)) {
            finish_homing(ax);
        }
        break;

    case AX_ST_READY:
        (void)check_follow(ax);
        break;

    case AX_ST_MOVE:
        if (axis_hw_take_limit_hit(ax)) {
            go_fault(ax, AX_FAULT_LIMIT);
        } else if (check_follow(ax) && !axis_hw_busy(ax)) {
            enter(a, AX_ST_READY);
        }
        break;

    case AX_ST_CYCLE_MOVE:
        if (axis_hw_take_limit_hit(ax)) {
            go_fault(ax, AX_FAULT_LIMIT);
            break;
        }
        if (!check_follow(ax)) {
            break;
        }
        if (a->cfg.cycle_sg_guard && !crash_guard(ax, now)) {
            break;
        }
        if (!axis_hw_busy(ax)) {
            if (a->cycle_dir < 0) {
                a->st.cycles++;
            }
            enter(a, AX_ST_CYCLE_DWELL);
        }
        break;

    case AX_ST_CYCLE_DWELL:
        if (check_follow(ax) && ((now - a->t_state) >= a->cfg.cycle_dwell_ms)) {
            a->cycle_dir = (int8_t)-a->cycle_dir;
            start_cycle_leg(ax);
        }
        break;

    case AX_ST_BOUNCE_SEEK:
        r = seek_poll(ax, now, &src);
        if (r == SEEK_STALL) {
            axis_hw_halt(ax);
            a->st.last_stall = src;
            enter(a, AX_ST_BOUNCE_SETTLE);
        } else if (r != SEEK_RUN) {
            seek_fail(ax, r);
        }
        break;

    case AX_ST_BOUNCE_SETTLE:
        if ((now - a->t_state) >= a->cfg.home_settle_ms) {
            bounce_measure(ax);
        }
        break;

    case AX_ST_BOUNCE_RETRACT:
        if (axis_hw_take_limit_hit(ax)) {
            go_fault(ax, AX_FAULT_LIMIT);
        } else if (!axis_hw_busy(ax) && ((now - a->t_state) >= a->cfg.cycle_dwell_ms)) {
            if (bounce_seek(ax, (int8_t)-a->seek_dir)) {
                enter(a, AX_ST_BOUNCE_SEEK);
            }
        }
        break;

    case AX_ST_STOPPING:
        if (!axis_hw_busy(ax)) {
            (void)axis_hw_take_limit_hit(ax);
            enter(a, a->stop_to);
        }
        break;

    default:
        go_fault(ax, AX_FAULT_INTERNAL);
        break;
    }
    update_status(ax);
}

void axis_ctrl_set_driver_ready(uint8_t ax, bool ready)
{
    axis_t *a = get(ax);

    if (a == NULL) {
        return;
    }
    a->driver_ready = ready;
    a->profile_valid = false; /* registers were (re)written by the driver layer */
    a->thr_raised = false;
    if (ready) {
        if (a->st.state == AX_ST_DISABLED) {
            enter(a, AX_ST_IDLE);
        }
        return;
    }
    switch (a->st.state) {
    case AX_ST_DISABLED:
    case AX_ST_FAULT:
        break;
    case AX_ST_IDLE:
        axis_hw_driver_enable(ax, false);
        enter(a, AX_ST_DISABLED);
        break;
    default:
        go_fault(ax, AX_FAULT_DRIVER);
        break;
    }
}

void axis_ctrl_config_changed(uint8_t ax)
{
    axis_t *a = get(ax);

    if (a != NULL) {
        a->profile_valid = false;
    }
}

bool axis_ctrl_home(uint8_t ax)
{
    axis_t *a = get(ax);

    if ((a == NULL) || !a->driver_ready ||
        ((a->st.state != AX_ST_IDLE) && (a->st.state != AX_ST_READY))) {
        return false;
    }
    a->st.homed = false;
    a->st.cycling = false;
    a->enc_cal = false;
    a->st.enc_ok = false;
    a->st.last_stall = STALL_NONE;
    if (!ensure_profile(ax, true)) {
        return false;
    }
    a->end1_dir = a->cfg.home_min_first ? -1 : 1;
    axis_hw_set_limits(ax, INT32_MIN, INT32_MAX);
    (void)axis_hw_take_limit_hit(ax);
    axis_hw_driver_enable(ax, true);
    enter(a, AX_ST_HOME_PREP);
    axis_hw_log("[HOME] %c: start - first end %s at %.1f mm/s", a->cfg.name,
                (a->end1_dir < 0) ? "MIN" : "MAX", (double)a->cfg.home_speed_mm_s);
    return true;
}

bool axis_ctrl_cycle(uint8_t ax, axis_cycle_mode_t mode)
{
    axis_t *a = get(ax);
    int32_t pos;
    int8_t dir;

    if ((a == NULL) || (a->st.state != AX_ST_READY) || !a->st.homed) {
        return false;
    }
    pos = axis_hw_position(ax);
    dir = ((pos - a->st.soft_min) < (a->st.soft_max - pos)) ? 1 : -1; /* farther end first */
    a->st.cycle_mode = mode;
    if (mode == AX_CYCLE_BOUNCE) {
        if (!ensure_profile(ax, true) || !bounce_seek(ax, dir)) {
            return false;
        }
        a->st.cycling = true;
        enter(a, AX_ST_BOUNCE_SEEK);
        axis_hw_log("[CYCLE] %c: bounce mode (StallGuard4 at every end) at %.1f mm/s",
                    a->cfg.name, (double)a->cfg.home_speed_mm_s);
    } else {
        if (!ensure_profile(ax, false)) {
            return false;
        }
        set_run_limits(ax);
        a->cycle_dir = dir;
        a->st.cycling = true;
        start_cycle_leg(ax);
        axis_hw_log("[CYCLE] %c: %.2f <-> %.2f mm at %.1f mm/s, %.0f mm/s^2",
                    a->cfg.name, (double)to_mm(a, a->st.soft_min), (double)to_mm(a, a->st.soft_max),
                    (double)a->cfg.cycle_speed_mm_s, (double)a->cfg.cycle_accel_mm_s2);
    }
    return true;
}

bool axis_ctrl_move_mm(uint8_t ax, float target_mm)
{
    axis_t *a = get(ax);
    int32_t target;

    if ((a == NULL) || (a->st.state != AX_ST_READY) || !a->st.homed) {
        return false;
    }
    target = mm_to_steps(a, target_mm);
    if (target < a->st.soft_min) {
        target = a->st.soft_min;
    }
    if (target > a->st.soft_max) {
        target = a->st.soft_max;
    }
    if (!ensure_profile(ax, false)) {
        return false;
    }
    set_run_limits(ax);
    axis_hw_move_to(ax, target, mm_to_stepsf(a, a->cfg.cycle_speed_mm_s),
                    mm_to_stepsf(a, a->cfg.cycle_accel_mm_s2));
    enter(a, AX_ST_MOVE);
    return true;
}

void axis_ctrl_stop(uint8_t ax)
{
    axis_t *a = get(ax);

    if (a == NULL) {
        return;
    }
    switch (a->st.state) {
    case AX_ST_HOME_PREP:
    case AX_ST_HOME_SETTLE1:
    case AX_ST_HOME_SETTLE2:
        axis_hw_halt(ax);
        a->st.homed = false;
        enter(a, AX_ST_IDLE);
        break;
    case AX_ST_HOME_BACKOFF:
    case AX_ST_HOME_SEEK1:
    case AX_ST_HOME_RETRACT1:
    case AX_ST_HOME_SEEK2:
    case AX_ST_HOME_RETRACT2:
        axis_hw_stop(ax);
        a->st.homed = false;
        a->stop_to = AX_ST_IDLE;
        enter(a, AX_ST_STOPPING);
        break;
    case AX_ST_MOVE:
    case AX_ST_CYCLE_MOVE:
    case AX_ST_BOUNCE_SEEK:
    case AX_ST_BOUNCE_RETRACT:
        axis_hw_stop(ax);
        a->st.cycling = false;
        a->stop_to = AX_ST_READY;
        enter(a, AX_ST_STOPPING);
        break;
    case AX_ST_CYCLE_DWELL:
    case AX_ST_BOUNCE_SETTLE:
        a->st.cycling = false;
        enter(a, AX_ST_READY);
        break;
    default:
        break;
    }
}

void axis_ctrl_disable(uint8_t ax)
{
    axis_t *a = get(ax);

    if (a == NULL) {
        return;
    }
    axis_hw_halt(ax);
    axis_hw_driver_enable(ax, false);
    a->st.homed = false;
    a->st.cycling = false;
    a->enc_cal = false;
    a->st.enc_ok = false;
    if (a->st.state != AX_ST_FAULT) {
        enter(a, a->driver_ready ? AX_ST_IDLE : AX_ST_DISABLED);
    }
}

void axis_ctrl_fault(uint8_t ax, axis_fault_t fault)
{
    axis_t *a = get(ax);

    if ((a != NULL) && (a->st.state != AX_ST_FAULT)) {
        go_fault(ax, fault);
    }
}

bool axis_ctrl_clear_fault(uint8_t ax)
{
    axis_t *a = get(ax);

    if ((a == NULL) || (a->st.state != AX_ST_FAULT)) {
        return false;
    }
    a->st.fault = AX_FAULT_NONE;
    a->follow_hits = 0U;
    enter(a, a->driver_ready ? AX_ST_IDLE : AX_ST_DISABLED);
    return true;
}

const axis_status_t *axis_ctrl_status(uint8_t ax)
{
    axis_t *a = get(ax);

    return (a != NULL) ? &a->st : NULL;
}

bool axis_ctrl_is_busy(uint8_t ax)
{
    axis_t *a = get(ax);

    if (a == NULL) {
        return false;
    }
    switch (a->st.state) {
    case AX_ST_DISABLED:
    case AX_ST_IDLE:
    case AX_ST_READY:
    case AX_ST_FAULT:
        return false;
    default:
        return true;
    }
}

float axis_ctrl_steps_to_mm(uint8_t ax, int32_t steps)
{
    axis_t *a = get(ax);

    return (a != NULL) ? to_mm(a, steps) : 0.0f;
}

const char *axis_ctrl_state_name(axis_state_t s)
{
    static const char *const names[AX_ST_COUNT] = {
        "DISABLED", "IDLE", "HOME-PREP", "HOME-BACKOFF", "HOME-SEEK1", "HOME-SETTLE1",
        "HOME-RETRACT1", "HOME-SEEK2", "HOME-SETTLE2", "HOME-RETRACT2", "READY", "MOVE",
        "CYCLE", "DWELL", "BOUNCE-SEEK", "BOUNCE-SETTLE", "BOUNCE-RETRACT", "STOPPING",
        "FAULT"
    };

    return ((unsigned)s < (unsigned)AX_ST_COUNT) ? names[s] : "?";
}

const char *axis_ctrl_fault_name(axis_fault_t f)
{
    switch (f) {
    case AX_FAULT_NONE:         return "none";
    case AX_FAULT_DRIVER:       return "driver fault";
    case AX_FAULT_SPI:          return "SPI error";
    case AX_FAULT_NO_STALL:     return "no stall detected";
    case AX_FAULT_TIMEOUT:      return "seek timeout";
    case AX_FAULT_SHORT_TRAVEL: return "travel too short";
    case AX_FAULT_STEP_LOSS:    return "step loss";
    case AX_FAULT_CRASH:        return "crash (StallGuard4)";
    case AX_FAULT_LIMIT:        return "software limit";
    case AX_FAULT_INTERNAL:
    default:                    return "internal error";
    }
}
