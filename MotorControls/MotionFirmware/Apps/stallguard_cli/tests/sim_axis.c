/* Host simulator implementing axis_hw.h (see sim_axis.h). */
#include "sim_axis.h"

#include "axis_hw.h"
#include "motion_profile.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define SIM_AXES     AXIS_CTRL_MAX_AXES
#define SIM_LOG_MAX  8192U
#define SIM_LOG_LEN  200U

typedef struct {
    sim_axis_cfg_t    cfg;
    bool              configured;
    step_core_t       core;
    double            t_next;
    bool              enabled;
    int32_t           rotor;
    int32_t           rotor_prev;    /* rotor 1 ms ago (models AS5600 filter lag) */
    int32_t           slip;          /* commanded - rotor */
    bool              ever_blocked;
    uint32_t          last_block_ms;
    bool              blocked_this_ms;
    uint32_t          blocked_ms;
    float             tcool_steps_s;
    uint8_t           sg_thr;
    axis_enc_sample_t enc;
    bool              enc_has;
    float             rmin;
    float             rmax;
} sim_t;

static sim_t    g_sim[SIM_AXES];
static uint32_t g_now;
static uint32_t g_rng;
static bool     g_verbose;
static char     g_log[SIM_LOG_MAX][SIM_LOG_LEN];
static unsigned g_log_n;

static float noise(float amplitude)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return amplitude * ((float)(g_rng % 20001U) / 10000.0f - 1.0f);
}

void sim_reset(uint32_t seed)
{
    memset(g_sim, 0, sizeof(g_sim));
    g_now = 1000U;
    g_rng = (seed != 0U) ? seed : 0x12345678U;
    g_log_n = 0U;
}

void sim_setup(uint8_t ax, const sim_axis_cfg_t *cfg)
{
    sim_t *s = &g_sim[ax];

    memset(s, 0, sizeof(*s));
    s->cfg = *cfg;
    s->configured = true;
    step_core_init(&s->core, 1000.0f, 1000.0f, 100.0f);
    s->rotor = cfg->start;
    s->rotor_prev = cfg->start;
    s->slip = s->core.pos - s->rotor;
    s->rmin = s->rmax = (float)s->rotor;
    s->tcool_steps_s = 1.0e9f;
}

static void block_at(sim_t *s, int32_t stop)
{
    s->rotor = stop;
    s->slip = s->core.pos - stop;
    s->ever_blocked = true;
    s->last_block_ms = g_now;
    s->blocked_this_ms = true;
}

static void apply_step(sim_t *s)
{
    int32_t desired;

    if (!s->enabled) {
        s->slip = s->core.pos - s->rotor; /* unpowered rotor does not follow */
        return;
    }
    desired = s->core.pos - s->slip;
    if (desired > s->cfg.stop_hi) {
        block_at(s, s->cfg.stop_hi);
    } else if (desired < s->cfg.stop_lo) {
        block_at(s, s->cfg.stop_lo);
    } else {
        s->rotor = desired;
    }
    if ((float)s->rotor < s->rmin) {
        s->rmin = (float)s->rotor;
    }
    if ((float)s->rotor > s->rmax) {
        s->rmax = (float)s->rotor;
    }
}

static void advance_axis(sim_t *s)
{
    const double t_end = (double)(g_now + 1U) / 1000.0;

    s->rotor_prev = s->rotor;
    s->blocked_this_ms = false;
    while (s->core.running && (s->t_next <= t_end)) {
        bool more = false;
        bool dir_changed = false;
        float v = 0.0f;

        if (step_core_expire(&s->core, &more, &v, &dir_changed)) {
            apply_step(s);
        }
        if (more) {
            s->t_next += 1.0 / (double)v;
        }
    }
    if (s->blocked_this_ms) {
        s->blocked_ms++;
    }
}

static void sample_encoder(sim_t *s)
{
    const sim_t *src = &g_sim[s->cfg.enc_source];

    if (!s->cfg.enc_present) {
        s->enc_has = false;
        return;
    }
    s->enc.counts = s->cfg.enc_offset + (int32_t)lround((double)src->rotor_prev * (double)s->cfg.enc_ratio);
    s->enc.pos = s->core.pos;
    s->enc.seq++;
    s->enc_has = true;
}

void sim_run_ms(uint32_t ms)
{
    uint32_t i;
    uint8_t ax;

    for (i = 0U; i < ms; i++) {
        for (ax = 0U; ax < SIM_AXES; ax++) {
            if (g_sim[ax].configured) {
                advance_axis(&g_sim[ax]);
            }
        }
        g_now++;
        if ((g_now % 2U) == 0U) {
            for (ax = 0U; ax < SIM_AXES; ax++) {
                if (g_sim[ax].configured) {
                    sample_encoder(&g_sim[ax]);
                }
            }
        }
        for (ax = 0U; ax < SIM_AXES; ax++) {
            if (g_sim[ax].configured) {
                axis_ctrl_step(ax);
            }
        }
    }
}

bool sim_run_until(bool (*pred)(void), uint32_t timeout_ms)
{
    uint32_t t;

    for (t = 0U; t < timeout_ms; t++) {
        if (pred()) {
            return true;
        }
        sim_run_ms(1U);
    }
    return pred();
}

uint32_t sim_now(void) { return g_now; }
float sim_rotor(uint8_t ax) { return (float)g_sim[ax].rotor; }
uint32_t sim_blocked_ms(uint8_t ax) { return g_sim[ax].blocked_ms; }
void sim_clear_blocked(uint8_t ax) { g_sim[ax].blocked_ms = 0U; }
void sim_set_spi_fail(uint8_t ax, bool fail) { g_sim[ax].cfg.spi_fail = fail; }
void sim_set_sg_broken(uint8_t ax, bool broken) { g_sim[ax].cfg.sg_broken = broken; }
void sim_set_verbose(bool verbose) { g_verbose = verbose; }

void sim_track_range(uint8_t ax, float *lo, float *hi, bool reset)
{
    sim_t *s = &g_sim[ax];

    if (lo != NULL) {
        *lo = s->rmin;
    }
    if (hi != NULL) {
        *hi = s->rmax;
    }
    if (reset) {
        s->rmin = s->rmax = (float)s->rotor;
    }
}

void sim_inject_slip(uint8_t ax, int32_t steps)
{
    sim_t *s = &g_sim[ax];

    s->rotor += steps;
    if (s->rotor > s->cfg.stop_hi) {
        s->rotor = s->cfg.stop_hi;
    }
    if (s->rotor < s->cfg.stop_lo) {
        s->rotor = s->cfg.stop_lo;
    }
    s->slip = s->core.pos - s->rotor;
}

unsigned sim_log_count(const char *needle)
{
    unsigned i;
    unsigned n = 0U;

    for (i = 0U; (i < g_log_n) && (i < SIM_LOG_MAX); i++) {
        if (strstr(g_log[i], needle) != NULL) {
            n++;
        }
    }
    return n;
}

/* ------------------------------------------------------------------------ */
/* axis_hw.h implementation                                                 */
/* ------------------------------------------------------------------------ */

uint32_t axis_hw_now_ms(void)
{
    return g_now;
}

void axis_hw_log(const char *fmt, ...)
{
    char line[SIM_LOG_LEN];
    va_list args;

    va_start(args, fmt);
    (void)vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (g_log_n < SIM_LOG_MAX) {
        memcpy(g_log[g_log_n], line, sizeof(line));
    }
    g_log_n++;
    if (g_verbose) {
        printf("      %8.3f s  %s\n", (double)g_now / 1000.0, line);
    }
}

void axis_hw_driver_enable(uint8_t ax, bool enable)
{
    g_sim[ax].enabled = enable;
}

axis_hw_status_t axis_hw_apply_profile(uint8_t ax, axis_profile_t profile, const axis_cfg_t *cfg)
{
    sim_t *s = &g_sim[ax];

    if (s->cfg.spi_fail) {
        return AXIS_HW_ERR_SPI;
    }
    s->sg_thr = cfg->tmc.sg4_thrs_home;
    if (profile == AXIS_PROFILE_HOMING) {
        s->tcool_steps_s = 0.5f * cfg->home_speed_mm_s * cfg->steps_per_mm;
    } else if (cfg->tmc.coolstep_enable || cfg->cycle_sg_guard) {
        s->tcool_steps_s = cfg->tmc.coolstep_min_mm_s * cfg->steps_per_mm;
    } else {
        s->tcool_steps_s = 1.0e9f;
    }
    return AXIS_HW_OK;
}

axis_hw_status_t axis_hw_set_sg_threshold(uint8_t ax, uint8_t thrs)
{
    if (g_sim[ax].cfg.spi_fail) {
        return AXIS_HW_ERR_SPI;
    }
    g_sim[ax].sg_thr = thrs;
    return AXIS_HW_OK;
}

axis_hw_status_t axis_hw_read_sg(uint8_t ax, uint16_t *sg4, bool *stall_flag)
{
    sim_t *s = &g_sim[ax];
    const float v = s->core.running ? s->core.mp.v : 0.0f;
    const bool blocked = s->ever_blocked && ((g_now - s->last_block_ms) <= 3U);
    float val;
    uint16_t out;

    if (s->cfg.spi_fail) {
        return AXIS_HW_ERR_SPI;
    }
    if (!s->enabled || (v < 1.0f)) {
        val = 0.0f;
    } else if (blocked && !s->cfg.sg_broken) {
        val = s->cfg.sg_stall + noise(s->cfg.sg_noise * 0.3f);
    } else {
        val = s->cfg.sg_free + noise(s->cfg.sg_noise);
    }
    if (val < 0.0f) {
        val = 0.0f;
    }
    if (val > 510.0f) {
        val = 510.0f;
    }
    out = (uint16_t)((uint16_t)val & 0x3FEU);
    *sg4 = out;
    *stall_flag = (v >= s->tcool_steps_s) && (out <= (uint16_t)(2U * s->sg_thr));
    return AXIS_HW_OK;
}

void axis_hw_set_vmin(uint8_t ax, float vmin)
{
    g_sim[ax].core.vmin = vmin;
}

static void maybe_start(sim_t *s, bool start, float v)
{
    if (start) {
        s->t_next = (double)g_now / 1000.0 + 1.0 / (double)v;
    }
}

void axis_hw_move_to(uint8_t ax, int32_t target, float vmax, float accel)
{
    sim_t *s = &g_sim[ax];
    float v = 0.0f;

    maybe_start(s, step_core_cmd_move(&s->core, target, vmax, accel, &v), v);
}

void axis_hw_run(uint8_t ax, int8_t dir, float vmax, float accel)
{
    sim_t *s = &g_sim[ax];
    float v = 0.0f;

    maybe_start(s, step_core_cmd_run(&s->core, dir, vmax, accel, &v), v);
}

void axis_hw_stop(uint8_t ax)
{
    step_core_cmd_stop(&g_sim[ax].core);
}

void axis_hw_halt(uint8_t ax)
{
    step_core_cmd_halt(&g_sim[ax].core);
}

bool axis_hw_busy(uint8_t ax)
{
    return g_sim[ax].core.running;
}

int32_t axis_hw_position(uint8_t ax)
{
    return g_sim[ax].core.pos;
}

bool axis_hw_set_position(uint8_t ax, int32_t pos)
{
    sim_t *s = &g_sim[ax];

    if (s->core.running) {
        return false;
    }
    s->core.pos = pos;
    s->slip = pos - s->rotor;
    return true;
}

float axis_hw_velocity(uint8_t ax)
{
    const sim_t *s = &g_sim[ax];

    return s->core.running ? s->core.mp.v * (float)s->core.step_dir : 0.0f;
}

bool axis_hw_at_cruise(uint8_t ax)
{
    return g_sim[ax].core.running && mp_at_cruise(&g_sim[ax].core.mp);
}

void axis_hw_set_limits(uint8_t ax, int32_t lo, int32_t hi)
{
    g_sim[ax].core.lim_lo = lo;
    g_sim[ax].core.lim_hi = hi;
}

bool axis_hw_take_limit_hit(uint8_t ax)
{
    const bool hit = g_sim[ax].core.limit_hit;

    g_sim[ax].core.limit_hit = false;
    return hit;
}

bool axis_hw_encoder(uint8_t ax, axis_enc_sample_t *sample)
{
    const sim_t *s = &g_sim[ax];

    if (!s->enc_has) {
        return false;
    }
    *sample = s->enc;
    return true;
}
