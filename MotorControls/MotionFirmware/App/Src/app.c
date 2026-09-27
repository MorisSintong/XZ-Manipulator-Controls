/**
 * @file    app.c
 * @brief   MotionFirmware application (see app.h).
 *
 * Main loop services (1 ms tick):
 *   encoder_poll        every 2 ms   AS5600 RAW ANGLE -> multi-turn counts
 *   axis_ctrl_step      every 1 ms   per-axis homing/cycle state machines
 *   driver probe        every 500 ms bring up TMC2240s that are not configured
 *   driver health       every 25 ms  DRV_STATUS / SPI status (round robin)
 *   driver ADC          every 1 s    chip temperature and VM
 *   sequencer           every 1 ms   homing order, cycle start, fault fan-out
 *   telemetry, LED, button, console commands
 */
#include "app.h"

#include "app_config.h"
#include "axis_ctrl.h"
#include "axis_hw.h"
#include "cmd_parse.h"
#include "console.h"
#include "encoder.h"
#include "main.h"
#include "stepgen.h"
#include "tmc_axis.h"

#include <stdio.h>
#include <string.h>

#define FW_NAME    "MotionFirmware"
#define FW_VERSION "2.0.0"

extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c3;
extern SPI_HandleTypeDef  hspi2;
extern UART_HandleTypeDef huart2;

typedef enum {
    SYS_WAIT_DRIVERS = 0, /* no TMC2240 answered yet (VM off?) */
    SYS_IDLE,             /* drivers ready, not running */
    SYS_HOMING,           /* sequential StallGuard4 homing */
    SYS_RUNNING,          /* axes cycling */
    SYS_PAUSED,           /* homed, holding position */
    SYS_FAULT
} sys_state_t;

static const char *const k_sys_names[] = {
    "WAIT-DRIVERS", "IDLE", "HOMING", "RUNNING", "PAUSED", "FAULT"
};

static sys_state_t       g_sys;
static axis_cycle_mode_t g_mode = APP_DEFAULT_CYCLE_MODE;
static uint8_t           g_log_level = APP_LOG_LEVEL_DEFAULT;
static bool              g_cycle_after_home;
static bool              g_home_pending[APP_AXIS_COUNT];
static bool              g_autostart_done;
static cmd_linebuf_t     g_line;
static const uint8_t     k_home_order[APP_AXIS_COUNT] = APP_HOME_ORDER;

static uint32_t g_last_tick;
static uint32_t g_t_enc;
static uint32_t g_t_enc_diag;
static uint32_t g_t_probe;
static uint32_t g_t_health;
static uint32_t g_t_adc;
static uint32_t g_t_status;
static uint32_t g_t_trace;
static uint8_t  g_health_rr;
static uint32_t g_probe_fail[APP_AXIS_COUNT];
static bool     g_warned[APP_AXIS_COUNT];
static bool     g_enc_was_ok[APP_AXIS_COUNT];
static bool     g_temp_warned[APP_AXIS_COUNT];

static bool     g_btn_raw;
static bool     g_btn_stable;
static bool     g_btn_long;
static uint32_t g_btn_t_change;
static uint32_t g_btn_t_press;

/* ------------------------------------------------------------------------ */
/* Helpers                                                                  */
/* ------------------------------------------------------------------------ */

#define LOG(...) axis_hw_log(__VA_ARGS__)

static char axis_name(uint8_t ax)
{
    return axis_ctrl_cfg(ax)->name;
}

static bool axis_present(uint8_t ax)
{
    return axis_ctrl_status(ax)->state != AX_ST_DISABLED;
}

static void set_sys(sys_state_t s)
{
    if (s != g_sys) {
        g_sys = s;
        LOG("[SYS] %s", k_sys_names[s]);
    }
}

static int parse_axis(const char *s)
{
    uint8_t ax;

    if ((s == NULL) || (s[0] == '\0') || (s[1] != '\0')) {
        return -1;
    }
    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        const char n = axis_name(ax);
        if ((s[0] == n) || (s[0] == (char)(n - 'A' + 'a'))) {
            return (int)ax;
        }
    }
    return -1;
}

static void print_c10(char *buf, size_t n, int16_t c10)
{
    const int a = (c10 < 0) ? -c10 : c10;

    (void)snprintf(buf, n, "%s%d.%dC", (c10 < 0) ? "-" : "", a / 10, a % 10);
}

static void print_axis_status(uint8_t ax)
{
    const axis_status_t *st = axis_ctrl_status(ax);
    const axis_cfg_t *cfg = axis_ctrl_cfg(ax);
    const tmc_axis_info_t *ti = tmc_axis_info(ax);
    const encoder_info_t *ei = encoder_info(ax);
    const float spm = cfg->steps_per_mm;
    const bool seeking = (st->state == AX_ST_HOME_BACKOFF) || (st->state == AX_ST_HOME_SEEK1) ||
                         (st->state == AX_ST_HOME_SEEK2) || (st->state == AX_ST_BOUNCE_SEEK);
    char enc[48];
    char drv[48];
    char temp[12];

    if (st->enc_ok && st->enc_valid) {
        (void)snprintf(enc, sizeof(enc), "enc %8.2f (err %+.2f)",
                       (double)((float)st->enc_pos_steps / spm), (double)((float)st->follow_err / spm));
    } else if ((ei != NULL) && ei->healthy) {
        (void)snprintf(enc, sizeof(enc), "enc raw %4u", (unsigned)ei->raw);
    } else {
        (void)snprintf(enc, sizeof(enc), "enc %s",
                       (ei != NULL) ? encoder_result_name(ei->last_result) : "n/a");
    }
    if ((ti != NULL) && ti->configured) {
        print_c10(temp, sizeof(temp), ti->temp_c10);
        (void)snprintf(drv, sizeof(drv), "SG %3u CS %2u %s", seeking ? (unsigned)st->sg : (unsigned)ti->sg_result,
                       (unsigned)ti->cs_actual, ti->adc_valid ? temp : "");
    } else {
        (void)snprintf(drv, sizeof(drv), "driver off");
    }
    console_printf("  %c %-14s pos%s %8.2f mm  v %+7.1f mm/s  %s  %s  cyc %lu\r\n",
                   cfg->name, axis_ctrl_state_name(st->state), st->homed ? " " : "?",
                   (double)((float)st->pos_steps / spm), (double)(st->speed_steps_s / spm), enc, drv,
                   (unsigned long)st->cycles);
}

static void print_status(void)
{
    uint8_t ax;
    const uint32_t t = HAL_GetTick();

    console_printf("[%5lu.%03lu] %s, mode %s\r\n", (unsigned long)(t / 1000U), (unsigned long)(t % 1000U),
                   k_sys_names[g_sys], (g_mode == AX_CYCLE_BOUNCE) ? "bounce" : "soft");
    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        print_axis_status(ax);
    }
}

/* ------------------------------------------------------------------------ */
/* Actions                                                                  */
/* ------------------------------------------------------------------------ */

static void start_cycles(void)
{
    uint8_t ax;
    bool any = false;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        any = any || axis_ctrl_status(ax)->homed;
    }
    /* The sequencer starts every homed axis as soon as it is READY (an axis
     * may still be decelerating from a previous stop). */
    set_sys(any ? SYS_RUNNING : SYS_IDLE);
}

static void action_start(void)
{
    uint8_t ax;
    bool need_home = false;

    if (g_sys == SYS_FAULT) {
        LOG("[SYS] fault active - send 'clear' or press B1 first");
        return;
    }
    if (g_sys == SYS_WAIT_DRIVERS) {
        LOG("[SYS] no TMC2240 ready yet");
        return;
    }
    if ((g_sys == SYS_RUNNING) || (g_sys == SYS_HOMING)) {
        return;
    }
    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        const axis_status_t *st = axis_ctrl_status(ax);
        if (axis_present(ax) && !st->homed) {
            g_home_pending[ax] = true;
            need_home = true;
        }
    }
    g_cycle_after_home = true;
    if (need_home) {
        set_sys(SYS_HOMING);
    } else {
        start_cycles();
    }
}

static void action_stop(void)
{
    uint8_t ax;
    bool homed = false;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        g_home_pending[ax] = false;
        axis_ctrl_stop(ax);
        homed = homed || axis_ctrl_status(ax)->homed;
    }
    if ((g_sys != SYS_FAULT) && (g_sys != SYS_WAIT_DRIVERS)) {
        set_sys(homed ? SYS_PAUSED : SYS_IDLE);
    }
}

static void action_home(void)
{
    uint8_t ax;

    if ((g_sys == SYS_FAULT) || (g_sys == SYS_WAIT_DRIVERS)) {
        LOG("[SYS] cannot home in state %s", k_sys_names[g_sys]);
        return;
    }
    if ((g_sys == SYS_RUNNING) || (g_sys == SYS_HOMING)) {
        LOG("[SYS] stop first");
        return;
    }
    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        g_home_pending[ax] = axis_present(ax);
    }
    g_cycle_after_home = false;
    set_sys(SYS_HOMING);
}

static void action_off(void)
{
    uint8_t ax;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        g_home_pending[ax] = false;
        axis_ctrl_disable(ax);
    }
    LOG("[SYS] drivers disabled (ENN high) - axes must be re-homed");
    if ((g_sys != SYS_FAULT) && (g_sys != SYS_WAIT_DRIVERS)) {
        set_sys(SYS_IDLE);
    }
}

static void action_clear(void)
{
    uint8_t ax;
    bool any = false;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        g_home_pending[ax] = false;
        any = axis_ctrl_clear_fault(ax) || any;
    }
    if (g_sys == SYS_FAULT) {
        LOG("[SYS] faults cleared%s", any ? " - drivers are re-initialised automatically" : "");
        set_sys(SYS_IDLE);
    }
}

/* ------------------------------------------------------------------------ */
/* Commands                                                                 */
/* ------------------------------------------------------------------------ */

typedef struct {
    const char *name;
    const char *args;
    const char *help;
    void (*fn)(int argc, char *argv[]);
} cmd_t;

static void cmd_help(int argc, char *argv[]);

static void cmd_status(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    print_status();
}

static void cmd_start(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    action_start();
}

static void cmd_stop(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    action_stop();
}

static void cmd_home(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    action_home();
}

static void cmd_off(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    action_off();
}

static void cmd_clear(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    action_clear();
}

static void cmd_mode(int argc, char *argv[])
{
    if ((argc >= 2) && (strcmp(argv[1], "soft") == 0)) {
        g_mode = AX_CYCLE_SOFT;
    } else if ((argc >= 2) && (strcmp(argv[1], "bounce") == 0)) {
        g_mode = AX_CYCLE_BOUNCE;
    } else {
        console_printf("usage: mode soft|bounce\r\n");
        return;
    }
    console_printf("cycle mode: %s (applies at the next start)\r\n",
                   (g_mode == AX_CYCLE_SOFT) ? "soft limits, step counting" : "bounce, StallGuard4 at every end");
}

/* Parse "<cmd> <axis> <number>" with range check. */
static bool axis_value(int argc, char *argv[], float lo, float hi, int *ax, float *value)
{
    if ((argc < 3) || ((*ax = parse_axis(argv[1])) < 0) || !cmd_parse_float(argv[2], value) ||
        (*value < lo) || (*value > hi)) {
        console_printf("usage: %s <axis> <%g..%g>\r\n", argv[0], (double)lo, (double)hi);
        return false;
    }
    return true;
}

static void cmd_move(int argc, char *argv[])
{
    int ax;
    float mm;

    if (!axis_value(argc, argv, -10000.0f, 10000.0f, &ax, &mm)) {
        return;
    }
    if ((g_sys == SYS_RUNNING) || (g_sys == SYS_HOMING)) {
        console_printf("stop the cycle first\r\n");
        return;
    }
    if (!axis_ctrl_move_mm((uint8_t)ax, mm)) {
        console_printf("%c: not homed or busy\r\n", axis_name((uint8_t)ax));
        return;
    }
    set_sys(SYS_PAUSED);
}

/* Tunable parameters for "set <axis> <param> <value>" / "get [axis]". */
typedef enum {
    P_SPEED = 0, P_ACCEL, P_MARGIN, P_DWELL, P_HSPEED, P_HACCEL, P_RETRACT,
    P_SG, P_SGRATIO, P_SGMIN, P_BLANK, P_CONFIRM, P_CURRENT, P_HCURRENT, P_COUNT
} param_id_t;

typedef struct {
    const char *name;
    float       min;
    float       max;
    const char *unit;
    const char *help;
} param_t;

static const param_t k_params[P_COUNT] = {
    [P_SPEED]    = { "speed",    0.5f,  1000.0f,  "mm/s",   "cycle/move speed" },
    [P_ACCEL]    = { "accel",    10.0f, 50000.0f, "mm/s2",  "cycle/move acceleration" },
    [P_MARGIN]   = { "margin",   0.0f,  100.0f,   "mm",     "soft limit distance from the stops (next homing)" },
    [P_DWELL]    = { "dwell",    0.0f,  10000.0f, "ms",     "pause at each end" },
    [P_HSPEED]   = { "hspeed",   0.5f,  500.0f,   "mm/s",   "StallGuard4 seek speed" },
    [P_HACCEL]   = { "haccel",   10.0f, 50000.0f, "mm/s2",  "seek acceleration" },
    [P_RETRACT]  = { "retract",  0.5f,  50.0f,    "mm",     "back-off after a stall" },
    [P_SG]       = { "sg",       0.0f,  255.0f,   "",       "static SG4_THRS (stall at SG4 <= 2x)" },
    [P_SGRATIO]  = { "sgratio",  0.1f,  0.9f,     "",       "stall below this fraction of free-run SG4" },
    [P_SGMIN]    = { "sgmin",    0.0f,  500.0f,   "",       "lowest trusted free-run SG4" },
    [P_BLANK]    = { "blank",    0.0f,  2000.0f,  "ms",     "SG4 ignored after reaching seek speed" },
    [P_CONFIRM]  = { "confirm",  1.0f,  50.0f,    "",       "consecutive SG4 samples to confirm a stall" },
    [P_CURRENT]  = { "current",  100.0f, 2000.0f, "mA",     "RMS run current (re-initialises driver)" },
    [P_HCURRENT] = { "hcurrent", 50.0f, 2000.0f,  "mA",     "RMS homing current (re-initialises driver)" },
};

static float param_get(const axis_cfg_t *c, param_id_t p)
{
    switch (p) {
    case P_SPEED:    return c->cycle_speed_mm_s;
    case P_ACCEL:    return c->cycle_accel_mm_s2;
    case P_MARGIN:   return c->cycle_margin_mm;
    case P_DWELL:    return (float)c->cycle_dwell_ms;
    case P_HSPEED:   return c->home_speed_mm_s;
    case P_HACCEL:   return c->home_accel_mm_s2;
    case P_RETRACT:  return c->home_retract_mm;
    case P_SG:       return (float)c->tmc.sg4_thrs_home;
    case P_SGRATIO:  return c->stall.drop_ratio;
    case P_SGMIN:    return (float)c->stall.min_baseline;
    case P_BLANK:    return (float)c->stall.blank_ms;
    case P_CONFIRM:  return (float)c->stall.confirm;
    case P_CURRENT:  return (float)c->tmc.run_current_ma;
    case P_HCURRENT: return (float)c->tmc.home_current_ma;
    default:         return 0.0f;
    }
}

static void reinit_driver(uint8_t ax)
{
    axis_ctrl_disable(ax);
    tmc_axis_mark_unconfigured(ax);
    axis_ctrl_set_driver_ready(ax, false); /* probe_drivers() re-configures it */
}

static bool param_set(uint8_t ax, param_id_t p, float v)
{
    axis_cfg_t *c = axis_ctrl_cfg(ax);

    if (((p == P_CURRENT) || (p == P_HCURRENT)) && axis_ctrl_is_busy(ax)) {
        console_printf("stop the axis first\r\n");
        return false;
    }
    switch (p) {
    case P_SPEED:   c->cycle_speed_mm_s = v; break;
    case P_ACCEL:   c->cycle_accel_mm_s2 = v; break;
    case P_MARGIN:  c->cycle_margin_mm = v; break;
    case P_DWELL:   c->cycle_dwell_ms = (uint16_t)v; break;
    case P_HSPEED:  c->home_speed_mm_s = v; axis_ctrl_config_changed(ax); break; /* TCOOLTHRS */
    case P_HACCEL:  c->home_accel_mm_s2 = v; break;
    case P_RETRACT: c->home_retract_mm = v; break;
    case P_SG:
        c->tmc.sg4_thrs_home = (uint8_t)v;
        c->stall.abs_threshold = (uint16_t)(2U * (uint16_t)c->tmc.sg4_thrs_home);
        axis_ctrl_config_changed(ax);
        break;
    case P_SGRATIO: c->stall.drop_ratio = v; break;
    case P_SGMIN:   c->stall.min_baseline = (uint16_t)v; break;
    case P_BLANK:   c->stall.blank_ms = (uint16_t)v; break;
    case P_CONFIRM: c->stall.confirm = (uint8_t)v; break;
    case P_CURRENT:
        c->tmc.run_current_ma = (uint16_t)v;
        if (c->tmc.home_current_ma > c->tmc.run_current_ma) {
            c->tmc.home_current_ma = c->tmc.run_current_ma;
        }
        reinit_driver(ax);
        break;
    case P_HCURRENT:
        c->tmc.home_current_ma = (uint16_t)((v > (float)c->tmc.run_current_ma) ? (float)c->tmc.run_current_ma : v);
        reinit_driver(ax);
        break;
    default:
        return false;
    }
    return true;
}

static void print_params(uint8_t ax)
{
    const axis_cfg_t *c = axis_ctrl_cfg(ax);
    unsigned i;

    console_printf("  %c:", c->name);
    for (i = 0U; i < (unsigned)P_COUNT; i++) {
        console_printf(" %s=%g", k_params[i].name, (double)param_get(c, (param_id_t)i));
    }
    console_printf("\r\n");
}

static void cmd_get(int argc, char *argv[])
{
    uint8_t ax;
    const int only = (argc >= 2) ? parse_axis(argv[1]) : -1;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        if ((only < 0) || ((int)ax == only)) {
            print_params(ax);
        }
    }
}

static void cmd_set(int argc, char *argv[])
{
    int ax;
    unsigned i;
    float v;

    if ((argc < 4) || ((ax = parse_axis(argv[1])) < 0)) {
        console_printf("usage: set <axis> <param> <value>; params:\r\n");
        for (i = 0U; i < (unsigned)P_COUNT; i++) {
            console_printf("  %-8s %g..%g %-5s %s\r\n", k_params[i].name, (double)k_params[i].min,
                           (double)k_params[i].max, k_params[i].unit, k_params[i].help);
        }
        return;
    }
    for (i = 0U; i < (unsigned)P_COUNT; i++) {
        if (strcmp(argv[2], k_params[i].name) == 0) {
            break;
        }
    }
    if (i == (unsigned)P_COUNT) {
        console_printf("unknown parameter '%s' (type 'set' for the list)\r\n", argv[2]);
        return;
    }
    if (!cmd_parse_float(argv[3], &v) || (v < k_params[i].min) || (v > k_params[i].max)) {
        console_printf("%s: value must be %g..%g %s\r\n", k_params[i].name, (double)k_params[i].min,
                       (double)k_params[i].max, k_params[i].unit);
        return;
    }
    if (param_set((uint8_t)ax, (param_id_t)i, v)) {
        console_printf("%c: %s = %g %s\r\n", axis_name((uint8_t)ax), k_params[i].name,
                       (double)param_get(axis_ctrl_cfg((uint8_t)ax), (param_id_t)i), k_params[i].unit);
    }
}
static void print_encoder(uint8_t ax)
{
    const encoder_info_t *ei = encoder_info(ax);

    if (ei == NULL) {
        return;
    }
    console_printf("  AS5600 %c: %s, last %s, raw %u, counts %ld, STATUS 0x%02X (%s%s%s), AGC %u, "
                   "magnitude %u, errors %lu, recoveries %lu, epoch %lu\r\n",
                   axis_name(ax), ei->healthy ? "healthy" : "NOT healthy",
                   encoder_result_name(ei->last_result), (unsigned)ei->raw, (long)ei->counts,
                   (unsigned)ei->status, (ei->status & AS5600_STATUS_MAGNET_FOUND) ? "magnet" : "NO magnet",
                   (ei->status & AS5600_STATUS_MAGNET_LOW) ? ", weak" : "",
                   (ei->status & AS5600_STATUS_MAGNET_HIGH) ? ", strong" : "", (unsigned)ei->agc,
                   (unsigned)ei->magnitude, (unsigned long)ei->errors, (unsigned long)ei->recoveries,
                   (unsigned long)ei->epoch);
}

static void cmd_diag(int argc, char *argv[])
{
    uint8_t ax;
    const int only = (argc >= 2) ? parse_axis(argv[1]) : -1;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        const axis_status_t *st = axis_ctrl_status(ax);
        if ((only >= 0) && ((int)ax != only)) {
            continue;
        }
        console_printf("--- axis %c: %s, fault '%s', homed %d, travel %.2f mm, soft %.2f..%.2f mm\r\n",
                       axis_name(ax), axis_ctrl_state_name(st->state), axis_ctrl_fault_name(st->fault),
                       (int)st->homed, (double)axis_ctrl_steps_to_mm(ax, st->travel_steps),
                       (double)axis_ctrl_steps_to_mm(ax, st->soft_min),
                       (double)axis_ctrl_steps_to_mm(ax, st->soft_max));
        console_printf("    StallGuard4: SG4 %u, baseline %.0f, SG4_THRS %u, last stall %s; "
                       "encoder ratio %.4f (%s), peak err %.3f mm; bounce dev %.3f/max %.3f mm\r\n",
                       (unsigned)st->sg, (double)st->sg_baseline, (unsigned)st->sg_thr,
                       stall_source_name(st->last_stall), (double)st->enc_ratio,
                       st->enc_ok ? "calibrated" : "not used",
                       (double)axis_ctrl_steps_to_mm(ax, st->follow_err_peak),
                       (double)axis_ctrl_steps_to_mm(ax, st->last_dev_steps),
                       (double)axis_ctrl_steps_to_mm(ax, st->max_dev_steps));
        tmc_axis_dump(ax, console_printf);
        print_encoder(ax);
    }
    console_printf("  console dropped %lu bytes, step timer %lu Hz\r\n",
                   (unsigned long)console_dropped_bytes(), (unsigned long)stepgen_timer_hz(0U));
}

static void cmd_log(int argc, char *argv[])
{
    uint32_t level;

    if ((argc < 2) || !cmd_parse_u32(argv[1], &level) || (level > 2U)) {
        console_printf("usage: log 0|1|2  (events | +status | +SG4 trace)\r\n");
        return;
    }
    g_log_level = (uint8_t)level;
}

static const cmd_t k_cmds[] = {
    { "help",    "",               "this list", cmd_help },
    { "status",  "",               "state of all axes", cmd_status },
    { "start",   "",               "home (if needed), then cycle all axes", cmd_start },
    { "stop",    "",               "decelerate and hold position", cmd_stop },
    { "home",    "",               "(re)home all axes with StallGuard4", cmd_home },
    { "off",     "",               "disable the drivers (motors free)", cmd_off },
    { "clear",   "",               "clear faults (drivers re-initialise)", cmd_clear },
    { "mode",    "soft|bounce",    "cycle between soft limits / stop to stop", cmd_mode },
    { "move",    "<axis> <mm>",    "move a homed, idle axis", cmd_move },
    { "set",     "<axis> <p> <v>", "change a parameter ('set' lists them)", cmd_set },
    { "get",     "[axis]",         "show the parameters", cmd_get },
    { "diag",    "[axis]",         "TMC2240 registers, StallGuard, AS5600", cmd_diag },
    { "log",     "0|1|2",          "telemetry: events / status / SG4 trace", cmd_log },
};

static void cmd_help(int argc, char *argv[])
{
    size_t i;

    (void)argc;
    (void)argv;
    console_printf("Commands (axis = %c or %c; B1 short press = start/stop, long press = off):\r\n",
                   axis_name(0U), axis_name(1U));
    for (i = 0U; i < sizeof(k_cmds) / sizeof(k_cmds[0]); i++) {
        console_printf("  %-8s %-18s %s\r\n", k_cmds[i].name, k_cmds[i].args, k_cmds[i].help);
    }
}

static void execute(char *line)
{
    char *argv[CMD_ARGV_MAX];
    const int argc = cmd_tokenize(line, argv, CMD_ARGV_MAX);
    size_t i;

    if (argc == 0) {
        return;
    }
    for (i = 0U; i < sizeof(k_cmds) / sizeof(k_cmds[0]); i++) {
        if (strcmp(argv[0], k_cmds[i].name) == 0) {
            k_cmds[i].fn(argc, argv);
            return;
        }
    }
    console_printf("unknown command '%s' - type 'help'\r\n", argv[0]);
}

/* ------------------------------------------------------------------------ */
/* Supervision                                                              */
/* ------------------------------------------------------------------------ */

static void probe_drivers(void)
{
    uint8_t ax;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        const tmc_axis_info_t *ti = tmc_axis_info(ax);
        TMC2240Status r;

        if ((ti == NULL) || ti->configured || (axis_ctrl_status(ax)->state == AX_ST_FAULT)) {
            continue;
        }
        r = tmc_axis_configure(ax, axis_ctrl_cfg(ax));
        if (r == TMC2240_OK) {
            LOG("[TMC] %c: TMC2240 #%u ready - %u mA run / %u mA hold / %u mA homing "
                "(range %u, GS %u), 1/%u step + MicroPlyer, StealthChop2%s, CoolStep %s%s",
                axis_name(ax), (unsigned)ax, (unsigned)ti->current.irun_ma, (unsigned)ti->current.ihold_ma,
                (unsigned)ti->current.ihome_ma, (unsigned)ti->current.range, (unsigned)ti->current.gs,
                (unsigned)axis_ctrl_cfg(ax)->tmc.microsteps,
                (axis_ctrl_cfg(ax)->tmc.stealth_max_mm_s > 0.0f) ? " + SpreadCycle" : "",
                axis_ctrl_cfg(ax)->tmc.coolstep_enable ? "on" : "off",
                ((ti->gstat_at_init & 0x02U) != 0U) ? " (previous driver error acknowledged)" : "");
            g_probe_fail[ax] = 0U;
            axis_ctrl_set_driver_ready(ax, true);
        } else {
            if ((g_probe_fail[ax] % 20U) == 0U) {
                LOG("[TMC] %c: TMC2240 #%u not ready (%s) - VM (24 V) on? SPI2/CS wiring?",
                    axis_name(ax), (unsigned)ax, tmc_status_name(r));
            }
            g_probe_fail[ax]++;
        }
    }
}

static void poll_health(void)
{
    const uint8_t ax = (uint8_t)(g_health_rr++ % APP_AXIS_COUNT);
    const tmc_axis_info_t *ti = tmc_axis_info(ax);
    tmc_health_t h;

    if ((ti == NULL) || !ti->configured) {
        return;
    }
    h = tmc_axis_poll_health(ax);
    switch (h) {
    case TMC_HEALTH_OK:
        g_warned[ax] = false;
        break;
    case TMC_HEALTH_WARN:
        if (!g_warned[ax]) {
            g_warned[ax] = true;
            LOG("[TMC] %c: warning -%s%s (DRV_STATUS 0x%08lX)", axis_name(ax),
                ti->otpw ? " over-temperature pre-warning" : "", ti->open_load ? " open load (motor wiring?)" : "",
                (unsigned long)ti->drv_status);
        }
        break;
    default:
        LOG("[TMC] %c: %s (SPI status 0x%02X, DRV_STATUS 0x%08lX) - driver stopped",
            axis_name(ax),
            (h == TMC_HEALTH_RESET) ? "chip reset (VM dropped?)"
            : (h == TMC_HEALTH_SPI) ? "SPI communication lost" : "driver shut down (over-temperature/short)",
            (unsigned)ti->spi_status, (unsigned long)ti->drv_status);
        tmc_axis_mark_unconfigured(ax);
        axis_ctrl_fault(ax, (h == TMC_HEALTH_SPI) ? AX_FAULT_SPI : AX_FAULT_DRIVER);
        axis_ctrl_set_driver_ready(ax, false);
        break;
    }
}

static void poll_adc(void)
{
    uint8_t ax;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        const tmc_axis_info_t *ti = tmc_axis_info(ax);
        if ((ti == NULL) || !ti->configured) {
            continue;
        }
        tmc_axis_poll_adc(ax);
        if (!ti->adc_valid) {
            continue;
        }
        if ((ti->temp_c10 >= APP_TEMP_WARN_C10) && !g_temp_warned[ax]) {
            char temp[12];
            print_c10(temp, sizeof(temp), ti->temp_c10);
            LOG("[TMC] %c: driver temperature %s - improve cooling or reduce current", axis_name(ax), temp);
            g_temp_warned[ax] = true;
        } else if (ti->temp_c10 < (APP_TEMP_WARN_C10 - 100)) {
            g_temp_warned[ax] = false;
        }
        if (ti->vm_mv < APP_VM_MIN_MV) {
            LOG("[TMC] %c: VM only %lu mV", axis_name(ax), (unsigned long)ti->vm_mv);
        }
    }
}

static void watch_encoders(void)
{
    uint8_t ax;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        const encoder_info_t *ei = encoder_info(ax);
        if ((ei == NULL) || (ei->healthy == g_enc_was_ok[ax])) {
            continue;
        }
        g_enc_was_ok[ax] = ei->healthy;
        if (ei->healthy) {
            LOG("[ENC] %c: AS5600 ok (raw %u, AGC %u, magnitude %u%s%s)", axis_name(ax), (unsigned)ei->raw,
                (unsigned)ei->agc, (unsigned)ei->magnitude,
                (ei->status & AS5600_STATUS_MAGNET_LOW) ? ", magnet weak" : "",
                (ei->status & AS5600_STATUS_MAGNET_HIGH) ? ", magnet strong" : "");
        } else {
            LOG("[ENC] %c: AS5600 unavailable (%s) - running open loop, retrying", axis_name(ax),
                encoder_result_name(ei->last_result));
        }
    }
}

static void sequence(void)
{
    uint8_t ax;
    uint8_t i;
    bool any_present = false;
    bool any_fault = false;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        const axis_status_t *st = axis_ctrl_status(ax);
        if (st->state == AX_ST_DISABLED) {
            continue;
        }
        any_present = true;
        any_fault = any_fault || (st->state == AX_ST_FAULT);
    }
    if (any_fault && (g_sys != SYS_FAULT)) {
        for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
            g_home_pending[ax] = false;
            axis_ctrl_stop(ax); /* bring the healthy axis to a controlled stop */
        }
        set_sys(SYS_FAULT);
        LOG("[SYS] fix the cause, then press B1 or send 'clear' ('diag' for details)");
        return;
    }

    switch (g_sys) {
    case SYS_WAIT_DRIVERS:
        if (any_present) {
            set_sys(SYS_IDLE);
            LOG("[SYS] press B1 or type 'start' to home and cycle ('help' for commands)");
        }
        break;
    case SYS_IDLE:
        if ((APP_AUTOSTART != 0) && !g_autostart_done && any_present) {
            g_autostart_done = true;
            action_start();
        }
        break;
    case SYS_HOMING:
        for (i = 0U; i < APP_AXIS_COUNT; i++) {
            ax = k_home_order[i];
            if (!axis_present(ax)) {
                g_home_pending[ax] = false;
                continue;
            }
            if (axis_ctrl_is_busy(ax)) {
                return; /* one axis at a time */
            }
            if (g_home_pending[ax]) {
                g_home_pending[ax] = false;
                if (!axis_ctrl_home(ax)) {
                    LOG("[SYS] %c: cannot start homing in state %s", axis_name(ax),
                        axis_ctrl_state_name(axis_ctrl_status(ax)->state));
                }
                return;
            }
        }
        if (g_cycle_after_home) {
            start_cycles();
        } else {
            set_sys(SYS_PAUSED);
        }
        break;
    case SYS_RUNNING:
        for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
            const axis_status_t *st = axis_ctrl_status(ax);
            if (st->homed && (st->state == AX_ST_READY)) {
                (void)axis_ctrl_cycle(ax, g_mode);
            }
        }
        break;
    case SYS_PAUSED:
    case SYS_FAULT:
    default:
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Telemetry, LED, button                                                   */
/* ------------------------------------------------------------------------ */

static void telemetry(uint32_t now)
{
    uint8_t ax;
    bool busy = false;

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        busy = busy || axis_ctrl_is_busy(ax);
    }
    if ((g_log_level >= 1U) &&
        ((now - g_t_status) >= (busy ? APP_STATUS_PERIOD_MS : APP_IDLE_STATUS_MS))) {
        g_t_status = now;
        print_status();
    }
    if ((g_log_level >= 2U) && ((now - g_t_trace) >= APP_SG_TRACE_PERIOD_MS)) {
        g_t_trace = now;
        for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
            const axis_status_t *st = axis_ctrl_status(ax);
            if ((st->state == AX_ST_HOME_BACKOFF) || (st->state == AX_ST_HOME_SEEK1) ||
                (st->state == AX_ST_HOME_SEEK2) || (st->state == AX_ST_BOUNCE_SEEK)) {
                console_printf("sg,%c,%lu,%.2f,%u,%.0f,%u,%d\r\n", axis_name(ax), (unsigned long)now,
                               (double)axis_ctrl_steps_to_mm(ax, st->pos_steps), (unsigned)st->sg,
                               (double)st->sg_baseline, (unsigned)st->sg_thr, (int)st->sg_flag);
            }
        }
    }
}

static void led_update(uint32_t now)
{
    bool on;

    switch (g_sys) {
    case SYS_WAIT_DRIVERS: on = (now % 1000U) < 500U; break;   /* 1 Hz */
    case SYS_IDLE:         on = (now % 2000U) < 60U;  break;   /* heartbeat */
    case SYS_HOMING:       on = (now % 250U) < 125U;  break;   /* 4 Hz */
    case SYS_RUNNING:      on = true;                 break;
    case SYS_PAUSED:       on = (now % 2000U) < 1000U; break;  /* 0.5 Hz */
    case SYS_FAULT:
    default:               on = (now % 100U) < 50U;   break;   /* 10 Hz */
    }
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void button_update(uint32_t now)
{
    const bool pressed = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;

    if (pressed != g_btn_raw) {
        g_btn_raw = pressed;
        g_btn_t_change = now;
    }
    if (((now - g_btn_t_change) >= 30U) && (g_btn_stable != g_btn_raw)) {
        g_btn_stable = g_btn_raw;
        if (g_btn_stable) {
            g_btn_t_press = now;
            g_btn_long = false;
        } else if (!g_btn_long) {
            if (g_sys == SYS_FAULT) {
                action_clear();
            } else if ((g_sys == SYS_RUNNING) || (g_sys == SYS_HOMING)) {
                action_stop();
            } else {
                action_start();
            }
        }
    }
    if (g_btn_stable && !g_btn_long && ((now - g_btn_t_press) >= APP_BUTTON_LONG_MS)) {
        g_btn_long = true;
        action_off();
    }
}

/* ------------------------------------------------------------------------ */
/* Entry points                                                             */
/* ------------------------------------------------------------------------ */

static void banner(void)
{
    uint8_t ax;

    console_printf("\r\n==============================================================\r\n");
    console_printf(" %s %s - XZ manipulator motion controller\r\n", FW_NAME, FW_VERSION);
    console_printf(" STM32F446RE @ %lu MHz, 2x TMC2240 (SPI2) + 2x AS5600 (I2C)\r\n",
                   (unsigned long)(SystemCoreClock / 1000000U));
    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        const axis_cfg_t *c = axis_ctrl_cfg(ax);
        console_printf(" %c: %.0f steps/mm, home %s first @ %.0f mm/s, cycle %.0f mm/s, %u/%u mA\r\n",
                       c->name, (double)c->steps_per_mm, c->home_min_first ? "MIN" : "MAX",
                       (double)c->home_speed_mm_s, (double)c->cycle_speed_mm_s,
                       (unsigned)c->tmc.run_current_ma, (unsigned)c->tmc.home_current_ma);
    }
    console_printf(" console: USART2 115200 8N1 or RTT terminal 0 - type 'help'\r\n");
    console_printf("==============================================================\r\n");
}

void app_init(void)
{
    static const TMC2240_SPIConfig_t spi_cfgs[APP_AXIS_COUNT] = {
        { &hspi2, TMC_CS0_GPIO_Port, TMC_CS0_Pin, 10U },
        { &hspi2, TMC_CS1_GPIO_Port, TMC_CS1_Pin, 10U },
    };
    static const stepgen_hw_t step_hw[APP_AXIS_COUNT] = {
        { TIM2, TIM2_IRQn, TMC_STEP0_GPIO_Port, TMC_STEP0_Pin, TMC_DIR0_GPIO_Port, TMC_DIR0_Pin },
        { TIM5, TIM5_IRQn, TMC_STEP1_GPIO_Port, TMC_STEP1_Pin, TMC_DIR1_GPIO_Port, TMC_DIR1_Pin },
    };
    I2C_HandleTypeDef *const enc_buses[APP_AXIS_COUNT] = { &APP_AXIS0_ENCODER_I2C, &APP_AXIS1_ENCODER_I2C };
    uint8_t ax;

    /* Power stages stay off (ENN high) until an axis is homed or moved. */
    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        axis_hw_driver_enable(ax, false);
    }
    console_init(&huart2);
    cmd_linebuf_init(&g_line);
    banner();

    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        stepgen_init(ax, &step_hw[ax]);
        axis_ctrl_init(ax, &g_app_axis_cfg[ax]);
    }
    if (!tmc_axis_bus_init(spi_cfgs, APP_AXIS_COUNT)) {
        LOG("[TMC] SPI transport initialisation failed");
    }
    encoder_init(APP_AXIS_COUNT, enc_buses);
    watch_encoders();
    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        const encoder_info_t *ei = encoder_info(ax);
        if ((ei != NULL) && !ei->healthy) {
            LOG("[ENC] %c: AS5600 not available (%s) - check wiring/magnet; homing still works "
                "with StallGuard4 only", axis_name(ax), encoder_result_name(ei->last_result));
        }
    }
    g_sys = SYS_WAIT_DRIVERS;
    g_last_tick = HAL_GetTick();
    g_t_probe = g_last_tick - APP_DRIVER_PROBE_MS; /* probe immediately */
}

void app_run(void)
{
    const uint32_t now = HAL_GetTick();
    uint8_t ax;
    int c;

    console_poll();
    while ((c = console_getc()) >= 0) {
        if (cmd_linebuf_push(&g_line, (char)c)) {
            execute(g_line.buf);
        }
    }
    if (now == g_last_tick) {
        return;
    }
    g_last_tick = now;

    button_update(now);
    if ((now - g_t_enc) >= APP_ENCODER_PERIOD_MS) {
        g_t_enc = now;
        encoder_poll();
    }
    for (ax = 0U; ax < APP_AXIS_COUNT; ax++) {
        axis_ctrl_step(ax);
    }
    if ((now - g_t_enc_diag) >= 250U) {
        g_t_enc_diag = now;
        encoder_poll_diag();
        watch_encoders();
    }
    if ((now - g_t_probe) >= APP_DRIVER_PROBE_MS) {
        g_t_probe = now;
        probe_drivers();
    }
    if ((now - g_t_health) >= (APP_HEALTH_PERIOD_MS / APP_AXIS_COUNT)) {
        g_t_health = now;
        poll_health();
    }
    if ((now - g_t_adc) >= APP_ADC_PERIOD_MS) {
        g_t_adc = now;
        poll_adc();
    }
    sequence();
    telemetry(now);
    led_update(now);
}
