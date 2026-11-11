/**
 * @file    axis_ctrl.h
 * @brief   Per-axis controller: StallGuard4 sensorless homing, continuous
 *          back-and-forth cycling, point-to-point moves and supervision.
 *
 * Homing (per axis):
 *   PREP      enable driver, HOMING profile, standstill for StealthChop AT#1
 *   BACKOFF   short move away from the first end (running start)
 *   SEEK1     constant-speed run towards the first end until StallGuard4
 *   RETRACT1  back off from the stop
 *   SEEK2     run across the whole axis to the other end (encoder ratio is
 *             calibrated on this traverse)
 *   RETRACT2  back off, then the travel is computed, MIN end = position 0,
 *             the RUN profile is applied and the axis is READY.
 *
 * Cycle modes:
 *   AX_CYCLE_SOFT    step-counted moves between [margin, travel - margin]
 *   AX_CYCLE_BOUNCE  every stroke ends on the hard stop detected by
 *                    StallGuard4 (reports repeatability per stroke)
 */
#ifndef AXIS_CTRL_H
#define AXIS_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include "axis_cfg.h"
#include "stall_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef AXIS_CTRL_MAX_AXES
#define AXIS_CTRL_MAX_AXES 2U
#endif

typedef enum {
    AX_ST_DISABLED = 0,  /* driver not configured */
    AX_ST_IDLE,          /* driver ready, not homed */
    AX_ST_HOME_PREP,
    AX_ST_HOME_BACKOFF,
    AX_ST_HOME_SEEK1,
    AX_ST_HOME_SETTLE1,
    AX_ST_HOME_RETRACT1,
    AX_ST_HOME_SEEK2,
    AX_ST_HOME_SETTLE2,
    AX_ST_HOME_RETRACT2,
    AX_ST_READY,         /* homed, holding position */
    AX_ST_MOVE,          /* single point-to-point move */
    AX_ST_CYCLE_MOVE,
    AX_ST_CYCLE_DWELL,
    AX_ST_BOUNCE_SEEK,
    AX_ST_BOUNCE_SETTLE,
    AX_ST_BOUNCE_RETRACT,
    AX_ST_STOPPING,      /* decelerating after a stop request */
    AX_ST_FAULT,
    AX_ST_COUNT
} axis_state_t;

typedef enum {
    AX_FAULT_NONE = 0,
    AX_FAULT_DRIVER,       /* TMC2240 reset / over-temperature / short */
    AX_FAULT_SPI,          /* SPI communication failure */
    AX_FAULT_NO_STALL,     /* travelled the seek limit without detecting the stop */
    AX_FAULT_TIMEOUT,      /* seek took longer than seek_timeout_ms */
    AX_FAULT_SHORT_TRAVEL, /* measured travel below min_travel_mm */
    AX_FAULT_STEP_LOSS,    /* encoder disagrees with the commanded position */
    AX_FAULT_CRASH,        /* StallGuard4 stall while cycling */
    AX_FAULT_LIMIT,        /* software travel limit reached unexpectedly */
    AX_FAULT_INTERNAL
} axis_fault_t;

typedef enum {
    AX_CYCLE_SOFT = 0,
    AX_CYCLE_BOUNCE
} axis_cycle_mode_t;

typedef struct {
    axis_state_t      state;
    axis_fault_t      fault;
    bool              homed;
    bool              cycling;
    axis_cycle_mode_t cycle_mode;

    int32_t  pos_steps;       /* commanded position (0 = MIN end) */
    float    speed_steps_s;   /* signed commanded velocity */
    int32_t  travel_steps;    /* measured MIN..MAX distance */
    int32_t  soft_min;
    int32_t  soft_max;
    int32_t  home_travel_cmd; /* from commanded steps */
    int32_t  home_travel_enc; /* from the encoder (0 if unavailable) */
    uint32_t home_count;

    uint16_t sg;              /* last SG4_RESULT */
    bool     sg_flag;         /* last TMC2240 stallGuard flag */
    float    sg_baseline;
    uint8_t  sg_thr;          /* SG4_THRS currently programmed */
    stall_source_t last_stall;

    bool     enc_valid;       /* fresh encoder sample available */
    bool     enc_ok;          /* encoder calibrated against the steps */
    float    enc_ratio;       /* counts per microstep (signed) */
    int32_t  enc_counts;      /* multi-turn counts */
    int32_t  enc_pos_steps;   /* encoder-derived position (if enc_ok) */
    int32_t  follow_err;      /* encoder - commanded, microsteps */
    int32_t  follow_err_peak;

    uint32_t cycles;          /* completed MIN->MAX->MIN cycles */
    uint32_t strokes;         /* bounce mode stall strokes */
    int32_t  last_dev_steps;  /* bounce mode end-stop deviation */
    int32_t  max_dev_steps;
    int32_t  last_travel_steps;
    uint32_t fault_count;
} axis_status_t;

void  axis_ctrl_init(uint8_t ax, const axis_cfg_t *cfg);
/** Runtime-mutable configuration (e.g. speed/threshold commands). */
axis_cfg_t *axis_ctrl_cfg(uint8_t ax);
/** Run the state machine; call every millisecond from the main loop. */
void  axis_ctrl_step(uint8_t ax);

void  axis_ctrl_set_driver_ready(uint8_t ax, bool ready);
/** Configuration was edited: re-write the driver profile before next use. */
void  axis_ctrl_config_changed(uint8_t ax);
bool  axis_ctrl_home(uint8_t ax);
bool  axis_ctrl_cycle(uint8_t ax, axis_cycle_mode_t mode);
bool  axis_ctrl_move_mm(uint8_t ax, float target_mm);
void  axis_ctrl_stop(uint8_t ax);
void  axis_ctrl_disable(uint8_t ax);
void  axis_ctrl_fault(uint8_t ax, axis_fault_t fault);
bool  axis_ctrl_clear_fault(uint8_t ax);

const axis_status_t *axis_ctrl_status(uint8_t ax);
bool  axis_ctrl_is_busy(uint8_t ax); /* homing/moving/stopping */
float axis_ctrl_steps_to_mm(uint8_t ax, int32_t steps);
const char *axis_ctrl_state_name(axis_state_t s);
const char *axis_ctrl_fault_name(axis_fault_t f);

#ifdef __cplusplus
}
#endif

#endif /* AXIS_CTRL_H */
