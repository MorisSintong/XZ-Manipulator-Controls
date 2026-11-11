#ifndef STEP_ENGINE_H
#define STEP_ENGINE_H
#include "motion_config.h"
#include "motion_profile.h"
#define STEP_LOOKAHEAD 64U
typedef struct {
    void *context;
    uint32_t (*lock)(void *);
    void (*unlock)(void *, uint32_t);
    void (*step)(void *, uint8_t, bool);
    void (*direction)(void *, uint8_t, int8_t);
    void (*schedule)(void *, uint8_t, uint32_t, bool);
    uint32_t (*now_us)(void *);
} step_engine_io_t;
typedef struct {
    uint32_t period[STEP_LOOKAHEAD];
    volatile uint32_t head, tail;
    step_core_t planner;
    volatile int32_t position, emitted;
    volatile uint32_t edges, deadline, rise_deadline, start_us, done_us;
    volatile bool running, high, aborted, stop_requested, timing_fault, plan_done;
    int8_t direction;
} step_axis_t;
typedef struct {
    step_engine_io_t io;
    const motion_config_t *config;
    step_axis_t axis[2];
} step_engine_t;
typedef struct {
    int32_t position, emitted;
    uint32_t edges, start_us, done_us;
    bool running, high, aborted, timing_fault;
} step_snapshot_t;
void step_engine_init(step_engine_t *, const motion_config_t *, step_engine_io_t);
bool step_engine_move(step_engine_t *, uint8_t, int32_t, uint32_t);
void step_engine_poll(step_engine_t *);
void step_engine_irq(step_engine_t *, uint8_t, uint32_t);
void step_engine_abort(step_engine_t *);
void step_engine_stop(step_engine_t *, uint8_t);
bool step_engine_clear(step_engine_t *);
void step_engine_zero(step_engine_t *, uint8_t);
step_snapshot_t step_engine_snapshot(step_engine_t *, uint8_t);
#endif
