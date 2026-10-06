#ifndef HOMING_H
#define HOMING_H
#include "step_engine.h"
#include "encoder_sampler.h"
#include "driver_evidence.h"
typedef enum {
    HOME_IDLE, HOME_PREP, HOME_SEEK, HOME_STOP, HOME_BACKOFF,
    HOME_LATCH, HOME_LATCH_STOP, HOME_CLEARANCE, HOME_SETTLE, HOME_DONE
} homing_state_t;
typedef struct {
    homing_state_t state;
    uint8_t axis, confirmations;
    uint32_t started_us, state_us, sg_poll_us;
    int32_t seek_start;
    bool repeat;
    diag_home_t result[2];
    step_engine_t *steps;
    encoder_sampler_t *encoders;
    driver_evidence_t *drivers;
    const motion_config_t *config;
} homing_t;
void homing_init(homing_t *, const motion_config_t *, step_engine_t *,
                 encoder_sampler_t *, driver_evidence_t *);
bool homing_begin(homing_t *, uint32_t);
void homing_poll(homing_t *, uint32_t);
bool homing_success(const homing_t *);
void homing_stop(homing_t *, uint8_t result, uint32_t now_us);
#endif
