#ifndef MOTION_EXECUTOR_H
#define MOTION_EXECUTOR_H
#include "homing.h"
#include "uart_transport.h"
typedef enum { EXEC_BOOT, EXEC_HOMING, EXEC_READY, EXEC_MOVING, EXEC_ABORTED, EXEC_FAULT } executor_state_t;
typedef enum { PHASE_START, PHASE_RUN, PHASE_SETTLE, PHASE_DWELL } executor_phase_state_t;
typedef struct {
    vision_cmd_t command;
    uint32_t seq;
    int32_t applied_x;
    bool clamped;
} queued_command_t;
typedef struct {
    const motion_config_t *config;
    step_engine_t *steps;
    encoder_sampler_t *encoders;
    driver_evidence_t *drivers;
    uart_transport_t *uart;
    homing_t home;
    executor_state_t state;
    executor_phase_state_t phase_state;
    queued_command_t queue[MOTION_QUEUE_SIZE];
    uint32_t qhead, qtail, command_seq, home_epoch, faults, rejected, status_us, settle_us;
    uint32_t status_rx_overflows, status_rx_crc_errors, status_rx_format_errors;
    uint8_t homed_mask, phase;
    bool active, home_pending;
    diag_record_t result, home_record;
    bool (*safe_inputs)(void *);
    void *safe_context;
} motion_executor_t;
void motion_executor_init(motion_executor_t *, const motion_config_t *, step_engine_t *,
                          encoder_sampler_t *, driver_evidence_t *, uart_transport_t *);
void motion_executor_boot(motion_executor_t *, uint32_t);
void motion_executor_poll(motion_executor_t *, uint32_t);
void motion_executor_submit(motion_executor_t *, const vision_cmd_t *, uint32_t);
void motion_executor_status(motion_executor_t *, const vision_cmd_t *, uint32_t, bool, uint32_t);
#endif
