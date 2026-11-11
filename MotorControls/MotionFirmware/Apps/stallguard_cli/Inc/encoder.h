/**
 * @file    encoder.h
 * @brief   Dual AS5600 magnetic encoder manager (one sensor per I2C bus).
 *
 * Uses the checked AS5600 driver (MotorControls/Drivers/AS5600) with its
 * STM32 HAL port in bare-metal mode. RAW ANGLE is read every 2 ms from the
 * main loop and unwrapped into a multi-turn count (enc_track.c); STATUS/AGC/
 * MAGNITUDE are checked periodically. Failed buses are recovered (9-clock
 * SCL bus clear + peripheral re-init) and the tracking epoch is incremented
 * whenever multi-turn continuity may have been lost.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "as5600.h"
#include "axis_hw.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_MAX 2U

typedef struct {
    bool            bound;       /* driver bound to its bus */
    bool            healthy;     /* reads succeed and a magnet is detected */
    uint8_t         status;      /* STATUS: MD 0x20, ML 0x10, MH 0x08 */
    uint8_t         agc;
    uint16_t        magnitude;
    uint16_t        raw;         /* last RAW ANGLE, 0..4095 */
    int32_t         counts;      /* multi-turn counts */
    int32_t         pos_snapshot;/* commanded position at the last reading */
    uint32_t        epoch;
    uint32_t        seq;
    uint32_t        t_last_ok;
    uint32_t        errors;
    uint8_t         error_run;
    uint32_t        recoveries;
    as5600_result_t last_result;
} encoder_info_t;

void  encoder_init(uint8_t count, I2C_HandleTypeDef *const buses[]);
/** Read RAW ANGLE of every healthy encoder (call every 2 ms). */
void  encoder_poll(void);
/** Magnet diagnostics and bus recovery (call every ~250 ms). */
void  encoder_poll_diag(void);
/** Fresh sample of a healthy encoder, false otherwise. */
bool  encoder_sample(uint8_t ax, axis_enc_sample_t *sample);
const encoder_info_t *encoder_info(uint8_t ax);
const char *encoder_result_name(as5600_result_t r);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
