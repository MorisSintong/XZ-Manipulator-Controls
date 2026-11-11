/**
 * @file    enc_track.h
 * @brief   Multi-turn tracking of a 12-bit absolute angle (AS5600 RAW ANGLE).
 *
 * Pure C. The AS5600 reports 0..4095 per revolution; successive samples are
 * unwrapped into a signed multi-turn count, which is valid as long as the
 * shaft turns less than half a revolution between two samples
 * (at a 2 ms sample period that allows up to 250 rev/s).
 */
#ifndef ENC_TRACK_H
#define ENC_TRACK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENC_COUNTS_PER_REV 4096

typedef struct {
    int32_t  counts;   /* multi-turn position in encoder counts */
    uint16_t last_raw;
    bool     primed;
} enc_track_t;

void    enc_track_reset(enc_track_t *t);
/** Feed a raw 12-bit angle, returns the updated multi-turn count. */
int32_t enc_track_update(enc_track_t *t, uint16_t raw12);
/** Shortest signed difference between two raw angles (-2048..2047). */
int32_t enc_track_delta(uint16_t from_raw, uint16_t to_raw);

#ifdef __cplusplus
}
#endif

#endif /* ENC_TRACK_H */
