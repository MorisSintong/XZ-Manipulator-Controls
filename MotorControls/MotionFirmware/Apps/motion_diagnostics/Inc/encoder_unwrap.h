#ifndef APP_ENCODER_UNWRAP_H
#define APP_ENCODER_UNWRAP_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t total_counts;
    uint16_t last_raw;
    uint16_t max_delta_counts;
    bool initialized;
    bool suspect_alias;
    bool overflow;
    uint32_t raw_errors;
    uint32_t alias_errors;
    uint32_t overflow_errors;
} enc_unwrap_t;

/** Initialize at raw, default threshold 2047; invalid raw awaits first valid sample. */
void enc_unwrap_init(enc_unwrap_t *e, uint16_t raw);
/** Accumulate shortest delta in (-2048,2048]; reject invalid raw, saturate overflow.
 * Exceeding caller-configured max_delta_counts marks sticky alias risk, not rejection.
 * A missed full turn cannot be detected from raw samples alone.
 */
int32_t enc_unwrap_update(enc_unwrap_t *e, uint16_t raw);
/** Zero accumulated displacement, retaining last raw, configuration and flags. */
void enc_unwrap_set_zero(enc_unwrap_t *e);
/** Return floor(total_counts / 4096), including negative displacement. */
int32_t enc_unwrap_turns(const enc_unwrap_t *e);
/** Return displacement within the turn in [0,4095]. */
uint16_t enc_unwrap_position(const enc_unwrap_t *e);

#endif
