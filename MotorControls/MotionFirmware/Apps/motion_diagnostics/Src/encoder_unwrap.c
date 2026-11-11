#include "encoder_unwrap.h"
#include <limits.h>

void enc_unwrap_init(enc_unwrap_t *e, uint16_t raw)
{
    *e = (enc_unwrap_t){0};
    e->max_delta_counts = 2047U;
    if (raw > 4095U) {
        e->raw_errors = 1U;
    } else {
        e->last_raw = raw;
        e->initialized = true;
    }
}

int32_t enc_unwrap_update(enc_unwrap_t *e, uint16_t raw)
{
    if (raw > 4095U) {
        ++e->raw_errors;
        return e->total_counts;
    }
    if (!e->initialized) {
        e->last_raw = raw;
        e->initialized = true;
        return e->total_counts;
    }
    int32_t delta = (int32_t)raw - (int32_t)e->last_raw;
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta <= -2048) {
        delta += 4096;
    }
    const int32_t magnitude = (delta < 0) ? -delta : delta;
    if (magnitude > (int32_t)e->max_delta_counts) {
        e->suspect_alias = true;
        ++e->alias_errors;
    }
    const int64_t sum = (int64_t)e->total_counts + (int64_t)delta;
    if ((sum > INT32_MAX) || (sum < INT32_MIN)) {
        e->overflow = true;
        ++e->overflow_errors;
        e->total_counts = (sum > INT32_MAX) ? INT32_MAX : INT32_MIN;
    } else {
        e->total_counts = (int32_t)sum;
    }
    e->last_raw = raw;
    return e->total_counts;
}

void enc_unwrap_set_zero(enc_unwrap_t *e)
{
    e->total_counts = 0;
}

int32_t enc_unwrap_turns(const enc_unwrap_t *e)
{
    int32_t turns = e->total_counts / 4096;
    if ((e->total_counts % 4096) < 0) {
        --turns;
    }
    return turns;
}

uint16_t enc_unwrap_position(const enc_unwrap_t *e)
{
    int32_t position = e->total_counts % 4096;
    if (position < 0) {
        position += 4096;
    }
    return (uint16_t)position;
}
