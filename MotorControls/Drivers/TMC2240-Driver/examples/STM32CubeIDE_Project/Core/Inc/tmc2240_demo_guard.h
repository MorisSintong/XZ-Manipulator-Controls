#ifndef TMC2240_DEMO_GUARD_H
#define TMC2240_DEMO_GUARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <limits.h>
#include <stdio.h>

typedef enum {
    TMC_DEMO_RUNNING = 0,
    TMC_DEMO_INVALID_CONFIGURATION,
    TMC_DEMO_DISTANCE_LIMIT,
    TMC_DEMO_TIME_LIMIT,
    TMC_DEMO_COMMUNICATION_FAILURE,
    TMC_DEMO_DRIVER_FAULT
} tmc_demo_stop_t;

typedef struct {
    uint32_t started_ms;
    uint32_t steps;
    uint32_t max_steps;
    uint32_t max_ms;
    tmc_demo_stop_t stop;
} tmc_demo_guard_t;

static inline bool tmc_demo_guard_init(tmc_demo_guard_t *guard, uint32_t now,
                                      uint32_t max_steps, uint32_t max_ms)
{
    if (guard == NULL) {
        return false;
    }
    guard->started_ms = now;
    guard->steps = 0U;
    guard->max_steps = max_steps;
    guard->max_ms = max_ms;
    guard->stop = TMC_DEMO_INVALID_CONFIGURATION;
    if (max_steps == 0U || max_steps > (uint32_t)INT32_MAX ||
        max_ms == 0U || max_ms > (uint32_t)INT32_MAX) {
        return false;
    }
    guard->stop = TMC_DEMO_RUNNING;
    return true;
}

static inline void tmc_demo_guard_stop(tmc_demo_guard_t *guard, tmc_demo_stop_t reason)
{
    if (guard != NULL && guard->stop == TMC_DEMO_RUNNING && reason != TMC_DEMO_RUNNING) {
        guard->stop = reason;
    }
}

static inline bool tmc_demo_guard_new_leg(tmc_demo_guard_t *guard, uint32_t now)
{
    if (guard == NULL || guard->stop != TMC_DEMO_RUNNING) {
        return false;
    }
    guard->steps = 0U;
    guard->started_ms = now;
    return true;
}

static inline bool tmc_demo_guard_step(tmc_demo_guard_t *guard, uint32_t now,
                                      int32_t position, int8_t direction)
{
    if (guard == NULL || guard->stop != TMC_DEMO_RUNNING) {
        return false;
    }
    if (direction != -1 && direction != 1) {
        guard->stop = TMC_DEMO_INVALID_CONFIGURATION;
        return false;
    }
    if ((uint32_t)(now - guard->started_ms) >= guard->max_ms) {
        guard->stop = TMC_DEMO_TIME_LIMIT;
        return false;
    }
    const int64_t next = (int64_t)position + (int64_t)direction;
    const int64_t bound = (int64_t)guard->max_steps;
    if (guard->steps >= guard->max_steps || next < -bound || next > bound) {
        guard->stop = TMC_DEMO_DISTANCE_LIMIT;
        return false;
    }
    guard->steps++;
    return true;
}

static inline bool tmc_demo_message_length(int length, size_t capacity, uint16_t *out)
{
    if (out == NULL || length < 0 || (size_t)length >= capacity ||
        (unsigned int)length > UINT16_MAX) {
        return false;
    }
    *out = (uint16_t)length;
    return true;
}

static inline bool tmc_demo_format_temperature(int16_t temperature, char *out, size_t capacity)
{
    if (out == NULL || capacity == 0U) {
        return false;
    }
    const int32_t magnitude = temperature < 0 ? -(int32_t)temperature : (int32_t)temperature;
    const int length = snprintf(out, capacity, "%s%ld.%ld", temperature < 0 ? "-" : "",
                               (long)(magnitude / 10), (long)(magnitude % 10));
    return length >= 0 && (size_t)length < capacity;
}

#endif
