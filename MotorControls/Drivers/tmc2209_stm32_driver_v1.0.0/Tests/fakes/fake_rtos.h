#ifndef FAKE_RTOS_H
#define FAKE_RTOS_H
#include "semphr.h"
#include "task.h"
typedef struct {
    fake_semaphore_t pool[8];
    BaseType_t allocation_fails, take_fails, give_fails, isr_fails, isr_woken;
    BaseType_t yielded;
    unsigned allocations, deletions, takes, gives, delays, isr_calls, yields;
    TickType_t last_wait, last_delay;
    uint64_t total_ticks;
} fake_rtos_t;
extern fake_rtos_t rtos_fake;
void rtos_reset(void);
#endif
