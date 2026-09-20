#include "fake_rtos.h"
#include <string.h>
#include "test_support.h"

fake_rtos_t rtos_fake;
void rtos_reset(void) { memset(&rtos_fake, 0, sizeof(rtos_fake)); }

static SemaphoreHandle_t allocate(BaseType_t mutex)
{
    if (rtos_fake.allocation_fails) return NULL;
    for (unsigned i = 0u; i < 8u; ++i) {
        if (!rtos_fake.pool[i].allocated) {
            rtos_fake.pool[i].allocated = pdTRUE;
            rtos_fake.pool[i].available = mutex;
            rtos_fake.pool[i].mutex = mutex;
            ++rtos_fake.allocations;
            return &rtos_fake.pool[i];
        }
    }
    return NULL;
}
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return allocate(pdTRUE); }
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return allocate(pdFALSE); }
void vSemaphoreDelete(SemaphoreHandle_t s)
{
    REQUIRE(s != NULL && s->allocated);
    ++rtos_fake.deletions;
    s->allocated = pdFALSE;
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks)
{
    REQUIRE(s != NULL && s->allocated);
    ++rtos_fake.takes;
    rtos_fake.last_wait = ticks;
    if (rtos_fake.take_fails || !s->available) return pdFALSE;
    s->available = pdFALSE;
    return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t s)
{
    REQUIRE(s != NULL && s->allocated);
    ++rtos_fake.gives;
    if (rtos_fake.give_fails || s->available) return pdFALSE;
    s->available = pdTRUE;
    return pdTRUE;
}
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t s, BaseType_t *woken)
{
    REQUIRE(s != NULL && s->allocated && !s->mutex);
    ++rtos_fake.isr_calls;
    *woken = rtos_fake.isr_woken;
    if (rtos_fake.isr_fails || s->available) return pdFALSE;
    s->available = pdTRUE;
    return pdTRUE;
}
void fake_rtos_yield(BaseType_t woken) { ++rtos_fake.yields; rtos_fake.yielded = woken; }
void vTaskDelay(TickType_t ticks)
{
    ++rtos_fake.delays;
    rtos_fake.last_delay = ticks;
    rtos_fake.total_ticks += ticks;
}
void vTaskSetTimeOutState(TimeOut_t *snapshot)
{
    snapshot->xTimeOnEntering = (TickType_t)rtos_fake.total_ticks;
    snapshot->xOverflowCount = (BaseType_t)(rtos_fake.total_ticks >> FAKE_TICK_BITS);
}
