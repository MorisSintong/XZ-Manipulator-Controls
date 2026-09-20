#ifndef FAKE_SEMPHR_H
#define FAKE_SEMPHR_H
#include "FreeRTOS.h"
typedef struct {
    BaseType_t allocated, available, mutex;
} fake_semaphore_t;
typedef fake_semaphore_t *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateBinary(void);
void vSemaphoreDelete(SemaphoreHandle_t s);
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks);
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t s, BaseType_t *woken);
#endif
