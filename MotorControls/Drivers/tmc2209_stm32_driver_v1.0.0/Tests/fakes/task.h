#ifndef FAKE_TASK_H
#define FAKE_TASK_H
#include "FreeRTOS.h"
typedef struct { BaseType_t xOverflowCount; TickType_t xTimeOnEntering; } TimeOut_t;
void vTaskDelay(TickType_t ticks);
void vTaskSetTimeOutState(TimeOut_t *snapshot);
#endif
