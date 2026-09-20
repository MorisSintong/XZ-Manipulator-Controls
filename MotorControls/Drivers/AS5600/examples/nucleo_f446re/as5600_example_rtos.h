#ifndef AS5600_EXAMPLE_RTOS_H
#define AS5600_EXAMPLE_RTOS_H

#include "as5600_stm32_hal.h"
#include "as5600_cmsis_rtos2.h"

as5600_result_t as5600_example_rtos_bind(
    I2C_HandleTypeDef *i2c, osMutexId_t shared_bus_mutex);
as5600_result_t as5600_example_rtos_poll(as5600_sample_t *sample);

#endif
