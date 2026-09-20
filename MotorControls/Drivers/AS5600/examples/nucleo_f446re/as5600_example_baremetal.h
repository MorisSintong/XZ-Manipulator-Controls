#ifndef AS5600_EXAMPLE_BAREMETAL_H
#define AS5600_EXAMPLE_BAREMETAL_H

#include "as5600_stm32_hal.h"

as5600_result_t as5600_example_baremetal_bind(I2C_HandleTypeDef *i2c);
as5600_result_t as5600_example_baremetal_poll(as5600_sample_t *sample);

#endif
