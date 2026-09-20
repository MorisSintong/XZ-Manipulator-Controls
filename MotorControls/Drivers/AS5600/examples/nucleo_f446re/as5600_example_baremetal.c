#include "as5600_example_baremetal.h"

#include <stddef.h>

static as5600_t sensor;
static as5600_stm32_hal_t hal_port;

as5600_result_t as5600_example_baremetal_bind(I2C_HandleTypeDef *i2c)
{
    return as5600_stm32_hal_init(&sensor, &hal_port, i2c, NULL);
}

as5600_result_t as5600_example_baremetal_poll(as5600_sample_t *sample)
{
    return as5600_read_sample(&sensor, sample, UINT32_C(50));
}
