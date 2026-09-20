#include "as5600_example_rtos.h"

#include "FreeRTOS.h"

static as5600_t sensor;
static as5600_stm32_hal_t hal_port;
static as5600_cmsis_rtos2_t rtos_port;

as5600_result_t as5600_example_rtos_bind(
    I2C_HandleTypeDef *i2c, osMutexId_t shared_bus_mutex)
{
    as5600_sync_t synchronization = {0};
    uint32_t max_wait_ticks = UINT32_C(0x7FFFFFFF);
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (sizeof(TickType_t) == sizeof(uint16_t))
    {
        max_wait_ticks = UINT32_C(0xFFFE);
    }
    result = as5600_cmsis_rtos2_init(
        &rtos_port, shared_bus_mutex, max_wait_ticks, &synchronization);
    if (result == AS5600_OK)
    {
        result = as5600_stm32_hal_init(
            &sensor, &hal_port, i2c, &synchronization);
    }

    return result;
}

as5600_result_t as5600_example_rtos_poll(as5600_sample_t *sample)
{
    return as5600_read_sample(&sensor, sample, UINT32_C(50));
}
