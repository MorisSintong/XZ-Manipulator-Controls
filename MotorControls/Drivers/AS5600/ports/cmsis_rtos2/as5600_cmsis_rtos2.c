#include "as5600_cmsis_rtos2.h"

#include <stddef.h>

static const as5600_cmsis_rtos2_t *cmsis_context(const void *context)
{
    return (const as5600_cmsis_rtos2_t *)context;
}

static uint64_t rounded_ticks(uint32_t milliseconds, uint32_t frequency)
{
    return (((uint64_t)milliseconds * (uint64_t)frequency) +
            UINT64_C(999)) / UINT64_C(1000);
}

static as5600_result_t cmsis_check_context(void *context)
{
    const as5600_cmsis_rtos2_t *adapter = cmsis_context(context);
    as5600_result_t result = AS5600_ERROR_CONTEXT;

    if ((adapter != NULL) && (adapter->mutex != NULL) &&
        (osKernelGetState() == osKernelRunning) && (osThreadGetId() != NULL))
    {
        result = AS5600_OK;
    }

    return result;
}

static as5600_result_t cmsis_lock(void *context, uint32_t timeout_ms)
{
    const as5600_cmsis_rtos2_t *adapter = cmsis_context(context);
    as5600_result_t result = cmsis_check_context(context);

    if (result == AS5600_OK)
    {
        const uint32_t frequency = osKernelGetTickFreq();

        result = AS5600_ERROR_ARGUMENT;
        if ((timeout_ms > 0U) && (timeout_ms <= AS5600_TIMEOUT_MAX_MS) &&
            (frequency > 0U))
        {
            const uint64_t ticks = rounded_ticks(timeout_ms, frequency);

            if (ticks <= (uint64_t)adapter->max_wait_ticks)
            {
                result = AS5600_ERROR_LOCK;
                if (osMutexGetOwner(adapter->mutex) != osThreadGetId())
                {
                    const osStatus_t status = osMutexAcquire(
                        adapter->mutex, (uint32_t)ticks);

                    switch (status)
                    {
                        case osOK:
                            result = AS5600_OK;
                            break;
                        case osErrorTimeout:
                            result = AS5600_ERROR_TIMEOUT;
                            break;
                        case osErrorResource:
                            result = AS5600_ERROR_BUSY;
                            break;
                        case osErrorISR:
                            result = AS5600_ERROR_CONTEXT;
                            break;
                        default:
                            result = AS5600_ERROR_LOCK;
                            break;
                    }
                }
            }
        }
    }

    return result;
}

static as5600_result_t cmsis_unlock(void *context)
{
    const as5600_cmsis_rtos2_t *adapter = cmsis_context(context);
    as5600_result_t result = cmsis_check_context(context);

    if (result == AS5600_OK)
    {
        result = AS5600_ERROR_UNLOCK;
        if (osMutexGetOwner(adapter->mutex) == osThreadGetId())
        {
            const osStatus_t status = osMutexRelease(adapter->mutex);

            if (status == osOK)
            {
                result = AS5600_OK;
            }
        }
    }

    return result;
}

static as5600_result_t cmsis_delay(
    void *context, uint32_t minimum_ms, uint32_t timeout_ms)
{
    const as5600_cmsis_rtos2_t *adapter = cmsis_context(context);
    as5600_result_t result = cmsis_check_context(context);

    if (result == AS5600_OK)
    {
        const uint32_t frequency = osKernelGetTickFreq();

        result = AS5600_ERROR_ARGUMENT;
        if ((minimum_ms > 0U) && (minimum_ms < timeout_ms) &&
            (timeout_ms <= AS5600_TIMEOUT_MAX_MS) && (frequency > 0U))
        {
            /* One extra tick guarantees the minimum across tick phase. */
            const uint64_t ticks = rounded_ticks(minimum_ms, frequency) +
                                   UINT64_C(1);
            const uint64_t budget_ticks =
                ((uint64_t)timeout_ms * (uint64_t)frequency) / UINT64_C(1000);

            result = AS5600_ERROR_TIMEOUT;
            if ((ticks < budget_ticks) &&
                (ticks <= (uint64_t)adapter->max_wait_ticks))
            {
                result = AS5600_ERROR_LOCK;
                if (osMutexGetOwner(adapter->mutex) == osThreadGetId())
                {
                    const osStatus_t status = osDelay((uint32_t)ticks);

                    if (status == osOK)
                    {
                        result = AS5600_OK;
                    }
                    else if (status == osErrorISR)
                    {
                        result = AS5600_ERROR_CONTEXT;
                    }
                    else
                    {
                        result = AS5600_ERROR_IO;
                    }
                }
            }
        }
    }

    return result;
}

as5600_result_t as5600_cmsis_rtos2_init(
    as5600_cmsis_rtos2_t *adapter, osMutexId_t mutex,
    uint32_t max_wait_ticks, as5600_sync_t *synchronization)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if ((adapter != NULL) && (mutex != NULL) && (synchronization != NULL) &&
        (max_wait_ticks > 0U) && (max_wait_ticks <= UINT32_C(0x7FFFFFFF)))
    {
        adapter->mutex = mutex;
        adapter->max_wait_ticks = max_wait_ticks;
        synchronization->context = adapter;
        synchronization->check_context = cmsis_check_context;
        synchronization->lock = cmsis_lock;
        synchronization->unlock = cmsis_unlock;
        synchronization->delay_ms = cmsis_delay;
        result = AS5600_OK;
    }

    return result;
}
