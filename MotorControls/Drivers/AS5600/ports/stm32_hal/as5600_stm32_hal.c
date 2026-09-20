#include "as5600_stm32_hal.h"

#include <stddef.h>

static const as5600_stm32_hal_t *stm32_context(const void *context)
{
    return (const as5600_stm32_hal_t *)context;
}

static bool transfer_is_valid(
    const as5600_stm32_hal_t *port, uint8_t address, uint16_t length,
    uint32_t timeout_ms)
{
    return (port != NULL) && (port->i2c != NULL) &&
           (address == AS5600_I2C_ADDRESS) &&
           ((length == UINT16_C(1)) || (length == UINT16_C(2))) &&
           (timeout_ms > 0U) && (timeout_ms <= AS5600_TIMEOUT_MAX_MS);
}

static as5600_result_t translate_hal_result(
    const I2C_HandleTypeDef *i2c, HAL_StatusTypeDef status)
{
    as5600_result_t result = AS5600_ERROR_IO;

    switch (status)
    {
        case HAL_OK:
            result = AS5600_OK;
            break;
        case HAL_BUSY:
            result = AS5600_ERROR_BUSY;
            break;
        case HAL_TIMEOUT:
            result = AS5600_ERROR_TIMEOUT;
            break;
        case HAL_ERROR:
            if ((i2c->ErrorCode & HAL_I2C_ERROR_TIMEOUT) != 0U)
            {
                result = AS5600_ERROR_TIMEOUT;
            }
            else if (i2c->ErrorCode == HAL_I2C_ERROR_AF)
            {
                result = AS5600_ERROR_NACK;
            }
            else
            {
                result = AS5600_ERROR_IO;
            }
            break;
        default:
            result = AS5600_ERROR_IO;
            break;
    }

    return result;
}

static as5600_result_t stm32_read(
    void *context, uint8_t address, uint8_t reg, uint8_t *data,
    uint16_t length, uint32_t timeout_ms)
{
    const as5600_stm32_hal_t *port = stm32_context(context);
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (transfer_is_valid(port, address, length, timeout_ms) && (data != NULL))
    {
        const uint16_t hal_address = (uint16_t)((uint32_t)address << 1U);
        const HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
            port->i2c, hal_address, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
            data, length, timeout_ms);

        result = translate_hal_result(port->i2c, status);
    }

    return result;
}

static as5600_result_t stm32_write(
    void *context, uint8_t address, uint8_t reg, const uint8_t *data,
    uint16_t length, uint32_t timeout_ms)
{
    const as5600_stm32_hal_t *port = stm32_context(context);
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (transfer_is_valid(port, address, length, timeout_ms) && (data != NULL))
    {
        uint8_t buffer[2] = {0U, 0U};
        const uint16_t hal_address = (uint16_t)((uint32_t)address << 1U);
        HAL_StatusTypeDef status = HAL_ERROR;

        /* HAL's write signature is mutable; never cast away caller constness. */
        buffer[0] = data[0];
        if (length == UINT16_C(2))
        {
            buffer[1] = data[1];
        }
        status = HAL_I2C_Mem_Write(
            port->i2c, hal_address, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
            buffer, length, timeout_ms);
        result = translate_hal_result(port->i2c, status);
    }

    return result;
}

static uint32_t stm32_now(void *context)
{
    (void)context;
    return HAL_GetTick();
}

static as5600_result_t stm32_check_context(void *context)
{
    const as5600_stm32_hal_t *port = stm32_context(context);
    as5600_result_t result = AS5600_ERROR_CONTEXT;

    /* Polling requires privileged thread mode and an interrupt-driven tick. */
    if ((port != NULL) && (__get_IPSR() == 0U) && (__get_PRIMASK() == 0U) &&
        (__get_BASEPRI() == 0U) && (__get_FAULTMASK() == 0U) &&
        ((__get_CONTROL() & UINT32_C(1)) == 0U) &&
        (HAL_GetTickFreq() == HAL_TICK_FREQ_1KHZ))
    {
        result = AS5600_OK;
        if (port->synchronized)
        {
            result = port->synchronization.check_context(
                port->synchronization.context);
        }
    }

    return result;
}

static as5600_result_t stm32_lock(void *context, uint32_t timeout_ms)
{
    const as5600_stm32_hal_t *port = stm32_context(context);

    return port->synchronization.lock(
        port->synchronization.context, timeout_ms);
}

static as5600_result_t stm32_unlock(void *context)
{
    const as5600_stm32_hal_t *port = stm32_context(context);

    return port->synchronization.unlock(port->synchronization.context);
}

static as5600_result_t stm32_delay(
    void *context, uint32_t minimum_ms, uint32_t timeout_ms)
{
    const as5600_stm32_hal_t *port = stm32_context(context);
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if ((port != NULL) && (minimum_ms > 0U) &&
        (timeout_ms > 0U) && (timeout_ms <= AS5600_TIMEOUT_MAX_MS))
    {
        result = AS5600_ERROR_TIMEOUT;
        if (minimum_ms < timeout_ms)
        {
            if (port->synchronized)
            {
                result = port->synchronization.delay_ms(
                    port->synchronization.context, minimum_ms, timeout_ms);
            }
            else
            {
                HAL_Delay(minimum_ms);
                result = AS5600_OK;
            }
        }
    }

    return result;
}

as5600_result_t as5600_stm32_hal_init(
    as5600_t *device, as5600_stm32_hal_t *port, I2C_HandleTypeDef *i2c,
    const as5600_sync_t *synchronization)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;
    bool synchronization_valid = true;

    if (synchronization != NULL)
    {
        synchronization_valid =
            (synchronization->check_context != NULL) &&
            (synchronization->lock != NULL) &&
            (synchronization->unlock != NULL) &&
            (synchronization->delay_ms != NULL);
    }
    if ((device != NULL) && (port != NULL) && (i2c != NULL) &&
        synchronization_valid)
    {
        if ((i2c->Instance != NULL) &&
            (i2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT) &&
            (i2c->Init.ClockSpeed > 0U) &&
            (i2c->Init.ClockSpeed <= UINT32_C(400000)) &&
            (i2c->State == HAL_I2C_STATE_READY))
        {
            as5600_t candidate_device = {0};
            as5600_stm32_hal_t candidate_port = {0};
            as5600_bus_t bus = {0};

            candidate_port.i2c = i2c;
            candidate_port.synchronized = (synchronization != NULL);
            if (synchronization != NULL)
            {
                candidate_port.synchronization = *synchronization;
            }
            bus.context = port;
            bus.read = stm32_read;
            bus.write = stm32_write;
            bus.now_ms = stm32_now;
            bus.check_context = stm32_check_context;
            bus.delay_ms = stm32_delay;
            if (synchronization != NULL)
            {
                bus.lock = stm32_lock;
                bus.unlock = stm32_unlock;
            }
            result = as5600_init(&candidate_device, &bus);
            if (result == AS5600_OK)
            {
                *port = candidate_port;
                *device = candidate_device;
            }
        }
    }

    return result;
}
