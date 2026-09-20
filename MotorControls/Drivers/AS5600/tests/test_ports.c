#include "as5600_stm32_hal.h"
#include "as5600_cmsis_rtos2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned long checks = 0UL;

#define CHECK(expression) do { \
    ++checks; \
    if (!(expression)) { \
        (void)fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

typedef struct
{
    uint8_t registers[256];
    uint32_t now;
    uint32_t ipsr;
    uint32_t primask;
    uint32_t basepri;
    uint32_t faultmask;
    uint32_t control;
    HAL_TickFreqTypeDef hal_tick_frequency;
    HAL_StatusTypeDef hal_result;
    uint32_t hal_error;
    uint32_t hal_cost;
    uint32_t hal_calls;
    uint16_t last_address;
    uint16_t last_register;
    uint16_t last_length;
    uint32_t last_timeout;
    uint32_t hal_delays;
    bool mutate_write_buffer;
    osKernelState_t kernel_state;
    uint32_t os_frequency;
    osThreadId_t current_thread;
    osThreadId_t owner;
    osStatus_t acquire_result;
    osStatus_t release_result;
    osStatus_t delay_result;
    uint32_t acquire_calls;
    uint32_t release_calls;
    uint32_t delay_calls;
    uint32_t acquire_ticks;
    uint32_t delay_ticks;
    uint32_t acquire_cost;
    uint32_t release_cost;
    uint32_t tick_reads;
} mock_t;

static mock_t mock;
static I2C_TypeDef peripheral;
static unsigned int mutex_token;
static unsigned int thread_token;
static unsigned int other_thread_token;

static void reset_mock(void)
{
    (void)memset(&mock, 0, sizeof(mock));
    mock.hal_tick_frequency = HAL_TICK_FREQ_1KHZ;
    mock.hal_result = HAL_OK;
    mock.kernel_state = osKernelRunning;
    mock.os_frequency = 1000U;
    mock.current_thread = &thread_token;
    mock.acquire_result = osOK;
    mock.release_result = osOK;
    mock.delay_result = osOK;
    mock.registers[0x0B] = AS5600_STATUS_MAGNET_FOUND;
    mock.registers[0x0C] = 0xABU;
    mock.registers[0x0D] = 0xCDU;
    mock.registers[0x0E] = 0x03U;
    mock.registers[0x0F] = 0x21U;
}

static I2C_HandleTypeDef make_i2c(void)
{
    I2C_HandleTypeDef i2c = {0};

    i2c.Instance = &peripheral;
    i2c.Init.ClockSpeed = 400000U;
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.State = HAL_I2C_STATE_READY;
    return i2c;
}

static void record_transfer(
    I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
    uint16_t reg_size, uint16_t length, uint32_t timeout)
{
    CHECK(i2c != NULL);
    CHECK(address == UINT16_C(0x6C));
    CHECK(reg_size == I2C_MEMADD_SIZE_8BIT);
    CHECK((length == 1U) || (length == 2U));
    CHECK((timeout > 0U) && (timeout <= AS5600_TIMEOUT_MAX_MS));
    CHECK((uint32_t)reg + (uint32_t)length <= 256U);
    ++mock.hal_calls;
    mock.last_address = address;
    mock.last_register = reg;
    mock.last_length = length;
    mock.last_timeout = timeout;
    mock.now += mock.hal_cost;
    i2c->ErrorCode = mock.hal_error;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(
    I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
    uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    record_transfer(i2c, address, reg, reg_size, length, timeout);
    CHECK(data != NULL);
    data[0] = mock.registers[reg];
    if (length == 2U)
    {
        data[1] = mock.registers[(uint32_t)reg + 1U];
    }
    return mock.hal_result;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(
    I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
    uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    record_transfer(i2c, address, reg, reg_size, length, timeout);
    CHECK(data != NULL);
    CHECK((reg == 1U) || (reg == 3U) || (reg == 5U) || (reg == 7U));
    if (mock.hal_result == HAL_OK)
    {
        mock.registers[reg] = data[0];
        if (length == 2U)
        {
            mock.registers[(uint32_t)reg + 1U] = data[1];
        }
    }
    if (mock.mutate_write_buffer)
    {
        data[0] = 0x55U;
    }
    return mock.hal_result;
}

uint32_t HAL_GetTick(void)
{
    ++mock.tick_reads;
    return mock.now;
}

HAL_TickFreqTypeDef HAL_GetTickFreq(void)
{
    return mock.hal_tick_frequency;
}

void HAL_Delay(uint32_t delay)
{
    ++mock.hal_delays;
    mock.now += delay + (uint32_t)mock.hal_tick_frequency;
}

uint32_t __get_IPSR(void)
{
    return mock.ipsr;
}

uint32_t __get_PRIMASK(void)
{
    return mock.primask;
}

uint32_t __get_BASEPRI(void)
{
    return mock.basepri;
}

uint32_t __get_FAULTMASK(void)
{
    return mock.faultmask;
}

uint32_t __get_CONTROL(void)
{
    return mock.control;
}

osKernelState_t osKernelGetState(void)
{
    return mock.kernel_state;
}

uint32_t osKernelGetTickFreq(void)
{
    return mock.os_frequency;
}

osThreadId_t osThreadGetId(void)
{
    return mock.current_thread;
}

osThreadId_t osMutexGetOwner(osMutexId_t mutex)
{
    CHECK(mutex == &mutex_token);
    return mock.owner;
}

osStatus_t osMutexAcquire(osMutexId_t mutex, uint32_t timeout)
{
    CHECK(mutex == &mutex_token);
    CHECK(timeout != osWaitForever);
    CHECK(timeout > 0U);
    ++mock.acquire_calls;
    mock.acquire_ticks = timeout;
    mock.now += mock.acquire_cost;
    if (mock.acquire_result == osOK)
    {
        mock.owner = mock.current_thread;
    }
    return mock.acquire_result;
}

osStatus_t osMutexRelease(osMutexId_t mutex)
{
    CHECK(mutex == &mutex_token);
    CHECK(mock.owner == mock.current_thread);
    ++mock.release_calls;
    mock.now += mock.release_cost;
    if (mock.release_result == osOK)
    {
        mock.owner = NULL;
    }
    return mock.release_result;
}

osStatus_t osDelay(uint32_t ticks)
{
    CHECK((ticks > 0U) && (ticks != osWaitForever));
    ++mock.delay_calls;
    mock.delay_ticks = ticks;
    if (mock.delay_result == osOK)
    {
        const uint64_t elapsed =
            (((uint64_t)ticks * UINT64_C(1000)) +
             (uint64_t)mock.os_frequency - UINT64_C(1)) /
            (uint64_t)mock.os_frequency;

        CHECK(elapsed <= UINT32_MAX);
        mock.now += (uint32_t)elapsed;
    }
    return mock.delay_result;
}

static void bind_rtos(
    as5600_t *device, as5600_stm32_hal_t *port,
    I2C_HandleTypeDef *i2c, as5600_cmsis_rtos2_t *adapter)
{
    as5600_sync_t synchronization = {0};

    CHECK(as5600_cmsis_rtos2_init(
        adapter, &mutex_token, UINT32_C(0x7FFFFFFF), &synchronization) ==
        AS5600_OK);
    CHECK(as5600_stm32_hal_init(
        device, port, i2c, &synchronization) == AS5600_OK);
}

static void test_binding(void)
{
    as5600_t device = {0};
    as5600_stm32_hal_t port = {0};
    as5600_cmsis_rtos2_t adapter = {0};
    as5600_sync_t synchronization = {0};
    I2C_HandleTypeDef i2c = make_i2c();
    uint16_t angle = 0U;

    reset_mock();
    CHECK(as5600_stm32_hal_init(NULL, &port, &i2c, NULL) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(as5600_stm32_hal_init(&device, NULL, &i2c, NULL) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(as5600_stm32_hal_init(&device, &port, NULL, NULL) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(as5600_stm32_hal_init(&device, &port, &i2c, &synchronization) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(!device.initialized);

    i2c.Instance = NULL;
    CHECK(as5600_stm32_hal_init(&device, &port, &i2c, NULL) ==
          AS5600_ERROR_ARGUMENT);
    i2c = make_i2c();
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
    CHECK(as5600_stm32_hal_init(&device, &port, &i2c, NULL) ==
          AS5600_ERROR_ARGUMENT);
    i2c = make_i2c();
    i2c.Init.ClockSpeed = 0U;
    CHECK(as5600_stm32_hal_init(&device, &port, &i2c, NULL) ==
          AS5600_ERROR_ARGUMENT);
    i2c.Init.ClockSpeed = 400001U;
    CHECK(as5600_stm32_hal_init(&device, &port, &i2c, NULL) ==
          AS5600_ERROR_ARGUMENT);
    i2c = make_i2c();
    i2c.State = HAL_I2C_STATE_RESET;
    CHECK(as5600_stm32_hal_init(&device, &port, &i2c, NULL) ==
          AS5600_ERROR_ARGUMENT);
    i2c = make_i2c();
    CHECK(as5600_stm32_hal_init(&device, &port, &i2c, NULL) == AS5600_OK);
    CHECK(mock.hal_calls == 0U);
    CHECK(mock.tick_reads == 0U);
    CHECK(device.bus.lock == NULL);
    CHECK(device.bus.unlock == NULL);
    CHECK(as5600_read_raw_angle(&device, &angle, 20U) == AS5600_OK);
    CHECK(angle == UINT16_C(0xBCD));
    CHECK(mock.last_address == UINT16_C(0x6C));
    CHECK(mock.last_register == UINT16_C(0x0C));
    CHECK(mock.last_length == 2U);
    CHECK(mock.last_timeout == 20U);

    CHECK(as5600_cmsis_rtos2_init(NULL, &mutex_token, 1U, &synchronization) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(as5600_cmsis_rtos2_init(&adapter, NULL, 1U, &synchronization) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(as5600_cmsis_rtos2_init(&adapter, &mutex_token, 1U, NULL) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(as5600_cmsis_rtos2_init(&adapter, &mutex_token, 0U, &synchronization) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(as5600_cmsis_rtos2_init(
        &adapter, &mutex_token, UINT32_MAX, &synchronization) ==
        AS5600_ERROR_ARGUMENT);
    CHECK(as5600_cmsis_rtos2_init(
        &adapter, &mutex_token, UINT32_C(0x7FFFFFFF), &synchronization) ==
        AS5600_OK);
    CHECK(synchronization.context == &adapter);
    {
        as5600_sync_t invalid = synchronization;
        as5600_t malformed_device = {0};

        invalid.check_context = NULL;
        CHECK(as5600_stm32_hal_init(&device, &port, &i2c, &invalid) ==
              AS5600_ERROR_ARGUMENT);
        invalid = synchronization;
        invalid.lock = NULL;
        CHECK(as5600_stm32_hal_init(&device, &port, &i2c, &invalid) ==
              AS5600_ERROR_ARGUMENT);
        invalid = synchronization;
        invalid.unlock = NULL;
        CHECK(as5600_stm32_hal_init(&device, &port, &i2c, &invalid) ==
              AS5600_ERROR_ARGUMENT);
        invalid = synchronization;
        invalid.delay_ms = NULL;
        CHECK(as5600_stm32_hal_init(&device, &port, &i2c, &invalid) ==
              AS5600_ERROR_ARGUMENT);
        malformed_device.initialized = true;
        CHECK(as5600_read_raw_angle(&malformed_device, &angle, 50U) ==
              AS5600_ERROR_NOT_INITIALIZED);
    }
}

static void test_hal_results_and_guards(void)
{
    static const struct
    {
        HAL_StatusTypeDef hal_status;
        uint32_t hal_error;
        as5600_result_t expected;
    } cases[] = {
        {HAL_OK, HAL_I2C_ERROR_AF, AS5600_OK},
        {HAL_BUSY, HAL_I2C_ERROR_AF, AS5600_ERROR_BUSY},
        {HAL_TIMEOUT, HAL_I2C_ERROR_NONE, AS5600_ERROR_TIMEOUT},
        {HAL_ERROR, HAL_I2C_ERROR_TIMEOUT, AS5600_ERROR_TIMEOUT},
        {HAL_ERROR, HAL_I2C_ERROR_AF, AS5600_ERROR_NACK},
        {HAL_ERROR, HAL_I2C_ERROR_NONE, AS5600_ERROR_IO},
        {HAL_ERROR, HAL_I2C_ERROR_BERR, AS5600_ERROR_IO},
        {HAL_ERROR, HAL_I2C_ERROR_ARLO, AS5600_ERROR_IO},
        {HAL_ERROR, HAL_I2C_ERROR_OVR, AS5600_ERROR_IO},
        {HAL_ERROR, HAL_I2C_ERROR_DMA, AS5600_ERROR_IO},
        {HAL_ERROR, HAL_I2C_ERROR_AF | HAL_I2C_ERROR_BERR, AS5600_ERROR_IO},
        {HAL_ERROR, HAL_I2C_ERROR_AF | HAL_I2C_ERROR_TIMEOUT,
         AS5600_ERROR_TIMEOUT},
        {(HAL_StatusTypeDef)99, 0U, AS5600_ERROR_IO}
    };
    as5600_t device = {0};
    as5600_stm32_hal_t port = {0};
    I2C_HandleTypeDef i2c = make_i2c();
    size_t index = 0U;

    reset_mock();
    CHECK(as5600_stm32_hal_init(&device, &port, &i2c, NULL) == AS5600_OK);
    for (index = 0U; index < (sizeof(cases) / sizeof(cases[0])); ++index)
    {
        uint16_t angle = 0xFFFFU;
        const uint8_t data[2] = {0x03U, 0xA5U};

        reset_mock();
        mock.hal_result = cases[index].hal_status;
        mock.hal_error = cases[index].hal_error;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              cases[index].expected);
        CHECK(angle == ((cases[index].expected == AS5600_OK) ?
                        UINT16_C(0xBCD) : UINT16_C(0xFFFF)));
        mock.mutate_write_buffer = true;
        CHECK(device.bus.write(
            device.bus.context, AS5600_I2C_ADDRESS, 7U, data, 2U, 50U) ==
            cases[index].expected);
        CHECK(data[0] == 0x03U);
        CHECK(data[1] == 0xA5U);
    }

    for (index = 0U; index < 6U; ++index)
    {
        uint16_t angle = 0xFFFFU;

        reset_mock();
        switch (index)
        {
            case 0U: mock.ipsr = 15U; break;
            case 1U: mock.primask = 1U; break;
            case 2U: mock.basepri = 0x50U; break;
            case 3U: mock.faultmask = 1U; break;
            case 4U: mock.control = 1U; break;
            default: mock.hal_tick_frequency = HAL_TICK_FREQ_100HZ; break;
        }
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              AS5600_ERROR_CONTEXT);
        CHECK(angle == 0xFFFFU);
        CHECK(mock.hal_calls == 0U);
        CHECK(mock.tick_reads == 0U);
    }
    for (index = 0U; index < 8U; ++index)
    {
        uint16_t angle = 0xFFFFU;
        const as5600_result_t expected =
            ((index & 1U) == 0U) ? AS5600_OK : AS5600_ERROR_CONTEXT;

        reset_mock();
        mock.control = (uint32_t)index;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) == expected);
        CHECK(angle == ((expected == AS5600_OK) ?
                        UINT16_C(0xBCD) : UINT16_C(0xFFFF)));
    }
    reset_mock();
    mock.hal_cost = 50U;
    {
        uint16_t angle = 0xFFFFU;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              AS5600_ERROR_TIMEOUT);
        CHECK(angle == 0xFFFFU);
    }
}

static void test_callback_validation_and_bare_delay(void)
{
    as5600_t device = {0};
    as5600_stm32_hal_t port = {0};
    I2C_HandleTypeDef i2c = make_i2c();
    uint8_t data[2] = {0x01U, 0x02U};
    as5600_config_t configuration = {0};

    reset_mock();
    CHECK(as5600_stm32_hal_init(&device, &port, &i2c, NULL) == AS5600_OK);
    CHECK(device.bus.read(NULL, 0x36U, 0x0CU, data, 2U, 50U) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.read(&port, 0x6CU, 0x0CU, data, 2U, 50U) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.read(&port, 0x36U, 0x0CU, NULL, 2U, 50U) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.read(&port, 0x36U, 0x0CU, data, 0U, 50U) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.read(&port, 0x36U, 0x0CU, data, 3U, 50U) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.read(&port, 0x36U, 0x0CU, data, 2U, 0U) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.read(&port, 0x36U, 0x0CU, data, 2U, UINT32_MAX) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.write(&port, 0x36U, 7U, NULL, 2U, 50U) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.write(&port, 0x6CU, 7U, data, 2U, 50U) ==
          AS5600_ERROR_ARGUMENT);
    port.i2c = NULL;
    CHECK(device.bus.read(&port, 0x36U, 0x0CU, data, 2U, 50U) ==
          AS5600_ERROR_ARGUMENT);
    port.i2c = &i2c;
    CHECK(device.bus.check_context(NULL) == AS5600_ERROR_CONTEXT);
    CHECK(device.bus.delay_ms(NULL, 1U, 50U) == AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.delay_ms(&port, 0U, 50U) == AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.delay_ms(&port, 1U, 0U) == AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.delay_ms(&port, 1U, UINT32_MAX) == AS5600_ERROR_ARGUMENT);
    CHECK(device.bus.delay_ms(&port, 50U, 50U) == AS5600_ERROR_TIMEOUT);
    CHECK(mock.hal_calls == 0U);
    CHECK(mock.hal_delays == 0U);
    CHECK(device.bus.write(&port, 0x36U, 7U, data, 1U, 50U) == AS5600_OK);
    CHECK(mock.last_length == 1U);

    CHECK(as5600_default_config(&configuration) == AS5600_OK);
    configuration.output = AS5600_OUTPUT_PWM;
    mock.registers[7] = 0xC0U;
    mock.registers[8] = 0U;
    CHECK(as5600_write_config(&device, &configuration, 50U) == AS5600_OK);
    CHECK(mock.registers[7] == 0xC0U);
    CHECK(mock.registers[8] == 0x20U);
    CHECK(mock.hal_delays == 1U);
}

static void test_rtos_conversion(void)
{
    static const uint32_t frequencies[] = {1U, 100U, 128U, 1000U, 1024U, 10000U};
    static const uint32_t waits[] = {1U, 2U, 9U, 10U, 11U, 999U, 1000U, 12345U};
    as5600_cmsis_rtos2_t adapter = {0};
    as5600_sync_t synchronization = {0};
    size_t frequency_index = 0U;
    size_t wait_index = 0U;

    reset_mock();
    CHECK(as5600_cmsis_rtos2_init(
        &adapter, &mutex_token, UINT32_C(0x7FFFFFFF), &synchronization) ==
        AS5600_OK);
    for (frequency_index = 0U;
         frequency_index < sizeof(frequencies) / sizeof(frequencies[0]);
         ++frequency_index)
    {
        for (wait_index = 0U;
             wait_index < sizeof(waits) / sizeof(waits[0]); ++wait_index)
        {
            const uint32_t milliseconds = waits[wait_index];
            const uint32_t frequency = frequencies[frequency_index];
            const uint32_t expected = (uint32_t)(
                (((uint64_t)milliseconds * (uint64_t)frequency) + 999U) / 1000U);

            reset_mock();
            mock.os_frequency = frequency;
            CHECK(synchronization.lock(&adapter, milliseconds) == AS5600_OK);
            CHECK(mock.acquire_ticks == expected);
            CHECK(mock.owner == &thread_token);
            CHECK(synchronization.unlock(&adapter) == AS5600_OK);
            CHECK(mock.owner == NULL);
        }
    }
    reset_mock();
    CHECK(synchronization.lock(&adapter, 0U) == AS5600_ERROR_ARGUMENT);
    CHECK(synchronization.lock(&adapter, UINT32_MAX) == AS5600_ERROR_ARGUMENT);
    mock.os_frequency = 0U;
    CHECK(synchronization.lock(&adapter, 10U) == AS5600_ERROR_ARGUMENT);
    mock.os_frequency = UINT32_MAX;
    CHECK(synchronization.lock(&adapter, AS5600_TIMEOUT_MAX_MS) ==
          AS5600_ERROR_ARGUMENT);
    CHECK(mock.acquire_calls == 0U);

    reset_mock();
    CHECK(as5600_cmsis_rtos2_init(
        &adapter, &mutex_token, UINT32_C(0xFFFE), &synchronization) == AS5600_OK);
    CHECK(synchronization.lock(&adapter, 65534U) == AS5600_OK);
    CHECK(mock.acquire_ticks == 65534U);
    CHECK(synchronization.unlock(&adapter) == AS5600_OK);
    CHECK(synchronization.lock(&adapter, 65535U) == AS5600_ERROR_ARGUMENT);
}

static void test_rtos_errors(void)
{
    static const osKernelState_t states[] = {
        osKernelInactive, osKernelReady, osKernelLocked, osKernelSuspended,
        osKernelError
    };
    static const struct
    {
        osStatus_t status;
        as5600_result_t expected;
    } cases[] = {
        {osErrorTimeout, AS5600_ERROR_TIMEOUT},
        {osErrorResource, AS5600_ERROR_BUSY},
        {osErrorISR, AS5600_ERROR_CONTEXT},
        {osErrorParameter, AS5600_ERROR_LOCK},
        {osErrorNoMemory, AS5600_ERROR_LOCK},
        {osError, AS5600_ERROR_LOCK}
    };
    as5600_t device = {0};
    as5600_stm32_hal_t port = {0};
    as5600_cmsis_rtos2_t adapter = {0};
    I2C_HandleTypeDef i2c = make_i2c();
    size_t index = 0U;

    reset_mock();
    bind_rtos(&device, &port, &i2c, &adapter);
    for (index = 0U; index < sizeof(states) / sizeof(states[0]); ++index)
    {
        uint16_t angle = 0xFFFFU;

        reset_mock();
        mock.kernel_state = states[index];
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              AS5600_ERROR_CONTEXT);
        CHECK(mock.hal_calls == 0U);
        CHECK(mock.acquire_calls == 0U);
        CHECK(mock.tick_reads == 0U);
        CHECK(angle == 0xFFFFU);
    }
    for (index = 0U; index < sizeof(cases) / sizeof(cases[0]); ++index)
    {
        uint16_t angle = 0xFFFFU;

        reset_mock();
        mock.acquire_result = cases[index].status;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              cases[index].expected);
        CHECK(mock.hal_calls == 0U);
        CHECK(mock.release_calls == 0U);
        CHECK(angle == 0xFFFFU);
    }
    reset_mock();
    {
        uint16_t angle = 0xFFFFU;

        mock.current_thread = NULL;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              AS5600_ERROR_CONTEXT);
        CHECK(mock.acquire_calls == 0U);
        mock.current_thread = &thread_token;
        mock.owner = &thread_token;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) == AS5600_ERROR_LOCK);
        CHECK(mock.acquire_calls == 0U);
        mock.owner = &other_thread_token;
        mock.acquire_result = osErrorTimeout;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              AS5600_ERROR_TIMEOUT);
        CHECK(mock.owner == &other_thread_token);
    }
    reset_mock();
    {
        uint16_t angle = 0xFFFFU;

        mock.release_result = osErrorResource;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              AS5600_ERROR_UNLOCK);
        CHECK(mock.release_calls == 1U);
        CHECK(angle == 0xFFFFU);
    }
    reset_mock();
    {
        uint16_t angle = 0xFFFFU;

        mock.acquire_cost = 50U;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              AS5600_ERROR_TIMEOUT);
        CHECK(mock.hal_calls == 0U);
        CHECK(mock.release_calls == 1U);
        CHECK(mock.owner == NULL);
    }
    reset_mock();
    {
        uint16_t angle = 0xFFFFU;

        mock.acquire_cost = 7U;
        mock.release_cost = 43U;
        CHECK(as5600_read_raw_angle(&device, &angle, 50U) ==
              AS5600_ERROR_TIMEOUT);
        CHECK(mock.last_timeout == 43U);
        CHECK(angle == 0xFFFFU);
        CHECK(mock.owner == NULL);
    }
}

static void test_rtos_settling(void)
{
    as5600_t device = {0};
    as5600_stm32_hal_t port = {0};
    as5600_cmsis_rtos2_t adapter = {0};
    I2C_HandleTypeDef i2c = make_i2c();
    as5600_config_t configuration = {0};
    as5600_sync_t synchronization = {0};

    reset_mock();
    bind_rtos(&device, &port, &i2c, &adapter);
    CHECK(as5600_default_config(&configuration) == AS5600_OK);
    configuration.power = AS5600_POWER_LOW_1;
    CHECK(as5600_write_config(&device, &configuration, 50U) == AS5600_OK);
    CHECK(mock.acquire_calls == 1U);
    CHECK(mock.release_calls == 1U);
    CHECK(mock.delay_calls == 1U);
    CHECK(mock.delay_ticks == 2U);
    CHECK(mock.hal_delays == 0U);
    CHECK(mock.hal_calls == 3U);
    CHECK(mock.owner == NULL);

    reset_mock();
    mock.os_frequency = 100U;
    CHECK(as5600_write_config(&device, &configuration, 50U) == AS5600_OK);
    CHECK(mock.delay_ticks == 2U);
    CHECK(mock.now >= 20U);
    reset_mock();
    mock.os_frequency = 100U;
    CHECK(as5600_write_config(&device, &configuration, 10U) ==
          AS5600_ERROR_TIMEOUT);
    CHECK(mock.delay_calls == 0U);
    CHECK(mock.release_calls == 1U);
    CHECK(mock.owner == NULL);
    reset_mock();
    mock.delay_result = osErrorISR;
    CHECK(as5600_write_config(&device, &configuration, 50U) ==
          AS5600_ERROR_CONTEXT);
    CHECK(mock.release_calls == 1U);
    reset_mock();
    mock.delay_result = osError;
    CHECK(as5600_write_config(&device, &configuration, 50U) == AS5600_ERROR_IO);
    CHECK(mock.release_calls == 1U);

    reset_mock();
    CHECK(as5600_cmsis_rtos2_init(
        &adapter, &mutex_token, UINT32_C(0x7FFFFFFF), &synchronization) ==
        AS5600_OK);
    CHECK(synchronization.check_context(NULL) == AS5600_ERROR_CONTEXT);
    CHECK(synchronization.lock(NULL, 50U) == AS5600_ERROR_CONTEXT);
    CHECK(synchronization.unlock(NULL) == AS5600_ERROR_CONTEXT);
    CHECK(synchronization.delay_ms(NULL, 1U, 50U) == AS5600_ERROR_CONTEXT);
    CHECK(synchronization.unlock(&adapter) == AS5600_ERROR_UNLOCK);
    CHECK(synchronization.delay_ms(&adapter, 1U, 50U) == AS5600_ERROR_LOCK);
    CHECK(synchronization.delay_ms(&adapter, 0U, 50U) == AS5600_ERROR_ARGUMENT);
    CHECK(synchronization.delay_ms(&adapter, 50U, 50U) == AS5600_ERROR_ARGUMENT);
    CHECK(synchronization.delay_ms(&adapter, 1U, UINT32_MAX) ==
          AS5600_ERROR_ARGUMENT);
    mock.os_frequency = 0U;
    CHECK(synchronization.delay_ms(&adapter, 1U, 50U) == AS5600_ERROR_ARGUMENT);
    adapter.mutex = NULL;
    CHECK(synchronization.check_context(&adapter) == AS5600_ERROR_CONTEXT);
    CHECK(as5600_cmsis_rtos2_init(
        &adapter, &mutex_token, 1U, &synchronization) == AS5600_OK);
    mock.os_frequency = 1000U;
    mock.owner = &thread_token;
    CHECK(synchronization.delay_ms(&adapter, 1U, 50U) == AS5600_ERROR_TIMEOUT);
    CHECK(mock.delay_calls == 0U);
}

int main(void)
{
    test_binding();
    test_hal_results_and_guards();
    test_callback_validation_and_bare_delay();
    test_rtos_conversion();
    test_rtos_errors();
    test_rtos_settling();
    (void)printf("ports: %lu checks passed\n", checks);
    return EXIT_SUCCESS;
}
