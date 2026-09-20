#include "as5600.h"
#include "as5600_cmsis_rtos2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "test_support.h"

#include <stddef.h>
#include <string.h>

enum { CONTROL, LOW, HIGH, MEDIUM, CLIENTS };
enum { IDLE_OPERATION, SAMPLE_OPERATION, CONFIG_OPERATION };
enum { INITIAL, INHERITANCE, ACQUIRE_TIMEOUT, IO_ERRORS, STRESS };
enum { CMD_PI = 0x100U, CMD_TIMEOUT = 0x200U, CMD_STRESS = 0x400U };
enum { IO_BUDGET = 100U, WAIT_BOUND = 2000U, STRESS_ITERATIONS = 16U };

typedef struct
{
    as5600_t device;
    osThreadId_t thread;
    unsigned index;
    unsigned operation;
    uint32_t last_transfers;
    uint32_t last_delays;
    uint32_t samples;
    uint32_t configurations;
    uint32_t failures;
} client_t;

typedef struct
{
    uint8_t registers[0x1DU];
    client_t *active;
    osThreadId_t owner;
    uint32_t attempts;
    uint32_t acquired;
    uint32_t released;
    uint32_t transfers;
    uint32_t transaction_transfers;
    uint32_t transaction_delays;
    uint32_t remaining_budget;
    uint32_t fail_at;
    as5600_result_t failure;
    uint32_t injected_errors;
} simulated_bus_t;

static client_t clients[CLIENTS];
static simulated_bus_t bus;
static as5600_cmsis_rtos2_t adapter;
static as5600_sync_t synchronization;
static osMutexId_t mutex;
static StaticSemaphore_t mutex_storage;
static StaticTask_t task_storage[CLIENTS];
static StackType_t stacks[CLIENTS][768] __attribute__((aligned(8)));
static volatile uint32_t switches;
static volatile uint32_t kernel_ticks;
static volatile unsigned phase;
static volatile unsigned inheritance_seen;
static volatile unsigned medium_ran;
static volatile unsigned timeout_seen;
static uint32_t timeout_elapsed;

void test_task_switched_in(void)
{
    ++switches;
}

void vApplicationTickHook(void)
{
    ++kernel_ticks;
    CHECK(kernel_ticks < 5000U);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    test_assert_failed("FreeRTOS stack overflow", name, 0U);
}

static void send_flags(osThreadId_t thread, uint32_t flags)
{
    CHECK((osThreadFlagsSet(thread, flags) & osFlagsError) == 0U);
}

static void wait_done(uint32_t flags)
{
    const uint32_t received = osThreadFlagsWait(flags, osFlagsWaitAll, WAIT_BOUND);
    CHECK((received & osFlagsError) == 0U);
    CHECK((received & flags) == flags);
}

static uint32_t now_ms(void *context)
{
    (void)context;
    return osKernelGetTickCount();
}

static as5600_result_t check_context(void *context)
{
    (void)context;
    return synchronization.check_context(synchronization.context);
}

static as5600_result_t acquire(void *context, uint32_t timeout_ms)
{
    client_t *client = context;
    as5600_result_t result;
    taskENTER_CRITICAL();
    ++bus.attempts;
    taskEXIT_CRITICAL();
    result = synchronization.lock(synchronization.context, timeout_ms);
    if (result == AS5600_OK)
    {
        CHECK(client->operation != IDLE_OPERATION);
        CHECK(bus.active == NULL);
        CHECK(osMutexGetOwner(mutex) == osThreadGetId());
        CHECK(osThreadGetId() == client->thread);
        bus.active = client;
        bus.owner = client->thread;
        bus.transaction_transfers = 0U;
        bus.transaction_delays = 0U;
        bus.remaining_budget = timeout_ms;
        ++bus.acquired;
    }
    return result;
}

static void assert_owner(client_t *client)
{
    CHECK(bus.active == client);
    CHECK(bus.owner == osThreadGetId());
    CHECK(bus.owner == client->thread);
    CHECK(osMutexGetOwner(mutex) == bus.owner);
    CHECK(client->operation != IDLE_OPERATION);
}

static as5600_result_t release(void *context)
{
    client_t *client = context;
    assert_owner(client);
    if ((phase == INHERITANCE) && (client->index == LOW))
    {
        CHECK(inheritance_seen == 1U);
        CHECK(medium_ran == 0U);
        CHECK(osThreadGetPriority(client->thread) == osPriorityHigh);
    }
    client->last_transfers = bus.transaction_transfers;
    client->last_delays = bus.transaction_delays;
    ++bus.released;
    /* A higher-priority waiter can run inside the real osMutexRelease call. */
    bus.active = NULL;
    bus.owner = NULL;
    return synchronization.unlock(synchronization.context);
}

static as5600_result_t settle(void *context, uint32_t minimum_ms,
                             uint32_t timeout_ms)
{
    client_t *client = context;
    const uint32_t transaction = bus.acquired;
    const uint32_t started = osKernelGetTickCount();
    as5600_result_t result;
    assert_owner(client);
    CHECK(client->operation == CONFIG_OPERATION);
    CHECK(bus.transaction_transfers == 2U);
    CHECK(minimum_ms == 1U);
    CHECK(timeout_ms <= bus.remaining_budget);
    bus.remaining_budget = timeout_ms;
    ++bus.transaction_delays;
    result = synchronization.delay_ms(synchronization.context,
                                      minimum_ms, timeout_ms);
    assert_owner(client);
    CHECK(bus.acquired == transaction);
    CHECK(result == AS5600_OK);
    CHECK(osKernelGetTickCount() - started >= 2U);
    return result;
}

static void begin_transfer(client_t *client, uint8_t address, uint8_t reg,
                           uint16_t length, uint32_t timeout_ms)
{
    assert_owner(client);
    CHECK(address == AS5600_I2C_ADDRESS);
    CHECK((length == 1U) || (length == 2U));
    CHECK((uint32_t)reg + length <= sizeof(bus.registers));
    CHECK((timeout_ms > 0U) && (timeout_ms <= bus.remaining_budget));
    bus.remaining_budget = timeout_ms;
    ++bus.transfers;
    ++bus.transaction_transfers;
}

static void exercise_contenders(client_t *client)
{
    if ((client->index != LOW) || (bus.transaction_transfers != 1U))
    {
        return;
    }
    if (phase == INHERITANCE)
    {
        CHECK(osThreadGetPriority(client->thread) == osPriorityLow);
        send_flags(clients[HIGH].thread, CMD_PI);
        CHECK(osThreadGetState(clients[HIGH].thread) == osThreadBlocked);
        CHECK(osThreadGetPriority(client->thread) == osPriorityHigh);
        inheritance_seen = 1U;
        send_flags(clients[MEDIUM].thread, CMD_PI);
        CHECK(osThreadGetState(clients[MEDIUM].thread) == osThreadReady);
        CHECK(osThreadYield() == osOK);
        CHECK(medium_ran == 0U);
        assert_owner(client);
    }
    else if (phase == ACQUIRE_TIMEOUT)
    {
        send_flags(clients[HIGH].thread, CMD_TIMEOUT);
        CHECK(osThreadGetState(clients[HIGH].thread) == osThreadBlocked);
        CHECK(osThreadGetPriority(client->thread) == osPriorityHigh);
        CHECK(osDelay(15U) == osOK);
        CHECK(timeout_seen == 1U);
        CHECK(osThreadGetPriority(client->thread) == osPriorityLow);
        assert_owner(client);
    }
}

static int inject_error(void)
{
    if (bus.transfers == bus.fail_at)
    {
        bus.fail_at = 0U;
        ++bus.injected_errors;
        return 1;
    }
    return 0;
}

static as5600_result_t read_registers(
    void *context, uint8_t address, uint8_t reg, uint8_t *data,
    uint16_t length, uint32_t timeout_ms)
{
    static const uint8_t sample_registers[6] = {
        0x0BU, 0x0CU, 0x0EU, 0x1AU, 0x1BU, 0x0BU
    };
    static const uint8_t sample_lengths[6] = {1U, 2U, 2U, 1U, 2U, 1U};
    client_t *client = context;
    begin_transfer(client, address, reg, length, timeout_ms);
    if (client->operation == SAMPLE_OPERATION)
    {
        CHECK(bus.transaction_transfers <= 6U);
        CHECK(reg == sample_registers[bus.transaction_transfers - 1U]);
        CHECK(length == sample_lengths[bus.transaction_transfers - 1U]);
    }
    else
    {
        CHECK(client->operation == CONFIG_OPERATION);
        CHECK((bus.transaction_transfers == 1U) ||
              (bus.transaction_transfers == 3U));
        CHECK((reg == 0x07U) && (length == 2U));
    }
    exercise_contenders(client);
    if (inject_error())
    {
        (void)memset(data, 0xEE, length);
        return bus.failure;
    }
    (void)memcpy(data, &bus.registers[reg], length);
    assert_owner(client);
    return AS5600_OK;
}

static as5600_result_t write_registers(
    void *context, uint8_t address, uint8_t reg, const uint8_t *data,
    uint16_t length, uint32_t timeout_ms)
{
    client_t *client = context;
    begin_transfer(client, address, reg, length, timeout_ms);
    CHECK(client->operation == CONFIG_OPERATION);
    CHECK(bus.transaction_transfers == 2U);
    CHECK((reg == 0x07U) && (length == 2U));
    CHECK((data[0] & 0xC0U) == 0xC0U);
    if (inject_error())
    {
        return bus.failure;
    }
    (void)memcpy(&bus.registers[reg], data, length);
    assert_owner(client);
    return AS5600_OK;
}

static void assert_sample(const as5600_sample_t *sample)
{
    CHECK(sample->raw_angle == 0xABCU);
    CHECK(sample->angle == 0x789U);
    CHECK(sample->diagnostics.status == AS5600_STATUS_MAGNET_FOUND);
    CHECK(sample->diagnostics.agc == 99U);
    CHECK(sample->diagnostics.magnitude == 0x543U);
}

static as5600_result_t sample_call(client_t *client, as5600_sample_t *sample,
                                  uint32_t timeout_ms)
{
    as5600_result_t result;
    CHECK(client->operation == IDLE_OPERATION);
    client->operation = SAMPLE_OPERATION;
    result = as5600_read_sample(&client->device, sample, timeout_ms);
    client->operation = IDLE_OPERATION;
    CHECK(bus.active != client);
    if (result == AS5600_OK)
    {
        assert_sample(sample);
        CHECK(client->last_transfers == 6U);
        ++client->samples;
    }
    else
    {
        ++client->failures;
    }
    return result;
}

static as5600_result_t configuration_call(client_t *client,
                                         const as5600_config_t *configuration)
{
    as5600_result_t result;
    CHECK(client->operation == IDLE_OPERATION);
    client->operation = CONFIG_OPERATION;
    result = as5600_write_config(&client->device, configuration, IO_BUDGET);
    client->operation = IDLE_OPERATION;
    CHECK(bus.active != client);
    if (result == AS5600_OK)
    {
        CHECK(client->last_transfers == 3U);
        CHECK(client->last_delays == 1U);
        ++client->configurations;
    }
    else
    {
        ++client->failures;
    }
    return result;
}

static void assert_bus_idle(void)
{
    CHECK(bus.active == NULL);
    CHECK(bus.owner == NULL);
    CHECK(osMutexGetOwner(mutex) == NULL);
    CHECK(bus.acquired == bus.released);
}

static void run_timeout_contender(client_t *client)
{
    as5600_sample_t output;
    as5600_sample_t unchanged;
    const uint32_t before_io = bus.transfers;
    const uint32_t before_acquire = bus.acquired;
    const uint32_t before_release = bus.released;
    const uint32_t started = osKernelGetTickCount();
    (void)memset(&output, 0xA5, sizeof(output));
    (void)memcpy(&unchanged, &output, sizeof(output));
    CHECK(osMutexGetOwner(mutex) == clients[LOW].thread);
    CHECK(sample_call(client, &output, 5U) == AS5600_ERROR_TIMEOUT);
    timeout_elapsed = osKernelGetTickCount() - started;
    CHECK((timeout_elapsed >= 5U) && (timeout_elapsed <= 7U));
    CHECK(memcmp(&output, &unchanged, sizeof(output)) == 0);
    CHECK(bus.transfers == before_io);
    CHECK(bus.acquired == before_acquire);
    CHECK(bus.released == before_release);
    CHECK(osMutexGetOwner(mutex) == clients[LOW].thread);
    CHECK(bus.active == &clients[LOW]);
    CHECK(osThreadGetPriority(clients[LOW].thread) == osPriorityLow);
    timeout_seen = 1U;
}

static void run_stress(client_t *client)
{
    for (unsigned i = 0U; i < STRESS_ITERATIONS; ++i)
    {
        as5600_sample_t sample;
        as5600_config_t configuration;
        CHECK(sample_call(client, &sample, IO_BUDGET) == AS5600_OK);
        CHECK(as5600_default_config(&configuration) == AS5600_OK);
        configuration.output = AS5600_OUTPUT_PWM;
        configuration.hysteresis = (as5600_hysteresis_t)client->index;
        configuration.pwm_frequency = (as5600_pwm_t)(i % 4U);
        configuration.watchdog = (i % 2U) != 0U;
        CHECK(configuration_call(client, &configuration) == AS5600_OK);
        CHECK(osDelay(1U) == osOK);
    }
}

static void worker(void *argument)
{
    client_t *client = argument;
    for (;;)
    {
        const uint32_t command = osThreadFlagsWait(
            CMD_PI | CMD_TIMEOUT | CMD_STRESS, osFlagsWaitAny, WAIT_BOUND);
        CHECK((command & osFlagsError) == 0U);
        if (command == CMD_STRESS)
        {
            run_stress(client);
        }
        else if ((command == CMD_PI) && (client->index == MEDIUM))
        {
            CHECK(inheritance_seen == 1U);
            CHECK(clients[HIGH].samples == 1U);
            CHECK(osThreadGetPriority(clients[LOW].thread) == osPriorityLow);
            medium_ran = 1U;
        }
        else if ((command == CMD_TIMEOUT) && (client->index == HIGH))
        {
            run_timeout_contender(client);
        }
        else
        {
            as5600_sample_t sample;
            CHECK((command == CMD_PI) || (command == CMD_TIMEOUT));
            CHECK(sample_call(client, &sample, IO_BUDGET) == AS5600_OK);
            if (client->index == LOW)
            {
                CHECK(osThreadGetPriority(client->thread) == osPriorityLow);
                CHECK((phase != INHERITANCE) || (medium_ran == 1U));
            }
        }
        send_flags(clients[CONTROL].thread, 1U << client->index);
    }
}

static void run_io_errors(client_t *client)
{
    static const as5600_result_t errors[3] = {
        AS5600_ERROR_NACK, AS5600_ERROR_IO, AS5600_ERROR_TIMEOUT
    };
    for (unsigned error = 0U; error < 3U; ++error)
    {
        for (unsigned position = 1U; position <= 6U; ++position)
        {
            as5600_sample_t output;
            as5600_sample_t unchanged;
            const uint32_t before_io = bus.transfers;
            (void)memset(&output, 0xA5, sizeof(output));
            (void)memcpy(&unchanged, &output, sizeof(output));
            bus.fail_at = before_io + position;
            bus.failure = errors[error];
            CHECK(sample_call(client, &output, IO_BUDGET) == errors[error]);
            CHECK(memcmp(&output, &unchanged, sizeof(output)) == 0);
            CHECK(bus.transfers == before_io + position);
            CHECK(client->last_transfers == position);
            CHECK(bus.fail_at == 0U);
            assert_bus_idle();
            CHECK(sample_call(client, &output, IO_BUDGET) == AS5600_OK);
            assert_bus_idle();
        }
    }
    for (unsigned position = 1U; position <= 3U; ++position)
    {
        as5600_config_t configuration;
        const uint32_t before_io = bus.transfers;
        CHECK(as5600_default_config(&configuration) == AS5600_OK);
        configuration.watchdog = (bus.registers[0x07U] & 0x20U) == 0U;
        bus.fail_at = before_io + position;
        bus.failure = errors[position - 1U];
        CHECK(configuration_call(client, &configuration) ==
              errors[position - 1U]);
        CHECK(bus.transfers == before_io + position);
        CHECK(client->last_transfers == position);
        CHECK(bus.fail_at == 0U);
        assert_bus_idle();
        /* A readback error need not roll back a write that already succeeded. */
        configuration.watchdog = (bus.registers[0x07U] & 0x20U) == 0U;
        CHECK(configuration_call(client, &configuration) == AS5600_OK);
        CHECK(((bus.registers[0x07U] & 0x20U) != 0U) == configuration.watchdog);
        assert_bus_idle();
    }
}

static void coordinator(void *argument)
{
    client_t *client = argument;
    as5600_sample_t sample;
    char kernel[32] = {0};
    uint32_t samples = 0U;
    uint32_t configurations = 0U;
    uint32_t failures = 0U;
    CHECK(osKernelGetState() == osKernelRunning);
    CHECK(osKernelGetTickFreq() == 1000U);
    CHECK(osKernelGetInfo(NULL, kernel, sizeof(kernel)) == osOK);
    test_puts("Actual CMSIS-RTOS2 / ");
    test_puts(kernel);
    test_puts("; Cortex-M4 MPS2-AN386; simulated AS5600 bus\n");
    test_puts("Static RTOS allocation only; real SysTick/SVC/PendSV scheduling\n");

    phase = INHERITANCE;
    send_flags(clients[LOW].thread, CMD_PI);
    wait_done((1U << LOW) | (1U << HIGH) | (1U << MEDIUM));
    CHECK(inheritance_seen == 1U);
    CHECK(medium_ran == 1U);
    CHECK(bus.transfers == 12U);
    CHECK(bus.acquired == 2U);
    assert_bus_idle();
    test_puts("PASS: low owner inherits high priority, excludes ready medium, disinherits\n");

    phase = ACQUIRE_TIMEOUT;
    send_flags(clients[LOW].thread, CMD_TIMEOUT);
    wait_done((1U << LOW) | (1U << HIGH));
    CHECK(timeout_seen == 1U);
    assert_bus_idle();
    CHECK(sample_call(client, &sample, IO_BUDGET) == AS5600_OK);
    assert_bus_idle();
    test_puts("PASS: finite acquire timeout ");
    test_put_u32(timeout_elapsed);
    test_puts(" ticks, unchanged output, no I/O/unlock after failed acquire, recovery\n");

    phase = IO_ERRORS;
    run_io_errors(client);
    CHECK(bus.injected_errors == 21U);
    test_puts("PASS: NACK/IO/timeout at all 6 sample transfers; 3 config error stages; recovery\n");

    phase = STRESS;
    send_flags(clients[HIGH].thread, CMD_STRESS);
    send_flags(clients[MEDIUM].thread, CMD_STRESS);
    send_flags(clients[LOW].thread, CMD_STRESS);
    wait_done((1U << LOW) | (1U << HIGH) | (1U << MEDIUM));
    assert_bus_idle();
    for (unsigned i = 0U; i < CLIENTS; ++i)
    {
        samples += clients[i].samples;
        configurations += clients[i].configurations;
        failures += clients[i].failures;
        CHECK(osThreadGetStackSpace(clients[i].thread) >= 128U);
    }
    CHECK(samples == 70U);
    CHECK(configurations == 51U);
    CHECK(failures == 22U);
    CHECK(bus.attempts == 143U);
    CHECK(bus.acquired == 142U);
    CHECK(bus.released == 142U);
    CHECK(bus.transfers == 642U);
    CHECK(switches >= 100U);
    CHECK(kernel_ticks > 0U);
    CHECK(osThreadGetPriority(clients[LOW].thread) == osPriorityLow);
    CHECK(osThreadGetPriority(clients[MEDIUM].thread) == osPriorityNormal);
    test_puts("PASS: 3 concurrent tasks, 48 samples + 48 config updates, whole-operation ownership\n");
    test_puts("Counts: samples=70 config=51 expected_failures=22 transfers=642 locks=142 releases=142\n");
    test_puts("Scheduler switches=");
    test_put_u32(switches);
    test_puts(" ticks=");
    test_put_u32(kernel_ticks);
    test_puts("\nRESULT: PASS\n");
    test_exit(0U);
}

static void check_divide_runtime(void)
{
    static volatile uint64_t numerators[] = {
        UINT64_C(0), UINT64_C(100999), UINT64_C(0xFFFFFFFFFFFFFFFF),
        UINT64_C(0xFEDCBA9876543210)
    };
    static volatile uint64_t denominators[] = {
        UINT64_C(1000), UINT64_C(1000), UINT64_C(0x8000000000000000),
        UINT64_C(0x100000000)
    };
    static const uint64_t quotients[] = {
        UINT64_C(0), UINT64_C(100), UINT64_C(1), UINT64_C(0xFEDCBA98)
    };
    static const uint64_t remainders[] = {
        UINT64_C(0), UINT64_C(999), UINT64_C(0x7FFFFFFFFFFFFFFF),
        UINT64_C(0x76543210)
    };
    for (unsigned i = 0U; i < 4U; ++i)
    {
        const uint64_t numerator = numerators[i];
        const uint64_t denominator = denominators[i];
        CHECK(numerator / denominator == quotients[i]);
        CHECK(numerator % denominator == remainders[i]);
    }
}

int main(void)
{
    static const osMutexAttr_t mutex_attributes = {
        .name = "shared-as5600",
        .attr_bits = osMutexPrioInherit,
        .cb_mem = &mutex_storage,
        .cb_size = sizeof(mutex_storage)
    };
    static const osPriority_t priorities[CLIENTS] = {
        osPriorityAboveNormal, osPriorityLow, osPriorityHigh, osPriorityNormal
    };
    static const char *const names[CLIENTS] = {
        "coordinator", "low-owner", "high-waiter", "medium"
    };
    as5600_sample_t output;
    as5600_sample_t unchanged;
    check_divide_runtime();
    CHECK(osKernelInitialize() == osOK);
    mutex = osMutexNew(&mutex_attributes);
    CHECK(mutex != NULL);
    CHECK(as5600_cmsis_rtos2_init(&adapter, mutex, 0x7FFFFFFFU,
                                 &synchronization) == AS5600_OK);
    bus.registers[0x07U] = 0xC0U;
    bus.registers[0x0BU] = AS5600_STATUS_MAGNET_FOUND;
    bus.registers[0x0CU] = 0x0AU;
    bus.registers[0x0DU] = 0xBCU;
    bus.registers[0x0EU] = 0x07U;
    bus.registers[0x0FU] = 0x89U;
    bus.registers[0x1AU] = 99U;
    bus.registers[0x1BU] = 0x05U;
    bus.registers[0x1CU] = 0x43U;
    for (unsigned i = 0U; i < CLIENTS; ++i)
    {
        const as5600_bus_t interface = {
            .context = &clients[i],
            .read = read_registers,
            .write = write_registers,
            .now_ms = now_ms,
            .check_context = check_context,
            .delay_ms = settle,
            .lock = acquire,
            .unlock = release
        };
        const osThreadAttr_t attributes = {
            .name = names[i],
            .cb_mem = &task_storage[i],
            .cb_size = sizeof(task_storage[i]),
            .stack_mem = &stacks[i][0],
            .stack_size = sizeof(stacks[i]),
            .priority = priorities[i]
        };
        clients[i].index = i;
        CHECK(as5600_init(&clients[i].device, &interface) == AS5600_OK);
        clients[i].thread = osThreadNew(
            (i == CONTROL) ? coordinator : worker, &clients[i], &attributes);
        CHECK(clients[i].thread != NULL);
    }
    (void)memset(&output, 0xA5, sizeof(output));
    (void)memcpy(&unchanged, &output, sizeof(output));
    CHECK(as5600_read_sample(&clients[CONTROL].device, &output, IO_BUDGET) ==
          AS5600_ERROR_CONTEXT);
    CHECK(memcmp(&output, &unchanged, sizeof(output)) == 0);
    CHECK(bus.attempts == 0U);
    CHECK(bus.transfers == 0U);
    CHECK(osKernelStart() == osOK);
    CHECK(0);
    return 1;
}
