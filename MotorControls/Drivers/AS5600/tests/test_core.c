#include "as5600.h"

#include <inttypes.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The oracle uses the register contract, not private driver definitions. */
enum
{
    REG_ZMCO = 0x00,
    REG_ZPOS = 0x01,
    REG_MPOS = 0x03,
    REG_MANG = 0x05,
    REG_CONF = 0x07,
    REG_STATUS = 0x0b,
    REG_RAW = 0x0c,
    REG_ANGLE = 0x0e,
    REG_AGC = 0x1a,
    REG_MAGNITUDE = 0x1b,
    LOG_CAPACITY = 96
};

typedef enum
{
    CONTEXT, CLOCK, LOCK, READ, WRITE, DELAY, UNLOCK, KIND_COUNT
} event_kind_t;

typedef struct
{
    event_kind_t kind;
    uint8_t address;
    uint8_t reg;
    uint16_t length;
    uint16_t word;
    uint32_t timeout;
    uint32_t tick;
} event_t;

typedef struct
{
    uint8_t registers[256];
    uint32_t tick;
    bool locking;
    bool held;
    bool context_ok;
    bool read_seen[256];
    bool written[256];
    bool settled[256];
    uint32_t written_at[256];
    event_t log[LOG_CAPACITY];
    size_t log_count;
    unsigned actions;
    unsigned calls[KIND_COUNT];
    uint32_t cost[KIND_COUNT];
    unsigned fault_at;
    as5600_result_t fault_result;
    as5600_result_t release_result;
    unsigned advance_at;
    uint32_t advance_ms;
    unsigned clock_jump_at;
    uint32_t clock_jump_ms;
    uint8_t corrupt_reg;
    uint16_t corrupt_mask;
    bool scripted_status;
    uint8_t first_status;
    uint8_t last_status;
    unsigned status_reads;
    bool moving_angle;
} fake_t;

typedef enum
{
    OP_RAW, OP_ANGLE, OP_DIAGNOSTICS, OP_SAMPLE, OP_CONFIG,
    OP_SETTINGS, OP_WRITE_CONFIG, OP_POSITIONS, OP_MAX_ANGLE, OP_COUNT
} operation_t;

typedef union
{
    uint16_t angle;
    as5600_diagnostics_t diagnostics;
    as5600_sample_t sample;
    as5600_config_t configuration;
    as5600_settings_t settings;
} output_t;

typedef struct
{
    event_kind_t kind;
    uint8_t reg;
    uint16_t length;
} phase_t;

static uint64_t assertions;
static uint64_t api_calls;
static uint64_t failures;
static const char *test_name = "startup";
static uint32_t case_number;

static void check_at(bool condition, const char *expression, unsigned line)
{
    ++assertions;
    if (!condition)
    {
        ++failures;
        if (failures <= UINT64_C(30))
        {
            (void)fprintf(stderr, "%s case=%" PRIu32 " line=%u: %s\n",
                          test_name, case_number, line, expression);
        }
    }
}

#define CHECK(expression) check_at((expression), #expression, __LINE__)
#define API(expression) (++api_calls, (expression))

static void expect_result_at(as5600_result_t actual,
                             as5600_result_t expected, unsigned line)
{
    ++assertions;
    if (actual != expected)
    {
        ++failures;
        if (failures <= UINT64_C(30))
        {
            (void)fprintf(stderr,
                          "%s case=%" PRIu32 " line=%u: result=%d expected=%d\n",
                          test_name, case_number, line, (int)actual,
                          (int)expected);
        }
    }
}

#define RESULT(expression, expected) \
    expect_result_at((expression), (expected), __LINE__)

static uint16_t get_word(const fake_t *fake, uint8_t reg)
{
    return (uint16_t)(((uint16_t)fake->registers[reg] * UINT16_C(256)) |
                      fake->registers[(unsigned)reg + 1U]);
}

static void put_word(fake_t *fake, uint8_t reg, uint16_t word)
{
    fake->registers[reg] = (uint8_t)(word / UINT16_C(256));
    fake->registers[(unsigned)reg + 1U] = (uint8_t)word;
}

static void clear_trace(fake_t *fake)
{
    CHECK(!fake->held);
    fake->context_ok = false;
    memset(fake->read_seen, 0, sizeof(fake->read_seen));
    memset(fake->written, 0, sizeof(fake->written));
    memset(fake->settled, 0, sizeof(fake->settled));
    memset(fake->written_at, 0, sizeof(fake->written_at));
    memset(fake->log, 0, sizeof(fake->log));
    memset(fake->calls, 0, sizeof(fake->calls));
    memset(fake->cost, 0, sizeof(fake->cost));
    fake->log_count = 0U;
    fake->actions = 0U;
    fake->fault_at = 0U;
    fake->fault_result = AS5600_OK;
    fake->release_result = AS5600_OK;
    fake->advance_at = 0U;
    fake->advance_ms = 0U;
    fake->clock_jump_at = 0U;
    fake->clock_jump_ms = 0U;
    fake->corrupt_reg = 0U;
    fake->corrupt_mask = 0U;
    fake->scripted_status = false;
    fake->status_reads = 0U;
    fake->moving_angle = false;
}

static void reset_fake(fake_t *fake)
{
    memset(fake, 0, sizeof(*fake));
    fake->locking = true;
    fake->registers[REG_ZMCO] = UINT8_C(0xfe);
    fake->registers[REG_STATUS] = UINT8_C(0xa5);
    fake->registers[REG_AGC] = UINT8_C(0x96);
    put_word(fake, REG_RAW, UINT16_C(0xf123));
    put_word(fake, REG_ANGLE, UINT16_C(0xa456));
    put_word(fake, REG_MAGNITUDE, UINT16_C(0xd789));
    put_word(fake, REG_ZPOS, UINT16_C(0xa102));
    put_word(fake, REG_MPOS, UINT16_C(0xb303));
    put_word(fake, REG_MANG, UINT16_C(0xc400));
    put_word(fake, REG_CONF, UINT16_C(0xd287));
}

static event_t *record_event(fake_t *fake, event_kind_t kind)
{
    event_t *event;
    CHECK(fake->log_count < LOG_CAPACITY);
    if (fake->log_count >= LOG_CAPACITY)
    {
        (void)fprintf(stderr, "Callback trace overflow; stopping safely.\n");
        exit(EXIT_FAILURE);
    }
    event = &fake->log[fake->log_count++];
    memset(event, 0, sizeof(*event));
    event->kind = kind;
    event->tick = fake->tick;
    ++fake->calls[kind];
    if (kind != CLOCK)
    {
        ++fake->actions;
    }
    return event;
}

static as5600_result_t callback_result(const fake_t *fake)
{
    return (fake->actions == fake->fault_at) ?
        fake->fault_result : AS5600_OK;
}

static void advance_clock(fake_t *fake, event_kind_t kind)
{
    fake->tick += fake->cost[kind];
    if (fake->actions == fake->advance_at)
    {
        fake->tick += fake->advance_ms;
    }
}

static void valid_budget(uint32_t timeout)
{
    CHECK(timeout >= UINT32_C(1));
    CHECK(timeout <= UINT32_C(2147483647));
}

static void require_bus_owner(const fake_t *fake)
{
    CHECK(fake->context_ok);
    CHECK(fake->calls[CLOCK] > 0U);
    CHECK(!fake->locking || fake->held);
}

static as5600_result_t fake_context(void *context)
{
    fake_t *fake = (fake_t *)context;
    as5600_result_t result;
    CHECK(fake->log_count == 0U);
    (void)record_event(fake, CONTEXT);
    result = callback_result(fake);
    fake->context_ok = result == AS5600_OK;
    advance_clock(fake, CONTEXT);
    return result;
}

static uint32_t fake_now(void *context)
{
    fake_t *fake = (fake_t *)context;
    CHECK(fake->context_ok);
    (void)record_event(fake, CLOCK);
    if (fake->calls[CLOCK] == fake->clock_jump_at)
    {
        fake->tick += fake->clock_jump_ms;
    }
    return fake->tick;
}

static as5600_result_t fake_lock(void *context, uint32_t timeout)
{
    fake_t *fake = (fake_t *)context;
    event_t *event;
    as5600_result_t result;
    CHECK(fake->context_ok);
    CHECK(fake->calls[CLOCK] >= 1U);
    CHECK(!fake->held);
    CHECK(fake->calls[LOCK] == 0U);
    event = record_event(fake, LOCK);
    event->timeout = timeout;
    valid_budget(timeout);
    result = callback_result(fake);
    fake->held = result == AS5600_OK;
    advance_clock(fake, LOCK);
    return result;
}

static as5600_result_t fake_unlock(void *context)
{
    fake_t *fake = (fake_t *)context;
    as5600_result_t result;
    CHECK(fake->held);
    CHECK(fake->calls[UNLOCK] == 0U);
    (void)record_event(fake, UNLOCK);
    result = callback_result(fake);
    if (fake->release_result != AS5600_OK)
    {
        result = fake->release_result;
    }
    /* Failed release is modeled as released, but reported as untrustworthy. */
    fake->held = false;
    advance_clock(fake, UNLOCK);
    return result;
}

static bool valid_read(uint8_t reg, uint16_t length)
{
    if ((reg == REG_ZMCO) || (reg == REG_STATUS) || (reg == REG_AGC))
    {
        return length == 1U;
    }
    return ((reg == REG_RAW) || (reg == REG_ANGLE) ||
            (reg == REG_MAGNITUDE) || (reg == REG_ZPOS) ||
            (reg == REG_MPOS) || (reg == REG_MANG) ||
            (reg == REG_CONF)) && (length == 2U);
}

static as5600_result_t fake_read(
    void *context, uint8_t address, uint8_t reg, uint8_t *data,
    uint16_t length, uint32_t timeout)
{
    fake_t *fake = (fake_t *)context;
    event_t *event = record_event(fake, READ);
    as5600_result_t result = callback_result(fake);
    uint16_t word;
    require_bus_owner(fake);
    valid_budget(timeout);
    CHECK(address == UINT8_C(0x36));
    CHECK(data != NULL);
    CHECK(valid_read(reg, length));
    event->address = address;
    event->reg = reg;
    event->length = length;
    event->timeout = timeout;
    /* A burst spanning a special register is rejected, not emulated as RAM. */
    if ((data == NULL) || !valid_read(reg, length))
    {
        return AS5600_ERROR_IO;
    }
    if (fake->written[reg])
    {
        CHECK(fake->settled[reg]);
        CHECK((uint32_t)(fake->tick - fake->written_at[reg]) >= 1U);
    }
    if (result != AS5600_OK)
    {
        data[0] = UINT8_C(0xee);
    }
    else if (length == 1U)
    {
        data[0] = fake->registers[reg];
        if (reg == REG_STATUS)
        {
            if (fake->scripted_status)
            {
                data[0] = (fake->status_reads == 0U) ?
                    fake->first_status : fake->last_status;
            }
            ++fake->status_reads;
        }
        event->word = data[0];
    }
    else
    {
        word = get_word(fake, reg);
        if (fake->written[reg] && (reg == fake->corrupt_reg))
        {
            word ^= fake->corrupt_mask;
        }
        data[0] = (uint8_t)(word / UINT16_C(256));
        data[1] = (uint8_t)word;
        event->word = word;
        if (fake->moving_angle && (reg == REG_RAW))
        {
            put_word(fake, REG_ANGLE, UINT16_C(0xe987));
        }
    }
    fake->read_seen[reg] = true;
    advance_clock(fake, READ);
    return result;
}

static as5600_result_t fake_write(
    void *context, uint8_t address, uint8_t reg, const uint8_t *data,
    uint16_t length, uint32_t timeout)
{
    fake_t *fake = (fake_t *)context;
    event_t *event = record_event(fake, WRITE);
    as5600_result_t result = callback_result(fake);
    uint16_t word;
    uint16_t reserved;
    bool allowed = ((reg == REG_ZPOS) || (reg == REG_MPOS) ||
                    (reg == REG_MANG) || (reg == REG_CONF)) &&
                   (length == 2U);
    require_bus_owner(fake);
    valid_budget(timeout);
    CHECK(address == UINT8_C(0x36));
    CHECK(data != NULL);
    CHECK(allowed);
    CHECK(reg != UINT8_C(0xff));
    event->address = address;
    event->reg = reg;
    event->length = length;
    event->timeout = timeout;
    if ((data == NULL) || !allowed)
    {
        return AS5600_ERROR_IO;
    }
    word = (uint16_t)(((uint16_t)data[0] * UINT16_C(256)) | data[1]);
    reserved = (reg == REG_CONF) ? UINT16_C(0xc000) : UINT16_C(0xf000);
    CHECK(fake->read_seen[reg]);
    CHECK((word & reserved) == (get_word(fake, reg) & reserved));
    CHECK(word != get_word(fake, reg));
    event->word = word;
    fake->registers[reg] = data[0];
    if (result == AS5600_OK)
    {
        fake->registers[(unsigned)reg + 1U] = data[1];
    }
    fake->written[reg] = true;
    fake->settled[reg] = false;
    fake->written_at[reg] = fake->tick;
    advance_clock(fake, WRITE);
    return result;
}

static as5600_result_t fake_delay(
    void *context, uint32_t minimum, uint32_t timeout)
{
    fake_t *fake = (fake_t *)context;
    event_t *event = record_event(fake, DELAY);
    as5600_result_t result = callback_result(fake);
    unsigned reg;
    require_bus_owner(fake);
    valid_budget(timeout);
    CHECK(minimum >= 1U);
    CHECK(minimum < timeout);
    event->timeout = timeout;
    event->word = (uint16_t)minimum;
    if (result == AS5600_OK)
    {
        fake->tick += minimum;
        for (reg = 0U; reg < 256U; ++reg)
        {
            if (fake->written[reg])
            {
                fake->settled[reg] = true;
            }
        }
    }
    advance_clock(fake, DELAY);
    return result;
}

static as5600_bus_t make_bus(fake_t *fake)
{
    as5600_bus_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.context = fake;
    bus.read = fake_read;
    bus.write = fake_write;
    bus.now_ms = fake_now;
    bus.check_context = fake_context;
    bus.delay_ms = fake_delay;
    if (fake->locking)
    {
        bus.lock = fake_lock;
        bus.unlock = fake_unlock;
    }
    return bus;
}

static void bind_device(fake_t *fake, as5600_t *device)
{
    as5600_bus_t bus = make_bus(fake);
    memset(device, 0, sizeof(*device));
    RESULT(API(as5600_init(device, &bus)), AS5600_OK);
    CHECK(fake->log_count == 0U);
}

static as5600_config_t config_oracle(uint16_t word)
{
    static const as5600_power_t powers[4] = {
        AS5600_POWER_NORMAL, AS5600_POWER_LOW_1,
        AS5600_POWER_LOW_2, AS5600_POWER_LOW_3
    };
    static const as5600_hysteresis_t hysteresis[4] = {
        AS5600_HYSTERESIS_OFF, AS5600_HYSTERESIS_1_LSB,
        AS5600_HYSTERESIS_2_LSB, AS5600_HYSTERESIS_3_LSB
    };
    static const as5600_output_t outputs[4] = {
        AS5600_OUTPUT_ANALOG_FULL, AS5600_OUTPUT_ANALOG_REDUCED,
        AS5600_OUTPUT_PWM, AS5600_OUTPUT_ANALOG_FULL
    };
    static const as5600_pwm_t pwm[4] = {
        AS5600_PWM_115_HZ, AS5600_PWM_230_HZ,
        AS5600_PWM_460_HZ, AS5600_PWM_920_HZ
    };
    static const as5600_slow_filter_t slow[4] = {
        AS5600_SLOW_FILTER_16X, AS5600_SLOW_FILTER_8X,
        AS5600_SLOW_FILTER_4X, AS5600_SLOW_FILTER_2X
    };
    static const as5600_fast_filter_t fast[8] = {
        AS5600_FAST_FILTER_OFF, AS5600_FAST_FILTER_6_LSB,
        AS5600_FAST_FILTER_7_LSB, AS5600_FAST_FILTER_9_LSB,
        AS5600_FAST_FILTER_18_LSB, AS5600_FAST_FILTER_21_LSB,
        AS5600_FAST_FILTER_24_LSB, AS5600_FAST_FILTER_10_LSB
    };
    as5600_config_t configuration;
    memset(&configuration, 0, sizeof(configuration));
    configuration.power = powers[word % 4U];
    configuration.hysteresis = hysteresis[(word / 4U) % 4U];
    configuration.output = outputs[(word / 16U) % 4U];
    configuration.pwm_frequency = pwm[(word / 64U) % 4U];
    configuration.slow_filter = slow[(word / 256U) % 4U];
    configuration.fast_filter = fast[(word / 1024U) % 8U];
    configuration.watchdog = ((word / 8192U) % 2U) != 0U;
    return configuration;
}

static void expect_config(const as5600_config_t *actual,
                          const as5600_config_t *expected)
{
    CHECK(actual->power == expected->power);
    CHECK(actual->hysteresis == expected->hysteresis);
    CHECK(actual->output == expected->output);
    CHECK(actual->pwm_frequency == expected->pwm_frequency);
    CHECK(actual->slow_filter == expected->slow_filter);
    CHECK(actual->fast_filter == expected->fast_filter);
    CHECK(actual->watchdog == expected->watchdog);
}

static as5600_result_t operate(operation_t operation, const as5600_t *device,
                               output_t *output, uint32_t timeout)
{
    as5600_config_t configuration = config_oracle(UINT16_C(0x3da6));
    switch (operation)
    {
        case OP_RAW:
            return API(as5600_read_raw_angle(device, &output->angle, timeout));
        case OP_ANGLE:
            return API(as5600_read_angle(device, &output->angle, timeout));
        case OP_DIAGNOSTICS:
            return API(as5600_read_diagnostics(
                device, &output->diagnostics, timeout));
        case OP_SAMPLE:
            return API(as5600_read_sample(device, &output->sample, timeout));
        case OP_CONFIG:
            return API(as5600_read_config(
                device, &output->configuration, timeout));
        case OP_SETTINGS:
            return API(as5600_read_settings(device, &output->settings, timeout));
        case OP_WRITE_CONFIG:
            return API(as5600_write_config(device, &configuration, timeout));
        case OP_POSITIONS:
            return API(as5600_write_positions(device, 1000U, 1500U, timeout));
        case OP_MAX_ANGLE:
            return API(as5600_write_max_angle(device, 600U, timeout));
        default:
            CHECK(false);
            return AS5600_ERROR_ARGUMENT;
    }
}

static size_t action_trace(const fake_t *fake, event_t *events)
{
    size_t index;
    size_t count = 0U;
    for (index = 0U; index < fake->log_count; ++index)
    {
        if (fake->log[index].kind != CLOCK)
        {
            events[count++] = fake->log[index];
        }
    }
    return count;
}

static void expect_phases(const fake_t *fake, const phase_t *expected,
                          size_t count)
{
    size_t index;
    size_t phase = 0U;
    for (index = 0U; index < fake->log_count; ++index)
    {
        const event_t *event = &fake->log[index];
        if ((event->kind == READ) || (event->kind == WRITE) ||
            (event->kind == DELAY))
        {
            CHECK(phase < count);
            if (phase < count)
            {
                CHECK(event->kind == expected[phase].kind);
                CHECK(event->reg == expected[phase].reg);
                CHECK(event->length == expected[phase].length);
            }
            ++phase;
        }
    }
    CHECK(phase == count);
    CHECK(fake->calls[CONTEXT] == 1U);
    CHECK(fake->calls[LOCK] == (fake->locking ? 1U : 0U));
    CHECK(fake->calls[UNLOCK] == (fake->locking ? 1U : 0U));
    CHECK(!fake->held);
}

static const phase_t raw_phases[] = {{READ, REG_RAW, 2U}};
static const phase_t angle_phases[] = {{READ, REG_ANGLE, 2U}};
static const phase_t diagnostic_phases[] = {
    {READ, REG_STATUS, 1U}, {READ, REG_AGC, 1U}, {READ, REG_MAGNITUDE, 2U}
};
static const phase_t sample_phases[] = {
    {READ, REG_STATUS, 1U}, {READ, REG_RAW, 2U}, {READ, REG_ANGLE, 2U},
    {READ, REG_AGC, 1U}, {READ, REG_MAGNITUDE, 2U}, {READ, REG_STATUS, 1U}
};
static const phase_t config_phases[] = {{READ, REG_CONF, 2U}};
static const phase_t settings_phases[] = {
    {READ, REG_ZPOS, 2U}, {READ, REG_MPOS, 2U}, {READ, REG_MANG, 2U},
    {READ, REG_CONF, 2U}, {READ, REG_ZMCO, 1U}
};
static const phase_t write_config_phases[] = {
    {READ, REG_CONF, 2U}, {WRITE, REG_CONF, 2U},
    {DELAY, 0U, 0U}, {READ, REG_CONF, 2U}
};
static const phase_t positions_phases[] = {
    {READ, REG_ZPOS, 2U}, {WRITE, REG_ZPOS, 2U}, {DELAY, 0U, 0U},
    {READ, REG_ZPOS, 2U}, {READ, REG_MPOS, 2U},
    {WRITE, REG_MPOS, 2U}, {DELAY, 0U, 0U}, {READ, REG_MPOS, 2U}
};
static const phase_t max_angle_phases[] = {
    {READ, REG_MANG, 2U}, {WRITE, REG_MANG, 2U},
    {DELAY, 0U, 0U}, {READ, REG_MANG, 2U}
};

#define PHASES(fake, array) \
    expect_phases((fake), (array), sizeof(array) / sizeof((array)[0]))

static void expect_complete_operation(const fake_t *fake,
                                      operation_t operation)
{
    switch (operation)
    {
        case OP_RAW: PHASES(fake, raw_phases); break;
        case OP_ANGLE: PHASES(fake, angle_phases); break;
        case OP_DIAGNOSTICS: PHASES(fake, diagnostic_phases); break;
        case OP_SAMPLE: PHASES(fake, sample_phases); break;
        case OP_CONFIG: PHASES(fake, config_phases); break;
        case OP_SETTINGS: PHASES(fake, settings_phases); break;
        case OP_WRITE_CONFIG: PHASES(fake, write_config_phases); break;
        case OP_POSITIONS: PHASES(fake, positions_phases); break;
        case OP_MAX_ANGLE: PHASES(fake, max_angle_phases); break;
        default: CHECK(false); break;
    }
}

static void expect_no_activity(const fake_t *fake, const uint8_t *registers)
{
    CHECK(fake->log_count == 0U);
    CHECK(fake->actions == 0U);
    CHECK(!fake->held);
    CHECK(memcmp(fake->registers, registers, sizeof(fake->registers)) == 0);
}

static void test_binding_and_arguments(void)
{
    fake_t fake;
    as5600_t device;
    as5600_t zero = {0};
    as5600_t before_device;
    as5600_bus_t bus;
    as5600_bus_t invalid;
    output_t output;
    output_t before;
    uint8_t registers[256];
    as5600_config_t configuration;
    operation_t operation;
    unsigned index;
    static const uint32_t bad_timeouts[] = {
        0U, UINT32_C(2147483648), UINT32_MAX
    };

    test_name = "binding/arguments";
    reset_fake(&fake);
    bus = make_bus(&fake);
    memcpy(registers, fake.registers, sizeof(registers));
    memset(&device, 0, sizeof(device));
    memcpy(&before_device, &device, sizeof(device));
    RESULT(API(as5600_init(NULL, &bus)), AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_init(&device, NULL)), AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_init(NULL, NULL)), AS5600_ERROR_ARGUMENT);
    CHECK(memcmp(&device, &before_device, sizeof(device)) == 0);
    for (index = 0U; index < 7U; ++index)
    {
        invalid = bus;
        switch (index)
        {
            case 0U: invalid.read = NULL; break;
            case 1U: invalid.write = NULL; break;
            case 2U: invalid.now_ms = NULL; break;
            case 3U: invalid.check_context = NULL; break;
            case 4U: invalid.delay_ms = NULL; break;
            case 5U: invalid.lock = NULL; break;
            default: invalid.unlock = NULL; break;
        }
        RESULT(API(as5600_init(&device, &invalid)), AS5600_ERROR_ARGUMENT);
        CHECK(memcmp(&device, &before_device, sizeof(device)) == 0);
        expect_no_activity(&fake, registers);
    }
    bind_device(&fake, &device);
    expect_no_activity(&fake, registers);
    memcpy(&before_device, &device, sizeof(device));
    invalid = bus;
    invalid.read = NULL;
    RESULT(API(as5600_init(&device, &invalid)), AS5600_ERROR_ARGUMENT);
    CHECK(memcmp(&device, &before_device, sizeof(device)) == 0);
    invalid = bus;
    invalid.context = NULL;
    RESULT(API(as5600_init(&zero, &invalid)), AS5600_OK);
    memset(&zero, 0, sizeof(zero));

    memset(&before, 0xa5, sizeof(before));
    for (operation = OP_RAW; operation < OP_COUNT; ++operation)
    {
        case_number = (uint32_t)operation;
        memcpy(&output, &before, sizeof(output));
        RESULT(operate(operation, NULL, &output, 100U), AS5600_ERROR_ARGUMENT);
        RESULT(operate(operation, &zero, &output, 100U),
               AS5600_ERROR_NOT_INITIALIZED);
        for (index = 0U;
             index < sizeof(bad_timeouts) / sizeof(bad_timeouts[0]); ++index)
        {
            RESULT(operate(operation, &device, &output, bad_timeouts[index]),
                   AS5600_ERROR_ARGUMENT);
        }
        CHECK(memcmp(&output, &before, sizeof(output)) == 0);
        expect_no_activity(&fake, registers);
    }
    RESULT(API(as5600_read_raw_angle(&device, NULL, 100U)),
           AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_read_angle(&device, NULL, 100U)), AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_read_diagnostics(&device, NULL, 100U)),
           AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_read_sample(&device, NULL, 100U)), AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_read_config(&device, NULL, 100U)), AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_read_settings(&device, NULL, 100U)),
           AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_write_config(&device, NULL, 100U)), AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_default_config(NULL)), AS5600_ERROR_ARGUMENT);
    RESULT(API(as5600_counts_to_millidegrees(0U, NULL)), AS5600_ERROR_ARGUMENT);
    expect_no_activity(&fake, registers);

    memset(&configuration, 0, sizeof(configuration));
    RESULT(API(as5600_default_config(&configuration)), AS5600_OK);
    {
        const as5600_config_t expected = config_oracle(0U);
        expect_config(&configuration, &expected);
    }
    expect_no_activity(&fake, registers);
    CHECK(AS5600_I2C_ADDRESS == 0x36U);
    CHECK(AS5600_ANGLE_MAX == 4095U);
    CHECK(AS5600_MIN_RANGE_COUNTS == 205U);
    CHECK(AS5600_TIMEOUT_MAX_MS == UINT32_C(2147483647));

    for (operation = OP_RAW; operation < OP_COUNT; ++operation)
    {
        reset_fake(&fake);
        fake.locking = false;
        bind_device(&fake, &device);
        RESULT(operate(operation, &device, &output, 100U), AS5600_OK);
        expect_complete_operation(&fake, operation);
        reset_fake(&fake);
        bind_device(&fake, &device);
        RESULT(operate(operation, &device, &output, UINT32_C(2147483647)),
               AS5600_OK);
        expect_complete_operation(&fake, operation);
    }
}

static void test_masking_and_conversion(void)
{
    fake_t fake;
    as5600_t device;
    output_t output;
    uint32_t word;
    uint32_t degrees;
    uint32_t previous = 0U;
    as5600_config_t expected_config = config_oracle(UINT16_C(0xd287));

    test_name = "all 65536 transport words";
    reset_fake(&fake);
    bind_device(&fake, &device);
    for (word = 0U; word <= UINT16_MAX; ++word)
    {
        const uint16_t expected = (uint16_t)(word % UINT32_C(4096));
        case_number = word;
        clear_trace(&fake);
        put_word(&fake, REG_RAW, (uint16_t)word);
        put_word(&fake, REG_ANGLE, (uint16_t)word);
        put_word(&fake, REG_MAGNITUDE, (uint16_t)word);
        fake.registers[REG_STATUS] = 0U;
        RESULT(operate(OP_RAW, &device, &output, 100U), AS5600_OK);
        CHECK(output.angle == expected);
        PHASES(&fake, raw_phases);
        clear_trace(&fake);
        RESULT(operate(OP_ANGLE, &device, &output, 100U), AS5600_OK);
        CHECK(output.angle == expected);
        PHASES(&fake, angle_phases);
        clear_trace(&fake);
        RESULT(operate(OP_DIAGNOSTICS, &device, &output, 100U), AS5600_OK);
        CHECK(output.diagnostics.status == 0U);
        CHECK(output.diagnostics.agc == UINT8_C(0x96));
        CHECK(output.diagnostics.magnitude == expected);
        PHASES(&fake, diagnostic_phases);
        clear_trace(&fake);
        fake.registers[REG_STATUS] = UINT8_C(0xa5);
        RESULT(operate(OP_SAMPLE, &device, &output, 100U), AS5600_OK);
        CHECK(output.sample.raw_angle == expected);
        CHECK(output.sample.angle == expected);
        CHECK(output.sample.diagnostics.magnitude == expected);
        PHASES(&fake, sample_phases);
        clear_trace(&fake);
        put_word(&fake, REG_ZPOS, (uint16_t)word);
        put_word(&fake, REG_MPOS, (uint16_t)word);
        put_word(&fake, REG_MANG, (uint16_t)word);
        fake.registers[REG_ZMCO] = (uint8_t)word;
        RESULT(operate(OP_SETTINGS, &device, &output, 100U), AS5600_OK);
        CHECK(output.settings.zero_position == expected);
        CHECK(output.settings.stop_position == expected);
        CHECK(output.settings.max_angle == expected);
        CHECK(output.settings.burn_count == (uint8_t)(word % 4U));
        expect_config(&output.settings.configuration, &expected_config);
        PHASES(&fake, settings_phases);
    }

    test_name = "all count conversions";
    for (word = 0U; word <= UINT16_MAX; ++word)
    {
        case_number = word;
        degrees = UINT32_C(0xdeadbeef);
        if (word < UINT32_C(4096))
        {
            uint64_t numerator = (uint64_t)word * UINT64_C(360000);
            uint32_t expected = (uint32_t)(numerator / UINT64_C(4096));
            if ((numerator % UINT64_C(4096)) >= UINT64_C(2048))
            {
                ++expected;
            }
            RESULT(API(as5600_counts_to_millidegrees((uint16_t)word, &degrees)),
                   AS5600_OK);
            CHECK(degrees == expected);
            CHECK((word == 0U) || (degrees > previous));
            previous = degrees;
        }
        else
        {
            RESULT(API(as5600_counts_to_millidegrees((uint16_t)word, &degrees)),
                   AS5600_ERROR_ARGUMENT);
            CHECK(degrees == UINT32_C(0xdeadbeef));
        }
    }
    CHECK(previous == UINT32_C(359912));
}

static void test_configurations(void)
{
    fake_t fake;
    as5600_t device;
    as5600_config_t configuration;
    as5600_config_t before_configuration;
    output_t output;
    output_t before;
    uint32_t word;
    unsigned field;
    unsigned invalid_index;
    uint8_t registers[256];
    static const int invalid_values[] = {-1, 8, 255, INT_MAX, INT_MIN};

    test_name = "all configuration words";
    reset_fake(&fake);
    bind_device(&fake, &device);
    memset(&before, 0xa5, sizeof(before));
    for (word = 0U; word <= UINT16_MAX; ++word)
    {
        case_number = word;
        clear_trace(&fake);
        put_word(&fake, REG_CONF, (uint16_t)word);
        memcpy(&output, &before, sizeof(output));
        if (((word / 16U) % 4U) == 3U)
        {
            RESULT(operate(OP_CONFIG, &device, &output, 100U),
                   AS5600_ERROR_DATA);
            CHECK(memcmp(&output, &before, sizeof(output)) == 0);
            PHASES(&fake, config_phases);
            clear_trace(&fake);
            RESULT(operate(OP_SETTINGS, &device, &output, 100U),
                   AS5600_ERROR_DATA);
            CHECK(memcmp(&output, &before, sizeof(output)) == 0);
            expect_phases(&fake, settings_phases, 4U);
        }
        else
        {
            configuration = config_oracle((uint16_t)word);
            RESULT(operate(OP_CONFIG, &device, &output, 100U), AS5600_OK);
            expect_config(&output.configuration, &configuration);
            PHASES(&fake, config_phases);
            clear_trace(&fake);
            put_word(&fake, REG_CONF, (uint16_t)(word ^ 1U));
            memcpy(&before_configuration, &configuration, sizeof(configuration));
            RESULT(API(as5600_write_config(&device, &configuration, 100U)),
                   AS5600_OK);
            CHECK(get_word(&fake, REG_CONF) == (uint16_t)word);
            CHECK(memcmp(&configuration, &before_configuration,
                         sizeof(configuration)) == 0);
            PHASES(&fake, write_config_phases);
            clear_trace(&fake);
            RESULT(API(as5600_write_config(&device, &configuration, 1U)),
                   AS5600_OK);
            PHASES(&fake, config_phases);
            CHECK(fake.calls[WRITE] == 0U);
            CHECK(fake.calls[DELAY] == 0U);
        }
    }
    test_name = "invalid configuration enums";
    clear_trace(&fake);
    memcpy(registers, fake.registers, sizeof(registers));
    for (field = 0U; field < 6U; ++field)
    {
        for (invalid_index = 0U; invalid_index <=
             sizeof(invalid_values) / sizeof(invalid_values[0]); ++invalid_index)
        {
            int invalid_value = (invalid_index == 0U) ?
                ((field == 2U) ? 3 : ((field == 5U) ? 8 : 4)) :
                invalid_values[invalid_index - 1U];
            case_number = field * 10U + invalid_index;
            configuration = config_oracle(0U);
            switch (field)
            {
                case 0U:
                    configuration.power = (as5600_power_t)invalid_value; break;
                case 1U:
                    configuration.hysteresis =
                        (as5600_hysteresis_t)invalid_value; break;
                case 2U:
                    configuration.output = (as5600_output_t)invalid_value; break;
                case 3U:
                    configuration.pwm_frequency =
                        (as5600_pwm_t)invalid_value; break;
                case 4U:
                    configuration.slow_filter =
                        (as5600_slow_filter_t)invalid_value; break;
                default:
                    configuration.fast_filter =
                        (as5600_fast_filter_t)invalid_value; break;
            }
            memcpy(&before_configuration, &configuration, sizeof(configuration));
            RESULT(API(as5600_write_config(&device, &configuration, 100U)),
                   AS5600_ERROR_ARGUMENT);
            CHECK(memcmp(&configuration, &before_configuration,
                         sizeof(configuration)) == 0);
            expect_no_activity(&fake, registers);
        }
    }
}

static void check_position_case(fake_t *fake, const as5600_t *device,
                                uint16_t zero, uint16_t span, unsigned nibble)
{
    uint16_t stop = (uint16_t)(((uint32_t)zero + span) % UINT32_C(4096));
    uint16_t old_zero = (uint16_t)((nibble * 4096U) | (zero ^ 1U));
    uint16_t old_stop = (uint16_t)(((15U - nibble) * 4096U) | (stop ^ 1U));
    uint8_t registers[256];
    clear_trace(fake);
    put_word(fake, REG_ZPOS, old_zero);
    put_word(fake, REG_MPOS, old_stop);
    if ((span == 0U) || (span >= 205U))
    {
        RESULT(API(as5600_write_positions(device, zero, stop, 100U)), AS5600_OK);
        CHECK(get_word(fake, REG_ZPOS) == (uint16_t)((nibble * 4096U) | zero));
        CHECK(get_word(fake, REG_MPOS) ==
              (uint16_t)(((15U - nibble) * 4096U) | stop));
        PHASES(fake, positions_phases);
    }
    else
    {
        memcpy(registers, fake->registers, sizeof(registers));
        RESULT(API(as5600_write_positions(device, zero, stop, 100U)),
               AS5600_ERROR_ARGUMENT);
        expect_no_activity(fake, registers);
    }
}

static void test_positions_and_ranges(void)
{
    fake_t fake;
    as5600_t device;
    uint32_t value;
    unsigned nibble;
    unsigned boundary;
    uint8_t registers[256];
    static const uint16_t boundaries[] = {0U, 1U, 2U, 203U, 204U, 205U,
                                         206U, 4094U, 4095U};
    static const phase_t no_change[] = {
        {READ, REG_ZPOS, 2U}, {READ, REG_MPOS, 2U}
    };
    static const phase_t only_stop[] = {
        {READ, REG_ZPOS, 2U}, {READ, REG_MPOS, 2U}, {WRITE, REG_MPOS, 2U},
        {DELAY, 0U, 0U}, {READ, REG_MPOS, 2U}
    };
    static const phase_t only_zero[] = {
        {READ, REG_ZPOS, 2U}, {WRITE, REG_ZPOS, 2U},
        {DELAY, 0U, 0U}, {READ, REG_ZPOS, 2U}, {READ, REG_MPOS, 2U}
    };
    static const phase_t max_no_change[] = {{READ, REG_MANG, 2U}};

    test_name = "position boundaries/all origins/all reserved nibbles";
    reset_fake(&fake);
    bind_device(&fake, &device);
    for (value = 0U; value < 4096U; ++value)
    {
        case_number = value;
        for (boundary = 0U; boundary <
             sizeof(boundaries) / sizeof(boundaries[0]); ++boundary)
        {
            for (nibble = 0U; nibble < 16U; ++nibble)
            {
                check_position_case(&fake, &device, (uint16_t)value,
                                    boundaries[boundary], nibble);
            }
        }
    }
    test_name = "all wrapped spans";
    for (value = 0U; value < 4096U; ++value)
    {
        case_number = value;
        check_position_case(&fake, &device, 0U, (uint16_t)value, 0U);
        check_position_case(&fake, &device, 2048U, (uint16_t)value, 8U);
        check_position_case(&fake, &device, 4095U, (uint16_t)value, 15U);
    }
    test_name = "all max angles/all reserved nibbles";
    for (value = 0U; value < 4096U; ++value)
    {
        case_number = value;
        for (nibble = 0U; nibble < 16U; ++nibble)
        {
            clear_trace(&fake);
            put_word(&fake, REG_MANG,
                     (uint16_t)((nibble * 4096U) | (value ^ 1U)));
            if ((value == 0U) || (value >= 205U))
            {
                RESULT(API(as5600_write_max_angle(
                    &device, (uint16_t)value, 100U)), AS5600_OK);
                CHECK(get_word(&fake, REG_MANG) ==
                      (uint16_t)((nibble * 4096U) | value));
                PHASES(&fake, max_angle_phases);
                clear_trace(&fake);
                RESULT(API(as5600_write_max_angle(
                    &device, (uint16_t)value, 1U)), AS5600_OK);
                PHASES(&fake, max_no_change);
            }
            else
            {
                memcpy(registers, fake.registers, sizeof(registers));
                RESULT(API(as5600_write_max_angle(
                    &device, (uint16_t)value, 100U)), AS5600_ERROR_ARGUMENT);
                expect_no_activity(&fake, registers);
            }
        }
    }
    test_name = "all out-of-domain position inputs";
    clear_trace(&fake);
    memcpy(registers, fake.registers, sizeof(registers));
    for (value = 4096U; value <= UINT16_MAX; ++value)
    {
        case_number = value;
        RESULT(API(as5600_write_positions(&device, (uint16_t)value, 0U, 100U)),
               AS5600_ERROR_ARGUMENT);
        RESULT(API(as5600_write_positions(&device, 0U, (uint16_t)value, 100U)),
               AS5600_ERROR_ARGUMENT);
        RESULT(API(as5600_write_max_angle(&device, (uint16_t)value, 100U)),
               AS5600_ERROR_ARGUMENT);
        expect_no_activity(&fake, registers);
    }
    test_name = "individual unchanged positions";
    clear_trace(&fake);
    put_word(&fake, REG_ZPOS, UINT16_C(0xf3e8));
    put_word(&fake, REG_MPOS, UINT16_C(0xa5dc));
    RESULT(API(as5600_write_positions(&device, 1000U, 1500U, 1U)), AS5600_OK);
    PHASES(&fake, no_change);
    clear_trace(&fake);
    RESULT(API(as5600_write_positions(&device, 1000U, 1501U, 100U)), AS5600_OK);
    PHASES(&fake, only_stop);
    clear_trace(&fake);
    RESULT(API(as5600_write_positions(&device, 999U, 1501U, 100U)), AS5600_OK);
    PHASES(&fake, only_zero);
}

static as5600_result_t magnet_oracle(uint8_t status)
{
    static const as5600_result_t outcomes[8] = {
        AS5600_ERROR_NO_MAGNET, AS5600_ERROR_NO_MAGNET,
        AS5600_ERROR_NO_MAGNET, AS5600_ERROR_DATA,
        AS5600_OK, AS5600_ERROR_MAGNET_STRONG,
        AS5600_ERROR_MAGNET_WEAK, AS5600_ERROR_DATA
    };
    return outcomes[(status / 8U) % 8U];
}

static void test_magnet_and_sample(void)
{
    fake_t fake;
    as5600_t device;
    output_t output;
    output_t before;
    unsigned status;
    unsigned placement;
    test_name = "all status bytes/both sample checks";
    reset_fake(&fake);
    bind_device(&fake, &device);
    memset(&before, 0x5a, sizeof(before));
    for (status = 0U; status <= UINT8_MAX; ++status)
    {
        as5600_result_t expected = magnet_oracle((uint8_t)status);
        case_number = status;
        RESULT(API(as5600_magnet_status((uint8_t)status)), expected);
        clear_trace(&fake);
        fake.registers[REG_STATUS] = (uint8_t)status;
        RESULT(operate(OP_DIAGNOSTICS, &device, &output, 100U), AS5600_OK);
        CHECK(output.diagnostics.status == (uint8_t)status);
        PHASES(&fake, diagnostic_phases);
        for (placement = 0U; placement < 2U; ++placement)
        {
            clear_trace(&fake);
            fake.scripted_status = true;
            fake.first_status = (placement == 0U) ? (uint8_t)status : 0x20U;
            fake.last_status = (placement == 0U) ? 0xa5U : (uint8_t)status;
            memcpy(&output, &before, sizeof(output));
            RESULT(operate(OP_SAMPLE, &device, &output, 100U), expected);
            if (expected != AS5600_OK)
            {
                CHECK(memcmp(&output, &before, sizeof(output)) == 0);
            }
            else
            {
                CHECK(output.sample.diagnostics.status == fake.last_status);
            }
            expect_phases(&fake, sample_phases,
                          ((placement == 0U) && (expected != AS5600_OK)) ?
                          1U : 6U);
        }
    }
    test_name = "sequential samples/no cached measurements";
    clear_trace(&fake);
    fake.registers[REG_STATUS] = 0x20U;
    fake.moving_angle = true;
    RESULT(operate(OP_SAMPLE, &device, &output, 100U), AS5600_OK);
    CHECK(output.sample.raw_angle == 0x123U);
    CHECK(output.sample.angle == 0x987U);
    PHASES(&fake, sample_phases);
    clear_trace(&fake);
    put_word(&fake, REG_RAW, UINT16_C(0x0345));
    RESULT(operate(OP_RAW, &device, &output, 100U), AS5600_OK);
    CHECK(output.angle == 0x345U);
}

static void expect_fault_trace(const fake_t *fake, const event_t *reference,
                               size_t fault_index)
{
    event_t actual[LOG_CAPACITY];
    size_t count = action_trace(fake, actual);
    size_t index;
    bool acquired = fault_index > 1U;
    bool fault_is_release = reference[fault_index].kind == UNLOCK;
    size_t expected = fault_index + 1U +
        ((acquired && !fault_is_release) ? 1U : 0U);
    CHECK(count == expected);
    for (index = 0U; (index <= fault_index) && (index < count); ++index)
    {
        CHECK(actual[index].kind == reference[index].kind);
        CHECK(actual[index].reg == reference[index].reg);
        CHECK(actual[index].length == reference[index].length);
    }
    CHECK(fake->calls[UNLOCK] == (acquired ? 1U : 0U));
    CHECK(!fake->held);
    if (acquired && (count > 0U))
    {
        CHECK(actual[count - 1U].kind == UNLOCK);
    }
    if (fault_index == 0U)
    {
        CHECK(fake->calls[CLOCK] == 0U);
    }
}

static void test_fault_matrix(void)
{
    fake_t fake;
    as5600_t device;
    output_t output;
    output_t before;
    event_t reference[LOG_CAPACITY];
    operation_t operation;
    size_t steps;
    size_t step;
    int error;
    test_name = "every callback/every error/every operation";
    memset(&before, 0xa5, sizeof(before));
    for (operation = OP_RAW; operation < OP_COUNT; ++operation)
    {
        reset_fake(&fake);
        bind_device(&fake, &device);
        RESULT(operate(operation, &device, &output, 100U), AS5600_OK);
        expect_complete_operation(&fake, operation);
        steps = action_trace(&fake, reference);
        for (step = 0U; step < steps; ++step)
        {
            for (error = (int)AS5600_ERROR_ARGUMENT;
                 error <= (int)AS5600_ERROR_MAGNET_STRONG; ++error)
            {
                case_number = (uint32_t)operation * 1000U +
                    (uint32_t)step * 20U + (uint32_t)error;
                reset_fake(&fake);
                bind_device(&fake, &device);
                fake.fault_at = (unsigned)step + 1U;
                fake.fault_result = (as5600_result_t)error;
                memcpy(&output, &before, sizeof(output));
                RESULT(operate(operation, &device, &output, 100U),
                       (reference[step].kind == UNLOCK) ?
                       AS5600_ERROR_UNLOCK : (as5600_result_t)error);
                CHECK(memcmp(&output, &before, sizeof(output)) == 0);
                expect_fault_trace(&fake, reference, step);
                if ((reference[step].kind == READ) ||
                    (reference[step].kind == WRITE) ||
                    (reference[step].kind == DELAY))
                {
                    /* Retain partially modified hardware for recovery. */
                    clear_trace(&fake);
                    RESULT(operate(operation, &device, &output, 100U),
                           AS5600_OK);
                    CHECK(fake.calls[LOCK] == 1U);
                    CHECK(fake.calls[UNLOCK] == 1U);
                    CHECK(!fake.held);
                }
            }
            if ((reference[step].kind != CONTEXT) &&
                (reference[step].kind != LOCK) &&
                (reference[step].kind != UNLOCK))
            {
                reset_fake(&fake);
                bind_device(&fake, &device);
                fake.fault_at = (unsigned)step + 1U;
                fake.fault_result = AS5600_ERROR_NACK;
                fake.release_result = AS5600_ERROR_IO;
                memcpy(&output, &before, sizeof(output));
                RESULT(operate(operation, &device, &output, 100U),
                       AS5600_ERROR_UNLOCK);
                CHECK(memcmp(&output, &before, sizeof(output)) == 0);
                expect_fault_trace(&fake, reference, step);
            }
        }
    }
}

static void test_verification_and_partial_writes(void)
{
    fake_t fake;
    as5600_t device;
    output_t output;
    unsigned reg_index;
    unsigned bit;
    static const uint8_t registers[] = {
        REG_CONF, REG_ZPOS, REG_MPOS, REG_MANG
    };
    static const operation_t operations[] = {
        OP_WRITE_CONFIG, OP_POSITIONS, OP_POSITIONS, OP_MAX_ANGLE
    };
    test_name = "fullword readback/every bit";
    for (reg_index = 0U; reg_index < 4U; ++reg_index)
    {
        for (bit = 0U; bit < 16U; ++bit)
        {
            case_number = reg_index * 16U + bit;
            reset_fake(&fake);
            bind_device(&fake, &device);
            fake.corrupt_reg = registers[reg_index];
            fake.corrupt_mask = (uint16_t)(UINT32_C(1) << bit);
            RESULT(operate(operations[reg_index], &device, &output, 100U),
                   AS5600_ERROR_VERIFY);
            CHECK(fake.calls[UNLOCK] == 1U);
            CHECK(fake.calls[WRITE] == ((reg_index == 2U) ? 2U : 1U));
            CHECK(fake.calls[DELAY] == fake.calls[WRITE]);
            if (reg_index == 1U)
            {
                CHECK(get_word(&fake, REG_ZPOS) == UINT16_C(0xa3e8));
                CHECK(get_word(&fake, REG_MPOS) == UINT16_C(0xb303));
            }
            clear_trace(&fake);
            RESULT(operate(operations[reg_index], &device, &output, 100U),
                   AS5600_OK);
        }
    }
    test_name = "partial two-register programming/no rollback";
    reset_fake(&fake);
    bind_device(&fake, &device);
    fake.fault_at = 8U; /* Second write: C L R W D R R W. */
    fake.fault_result = AS5600_ERROR_NACK;
    RESULT(operate(OP_POSITIONS, &device, &output, 100U), AS5600_ERROR_NACK);
    CHECK(get_word(&fake, REG_ZPOS) == UINT16_C(0xa3e8));
    CHECK(get_word(&fake, REG_MPOS) == UINT16_C(0xb503));
    CHECK(fake.calls[WRITE] == 2U);
    CHECK(fake.calls[READ] == 3U);
    CHECK(fake.calls[DELAY] == 1U);
    CHECK(fake.calls[UNLOCK] == 1U);
    clear_trace(&fake);
    RESULT(operate(OP_POSITIONS, &device, &output, 100U), AS5600_OK);
    CHECK(get_word(&fake, REG_MPOS) == UINT16_C(0xb5dc));
}

static void expect_budgets(const fake_t *fake, uint32_t started,
                           uint32_t budget)
{
    size_t index;
    for (index = 0U; index < fake->log_count; ++index)
    {
        const event_t *event = &fake->log[index];
        if ((event->kind == LOCK) || (event->kind == READ) ||
            (event->kind == WRITE) || (event->kind == DELAY))
        {
            uint32_t elapsed = event->tick - started;
            CHECK(elapsed < budget);
            CHECK(event->timeout == budget - elapsed);
        }
    }
}

static void set_costs(fake_t *fake)
{
    fake->cost[LOCK] = 2U;
    fake->cost[READ] = 3U;
    fake->cost[WRITE] = 4U;
    fake->cost[UNLOCK] = 2U;
}

static void test_deadlines(void)
{
    fake_t fake;
    as5600_t device;
    output_t output;
    output_t before;
    operation_t operation;
    event_t reference[LOG_CAPACITY];
    size_t actions;
    size_t action;
    unsigned clocks;
    unsigned clock;
    uint32_t elapsed;
    uint32_t budget;
    unsigned origin;
    static const uint32_t origins[] = {
        0U, UINT32_MAX - 1U, UINT32_MAX - 30U
    };
    memset(&before, 0x96, sizeof(before));
    test_name = "single deadline/remainders/wrap/every integer budget";
    for (operation = OP_RAW; operation < OP_COUNT; ++operation)
    {
        reset_fake(&fake);
        bind_device(&fake, &device);
        set_costs(&fake);
        RESULT(operate(operation, &device, &output, 100U), AS5600_OK);
        elapsed = fake.tick;
        actions = action_trace(&fake, reference);
        clocks = fake.calls[CLOCK];
        expect_budgets(&fake, 0U, 100U);
        for (origin = 0U; origin < sizeof(origins) / sizeof(origins[0]); ++origin)
        {
            for (budget = 1U; budget <= elapsed + 2U; ++budget)
            {
                case_number = (uint32_t)operation * 1000U +
                    origin * 100U + budget;
                reset_fake(&fake);
                bind_device(&fake, &device);
                fake.tick = origins[origin];
                set_costs(&fake);
                memcpy(&output, &before, sizeof(output));
                RESULT(operate(operation, &device, &output, budget),
                       (budget > elapsed) ? AS5600_OK : AS5600_ERROR_TIMEOUT);
                if (budget <= elapsed)
                {
                    CHECK(memcmp(&output, &before, sizeof(output)) == 0);
                }
                CHECK(fake.calls[LOCK] == 1U);
                CHECK(fake.calls[UNLOCK] == 1U);
                CHECK(!fake.held);
                expect_budgets(&fake, origins[origin], budget);
            }
        }

        test_name = "late successful callbacks/every phase";
        for (action = 1U; action < actions; ++action)
        {
            case_number = (uint32_t)operation * 100U + (uint32_t)action;
            reset_fake(&fake);
            bind_device(&fake, &device);
            fake.advance_at = (unsigned)action + 1U;
            fake.advance_ms = 100U;
            memcpy(&output, &before, sizeof(output));
            RESULT(operate(operation, &device, &output, 100U),
                   AS5600_ERROR_TIMEOUT);
            CHECK(memcmp(&output, &before, sizeof(output)) == 0);
            CHECK(fake.calls[UNLOCK] == 1U);
            CHECK(!fake.held);
            expect_budgets(&fake, 0U, 100U);
            clear_trace(&fake);
            RESULT(operate(OP_RAW, &device, &output, 100U), AS5600_OK);
        }
        test_name = "expiry at every clock observation";
        for (clock = 2U; clock <= clocks; ++clock)
        {
            case_number = (uint32_t)operation * 100U + clock;
            reset_fake(&fake);
            bind_device(&fake, &device);
            fake.clock_jump_at = clock;
            fake.clock_jump_ms = 100U;
            memcpy(&output, &before, sizeof(output));
            RESULT(operate(operation, &device, &output, 100U),
                   AS5600_ERROR_TIMEOUT);
            CHECK(memcmp(&output, &before, sizeof(output)) == 0);
            CHECK(fake.calls[UNLOCK] == 1U);
            CHECK(!fake.held);
        }
        test_name = "single deadline/remainders/wrap/every integer budget";
    }
    test_name = "maximum signed deadline across wrap";
    reset_fake(&fake);
    bind_device(&fake, &device);
    fake.tick = UINT32_C(0x80000005);
    fake.cost[LOCK] = UINT32_C(2147483645);
    RESULT(operate(OP_RAW, &device, &output, UINT32_C(2147483647)), AS5600_OK);
    expect_budgets(&fake, UINT32_C(0x80000005), UINT32_C(2147483647));
    clear_trace(&fake);
    fake.tick = UINT32_C(0x80000005);
    fake.cost[LOCK] = UINT32_C(2147483645);
    fake.cost[READ] = 2U;
    memcpy(&output, &before, sizeof(output));
    RESULT(operate(OP_RAW, &device, &output, UINT32_C(2147483647)),
           AS5600_ERROR_TIMEOUT);
    CHECK(memcmp(&output, &before, sizeof(output)) == 0);
    expect_budgets(&fake, UINT32_C(0x80000005), UINT32_C(2147483647));

    test_name = "release failure overrides timeout";
    reset_fake(&fake);
    bind_device(&fake, &device);
    fake.cost[READ] = 100U;
    fake.release_result = AS5600_ERROR_BUSY;
    memcpy(&output, &before, sizeof(output));
    RESULT(operate(OP_SAMPLE, &device, &output, 100U), AS5600_ERROR_UNLOCK);
    CHECK(memcmp(&output, &before, sizeof(output)) == 0);
    CHECK(fake.calls[UNLOCK] == 1U);
}

static void test_independent_handles(void)
{
    fake_t first;
    fake_t second;
    as5600_t one;
    as5600_t alias;
    as5600_t two;
    output_t output;
    as5600_bus_t shared;
    test_name = "independent devices/shared physical bus/no cache";
    reset_fake(&first);
    reset_fake(&second);
    bind_device(&first, &one);
    bind_device(&second, &two);
    shared = make_bus(&first);
    memset(&alias, 0, sizeof(alias));
    RESULT(API(as5600_init(&alias, &shared)), AS5600_OK);
    CHECK(alias.bus.context == one.bus.context);
    CHECK(alias.bus.lock == one.bus.lock);
    CHECK(alias.bus.unlock == one.bus.unlock);
    put_word(&first, REG_RAW, 123U);
    put_word(&second, REG_RAW, 456U);
    RESULT(operate(OP_RAW, &one, &output, 100U), AS5600_OK);
    CHECK(output.angle == 123U);
    RESULT(operate(OP_RAW, &two, &output, 100U), AS5600_OK);
    CHECK(output.angle == 456U);
    clear_trace(&first);
    put_word(&first, REG_RAW, 789U);
    RESULT(operate(OP_RAW, &alias, &output, 100U), AS5600_OK);
    CHECK(output.angle == 789U);
    clear_trace(&first);
    RESULT(operate(OP_RAW, &one, &output, 100U), AS5600_OK);
    CHECK(output.angle == 789U);
    clear_trace(&first);
    RESULT(operate(OP_WRITE_CONFIG, &alias, &output, 100U), AS5600_OK);
    clear_trace(&first);
    RESULT(operate(OP_CONFIG, &one, &output, 100U), AS5600_OK);
    {
        const as5600_config_t expected = config_oracle(UINT16_C(0x3da6));
        expect_config(&output.configuration, &expected);
    }
    CHECK(second.calls[READ] == 1U);
}

int main(void)
{
    test_binding_and_arguments();
    test_masking_and_conversion();
    test_configurations();
    test_positions_and_ranges();
    test_magnet_and_sample();
    test_fault_matrix();
    test_verification_and_partial_writes();
    test_deadlines();
    test_independent_handles();
    (void)printf("core: %" PRIu64 " API calls, %" PRIu64
                 " assertions, %" PRIu64 " failures\n",
                 api_calls, assertions, failures);
    return (failures == 0U) ? EXIT_SUCCESS : EXIT_FAILURE;
}
