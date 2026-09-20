#include "as5600.h"

#include <limits.h>
#include <stddef.h>

#if CHAR_BIT != 8
#error "The AS5600 driver requires 8-bit bytes."
#endif

#define AS5600_REG_ZMCO       UINT8_C(0x00)
#define AS5600_REG_ZPOS       UINT8_C(0x01)
#define AS5600_REG_MPOS       UINT8_C(0x03)
#define AS5600_REG_MANG       UINT8_C(0x05)
#define AS5600_REG_CONF       UINT8_C(0x07)
#define AS5600_REG_STATUS     UINT8_C(0x0B)
#define AS5600_REG_RAW_ANGLE  UINT8_C(0x0C)
#define AS5600_REG_ANGLE      UINT8_C(0x0E)
#define AS5600_REG_AGC        UINT8_C(0x1A)
#define AS5600_REG_MAGNITUDE  UINT8_C(0x1B)
#define AS5600_CONF_MASK      UINT16_C(0x3FFF)
#define AS5600_SETTLE_MS      UINT32_C(1)

typedef struct
{
    const as5600_t *device;
    uint32_t started_ms;
    uint32_t timeout_ms;
    bool active;
    bool locked;
} as5600_transaction_t;

static bool bus_is_valid(const as5600_bus_t *bus)
{
    return (bus->read != NULL) && (bus->write != NULL) &&
           (bus->now_ms != NULL) && (bus->check_context != NULL) &&
           (bus->delay_ms != NULL) &&
           (((bus->lock == NULL) && (bus->unlock == NULL)) ||
            ((bus->lock != NULL) && (bus->unlock != NULL)));
}

static bool config_is_valid(const as5600_config_t *configuration)
{
    return
        ((uint32_t)configuration->power <= (uint32_t)AS5600_POWER_LOW_3) &&
        ((uint32_t)configuration->hysteresis <=
         (uint32_t)AS5600_HYSTERESIS_3_LSB) &&
        ((uint32_t)configuration->output <= (uint32_t)AS5600_OUTPUT_PWM) &&
        ((uint32_t)configuration->pwm_frequency <=
         (uint32_t)AS5600_PWM_920_HZ) &&
        ((uint32_t)configuration->slow_filter <=
         (uint32_t)AS5600_SLOW_FILTER_2X) &&
        ((uint32_t)configuration->fast_filter <=
         (uint32_t)AS5600_FAST_FILTER_10_LSB);
}

static as5600_result_t begin_transaction(
    const as5600_t *device, uint32_t timeout_ms,
    as5600_transaction_t *transaction)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    transaction->device = device;
    transaction->started_ms = 0U;
    transaction->timeout_ms = timeout_ms;
    transaction->active = false;
    transaction->locked = false;

    if ((device != NULL) && (timeout_ms > 0U) &&
        (timeout_ms <= AS5600_TIMEOUT_MAX_MS))
    {
        result = AS5600_ERROR_NOT_INITIALIZED;
        if (device->initialized && bus_is_valid(&device->bus))
        {
            result = device->bus.check_context(device->bus.context);
            if (result == AS5600_OK)
            {
                transaction->started_ms =
                    device->bus.now_ms(device->bus.context);
                transaction->active = true;
                if (device->bus.lock != NULL)
                {
                    result = device->bus.lock(device->bus.context, timeout_ms);
                    transaction->locked = (result == AS5600_OK);
                }
            }
        }
    }

    return result;
}

static as5600_result_t remaining_time(
    const as5600_transaction_t *transaction, uint32_t *remaining_ms)
{
    const as5600_bus_t *bus = &transaction->device->bus;
    const uint32_t now = bus->now_ms(bus->context);
    /* Modulo subtraction deliberately supports one uint32_t clock wrap. */
    const uint32_t elapsed = now - transaction->started_ms;
    as5600_result_t result = AS5600_ERROR_TIMEOUT;

    if (elapsed < transaction->timeout_ms)
    {
        *remaining_ms = transaction->timeout_ms - elapsed;
        result = AS5600_OK;
    }

    return result;
}

static as5600_result_t end_transaction(
    const as5600_transaction_t *transaction, as5600_result_t operation_result)
{
    as5600_result_t result = operation_result;
    uint32_t remaining = 0U;

    if (transaction->locked)
    {
        const as5600_bus_t *bus = &transaction->device->bus;
        const as5600_result_t unlock_result = bus->unlock(bus->context);

        if (unlock_result != AS5600_OK)
        {
            /* Loss of bus ownership takes precedence over a transfer error. */
            result = AS5600_ERROR_UNLOCK;
        }
    }
    if ((result == AS5600_OK) && transaction->active)
    {
        result = remaining_time(transaction, &remaining);
    }

    return result;
}

static as5600_result_t read_bytes(
    const as5600_transaction_t *transaction, uint8_t reg, uint8_t *data,
    uint16_t length)
{
    uint32_t remaining = 0U;
    as5600_result_t result = remaining_time(transaction, &remaining);

    if (result == AS5600_OK)
    {
        const as5600_bus_t *bus = &transaction->device->bus;

        result = bus->read(bus->context, AS5600_I2C_ADDRESS, reg, data,
                           length, remaining);
        if (result == AS5600_OK)
        {
            result = remaining_time(transaction, &remaining);
        }
    }

    return result;
}

static as5600_result_t read_word(
    const as5600_transaction_t *transaction, uint8_t reg, uint16_t *word)
{
    uint8_t bytes[2] = {0U, 0U};
    const as5600_result_t result =
        read_bytes(transaction, reg, bytes, UINT16_C(2));

    if (result == AS5600_OK)
    {
        *word = (uint16_t)(((uint32_t)bytes[0] << 8U) | (uint32_t)bytes[1]);
    }

    return result;
}

static as5600_result_t read_angle_word(
    const as5600_transaction_t *transaction, uint8_t reg, uint16_t *word)
{
    uint16_t raw = 0U;
    const as5600_result_t result = read_word(transaction, reg, &raw);

    if (result == AS5600_OK)
    {
        *word = (uint16_t)((uint32_t)raw & (uint32_t)AS5600_ANGLE_MAX);
    }

    return result;
}

static as5600_result_t write_word(
    const as5600_transaction_t *transaction, uint8_t reg, uint16_t word)
{
    uint8_t bytes[2] = {0U, 0U};
    uint32_t remaining = 0U;
    as5600_result_t result = remaining_time(transaction, &remaining);

    bytes[0] = (uint8_t)((uint32_t)word >> 8U);
    bytes[1] = (uint8_t)((uint32_t)word & UINT32_C(0xFF));
    if (result == AS5600_OK)
    {
        const as5600_bus_t *bus = &transaction->device->bus;

        result = bus->write(bus->context, AS5600_I2C_ADDRESS, reg, bytes,
                            UINT16_C(2), remaining);
        if (result == AS5600_OK)
        {
            result = remaining_time(transaction, &remaining);
        }
    }

    return result;
}

static as5600_result_t settle_configuration(
    const as5600_transaction_t *transaction)
{
    uint32_t remaining = 0U;
    as5600_result_t result = remaining_time(transaction, &remaining);

    if (result == AS5600_OK)
    {
        result = AS5600_ERROR_TIMEOUT;
        if (remaining > AS5600_SETTLE_MS)
        {
            const as5600_bus_t *bus = &transaction->device->bus;

            result = bus->delay_ms(bus->context, AS5600_SETTLE_MS, remaining);
            if (result == AS5600_OK)
            {
                result = remaining_time(transaction, &remaining);
            }
        }
    }

    return result;
}

static as5600_result_t update_word(
    const as5600_transaction_t *transaction, uint8_t reg, uint16_t mask,
    uint16_t value)
{
    uint16_t old_word = 0U;
    uint16_t readback = 0U;
    as5600_result_t result = read_word(transaction, reg, &old_word);

    if (result == AS5600_OK)
    {
        const uint16_t new_word = (uint16_t)(
            ((uint32_t)old_word & ~(uint32_t)mask) | (uint32_t)value);

        if (new_word != old_word)
        {
            result = write_word(transaction, reg, new_word);
            if (result == AS5600_OK)
            {
                result = settle_configuration(transaction);
            }
            if (result == AS5600_OK)
            {
                result = read_word(transaction, reg, &readback);
            }
            if ((result == AS5600_OK) && (readback != new_word))
            {
                result = AS5600_ERROR_VERIFY;
            }
        }
    }

    return result;
}

static uint16_t encode_config(const as5600_config_t *configuration)
{
    uint32_t value =
        (uint32_t)configuration->power |
        ((uint32_t)configuration->hysteresis << 2U) |
        ((uint32_t)configuration->output << 4U) |
        ((uint32_t)configuration->pwm_frequency << 6U) |
        ((uint32_t)configuration->slow_filter << 8U) |
        ((uint32_t)configuration->fast_filter << 10U);

    if (configuration->watchdog)
    {
        value |= UINT32_C(0x2000);
    }

    return (uint16_t)value;
}

static as5600_result_t decode_config(
    uint16_t word, as5600_config_t *configuration)
{
    static const as5600_power_t powers[4] = {
        AS5600_POWER_NORMAL, AS5600_POWER_LOW_1,
        AS5600_POWER_LOW_2, AS5600_POWER_LOW_3
    };
    static const as5600_hysteresis_t hysteresis[4] = {
        AS5600_HYSTERESIS_OFF, AS5600_HYSTERESIS_1_LSB,
        AS5600_HYSTERESIS_2_LSB, AS5600_HYSTERESIS_3_LSB
    };
    static const as5600_output_t outputs[3] = {
        AS5600_OUTPUT_ANALOG_FULL, AS5600_OUTPUT_ANALOG_REDUCED,
        AS5600_OUTPUT_PWM
    };
    static const as5600_pwm_t frequencies[4] = {
        AS5600_PWM_115_HZ, AS5600_PWM_230_HZ,
        AS5600_PWM_460_HZ, AS5600_PWM_920_HZ
    };
    static const as5600_slow_filter_t slow_filters[4] = {
        AS5600_SLOW_FILTER_16X, AS5600_SLOW_FILTER_8X,
        AS5600_SLOW_FILTER_4X, AS5600_SLOW_FILTER_2X
    };
    static const as5600_fast_filter_t fast_filters[8] = {
        AS5600_FAST_FILTER_OFF, AS5600_FAST_FILTER_6_LSB,
        AS5600_FAST_FILTER_7_LSB, AS5600_FAST_FILTER_9_LSB,
        AS5600_FAST_FILTER_18_LSB, AS5600_FAST_FILTER_21_LSB,
        AS5600_FAST_FILTER_24_LSB, AS5600_FAST_FILTER_10_LSB
    };
    const uint32_t value = (uint32_t)word;
    const uint32_t output = (value >> 4U) & UINT32_C(3);
    as5600_result_t result = AS5600_ERROR_DATA;

    if (output < UINT32_C(3))
    {
        configuration->power = powers[value & UINT32_C(3)];
        configuration->hysteresis = hysteresis[(value >> 2U) & UINT32_C(3)];
        configuration->output = outputs[output];
        configuration->pwm_frequency = frequencies[(value >> 6U) & UINT32_C(3)];
        configuration->slow_filter = slow_filters[(value >> 8U) & UINT32_C(3)];
        configuration->fast_filter = fast_filters[(value >> 10U) & UINT32_C(7)];
        configuration->watchdog = (value & UINT32_C(0x2000)) != 0U;
        result = AS5600_OK;
    }

    return result;
}

static as5600_result_t read_config_locked(
    const as5600_transaction_t *transaction, as5600_config_t *configuration)
{
    uint16_t word = 0U;
    as5600_result_t result = read_word(transaction, AS5600_REG_CONF, &word);

    if (result == AS5600_OK)
    {
        result = decode_config(word, configuration);
    }

    return result;
}

static as5600_result_t read_measurement(
    const as5600_t *device, uint8_t reg, uint16_t *measurement,
    uint32_t timeout_ms)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (measurement != NULL)
    {
        as5600_transaction_t transaction = {0};
        uint16_t value = 0U;

        result = begin_transaction(device, timeout_ms, &transaction);
        if (result == AS5600_OK)
        {
            result = read_angle_word(&transaction, reg, &value);
        }
        result = end_transaction(&transaction, result);
        if (result == AS5600_OK)
        {
            *measurement = value;
        }
    }

    return result;
}

as5600_result_t as5600_init(as5600_t *device, const as5600_bus_t *bus)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if ((device != NULL) && (bus != NULL))
    {
        if (bus_is_valid(bus))
        {
            device->bus = *bus;
            device->initialized = true;
            result = AS5600_OK;
        }
    }

    return result;
}

as5600_result_t as5600_default_config(as5600_config_t *configuration)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (configuration != NULL)
    {
        configuration->power = AS5600_POWER_NORMAL;
        configuration->hysteresis = AS5600_HYSTERESIS_OFF;
        configuration->output = AS5600_OUTPUT_ANALOG_FULL;
        configuration->pwm_frequency = AS5600_PWM_115_HZ;
        configuration->slow_filter = AS5600_SLOW_FILTER_16X;
        configuration->fast_filter = AS5600_FAST_FILTER_OFF;
        configuration->watchdog = false;
        result = AS5600_OK;
    }

    return result;
}

as5600_result_t as5600_read_raw_angle(
    const as5600_t *device, uint16_t *angle, uint32_t timeout_ms)
{
    return read_measurement(device, AS5600_REG_RAW_ANGLE, angle, timeout_ms);
}

as5600_result_t as5600_read_angle(
    const as5600_t *device, uint16_t *angle, uint32_t timeout_ms)
{
    return read_measurement(device, AS5600_REG_ANGLE, angle, timeout_ms);
}

as5600_result_t as5600_read_diagnostics(
    const as5600_t *device, as5600_diagnostics_t *diagnostics,
    uint32_t timeout_ms)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (diagnostics != NULL)
    {
        as5600_transaction_t transaction = {0};
        as5600_diagnostics_t value = {0};

        result = begin_transaction(device, timeout_ms, &transaction);
        if (result == AS5600_OK)
        {
            result = read_bytes(&transaction, AS5600_REG_STATUS,
                                &value.status, UINT16_C(1));
        }
        if (result == AS5600_OK)
        {
            result = read_bytes(&transaction, AS5600_REG_AGC,
                                &value.agc, UINT16_C(1));
        }
        if (result == AS5600_OK)
        {
            result = read_angle_word(&transaction, AS5600_REG_MAGNITUDE,
                                     &value.magnitude);
        }
        result = end_transaction(&transaction, result);
        if (result == AS5600_OK)
        {
            *diagnostics = value;
        }
    }

    return result;
}

as5600_result_t as5600_read_sample(
    const as5600_t *device, as5600_sample_t *sample, uint32_t timeout_ms)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (sample != NULL)
    {
        as5600_transaction_t transaction = {0};
        as5600_sample_t value = {0};

        result = begin_transaction(device, timeout_ms, &transaction);
        if (result == AS5600_OK)
        {
            result = read_bytes(&transaction, AS5600_REG_STATUS,
                                &value.diagnostics.status, UINT16_C(1));
        }
        if (result == AS5600_OK)
        {
            result = as5600_magnet_status(value.diagnostics.status);
        }
        if (result == AS5600_OK)
        {
            result = read_angle_word(&transaction, AS5600_REG_RAW_ANGLE,
                                     &value.raw_angle);
        }
        if (result == AS5600_OK)
        {
            result = read_angle_word(&transaction, AS5600_REG_ANGLE,
                                     &value.angle);
        }
        if (result == AS5600_OK)
        {
            result = read_bytes(&transaction, AS5600_REG_AGC,
                                &value.diagnostics.agc, UINT16_C(1));
        }
        if (result == AS5600_OK)
        {
            result = read_angle_word(&transaction, AS5600_REG_MAGNITUDE,
                                     &value.diagnostics.magnitude);
        }
        if (result == AS5600_OK)
        {
            result = read_bytes(&transaction, AS5600_REG_STATUS,
                                &value.diagnostics.status, UINT16_C(1));
        }
        if (result == AS5600_OK)
        {
            result = as5600_magnet_status(value.diagnostics.status);
        }
        result = end_transaction(&transaction, result);
        if (result == AS5600_OK)
        {
            *sample = value;
        }
    }

    return result;
}

as5600_result_t as5600_read_config(
    const as5600_t *device, as5600_config_t *configuration,
    uint32_t timeout_ms)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (configuration != NULL)
    {
        as5600_transaction_t transaction = {0};
        as5600_config_t value = {0};

        result = begin_transaction(device, timeout_ms, &transaction);
        if (result == AS5600_OK)
        {
            result = read_config_locked(&transaction, &value);
        }
        result = end_transaction(&transaction, result);
        if (result == AS5600_OK)
        {
            *configuration = value;
        }
    }

    return result;
}

as5600_result_t as5600_read_settings(
    const as5600_t *device, as5600_settings_t *settings, uint32_t timeout_ms)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (settings != NULL)
    {
        as5600_transaction_t transaction = {0};
        as5600_settings_t value = {0};

        result = begin_transaction(device, timeout_ms, &transaction);
        if (result == AS5600_OK)
        {
            result = read_angle_word(&transaction, AS5600_REG_ZPOS,
                                     &value.zero_position);
        }
        if (result == AS5600_OK)
        {
            result = read_angle_word(&transaction, AS5600_REG_MPOS,
                                     &value.stop_position);
        }
        if (result == AS5600_OK)
        {
            result = read_angle_word(&transaction, AS5600_REG_MANG,
                                     &value.max_angle);
        }
        if (result == AS5600_OK)
        {
            result = read_config_locked(&transaction, &value.configuration);
        }
        if (result == AS5600_OK)
        {
            result = read_bytes(&transaction, AS5600_REG_ZMCO,
                                &value.burn_count, UINT16_C(1));
        }
        value.burn_count = (uint8_t)((uint32_t)value.burn_count & UINT32_C(3));
        result = end_transaction(&transaction, result);
        if (result == AS5600_OK)
        {
            *settings = value;
        }
    }

    return result;
}

as5600_result_t as5600_write_config(
    const as5600_t *device, const as5600_config_t *configuration,
    uint32_t timeout_ms)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if (configuration != NULL)
    {
        if (config_is_valid(configuration))
        {
            as5600_transaction_t transaction = {0};
            const uint16_t word = encode_config(configuration);

            result = begin_transaction(device, timeout_ms, &transaction);
            if (result == AS5600_OK)
            {
                result = update_word(&transaction, AS5600_REG_CONF,
                                     AS5600_CONF_MASK, word);
            }
            result = end_transaction(&transaction, result);
        }
    }

    return result;
}

as5600_result_t as5600_write_positions(
    const as5600_t *device, uint16_t zero_position, uint16_t stop_position,
    uint32_t timeout_ms)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if ((zero_position <= AS5600_ANGLE_MAX) &&
        (stop_position <= AS5600_ANGLE_MAX))
    {
        const uint32_t span =
            ((uint32_t)stop_position + UINT32_C(4096) -
             (uint32_t)zero_position) & UINT32_C(0xFFF);

        if ((span == 0U) || (span >= (uint32_t)AS5600_MIN_RANGE_COUNTS))
        {
            as5600_transaction_t transaction = {0};

            result = begin_transaction(device, timeout_ms, &transaction);
            if (result == AS5600_OK)
            {
                result = update_word(&transaction, AS5600_REG_ZPOS,
                                     AS5600_ANGLE_MAX, zero_position);
            }
            if (result == AS5600_OK)
            {
                result = update_word(&transaction, AS5600_REG_MPOS,
                                     AS5600_ANGLE_MAX, stop_position);
            }
            result = end_transaction(&transaction, result);
        }
    }

    return result;
}

as5600_result_t as5600_write_max_angle(
    const as5600_t *device, uint16_t max_angle, uint32_t timeout_ms)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if ((max_angle == 0U) ||
        ((max_angle >= AS5600_MIN_RANGE_COUNTS) &&
         (max_angle <= AS5600_ANGLE_MAX)))
    {
        as5600_transaction_t transaction = {0};

        result = begin_transaction(device, timeout_ms, &transaction);
        if (result == AS5600_OK)
        {
            result = update_word(&transaction, AS5600_REG_MANG,
                                 AS5600_ANGLE_MAX, max_angle);
        }
        result = end_transaction(&transaction, result);
    }

    return result;
}

as5600_result_t as5600_magnet_status(uint8_t status)
{
    const bool weak = (status & AS5600_STATUS_MAGNET_LOW) != 0U;
    const bool strong = (status & AS5600_STATUS_MAGNET_HIGH) != 0U;
    as5600_result_t result = AS5600_OK;

    if (weak && strong)
    {
        result = AS5600_ERROR_DATA;
    }
    else if ((status & AS5600_STATUS_MAGNET_FOUND) == 0U)
    {
        result = AS5600_ERROR_NO_MAGNET;
    }
    else if (weak)
    {
        result = AS5600_ERROR_MAGNET_WEAK;
    }
    else if (strong)
    {
        result = AS5600_ERROR_MAGNET_STRONG;
    }
    else
    {
        /* All diagnostic flags are healthy. */
    }

    return result;
}

as5600_result_t as5600_counts_to_millidegrees(
    uint16_t counts, uint32_t *millidegrees)
{
    as5600_result_t result = AS5600_ERROR_ARGUMENT;

    if ((counts <= AS5600_ANGLE_MAX) && (millidegrees != NULL))
    {
        *millidegrees =
            (((uint32_t)counts * UINT32_C(360000)) + UINT32_C(2048)) >> 12U;
        result = AS5600_OK;
    }

    return result;
}
