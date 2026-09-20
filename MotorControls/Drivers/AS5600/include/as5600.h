#ifndef AS5600_H
#define AS5600_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_I2C_ADDRESS          UINT8_C(0x36)
#define AS5600_ANGLE_MAX            UINT16_C(4095)
#define AS5600_MIN_RANGE_COUNTS     UINT16_C(205)
#define AS5600_POWER_UP_MS          UINT32_C(10)
#define AS5600_TIMEOUT_MAX_MS       UINT32_C(2147483647)
#define AS5600_STATUS_MAGNET_HIGH   UINT8_C(0x08)
#define AS5600_STATUS_MAGNET_LOW    UINT8_C(0x10)
#define AS5600_STATUS_MAGNET_FOUND  UINT8_C(0x20)

typedef enum
{
    AS5600_OK = 0,
    AS5600_ERROR_ARGUMENT,
    AS5600_ERROR_NOT_INITIALIZED,
    AS5600_ERROR_CONTEXT,
    AS5600_ERROR_BUSY,
    AS5600_ERROR_TIMEOUT,
    AS5600_ERROR_NACK,
    AS5600_ERROR_IO,
    AS5600_ERROR_LOCK,
    AS5600_ERROR_UNLOCK,
    AS5600_ERROR_VERIFY,
    AS5600_ERROR_DATA,
    AS5600_ERROR_NO_MAGNET,
    AS5600_ERROR_MAGNET_WEAK,
    AS5600_ERROR_MAGNET_STRONG
} as5600_result_t;

typedef enum
{
    AS5600_POWER_NORMAL = 0,
    AS5600_POWER_LOW_1,
    AS5600_POWER_LOW_2,
    AS5600_POWER_LOW_3
} as5600_power_t;

typedef enum
{
    AS5600_HYSTERESIS_OFF = 0,
    AS5600_HYSTERESIS_1_LSB,
    AS5600_HYSTERESIS_2_LSB,
    AS5600_HYSTERESIS_3_LSB
} as5600_hysteresis_t;

typedef enum
{
    AS5600_OUTPUT_ANALOG_FULL = 0,
    AS5600_OUTPUT_ANALOG_REDUCED,
    AS5600_OUTPUT_PWM
} as5600_output_t;

typedef enum
{
    AS5600_PWM_115_HZ = 0,
    AS5600_PWM_230_HZ,
    AS5600_PWM_460_HZ,
    AS5600_PWM_920_HZ
} as5600_pwm_t;

typedef enum
{
    AS5600_SLOW_FILTER_16X = 0,
    AS5600_SLOW_FILTER_8X,
    AS5600_SLOW_FILTER_4X,
    AS5600_SLOW_FILTER_2X
} as5600_slow_filter_t;

typedef enum
{
    AS5600_FAST_FILTER_OFF = 0,
    AS5600_FAST_FILTER_6_LSB,
    AS5600_FAST_FILTER_7_LSB,
    AS5600_FAST_FILTER_9_LSB,
    AS5600_FAST_FILTER_18_LSB,
    AS5600_FAST_FILTER_21_LSB,
    AS5600_FAST_FILTER_24_LSB,
    AS5600_FAST_FILTER_10_LSB
} as5600_fast_filter_t;

typedef struct
{
    as5600_power_t power;
    as5600_hysteresis_t hysteresis;
    as5600_output_t output;
    as5600_pwm_t pwm_frequency;
    as5600_slow_filter_t slow_filter;
    as5600_fast_filter_t fast_filter;
    bool watchdog;
} as5600_config_t;

typedef struct
{
    uint8_t status;
    uint8_t agc;
    uint16_t magnitude;
} as5600_diagnostics_t;

typedef struct
{
    uint16_t raw_angle;
    uint16_t angle;
    as5600_diagnostics_t diagnostics;
} as5600_sample_t;

typedef struct
{
    uint16_t zero_position;
    uint16_t stop_position;
    uint16_t max_angle;
    uint8_t burn_count;
    as5600_config_t configuration;
} as5600_settings_t;

/* Transfers are synchronous, use a 7-bit address, and contain 1 or 2 bytes. */
typedef as5600_result_t (*as5600_read_fn)(
    void *context, uint8_t address, uint8_t reg, uint8_t *data,
    uint16_t length, uint32_t timeout_ms);
typedef as5600_result_t (*as5600_write_fn)(
    void *context, uint8_t address, uint8_t reg, const uint8_t *data,
    uint16_t length, uint32_t timeout_ms);
typedef uint32_t (*as5600_now_fn)(void *context);
typedef as5600_result_t (*as5600_context_fn)(void *context);
typedef as5600_result_t (*as5600_lock_fn)(void *context, uint32_t timeout_ms);
typedef as5600_result_t (*as5600_unlock_fn)(void *context);
typedef as5600_result_t (*as5600_delay_fn)(
    void *context, uint32_t minimum_ms, uint32_t timeout_ms);

typedef struct
{
    void *context;
    as5600_read_fn read;
    as5600_write_fn write;
    as5600_now_fn now_ms;
    as5600_context_fn check_context;
    as5600_delay_fn delay_ms;
    as5600_lock_fn lock;
    as5600_unlock_fn unlock;
} as5600_bus_t;

/* A synchronization adapter is shared by every client of one physical bus. */
typedef struct
{
    void *context;
    as5600_context_fn check_context;
    as5600_lock_fn lock;
    as5600_unlock_fn unlock;
    as5600_delay_fn delay_ms;
} as5600_sync_t;

/* Zero-initialize before binding; do not modify or rebind while in use. */
typedef struct
{
    as5600_bus_t bus;
    bool initialized;
} as5600_t;

/* Binding does not access the device or change its power-on configuration. */
as5600_result_t as5600_init(as5600_t *device, const as5600_bus_t *bus);
as5600_result_t as5600_default_config(as5600_config_t *configuration);

/*
 * Hardware calls require 1..AS5600_TIMEOUT_MAX_MS and task/main context.
 * The budget covers mutex acquisition, transfers, settling and release.
 * Read outputs change only on AS5600_OK, including successful release.
 */
as5600_result_t as5600_read_raw_angle(
    const as5600_t *device, uint16_t *angle, uint32_t timeout_ms);
as5600_result_t as5600_read_angle(
    const as5600_t *device, uint16_t *angle, uint32_t timeout_ms);
as5600_result_t as5600_read_diagnostics(
    const as5600_t *device, as5600_diagnostics_t *diagnostics,
    uint32_t timeout_ms);
as5600_result_t as5600_read_sample(
    const as5600_t *device, as5600_sample_t *sample, uint32_t timeout_ms);
as5600_result_t as5600_read_config(
    const as5600_t *device, as5600_config_t *configuration,
    uint32_t timeout_ms);
as5600_result_t as5600_read_settings(
    const as5600_t *device, as5600_settings_t *settings, uint32_t timeout_ms);

/* Volatile writes only. Failures can leave partially changed hardware. */
as5600_result_t as5600_write_config(
    const as5600_t *device, const as5600_config_t *configuration,
    uint32_t timeout_ms);
as5600_result_t as5600_write_positions(
    const as5600_t *device, uint16_t zero_position, uint16_t stop_position,
    uint32_t timeout_ms);
as5600_result_t as5600_write_max_angle(
    const as5600_t *device, uint16_t max_angle, uint32_t timeout_ms);

as5600_result_t as5600_magnet_status(uint8_t status);
as5600_result_t as5600_counts_to_millidegrees(
    uint16_t counts, uint32_t *millidegrees);

#ifdef __cplusplus
}
#endif

#endif
