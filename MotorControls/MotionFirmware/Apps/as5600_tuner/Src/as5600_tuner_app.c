/**
 * @file    as5600_tuner_app.c
 * @brief   Dedicated passive AS5600 encoder tuning, airgap alignment, and
 *          filter/EMI characterization firmware profile (fw_app.h).
 *
 * Safety invariants:
 *  - Boots with all TMC2240 ENN pins HIGH (motors unpowered) and zero STEP edges.
 *  - Never writes AS5600 OTP register 0xFF (BURN_ANGLE / BURN_SETTING) or range registers.
 *  - Only modifies volatile CONF (0x07/0x08) via as5600_write_config().
 *  - Native USB-C CDC exclusively (USART2 untouched); NDJSON output compatible
 *    with MotorControls/matlab/scripts/analyze_as5600_noise.m.
 *
 * Commands (newline-terminated over USB-C CDC):
 *   HELP
 *   STATUS
 *   CONF <x|z> <sf:2|4|8|16> [hyst:0..3] [fth:0..7]
 *   CAPTURE <x|z> [rate_hz:50..1000] [n:64..4096]
 *   STREAM <x|z> [rate_hz:10..500]
 *   STOP
 */
#include "as5600.h"
#include "as5600_stm32_hal.h"
#if defined(ENABLE_SCRUTINY)
#include "as5600_scrutiny_bridge.h"
#endif
#include "cmd_parse.h"
#include "encoder_unwrap.h"
#include "fw_app.h"
#include "main.h"
#include "usb_cdc_port.h"

#include <limits.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TUNER_PROFILE_NAME     "as5600_tuner"
#define TUNER_SCHEMA_VERSION   1U
#define TUNER_I2C_TIMEOUT_MS   3U
#define TUNER_TX_LINE_MAX      320U
#define TUNER_I2C_FAST_TIMINGR 0x30A31A23U /* Proposed 400 kHz candidate; requires verified 100 MHz I2C kernel */

extern I2C_HandleTypeDef hi2c1; /* Bus 1: X-axis (PB8/PB9 J73) */
extern I2C_HandleTypeDef hi2c4; /* Bus 0: Z-axis (PB6/PB7 J43) */

typedef enum {
    TUNER_MODE_IDLE = 0U,
    TUNER_MODE_CAPTURE,
    TUNER_MODE_STREAM
} tuner_mode_t;

typedef struct {
    as5600_t           dev;
    as5600_stm32_hal_t port;
    bool               bound;
    uint8_t            sf_code;   /* 0=16x, 1=8x, 2=4x, 3=2x */
    uint8_t            hyst_code; /* 0..3 */
    uint8_t            fth_code;  /* 0..7 */
    uint16_t           conf_rb;
} tuner_enc_t;

static tuner_enc_t  g_enc[2]; /* 0 = Z (I2C4), 1 = X (I2C1) */
static cmd_linebuf_t g_rx;
static tuner_mode_t g_mode = TUNER_MODE_IDLE;
static uint8_t      g_bus = 1U;
static uint32_t     g_seq = 0U;
static uint32_t     g_trial_id = 0U;
static uint32_t     g_rate_hz = 500U;
static uint32_t     g_period_us = 2000U;
static uint32_t     g_next_us = 0U;
static uint32_t     g_target_n = 2048U;
static uint32_t     g_idx = 0U;
static uint32_t     g_ok_n = 0U;
static uint32_t     g_err_n = 0U;
static uint32_t     g_overruns = 0U;
static uint32_t     g_tx_drops = 0U;
static enc_unwrap_t g_uw;
static int32_t      g_min_c = 0;
static int32_t      g_max_c = 0;
static int64_t      g_sum_c = 0;
static uint8_t      g_last_status = 0U;
static uint8_t      g_last_agc = 0U;
static uint16_t     g_last_mag = 0U;
static uint32_t     g_last_led_ms = 0U;
#if defined(ENABLE_SCRUTINY)
static enc_unwrap_t g_scrutiny_uw[2];
static uint32_t     g_scrutiny_next_us = 0U;
static uint32_t     g_scrutiny_diag_div = 0U;
#endif

static uint32_t now_us(void) { return TIM2->CNT; }

static const char *bus_name(uint8_t bus) { return (bus == 0U) ? "z" : "x"; }

static I2C_HandleTypeDef *bus_hi2c(uint8_t bus) {
    return (bus == 0U) ? &hi2c4 : &hi2c1;
}

static uint8_t sf_code_to_factor(uint8_t code) {
    switch (code & 3U) {
        case 0U: return 16U;
        case 1U: return 8U;
        case 2U: return 4U;
        default: return 2U;
    }
}

static bool parse_bus(const char *s, uint8_t *bus_out) {
    if (s == NULL || bus_out == NULL) return false;
    if (strcmp(s, "z") == 0 || strcmp(s, "Z") == 0 || strcmp(s, "0") == 0) {
        *bus_out = 0U;
        return true;
    }
    if (strcmp(s, "x") == 0 || strcmp(s, "X") == 0 || strcmp(s, "1") == 0) {
        *bus_out = 1U;
        return true;
    }
    return false;
}

static bool cdc_write_bytes(const char *buf, uint32_t len) {
    uint32_t off = 0U;
    const uint32_t t0 = HAL_GetTick();
    while (off < len && usb_cdc_port_is_mounted() && (HAL_GetTick() - t0) < 20U) {
        usb_cdc_port_poll();
        off += usb_cdc_port_write((const uint8_t *)(buf + off), len - off);
    }
    if (off < len) {
        g_tx_drops++;
        return false;
    }
    return true;
}

static bool cdc_write_line(const char *fmt, ...) {
    char buf[TUNER_TX_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof(buf) - 1U, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= sizeof(buf) - 1U) {
        static const char kTxTruncated[] = "{\"kind\":\"error\",\"reason\":\"tx_truncated\"}\n";
        (void)cdc_write_bytes(kTxTruncated, sizeof(kTxTruncated) - 1U);
        return false;
    }
    buf[n] = '\n';
    return cdc_write_bytes(buf, (uint32_t)n + 1U);
}

static void disable_all_drivers(void) {
    HAL_GPIO_WritePin(TMC_ENN0_GPIO_Port, TMC_ENN0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_ENN1_GPIO_Port, TMC_ENN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_ENN2_GPIO_Port, TMC_ENN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_CS0_GPIO_Port, TMC_CS0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_CS1_GPIO_Port, TMC_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_CS2_GPIO_Port, TMC_CS2_Pin, GPIO_PIN_SET);
}

static bool enc_bind(uint8_t bus) {
    if (bus > 1U) return false;
    if (!g_enc[bus].bound) {
        memset(&g_enc[bus], 0, sizeof(g_enc[bus]));
        g_enc[bus].sf_code = 3U; /* 2x */
        if (as5600_stm32_hal_init(&g_enc[bus].dev, &g_enc[bus].port,
                                  bus_hi2c(bus), NULL) != AS5600_OK) {
            return false;
        }
        g_enc[bus].bound = true;
    }
    return true;
}

static bool enc_apply_conf(uint8_t bus, uint8_t sf_code, uint8_t hyst_code,
                           uint8_t fth_code, uint16_t *conf_rb) {
    if (!enc_bind(bus) || sf_code > 3U || hyst_code > 3U || fth_code > 7U) {
        return false;
    }
    as5600_config_t cfg;
    if (as5600_default_config(&cfg) != AS5600_OK) return false;
    cfg.power = AS5600_POWER_NORMAL;
    cfg.hysteresis = (as5600_hysteresis_t)hyst_code;
    cfg.output = AS5600_OUTPUT_ANALOG_FULL;
    cfg.pwm_frequency = AS5600_PWM_115_HZ;
    cfg.slow_filter = (as5600_slow_filter_t)sf_code;
    cfg.fast_filter = (as5600_fast_filter_t)fth_code;
    cfg.watchdog = false;
    if (as5600_write_config(&g_enc[bus].dev, &cfg, 5U) != AS5600_OK) {
        return false;
    }
    HAL_Delay(AS5600_POWER_UP_MS);
    uint8_t raw_bytes[2] = {0U, 0U};
    if (HAL_I2C_Mem_Read(bus_hi2c(bus), (uint16_t)(AS5600_I2C_ADDRESS << 1U),
                         0x07U, I2C_MEMADD_SIZE_8BIT, raw_bytes, 2U, 5U) != HAL_OK) {
        return false;
    }
    const uint16_t rb = (uint16_t)(((uint16_t)raw_bytes[0] << 8U) | raw_bytes[1]);
    g_enc[bus].sf_code = sf_code;
    g_enc[bus].hyst_code = hyst_code;
    g_enc[bus].fth_code = fth_code;
    g_enc[bus].conf_rb = rb;
    if (conf_rb != NULL) *conf_rb = rb;
    return true;
}

#if defined(ENABLE_SCRUTINY)
bool as5600_tuner_apply_conf_from_scrutiny(uint8_t bus, uint8_t sf_code,
                                           uint8_t hyst_code, uint8_t fth_code,
                                           uint16_t *conf_rb) {
    if (g_mode != TUNER_MODE_IDLE) {
        return false;
    }
    return enc_apply_conf(bus, sf_code, hyst_code, fth_code, conf_rb);
}

void as5600_tuner_get_conf_for_scrutiny(uint8_t bus, uint8_t *sf_code,
                                        uint8_t *hyst_code, uint8_t *fth_code,
                                        uint16_t *conf_rb) {
    if (bus > 1U) {
        return;
    }
    if (sf_code != NULL) *sf_code = g_enc[bus].sf_code;
    if (hyst_code != NULL) *hyst_code = g_enc[bus].hyst_code;
    if (fth_code != NULL) *fth_code = g_enc[bus].fth_code;
    if (conf_rb != NULL) *conf_rb = g_enc[bus].conf_rb;
}

static void poll_scrutiny_axis(uint8_t bus, bool read_diag) {
    as5600_scrutiny_axis_t *dst = (bus == 1U) ? &g_scrutiny_as5600.x : &g_scrutiny_as5600.z;
    if (!enc_bind(bus)) {
        dst->ok = 0U;
        dst->err_count++;
        return;
    }
    uint8_t burst[4] = {0U, 0U, 0U, 0U};
    const HAL_StatusTypeDef st = HAL_I2C_Mem_Read(bus_hi2c(bus), (uint16_t)(AS5600_I2C_ADDRESS << 1U),
                                                  0x0CU, I2C_MEMADD_SIZE_8BIT, burst, sizeof(burst),
                                                  TUNER_I2C_TIMEOUT_MS);
    const uint16_t raw = (uint16_t)(((uint16_t)burst[0] << 8U) | burst[1]);
    const uint16_t angle = (uint16_t)(((uint16_t)burst[2] << 8U) | burst[3]);
    if (st == HAL_OK && raw <= 4095U) {
        if (!g_scrutiny_uw[bus].initialized) {
            enc_unwrap_init(&g_scrutiny_uw[bus], raw);
        } else {
            (void)enc_unwrap_update(&g_scrutiny_uw[bus], raw);
        }
        dst->raw = raw;
        dst->angle = (angle <= 4095U) ? angle : raw;
        dst->unwrapped = g_scrutiny_uw[bus].total_counts;
        dst->deg = ((float)dst->unwrapped) * (360.0f / 4096.0f);
        dst->conf = g_enc[bus].conf_rb;
        dst->ok = 1U;
    } else {
        dst->ok = 0U;
        dst->err_count++;
    }
    if (read_diag) {
        as5600_diagnostics_t d;
        if (as5600_read_diagnostics(&g_enc[bus].dev, &d, TUNER_I2C_TIMEOUT_MS) == AS5600_OK) {
            dst->status = d.status;
            dst->agc = d.agc;
            dst->mag = d.magnitude;
        }
    }
}

static void poll_scrutiny_telemetry(void) {
    if (g_mode != TUNER_MODE_IDLE) {
        /* Avoid extra I2C bus contention during USB-CDC CAPTURE/STREAM runs. */
        return;
    }
    const uint32_t t_now = now_us();
    if ((int32_t)(t_now - g_scrutiny_next_us) < 0) {
        return;
    }
    if ((uint32_t)(t_now - g_scrutiny_next_us) > 2000U) {
        g_scrutiny_next_us = t_now + 2000U;
    } else {
        g_scrutiny_next_us += 2000U;
    }
    const bool read_diag = ((g_scrutiny_diag_div++ % 25U) == 0U); /* 20 Hz status/AGC/MAG */
    poll_scrutiny_axis(1U, read_diag); /* X (I2C1) */
    poll_scrutiny_axis(0U, read_diag); /* Z (I2C4) */
    as5600_scrutiny_tick_500hz();
}
#endif

static void emit_status_bus(uint8_t bus) {
    if (!enc_bind(bus)) {
        cdc_write_line("{\"schema_version\":%u,\"seq\":%lu,\"kind\":\"status\",\"axis\":\"%s\",\"ok\":false}",
                       (unsigned)TUNER_SCHEMA_VERSION, (unsigned long)(++g_seq), bus_name(bus));
        return;
    }
    as5600_diagnostics_t d = {0};
    uint16_t raw = 0U;
    uint8_t conf_bytes[2] = {0U, 0U};
    const bool d_ok = (as5600_read_diagnostics(&g_enc[bus].dev, &d, TUNER_I2C_TIMEOUT_MS) == AS5600_OK);
    const bool r_ok = (as5600_read_raw_angle(&g_enc[bus].dev, &raw, TUNER_I2C_TIMEOUT_MS) == AS5600_OK);
    const bool c_ok = (HAL_I2C_Mem_Read(bus_hi2c(bus), (uint16_t)(AS5600_I2C_ADDRESS << 1U),
                                        0x07U, I2C_MEMADD_SIZE_8BIT, conf_bytes, 2U, 5U) == HAL_OK);
    const uint16_t conf = (uint16_t)(((uint16_t)conf_bytes[0] << 8U) | conf_bytes[1]);
    const uint8_t sf_code = (uint8_t)((conf >> 8U) & 3U);
    cdc_write_line(
        "{\"schema_version\":%u,\"seq\":%lu,\"kind\":\"status\",\"profile\":\"%s\","
        "\"axis\":\"%s\",\"ok\":%s,\"t_us\":%lu,\"raw\":%u,\"conf\":%u,\"sf\":%u,"
        "\"status\":%u,\"md\":%s,\"ml\":%s,\"mh\":%s,\"agc\":%u,\"mag\":%u}",
        (unsigned)TUNER_SCHEMA_VERSION, (unsigned long)(++g_seq), TUNER_PROFILE_NAME,
        bus_name(bus), (d_ok && r_ok && c_ok) ? "true" : "false",
        (unsigned long)now_us(), (unsigned)raw, (unsigned)conf,
        (unsigned)sf_code_to_factor(sf_code),
        (unsigned)d.status,
        ((d.status & AS5600_STATUS_MAGNET_FOUND) != 0U) ? "true" : "false",
        ((d.status & AS5600_STATUS_MAGNET_LOW) != 0U) ? "true" : "false",
        ((d.status & AS5600_STATUS_MAGNET_HIGH) != 0U) ? "true" : "false",
        (unsigned)d.agc, (unsigned)d.magnitude);
}

static void start_capture_or_stream(uint8_t bus, uint32_t rate_hz, uint32_t target_n,
                                    bool is_stream) {
    if (!enc_bind(bus)) {
        cdc_write_line("{\"kind\":\"error\",\"reason\":\"bus_init_failed\"}");
        return;
    }
    as5600_diagnostics_t d;
    if (as5600_read_diagnostics(&g_enc[bus].dev, &d, TUNER_I2C_TIMEOUT_MS) != AS5600_OK) {
        cdc_write_line("{\"kind\":\"error\",\"axis\":\"%s\",\"reason\":\"i2c_read_failed\"}",
                       bus_name(bus));
        return;
    }
    g_bus = bus;
    g_rate_hz = rate_hz;
    g_period_us = 1000000U / rate_hz;
    g_next_us = now_us();
    g_target_n = target_n;
    g_idx = 0U;
    g_ok_n = 0U;
    g_err_n = 0U;
    g_overruns = 0U;
    g_tx_drops = 0U;
    memset(&g_uw, 0, sizeof(g_uw));
    g_min_c = INT32_MAX;
    g_max_c = INT32_MIN;
    g_sum_c = 0;
    g_last_status = d.status;
    g_last_agc = d.agc;
    g_last_mag = d.magnitude;
    g_trial_id++;
    g_mode = is_stream ? TUNER_MODE_STREAM : TUNER_MODE_CAPTURE;
    cdc_write_line("{\"schema_version\":%u,\"seq\":%lu,\"trial_id\":%lu,\"kind\":\"ack\","
                   "\"axis\":\"%s\",\"mode\":\"%s\",\"rate_hz\":%lu,\"n\":%lu,\"sf\":%u}",
                   (unsigned)TUNER_SCHEMA_VERSION, (unsigned long)(++g_seq),
                   (unsigned long)g_trial_id, bus_name(bus),
                   is_stream ? "stream" : "capture",
                   (unsigned long)rate_hz, (unsigned long)target_n,
                   (unsigned)sf_code_to_factor(g_enc[bus].sf_code));
}

static void handle_line(char *line) {
    char *argv[8];
    const int argc = cmd_tokenize(line, argv, 8);
    if (argc <= 0) return;

    if (strcmp(argv[0], "stop") == 0) {
        g_mode = TUNER_MODE_IDLE;
        cdc_write_line("{\"schema_version\":%u,\"seq\":%lu,\"kind\":\"ack\",\"cmd\":\"stop\"}",
                       (unsigned)TUNER_SCHEMA_VERSION, (unsigned long)(++g_seq));
        return;
    }
    if (strcmp(argv[0], "help") == 0 || strcmp(argv[0], "hello") == 0) {
        cdc_write_line(
            "{\"schema_version\":%u,\"seq\":%lu,\"kind\":\"hello\",\"profile\":\"%s\","
            "\"commands\":[\"STATUS\",\"CONF <x|z> <sf:2|4|8|16> [hyst:0..3] [fth:0..7]\","
            "\"CAPTURE <x|z> [rate_hz:50..1000] [n:64..4096]\","
            "\"STREAM <x|z> [rate_hz:10..500]\",\"STOP\"]}",
            (unsigned)TUNER_SCHEMA_VERSION, (unsigned long)(++g_seq), TUNER_PROFILE_NAME);
        return;
    }
    if (strcmp(argv[0], "status") == 0) {
        g_mode = TUNER_MODE_IDLE;
        emit_status_bus(1U); /* X */
        emit_status_bus(0U); /* Z */
        return;
    }
    if (strcmp(argv[0], "conf") == 0) {
        g_mode = TUNER_MODE_IDLE;
        uint8_t bus = 1U;
        uint32_t sf = 2U, hyst = 0U, fth = 0U;
        if (argc < 3 || !parse_bus(argv[1], &bus) || !cmd_parse_u32(argv[2], &sf)) {
            cdc_write_line("{\"kind\":\"error\",\"reason\":\"usage: CONF <x|z> <sf:2|4|8|16> [hyst:0..3] [fth:0..7]\"}");
            return;
        }
        uint8_t sf_code = 3U;
        if (sf == 16U) sf_code = 0U;
        else if (sf == 8U) sf_code = 1U;
        else if (sf == 4U) sf_code = 2U;
        else if (sf == 2U) sf_code = 3U;
        else {
            cdc_write_line("{\"kind\":\"error\",\"reason\":\"invalid_sf\"}");
            return;
        }
        if (argc >= 4 && (!cmd_parse_u32(argv[3], &hyst) || hyst > 3U)) {
            cdc_write_line("{\"kind\":\"error\",\"reason\":\"invalid_hyst\"}");
            return;
        }
        if (argc >= 5 && (!cmd_parse_u32(argv[4], &fth) || fth > 7U)) {
            cdc_write_line("{\"kind\":\"error\",\"reason\":\"invalid_fth\"}");
            return;
        }
        uint16_t rb = 0U;
        const bool ok = enc_apply_conf(bus, sf_code, (uint8_t)hyst, (uint8_t)fth, &rb);
        cdc_write_line("{\"schema_version\":%u,\"seq\":%lu,\"kind\":\"conf\",\"axis\":\"%s\","
                       "\"ok\":%s,\"sf\":%u,\"hyst\":%lu,\"fth\":%lu,\"conf\":%u}",
                       (unsigned)TUNER_SCHEMA_VERSION, (unsigned long)(++g_seq),
                       bus_name(bus), ok ? "true" : "false", (unsigned)sf,
                       (unsigned long)hyst, (unsigned long)fth, (unsigned)rb);
        return;
    }
    if (strcmp(argv[0], "capture") == 0) {
        uint8_t bus = 1U;
        uint32_t rate_hz = 500U, n = 2048U;
        if (argc < 2 || !parse_bus(argv[1], &bus)) {
            cdc_write_line("{\"kind\":\"error\",\"reason\":\"usage: CAPTURE <x|z> [rate_hz:50..1000] [n:64..4096]\"}");
            return;
        }
        if (argc >= 3 && (!cmd_parse_u32(argv[2], &rate_hz) || rate_hz < 50U || rate_hz > 1000U)) {
            cdc_write_line("{\"kind\":\"error\",\"reason\":\"invalid_rate_hz\"}");
            return;
        }
        if (argc >= 4 && (!cmd_parse_u32(argv[3], &n) || n < 64U || n > 4096U)) {
            cdc_write_line("{\"kind\":\"error\",\"reason\":\"invalid_n\"}");
            return;
        }
        start_capture_or_stream(bus, rate_hz, n, false);
        return;
    }
    if (strcmp(argv[0], "stream") == 0) {
        uint8_t bus = 1U;
        uint32_t rate_hz = 50U;
        if (argc < 2 || !parse_bus(argv[1], &bus)) {
            cdc_write_line("{\"kind\":\"error\",\"reason\":\"usage: STREAM <x|z> [rate_hz:10..500]\"}");
            return;
        }
        if (argc >= 3 && (!cmd_parse_u32(argv[2], &rate_hz) || rate_hz < 10U || rate_hz > 500U)) {
            cdc_write_line("{\"kind\":\"error\",\"reason\":\"invalid_rate_hz\"}");
            return;
        }
        start_capture_or_stream(bus, rate_hz, 0U, true);
        return;
    }

    cdc_write_line("{\"kind\":\"error\",\"reason\":\"unknown_command\"}");
}

static void poll_sampling(void) {
    if (g_mode == TUNER_MODE_IDLE) return;
    const uint32_t t_now = now_us();
    if ((int32_t)(t_now - g_next_us) < 0) return;

    if ((uint32_t)(t_now - g_next_us) > g_period_us) {
        g_overruns++;
        g_next_us = t_now + g_period_us;
    } else {
        g_next_us += g_period_us;
    }

    uint16_t raw = 0U;
    const as5600_result_t r = as5600_read_raw_angle(&g_enc[g_bus].dev, &raw, TUNER_I2C_TIMEOUT_MS);
    const uint32_t idx = g_idx++;
    if (r == AS5600_OK && raw <= 4095U) {
        if (!g_uw.initialized) {
            enc_unwrap_init(&g_uw, raw);
        } else {
            (void)enc_unwrap_update(&g_uw, raw);
        }
        const int32_t c = g_uw.total_counts;
        if (c < g_min_c) g_min_c = c;
        if (c > g_max_c) g_max_c = c;
        g_sum_c += c;
        g_ok_n++;
        if (g_mode == TUNER_MODE_STREAM && (idx % 10U) == 0U) {
            as5600_diagnostics_t d;
            if (as5600_read_diagnostics(&g_enc[g_bus].dev, &d, TUNER_I2C_TIMEOUT_MS) == AS5600_OK) {
                g_last_status = d.status;
                g_last_agc = d.agc;
                g_last_mag = d.magnitude;
            }
            cdc_write_line(
                "{\"schema_version\":%u,\"seq\":%lu,\"trial_id\":%lu,\"kind\":\"sample\","
                "\"axis\":\"%s\",\"t_us\":%lu,\"test\":\"enc_noise\",\"idx\":%lu,"
                "\"raw\":%u,\"unwrapped\":%ld,\"status\":%u,\"agc\":%u,\"mag\":%u}",
                (unsigned)TUNER_SCHEMA_VERSION, (unsigned long)(++g_seq),
                (unsigned long)g_trial_id, bus_name(g_bus), (unsigned long)t_now,
                (unsigned long)idx, (unsigned)raw, (long)c,
                (unsigned)g_last_status, (unsigned)g_last_agc, (unsigned)g_last_mag);
        } else {
            cdc_write_line(
                "{\"schema_version\":%u,\"seq\":%lu,\"trial_id\":%lu,\"kind\":\"sample\","
                "\"axis\":\"%s\",\"t_us\":%lu,\"test\":\"enc_noise\",\"idx\":%lu,"
                "\"raw\":%u,\"unwrapped\":%ld}",
                (unsigned)TUNER_SCHEMA_VERSION, (unsigned long)(++g_seq),
                (unsigned long)g_trial_id, bus_name(g_bus), (unsigned long)t_now,
                (unsigned long)idx, (unsigned)raw, (long)c);
        }
    } else {
        g_err_n++;
    }

    if (g_mode == TUNER_MODE_CAPTURE && g_idx >= g_target_n) {
        const int32_t p2p = (g_ok_n > 0U) ? (g_max_c - g_min_c) : 0;
        const int32_t mean_x100 = (g_ok_n > 0U)
                                      ? (int32_t)((g_sum_c * 100LL) / (int64_t)g_ok_n)
                                      : 0;
        const bool ok = (g_ok_n == g_target_n) && (g_err_n == 0U) && !g_uw.suspect_alias &&
                        (g_tx_drops == 0U);
        cdc_write_line(
            "{\"schema_version\":%u,\"seq\":%lu,\"trial_id\":%lu,\"kind\":\"result\","
            "\"axis\":\"%s\",\"t_us\":%lu,\"ok\":%s,\"test\":\"enc_noise\","
            "\"sf\":%u,\"conf\":%u,\"rate_hz\":%lu,\"hold\":false,"
            "\"n\":%lu,\"errors\":%lu,\"overruns\":%lu,\"tx_drops\":%lu,\"mean_rel_x100\":%ld,"
            "\"p2p_cnt\":%ld,\"status\":%u,\"agc\":%u,\"mag\":%u}",
            (unsigned)TUNER_SCHEMA_VERSION, (unsigned long)(++g_seq),
            (unsigned long)g_trial_id, bus_name(g_bus), (unsigned long)t_now,
            ok ? "true" : "false",
            (unsigned)sf_code_to_factor(g_enc[g_bus].sf_code),
            (unsigned)g_enc[g_bus].conf_rb,
            (unsigned long)g_rate_hz,
            (unsigned long)g_ok_n, (unsigned long)g_err_n, (unsigned long)g_overruns,
            (unsigned long)g_tx_drops,
            (long)mean_x100, (long)p2p,
            (unsigned)g_last_status, (unsigned)g_last_agc, (unsigned)g_last_mag);
        g_mode = TUNER_MODE_IDLE;
    }
}

/* ------------------------------------------------------------------------- */
/* Shared Core/ entry-point bindings (fw_app.h)                              */
/* ------------------------------------------------------------------------- */

static void i2c_try_fast_mode(uint8_t bus) {
    I2C_HandleTypeDef *hi2c = bus_hi2c(bus);
    const uint32_t clk = (bus == 0U)
                             ? HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C4)
                             : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C1);
    if (clk < 99000000U || clk > 101000000U) {
        return; /* Keep CubeMX default Timing (100 kHz) if kernel clock is not ~100 MHz */
    }
    const uint32_t saved_timing = hi2c->Init.Timing;
    const uint32_t saved_df = (hi2c->Instance->CR1 & I2C_CR1_DNF_Msk) >> I2C_CR1_DNF_Pos;
    hi2c->Init.Timing = TUNER_I2C_FAST_TIMINGR;
    if (HAL_I2C_Init(hi2c) != HAL_OK ||
        HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK ||
        HAL_I2CEx_ConfigDigitalFilter(hi2c, saved_df) != HAL_OK) {
        hi2c->Init.Timing = saved_timing;
        (void)HAL_I2C_Init(hi2c);
        (void)HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
        (void)HAL_I2CEx_ConfigDigitalFilter(hi2c, saved_df);
    }
}

void fw_app_init(void) {
    disable_all_drivers();
    i2c_try_fast_mode(1U); /* X: I2C1 */
    i2c_try_fast_mode(0U); /* Z: I2C4 */
    /* Start TIM2 free-running at 1 MHz for uniform sampling timestamps. */
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CNT = 0U;
    TIM2->CR1 |= TIM_CR1_CEN;

    cmd_linebuf_init(&g_rx);
    (void)usb_cdc_port_init();
    (void)enc_apply_conf(1U, 3U, 0U, 0U, NULL); /* X: SF=2x default */
    (void)enc_apply_conf(0U, 3U, 0U, 0U, NULL); /* Z: SF=2x default */
#if defined(ENABLE_SCRUTINY)
    memset(g_scrutiny_uw, 0, sizeof(g_scrutiny_uw));
    g_scrutiny_next_us = now_us();
    g_scrutiny_diag_div = 0U;
    as5600_scrutiny_init(now_us());
#endif
}

void fw_app_poll(void) {
    usb_cdc_port_poll();
    uint8_t rx[64];
    const uint32_t n = usb_cdc_port_read(rx, sizeof(rx));
    for (uint32_t i = 0U; i < n; ++i) {
        if (cmd_linebuf_push(&g_rx, (char)rx[i])) {
            handle_line(g_rx.buf);
            cmd_linebuf_init(&g_rx);
        }
    }
    poll_sampling();
#if defined(ENABLE_SCRUTINY)
    poll_scrutiny_telemetry();
    as5600_scrutiny_poll(now_us());
#endif

    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - g_last_led_ms) >= ((g_mode == TUNER_MODE_IDLE) ? 500U : 100U)) {
        g_last_led_ms = now_ms;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}

void fw_app_tim_irq(uint8_t motor) { (void)motor; }

bool fw_app_usart2_irq(void) { return false; }

void fw_app_error_handler(void) {
    disable_all_drivers();
    g_mode = TUNER_MODE_IDLE;
}
