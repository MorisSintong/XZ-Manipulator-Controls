/**
 * @file    encoder.c
 * @brief   Dual AS5600 magnetic encoder manager (see encoder.h).
 */
#include "encoder.h"

#include "as5600_stm32_hal.h"
#include "enc_track.h"
#include "stepgen.h"

#include <string.h>

#define ENC_READ_TIMEOUT_MS 5U
#define ENC_DIAG_TIMEOUT_MS 10U
#define ENC_CONF_TIMEOUT_MS 20U
#define ENC_FRESH_MS        20U
#define ENC_ERROR_LIMIT     3U
#define ENC_RETRY_MS        500U

typedef struct {
    I2C_HandleTypeDef *i2c;
    as5600_t           dev;
    as5600_stm32_hal_t port;
    enc_track_t        track;
    encoder_info_t     info;
    uint32_t           t_retry;
} enc_t;

/* SCL/SDA pins of the encoder buses, used for the bus-clear sequence. They
 * must match the pin assignment in MotionFirmware.ioc / stm32f4xx_hal_msp.c. */
typedef struct {
    const I2C_TypeDef *inst;
    GPIO_TypeDef      *scl_port;
    uint16_t           scl_pin;
    GPIO_TypeDef      *sda_port;
    uint16_t           sda_pin;
} i2c_pins_t;

static const i2c_pins_t k_pins[] = {
    { I2C1, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9 }, /* PB8 SCL, PB9 SDA */
    { I2C3, GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9 }, /* PA8 SCL, PC9 SDA */
};

static enc_t   g_enc[ENCODER_MAX];
static uint8_t g_enc_count;

static void dwt_enable(void)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

static void delay_us(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks) {
    }
}

static const i2c_pins_t *find_pins(const I2C_TypeDef *inst)
{
    size_t i;

    for (i = 0U; i < sizeof(k_pins) / sizeof(k_pins[0]); i++) {
        if (k_pins[i].inst == inst) {
            return &k_pins[i];
        }
    }
    return NULL;
}

/* Release a slave that holds SDA low (e.g. after an MCU reset in the middle
 * of a read): clock SCL until SDA is free, send STOP, re-initialise I2C
 * (HAL_I2C_Init also pulses SWRST, clearing a stuck BUSY flag). */
static void bus_recover(I2C_HandleTypeDef *h)
{
    const i2c_pins_t *p = find_pins(h->Instance);
    GPIO_InitTypeDef g = { 0 };
    int i;

    if (p == NULL) {
        return;
    }
    (void)HAL_I2C_DeInit(h);
    HAL_GPIO_WritePin(p->scl_port, p->scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p->sda_port, p->sda_pin, GPIO_PIN_SET);
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = p->scl_pin;
    HAL_GPIO_Init(p->scl_port, &g);
    g.Pin = p->sda_pin;
    HAL_GPIO_Init(p->sda_port, &g);
    delay_us(10U);
    for (i = 0; (i < 9) && (HAL_GPIO_ReadPin(p->sda_port, p->sda_pin) == GPIO_PIN_RESET); i++) {
        HAL_GPIO_WritePin(p->scl_port, p->scl_pin, GPIO_PIN_RESET);
        delay_us(5U);
        HAL_GPIO_WritePin(p->scl_port, p->scl_pin, GPIO_PIN_SET);
        delay_us(5U);
    }
    HAL_GPIO_WritePin(p->scl_port, p->scl_pin, GPIO_PIN_RESET);
    delay_us(5U);
    HAL_GPIO_WritePin(p->sda_port, p->sda_pin, GPIO_PIN_RESET);
    delay_us(5U);
    HAL_GPIO_WritePin(p->scl_port, p->scl_pin, GPIO_PIN_SET);
    delay_us(5U);
    HAL_GPIO_WritePin(p->sda_port, p->sda_pin, GPIO_PIN_SET); /* STOP */
    delay_us(5U);
    (void)HAL_I2C_Init(h);
}

static bool is_magnet_result(as5600_result_t r)
{
    return (r == AS5600_ERROR_NO_MAGNET) || (r == AS5600_ERROR_MAGNET_WEAK) ||
           (r == AS5600_ERROR_MAGNET_STRONG);
}

static as5600_result_t read_diag(enc_t *e)
{
    as5600_diagnostics_t d;
    as5600_result_t r = as5600_read_diagnostics(&e->dev, &d, ENC_DIAG_TIMEOUT_MS);

    if (r == AS5600_OK) {
        e->info.status = d.status;
        e->info.agc = d.agc;
        e->info.magnitude = d.magnitude;
        r = as5600_magnet_status(d.status);
    }
    return r;
}

static void mark_failed(enc_t *e, as5600_result_t r)
{
    e->info.last_result = r;
    if (e->info.healthy) {
        e->info.healthy = false;
        e->info.epoch++; /* multi-turn continuity lost */
    }
    e->t_retry = HAL_GetTick();
}

/* Bind the driver, program the volatile filter settings and check the magnet.
 * Tracking restarts from the absolute angle (new epoch). */
static void start(enc_t *e)
{
    as5600_config_t cfg;
    as5600_result_t r = as5600_stm32_hal_init(&e->dev, &e->port, e->i2c, NULL);

    e->info.bound = (r == AS5600_OK);
    if (!e->info.bound) {
        mark_failed(e, r);
        return;
    }
    /* Slow filter 8x for low noise at standstill, fast filter (6 LSB) for a
     * short lag while the motor turns. Volatile write: OTP is not touched. */
    (void)as5600_default_config(&cfg);
    cfg.slow_filter = AS5600_SLOW_FILTER_8X;
    cfg.fast_filter = AS5600_FAST_FILTER_6_LSB;
    r = as5600_write_config(&e->dev, &cfg, ENC_CONF_TIMEOUT_MS);
    if (r == AS5600_OK) {
        r = read_diag(e);
    }
    e->info.last_result = r;
    if ((r == AS5600_OK) || (r == AS5600_ERROR_MAGNET_WEAK) || (r == AS5600_ERROR_MAGNET_STRONG)) {
        enc_track_reset(&e->track);
        e->info.error_run = 0U;
        e->info.healthy = true;
        e->info.epoch++;
    } else {
        mark_failed(e, r);
    }
}

void encoder_init(uint8_t count, I2C_HandleTypeDef *const buses[])
{
    uint8_t i;

    dwt_enable();
    g_enc_count = (count > ENCODER_MAX) ? ENCODER_MAX : count;
    for (i = 0U; i < g_enc_count; i++) {
        enc_t *e = &g_enc[i];
        memset(e, 0, sizeof(*e));
        e->i2c = buses[i];
        if (e->i2c == NULL) {
            continue;
        }
        bus_recover(e->i2c);
        start(e);
    }
}

void encoder_poll(void)
{
    uint8_t i;

    for (i = 0U; i < g_enc_count; i++) {
        enc_t *e = &g_enc[i];
        uint16_t raw = 0U;
        int32_t pos;
        as5600_result_t r;

        if (!e->info.bound || !e->info.healthy) {
            continue;
        }
        pos = stepgen_position(i); /* commanded position at the time of reading */
        r = as5600_read_raw_angle(&e->dev, &raw, ENC_READ_TIMEOUT_MS);
        if (r == AS5600_OK) {
            e->info.raw = raw;
            e->info.counts = enc_track_update(&e->track, raw);
            e->info.pos_snapshot = pos;
            e->info.seq++;
            e->info.t_last_ok = HAL_GetTick();
            e->info.error_run = 0U;
        } else {
            e->info.errors++;
            e->info.last_result = r;
            e->info.error_run++;
            if (e->info.error_run >= ENC_ERROR_LIMIT) {
                mark_failed(e, r);
            }
        }
    }
}

void encoder_poll_diag(void)
{
    uint8_t i;

    for (i = 0U; i < g_enc_count; i++) {
        enc_t *e = &g_enc[i];
        as5600_result_t r;

        if (e->i2c == NULL) {
            continue;
        }
        if (!e->info.bound || !e->info.healthy) {
            if ((HAL_GetTick() - e->t_retry) >= ENC_RETRY_MS) {
                e->info.recoveries++;
                bus_recover(e->i2c);
                start(e);
            }
            continue;
        }
        r = read_diag(e);
        if (r == AS5600_ERROR_NO_MAGNET) {
            mark_failed(e, r);
        } else if (!is_magnet_result(r) && (r != AS5600_OK)) {
            e->info.errors++;
            e->info.error_run++;
            e->info.last_result = r;
            if (e->info.error_run >= ENC_ERROR_LIMIT) {
                mark_failed(e, r);
            }
        }
    }
}

bool encoder_sample(uint8_t ax, axis_enc_sample_t *sample)
{
    const enc_t *e;

    if ((ax >= g_enc_count) || (sample == NULL)) {
        return false;
    }
    e = &g_enc[ax];
    if (!e->info.healthy || (e->info.seq == 0U) ||
        ((HAL_GetTick() - e->info.t_last_ok) > ENC_FRESH_MS)) {
        return false;
    }
    sample->counts = e->info.counts;
    sample->pos = e->info.pos_snapshot;
    sample->epoch = e->info.epoch;
    sample->seq = e->info.seq;
    return true;
}

const encoder_info_t *encoder_info(uint8_t ax)
{
    return (ax < g_enc_count) ? &g_enc[ax].info : NULL;
}

const char *encoder_result_name(as5600_result_t r)
{
    switch (r) {
    case AS5600_OK:                   return "ok";
    case AS5600_ERROR_ARGUMENT:       return "argument";
    case AS5600_ERROR_NOT_INITIALIZED:return "not initialised";
    case AS5600_ERROR_CONTEXT:        return "context";
    case AS5600_ERROR_BUSY:           return "I2C busy";
    case AS5600_ERROR_TIMEOUT:        return "I2C timeout";
    case AS5600_ERROR_NACK:           return "no ACK (not connected?)";
    case AS5600_ERROR_IO:             return "I2C error";
    case AS5600_ERROR_LOCK:           return "lock";
    case AS5600_ERROR_UNLOCK:         return "unlock";
    case AS5600_ERROR_VERIFY:         return "verify";
    case AS5600_ERROR_DATA:           return "data";
    case AS5600_ERROR_NO_MAGNET:      return "no magnet";
    case AS5600_ERROR_MAGNET_WEAK:    return "magnet too weak";
    case AS5600_ERROR_MAGNET_STRONG:  return "magnet too strong";
    default:                          return "?";
    }
}
