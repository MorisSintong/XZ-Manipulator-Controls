#ifndef AS5600_SCRUTINY_BRIDGE_H
#define AS5600_SCRUTINY_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Live telemetry snapshot per axis exposed to Scrutiny (DWARF variables + 500 Hz datalogger).
 */
typedef struct {
    uint16_t raw;        /* 12-bit RAW ANGLE (0..4095) */
    uint16_t angle;      /* 12-bit filtered ANGLE (0..4095) */
    int32_t  unwrapped;  /* Unwrapped signed counts */
    float    deg;        /* Unwrapped angle in degrees */
    uint16_t mag;        /* CORDIC magnitude (0..4095) */
    uint16_t conf;       /* Current CONF register readback (0x07/0x08) */
    uint8_t  status;     /* STATUS register (MD/ML/MH) */
    uint8_t  agc;        /* AGC gain (0..255; ideal ~128) */
    uint8_t  ok;         /* 1 if last I2C read succeeded, 0 otherwise */
    uint32_t err_count;  /* Cumulative I2C error count */
} as5600_scrutiny_axis_t;

typedef struct {
    as5600_scrutiny_axis_t x; /* Bus 1: I2C1 (PB8/PB9 J73) */
    as5600_scrutiny_axis_t z; /* Bus 0: I2C4 (PB6/PB7 J43) */
    uint32_t loop_count;      /* 500 Hz sample counter */
    uint32_t rtt_rx_bytes;    /* Total bytes received from RTT ch0 */
    uint32_t rtt_tx_bytes;    /* Total bytes written to RTT ch0 */
} as5600_scrutiny_state_t;

extern as5600_scrutiny_state_t g_scrutiny_as5600;

/**
 * Callback implemented by as5600_tuner_app.c so Scrutiny RPV writes can
 * update volatile AS5600 CONF (SF / HYST / FTH) safely without OTP writes.
 */
bool as5600_tuner_apply_conf_from_scrutiny(uint8_t bus, uint8_t sf_code,
                                           uint8_t hyst_code, uint8_t fth_code,
                                           uint16_t *conf_rb);

/**
 * Query current cached CONF fields for a bus (0=Z, 1=X).
 */
void as5600_tuner_get_conf_for_scrutiny(uint8_t bus, uint8_t *sf_code,
                                        uint8_t *hyst_code, uint8_t *fth_code,
                                        uint16_t *conf_rb);

/**
 * Initialize Scrutiny MainHandler + 500 Hz FixedFrequencyLoopHandler + RPVs over SEGGER RTT ch0.
 */
void as5600_scrutiny_init(uint32_t now_us);

/**
 * Pump RTT ch0 <-> Scrutiny MainHandler (call from fw_app_poll).
 */
void as5600_scrutiny_poll(uint32_t now_us);

/**
 * Advance Scrutiny's 500 Hz datalogger loop (call once per 2 ms encoder sample tick).
 */
void as5600_scrutiny_tick_500hz(void);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_SCRUTINY_BRIDGE_H */
