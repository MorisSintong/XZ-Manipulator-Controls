/*******************************************************************************
 * Copyright © 2024 STM32 HAL Integration for TMC2240
 *
 * STM32F4 HAL SPI adapter. F446RE is the software integration target.
 * Caller must serialize entire public operations and shared buses, not merely
 * HAL_SPI_TransmitReceive calls. No internal locks, ISR, or RTOS-safety promise.
 *******************************************************************************/
#ifndef TMC2240_HAL_H_
#define TMC2240_HAL_H_

#include "stm32f4xx_hal.h"
#include "tmc2240_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TMC2240_HAL_MAX_ICS
#define TMC2240_HAL_MAX_ICS 4U
#endif
#if (TMC2240_HAL_MAX_ICS < 1) || (TMC2240_HAL_MAX_ICS > 255)
#error TMC2240_HAL_MAX_ICS must be in 1..255
#endif
#define TMC2240_SPI_FRAME_SIZE 5U
#ifndef TMC2240_HAL_SPI_TIMEOUT_MS
#define TMC2240_HAL_SPI_TIMEOUT_MS 10U /* Optional application-selected value. */
#endif
#define TMC2240_SPI_MODE 3U /* Protocol mode number, not SPI_InitTypeDef.Mode. */

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t spi_timeout_ms;
} TMC2240_SPIConfig_t;

/* Handles/pins must remain valid and unchanged until deinit. SPI master, mode
 * 3, 8-bit, MSB-first, software NSS, no TI/CRC, <=10 MHz. SystemCoreClock and
 * RCC clocks must reflect reality. Timeouts are finite, 1..0x7FFFFFFF ms.
 * Init validates every entry before any GPIO/SPI access. It sets CS inactive,
 * clears software state, and NEVER writes motor registers or acknowledges
 * faults. An already initialized adapter must be deinitialized first.
 * Keep external ENN high during setup; transport init cannot disable a motor
 * left active by another host/reset domain. Deinit also does not disable it.
 */
TMC2240Status tmc2240_hal_init(const TMC2240_SPIConfig_t *configs, uint8_t count);
TMC2240Status tmc2240_hal_deinit(void);
bool tmc2240_hal_isInitialized(void);
TMC2240Status tmc2240_hal_testConnection(uint16_t icID); /* IOIN.VERSION == 0x40 */
/* Last frame, diagnostic only; HAL_ERROR before any transfer. */
HAL_StatusTypeDef tmc2240_hal_spiStatus(uint16_t icID);
TMC2240Status tmc2240_hal_getSPIStatus(uint16_t icID, uint8_t *status);

/* All six values are explicit application choices, not universal motor
 * defaults. chopconf must have TOFF=0. configureMotor disables TOFF first,
 * writes/reads back all six registers, and leaves TOFF=0. It never clears GSTAT.
 * activateMotor requires successful configuration, matching readback, and
 * cleared GSTAT. Failure after attempting the activation write triggers a
 * best-effort disable, preserves the FIRST error, and requires reconfiguration.
 * Argument/precondition failures do not write TOFF. A transfer cannot
 * guarantee physical disable; keep a hardware shutdown path.
 * Raw core writes intentionally remain low-level and can bypass this policy.
 */
typedef struct {
    uint32_t gconf;
    uint32_t drv_conf;
    uint32_t global_scaler;
    uint32_t ihold_irun;
    uint32_t chopconf;
    uint32_t pwmconf;
} TMC2240_MotorConfig_t;

TMC2240Status tmc2240_hal_configureMotor(uint16_t icID,
                                        const TMC2240_MotorConfig_t *config);
TMC2240Status tmc2240_hal_activateMotor(uint16_t icID, uint8_t toff);
TMC2240Status tmc2240_hal_disableMotor(uint16_t icID);

/* Nominal RMS calculation, Rev.2 pp.46-48,87-88:
 * RREF 12000..60000 ohm; ranges 0/1/2/3 use KIFS 11.75/24/36/36 A*kOhm.
 * GS=0 means 256; 1..31 are forbidden. Quantize DOWN, not saturate.
 * Zero run current, zero hold percent, and unrepresentable currents return
 * RANGE (CS=0 means 1/32 current, not zero). Use disableMotor for zero current.
 * Holds are floor((IRUN+1)*holdPercent/100)-1. Delays each range 0..15.
 * Output includes both delays. setCurrent preserves the existing IRUNDELAY.
 */
TMC2240Status tmc2240_calculateCurrent(uint16_t runCurrent_mA, uint16_t rRef_Ohm,
    uint8_t currentRange, uint8_t globalScaler, uint8_t holdPercent,
    uint8_t holdDelay, uint8_t runDelay, uint32_t *ihold_irun);
TMC2240Status tmc2240_hal_setCurrent(uint16_t icID, uint16_t runCurrent_mA,
    uint16_t rRef_Ohm, uint8_t holdPercent, uint8_t holdDelay);
/* 1 (and legacy 0) means fullstep. Other values: 2,4,...,256. */
TMC2240Status tmc2240_hal_setMicrosteps(uint16_t icID, uint16_t usteps);

/* Helpers preserve unspecified fields; no hidden PWM/default programming.
 * Changes after configureMotor require reconfiguration before activation.
 * StealthChop enable on an active chopper requires standstill and IHOLD=IRUN.
 * StallGuard4 is for StealthChop; threshold compares to SG4_RESULT (bits9,0=0).
 * CoolStep SEMIN is a load threshold (0 disables), not a current fraction.
 * Set TCOOLTHRS explicitly for velocity-gated StallGuard DIAG/CoolStep.
 */
TMC2240Status tmc2240_stealthchop_enable(uint16_t icID, uint8_t enable);
TMC2240Status tmc2240_stallguard_set_threshold(uint16_t icID, uint8_t threshold);
TMC2240Status tmc2240_stallguard_read(uint16_t icID, uint16_t *result);
TMC2240Status tmc2240_coolstep_configure(uint16_t icID, uint8_t semin, uint8_t semax,
    uint8_t seup, uint8_t sedn, int8_t stall_thr);
TMC2240Status tmc2240_driver_status(uint16_t icID, uint32_t *status);
/* Observe GSTAT bits 0..4 without acknowledgement. Explicit W1C uses the same
 * bit positions and checks selected flags cleared; persistent flags -> FAULT. */
TMC2240Status tmc2240_check_faults(uint16_t icID, uint8_t *faults);
TMC2240Status tmc2240_clear_faults(uint16_t icID, uint8_t mask);

/* Reject IOIN.ADC_ERR; mask off ADC_AIN/reserved upper half. Integer conversions
 * truncate towards zero: supply=ADC*9732/1000 mV, temp=(ADC-2038)*100/77 c10.
 */
TMC2240Status tmc2240_read_vsupply(uint16_t icID, uint16_t *adc);
TMC2240Status tmc2240_get_vsupply_mV(uint16_t icID, uint32_t *millivolts);
TMC2240Status tmc2240_read_temperature(uint16_t icID, uint16_t *adc);
TMC2240Status tmc2240_get_temperature_c10(uint16_t icID, int16_t *temperature);

/* Binary signed 16.16 scaling (65536 = +1, -65536 = -1); INT32_MIN is outside
 * the documented +/-32767.999... range. Zero scale is allowed. Explicitly
 * zeros XENC, then arms one latch+clear on either N edge, independent of A/B.
 * POS_NEG_EDGE controls N-event sensitivity, not quadrature A/B counting.
 */
TMC2240Status tmc2240_encoder_init(uint16_t icID, int32_t enc_constant);
TMC2240Status tmc2240_encoder_read(uint16_t icID, int32_t *position);
TMC2240Status tmc2240_encoder_read_latch(uint16_t icID, int32_t *position);
TMC2240Status tmc2240_encoder_get_status(uint16_t icID, uint8_t *n_event);
TMC2240Status tmc2240_encoder_clear_n_event(uint16_t icID);

/* TSTEP is time per 1/256 microstep regardless of MRES (Rev.2 p.89).
 * All thresholds are 20-bit. Larger TPWMTHRS means a LOWER switching speed.
 */
TMC2240Status tmc2240_set_tpwmthrs(uint16_t icID, uint32_t threshold);
TMC2240Status tmc2240_set_tcoolthrs(uint16_t icID, uint32_t threshold);
TMC2240Status tmc2240_set_thigh(uint16_t icID, uint32_t threshold);
/* One RMW, never briefly enable TOFF with old hysteresis. Nonzero TOFF uses
 * the same configuration/fault/readback gate as activateMotor.
 * TBL=0 requires an external <=8MHz clock; TBL=1 requires <=13MHz (caller).
 * TOFF=1 requires TBL>=2. Checked activation/configuration also enforce effective
 * HEND+HSTRT<=16 at CS=31 in SpreadCycle (Rev.2 pp.44,111-112).
 */
TMC2240Status tmc2240_set_chopper(uint16_t icID, uint8_t toff,
                                uint8_t hstrt, uint8_t hend, uint8_t tbl);
TMC2240Status tmc2240_set_tpowerdown(uint16_t icID, uint8_t delay);
TMC2240Status tmc2240_diag_configure(uint16_t icID, uint8_t diag0_error,
    uint8_t diag0_otpw, uint8_t diag0_stall, uint8_t diag1_stall,
    uint8_t diag1_index, uint8_t diag1_onstate, uint8_t pushpull);
/* Either output may be NULL, but not both; outputs unchanged on failure. */
TMC2240Status tmc2240_pwm_get_scale(uint16_t icID, uint16_t *sum, int16_t *autoScale);
TMC2240Status tmc2240_get_microstep_counter(uint16_t icID, uint16_t *counter);
TMC2240Status tmc2240_get_microstep_current(uint16_t icID, int16_t *cur_a, int16_t *cur_b);

#ifdef __cplusplus
}
#endif
#endif
