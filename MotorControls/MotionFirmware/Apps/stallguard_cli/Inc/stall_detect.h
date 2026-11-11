/**
 * @file    stall_detect.h
 * @brief   StallGuard4 end-stop detector (pure C, host testable).
 *
 * Combines three stall indications of the TMC2240 while the axis runs at a
 * constant homing speed:
 *   1. the chip's own comparator (SPI status bit "sg2" = DRV_STATUS.stallGuard,
 *      set while SG4_RESULT <= 2 * SG4_THRS and TSTEP <= TCOOLTHRS),
 *   2. an absolute software threshold on SG4_RESULT (valid from the start of
 *      a seek, e.g. when the carriage already rests against the stop), and
 *   3. an adaptive drop relative to the free-running SG4 baseline, which makes
 *      detection independent of the particular motor, current and speed.
 * A candidate must persist for 'confirm' consecutive samples. Samples taken
 * during acceleration and during a blanking period after reaching cruise
 * speed are ignored because SG4 is not meaningful there.
 */
#ifndef STALL_DETECT_H
#define STALL_DETECT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t blank_ms;         /* ignore samples this long after cruise starts */
    uint16_t baseline_samples; /* samples averaged into the initial baseline */
    uint16_t min_baseline;     /* baselines below this are not trusted */
    uint16_t no_baseline_ms;   /* stall if no trusted baseline this long after
                                  blanking (0 = disabled) */
    uint16_t abs_threshold;    /* stall while SG4_RESULT <= this */
    float    drop_ratio;       /* stall while SG4_RESULT < baseline * ratio */
    uint8_t  confirm;          /* consecutive candidate samples to confirm */
    uint8_t  ema_shift;        /* baseline tracking weight = 1 / 2^shift */
} stall_cfg_t;

typedef enum {
    STALL_NONE = 0,
    STALL_HW_FLAG,     /* TMC2240 comparator (SPI status / DRV_STATUS) */
    STALL_SG_ABS,      /* SG4_RESULT below the absolute threshold */
    STALL_SG_DROP,     /* SG4_RESULT dropped below baseline * drop_ratio */
    STALL_NO_BASELINE, /* SG4 never rose to a trustworthy free-run level */
    STALL_ENCODER      /* reported by the caller: encoder shows no motion */
} stall_source_t;

typedef struct {
    stall_cfg_t    cfg;
    uint32_t       acc_sum;
    uint16_t       acc_n;
    float          baseline;
    bool           baseline_valid;
    uint8_t        hits;
    stall_source_t candidate;
    uint16_t       last_sg;
    uint16_t       min_sg;   /* minimum accepted (non-candidate) sample */
} stall_det_t;

void stall_det_reset(stall_det_t *d, const stall_cfg_t *cfg);

/**
 * Feed one SG4 sample.
 * @param since_cruise_ms  ms since the commanded speed reached cruise, or a
 *                         negative value while still accelerating.
 * @param sg4              SG4_RESULT (0..510, larger = lower load).
 * @param hw_flag          TMC2240 stallGuard status bit.
 * @return STALL_NONE, or the source of a confirmed stall.
 */
stall_source_t stall_det_update(stall_det_t *d, int32_t since_cruise_ms,
                                uint16_t sg4, bool hw_flag);

/** SG4_THRS that makes the chip comparator agree with the adaptive threshold
 *  (0 if no baseline yet). */
uint8_t stall_det_hw_threshold(const stall_det_t *d);

const char *stall_source_name(stall_source_t s);

#ifdef __cplusplus
}
#endif

#endif /* STALL_DETECT_H */
