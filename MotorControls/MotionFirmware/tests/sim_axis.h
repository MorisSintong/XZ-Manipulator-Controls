/* Host simulator for the axis_hw.h seam: step sequencing (the real
 * step_core), a rotor with rigid end stops, a StallGuard4 load model and
 * AS5600 multi-turn readings. */
#ifndef SIM_AXIS_H
#define SIM_AXIS_H

#include <stdbool.h>
#include <stdint.h>

#include "axis_ctrl.h"

typedef struct {
    int32_t stop_lo;     /* true hard stop positions, microsteps */
    int32_t stop_hi;
    int32_t start;       /* true start position */
    float   sg_free;     /* SG4_RESULT while running freely */
    float   sg_noise;    /* +/- uniform noise */
    float   sg_stall;    /* SG4_RESULT while pushing against a stop */
    bool    sg_broken;   /* SG4 never indicates a stall */
    bool    enc_present;
    float   enc_ratio;   /* counts per microstep, signed */
    int32_t enc_offset;  /* counts at true position 0 */
    uint8_t enc_source;  /* rotor seen by this encoder (swap test) */
    bool    spi_fail;
} sim_axis_cfg_t;

void     sim_reset(uint32_t seed);
void     sim_setup(uint8_t ax, const sim_axis_cfg_t *cfg);
void     sim_run_ms(uint32_t ms);
/** Run until pred() is true or 'timeout_ms' elapsed. Returns pred(). */
bool     sim_run_until(bool (*pred)(void), uint32_t timeout_ms);
uint32_t sim_now(void);
float    sim_rotor(uint8_t ax);
uint32_t sim_blocked_ms(uint8_t ax);
void     sim_clear_blocked(uint8_t ax);
void     sim_track_range(uint8_t ax, float *lo, float *hi, bool reset);
void     sim_inject_slip(uint8_t ax, int32_t steps);
void     sim_set_spi_fail(uint8_t ax, bool fail);
void     sim_set_sg_broken(uint8_t ax, bool broken);
void     sim_set_verbose(bool verbose);
unsigned sim_log_count(const char *needle);

#endif /* SIM_AXIS_H */
