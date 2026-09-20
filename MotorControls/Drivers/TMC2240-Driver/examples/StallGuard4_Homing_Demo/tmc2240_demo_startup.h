#ifndef TMC2240_DEMO_STARTUP_H
#define TMC2240_DEMO_STARTUP_H
#include "tmc2240_hal.h"

/* Demo-specific profile, not a universal motor default. The caller must hold
 * ENN inactive, check the result, and separately request activation. Only
 * observed startup flags explicitly selected by the caller are acknowledged.
 * Persistent/unselected faults and any late fault abort preparation.
 */
TMC2240Status tmc_demo_prepare_motor(uint16_t id, uint16_t run_mA, uint16_t rref_ohm,
                                   uint8_t acknowledge_startup_flags);

#endif
