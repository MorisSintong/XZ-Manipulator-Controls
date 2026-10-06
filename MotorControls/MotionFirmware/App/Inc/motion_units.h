#ifndef APP_MOTION_UNITS_H
#define APP_MOTION_UNITS_H

#include <stdint.h>

/** Convert AS5600 counts (4096/revolution), rounding half away from zero, saturating. */
int32_t enc_counts_to_um(int32_t counts, uint32_t um_per_rev);
/** Convert microsteps with half-away rounding and saturation; zero divisor returns zero. */
int32_t steps_to_um(int32_t microsteps, uint32_t microsteps_per_rev,
                    uint32_t um_per_rev);
/** Convert distance with half-away rounding and saturation; zero parameters return zero.
 * Optional residual is requested minus represented distance in nm, rounded/saturated.
 */
int32_t um_to_steps(int32_t um, uint32_t microsteps_per_rev,
                    uint32_t um_per_rev, int32_t *residual_nm_out);

#endif
