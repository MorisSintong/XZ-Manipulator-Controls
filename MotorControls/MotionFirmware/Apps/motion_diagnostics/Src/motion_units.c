#include "motion_units.h"
#include <limits.h>
#include <stddef.h>

static int32_t clamp_i32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static int64_t rounded_div(int64_t numerator, uint32_t denominator)
{
    const int64_t d = (int64_t)denominator;
    int64_t q = numerator / d;
    const int64_t r = numerator % d;
    const int64_t magnitude = (r < 0) ? -r : r;
    if ((magnitude * 2) >= d) {
        q += (numerator < 0) ? -1 : 1;
    }
    return q;
}

int32_t enc_counts_to_um(int32_t counts, uint32_t um_per_rev)
{
    return steps_to_um(counts, 4096U, um_per_rev);
}

int32_t steps_to_um(int32_t microsteps, uint32_t microsteps_per_rev,
                    uint32_t um_per_rev)
{
    if (microsteps_per_rev == 0U) {
        return 0;
    }
    return clamp_i32(rounded_div((int64_t)microsteps * (int64_t)um_per_rev,
                                microsteps_per_rev));
}

int32_t um_to_steps(int32_t um, uint32_t microsteps_per_rev,
                    uint32_t um_per_rev, int32_t *residual_nm_out)
{
    if (residual_nm_out != NULL) {
        *residual_nm_out = 0;
    }
    if ((um_per_rev == 0U) || (microsteps_per_rev == 0U)) {
        return 0;
    }
    const int64_t numerator = (int64_t)um * (int64_t)microsteps_per_rev;
    const int32_t steps = clamp_i32(rounded_div(numerator, um_per_rev));
    if (residual_nm_out != NULL) {
        const int64_t residual = numerator - (int64_t)steps * (int64_t)um_per_rev;
        const int64_t whole = residual / (int64_t)microsteps_per_rev;
        const int64_t remainder = residual % (int64_t)microsteps_per_rev;
        if (whole > INT32_MAX) {
            *residual_nm_out = INT32_MAX;
        } else if (whole < INT32_MIN) {
            *residual_nm_out = INT32_MIN;
        } else {
            *residual_nm_out = clamp_i32(whole * 1000
                + rounded_div(remainder * 1000, microsteps_per_rev));
        }
    }
    return steps;
}
