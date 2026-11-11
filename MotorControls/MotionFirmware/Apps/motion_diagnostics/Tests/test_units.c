#include "motion_units.h"
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>

int main(void)
{
    int32_t residual = INT32_MAX;
    assert(um_to_steps(100000, 3200U, 40000U, &residual) == 8000);
    assert(residual == 0);
    assert(um_to_steps(1000, 3200U, 8000U, &residual) == 400);
    assert(residual == 0);
    assert(steps_to_um(8000, 3200U, 40000U) == 100000);
    assert(steps_to_um(400, 3200U, 8000U) == 1000);
    assert(enc_counts_to_um(4096, 40000U) == 40000);
    assert(enc_counts_to_um(-4096, 8000U) == -8000);
    assert(enc_counts_to_um(1, 40000U) == 10);
    assert(enc_counts_to_um(-1, 40000U) == -10);
    assert(enc_counts_to_um(1, 8000U) == 2);
    assert(steps_to_um(1, 3200U, 40000U) == 13);
    assert(steps_to_um(-1, 3200U, 40000U) == -13);
    assert(steps_to_um(1, 3200U, 8000U) == 3);
    assert(steps_to_um(-1, 3200U, 8000U) == -3);
    assert(um_to_steps(6, 3200U, 40000U, &residual) == 0 && residual == 6000);
    assert(um_to_steps(7, 3200U, 40000U, &residual) == 1 && residual == -5500);
    assert(um_to_steps(-7, 3200U, 40000U, &residual) == -1 && residual == 5500);
    assert(um_to_steps(-6, 3200U, 40000U, &residual) == 0 && residual == -6000);
    assert(um_to_steps(5, 1U, 10U, &residual) == 1 && residual == -5000);
    assert(um_to_steps(-5, 1U, 10U, &residual) == -1 && residual == 5000);
    assert(um_to_steps(1, 3U, 2U, &residual) == 2 && residual == -333);
    assert(um_to_steps(-1, 3U, 2U, &residual) == -2 && residual == 333);
    assert(um_to_steps(1, 16U, 17U, &residual) == 1 && residual == -63);
    assert(um_to_steps(-1, 16U, 17U, &residual) == -1 && residual == 63);
    for (int32_t um = -300000; um <= 300000; um += 1000) {
        const int32_t x = um_to_steps(um, 3200U, 40000U, &residual);
        assert(x == (um / 1000) * 80 && residual == 0);
        assert(steps_to_um(x, 3200U, 40000U) == um);
        const int32_t z = um_to_steps(um, 3200U, 8000U, &residual);
        assert(z == (um / 1000) * 400 && residual == 0);
        assert(steps_to_um(z, 3200U, 8000U) == um);
    }
    assert(steps_to_um(1, 0U, 40000U) == 0);
    assert(um_to_steps(1, 0U, 40000U, &residual) == 0 && residual == 0);
    residual = INT32_MAX;
    assert(um_to_steps(1, 3200U, 0U, &residual) == 0 && residual == 0);
    assert(um_to_steps(1000, 3200U, 8000U, NULL) == 400);
    assert(steps_to_um(INT32_MAX, UINT32_MAX, UINT32_MAX) == INT32_MAX);
    assert(steps_to_um(INT32_MIN, UINT32_MAX, UINT32_MAX) == INT32_MIN);
    assert(steps_to_um(INT32_MAX, 1U, UINT32_MAX) == INT32_MAX);
    assert(steps_to_um(INT32_MIN, 1U, UINT32_MAX) == INT32_MIN);
    assert(um_to_steps(INT32_MAX, UINT32_MAX, UINT32_MAX, &residual) == INT32_MAX);
    assert(residual == 0);
    assert(um_to_steps(INT32_MIN, UINT32_MAX, UINT32_MAX, &residual) == INT32_MIN);
    assert(residual == 0);
    assert(um_to_steps(INT32_MAX, UINT32_MAX, 1U, &residual) == INT32_MAX);
    assert(residual == INT32_MAX);
    assert(um_to_steps(INT32_MIN, UINT32_MAX, 1U, &residual) == INT32_MIN);
    assert(residual == INT32_MIN);
    puts("Units: X/Z, signed rounding, residuals, 601 distances, full-range saturation PASS");
    return 0;
}
