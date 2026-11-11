#include "encoder_unwrap.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

static void many_turns(int32_t sign)
{
    enc_unwrap_t e;
    enc_unwrap_init(&e, 0U);
    int32_t expected = 0;
    for (unsigned int i = 0U; i < 205U; ++i) {
        expected += sign * 1000;
        int32_t raw = expected % 4096;
        if (raw < 0) {
            raw += 4096;
        }
        assert(enc_unwrap_update(&e, (uint16_t)raw) == expected);
        int32_t turns = expected / 4096;
        if ((expected % 4096) < 0) {
            --turns;
        }
        assert(enc_unwrap_turns(&e) == turns);
        assert(enc_unwrap_position(&e) == (uint16_t)raw);
    }
    assert(!e.suspect_alias && e.alias_errors == 0U);
    assert(!e.overflow && e.raw_errors == 0U);
}

int main(void)
{
    enc_unwrap_t e;
    enc_unwrap_init(&e, 4095U);
    assert(enc_unwrap_update(&e, 0U) == 1);
    assert(enc_unwrap_update(&e, 4095U) == 0);
    assert(enc_unwrap_update(&e, 4094U) == -1);
    assert(enc_unwrap_turns(&e) == -1 && enc_unwrap_position(&e) == 4095U);
    enc_unwrap_set_zero(&e);
    assert(e.total_counts == 0 && e.last_raw == 4094U);
    assert(enc_unwrap_update(&e, 4095U) == 1);
    enc_unwrap_init(&e, 0U);
    assert(enc_unwrap_update(&e, 4095U) == -1);
    assert(enc_unwrap_update(&e, 0U) == 0);
    assert(enc_unwrap_update(&e, 2048U) == 2048);
    assert(enc_unwrap_update(&e, 0U) == 4096);
    assert(e.suspect_alias && e.alias_errors == 2U);
    assert(enc_unwrap_turns(&e) == 1 && enc_unwrap_position(&e) == 0U);
    assert(enc_unwrap_update(&e, 4096U) == 4096);
    assert(enc_unwrap_update(&e, UINT16_MAX) == 4096);
    assert(e.raw_errors == 2U && e.last_raw == 0U);
    assert(enc_unwrap_update(&e, 1U) == 4097);
    enc_unwrap_set_zero(&e);
    assert(e.suspect_alias && e.alias_errors == 2U && e.raw_errors == 2U);
    enc_unwrap_init(&e, UINT16_MAX);
    assert(!e.initialized && e.raw_errors == 1U);
    assert(enc_unwrap_update(&e, 3000U) == 0 && e.initialized);
    assert(enc_unwrap_update(&e, 3001U) == 1);
    enc_unwrap_init(&e, 0U);
    e.max_delta_counts = 100U;
    assert(enc_unwrap_update(&e, 100U) == 100 && !e.suspect_alias);
    assert(enc_unwrap_update(&e, 201U) == 201 && e.suspect_alias);
    assert(enc_unwrap_update(&e, 100U) == 100 && e.alias_errors == 2U);
    assert(enc_unwrap_update(&e, 99U) == 99 && e.suspect_alias);
    enc_unwrap_init(&e, 0U);
    e.total_counts = INT32_MAX;
    assert(enc_unwrap_update(&e, 1U) == INT32_MAX);
    assert(e.overflow && e.overflow_errors == 1U);
    assert(enc_unwrap_update(&e, 0U) == INT32_MAX - 1);
    e.total_counts = INT32_MIN;
    assert(enc_unwrap_update(&e, 4095U) == INT32_MIN);
    assert(e.overflow_errors == 2U);
    assert(enc_unwrap_turns(&e) == -524288 && enc_unwrap_position(&e) == 0U);
    e.total_counts = -4097;
    assert(enc_unwrap_turns(&e) == -2 && enc_unwrap_position(&e) == 4095U);
    e.total_counts = -4096;
    assert(enc_unwrap_turns(&e) == -1 && enc_unwrap_position(&e) == 0U);
    many_turns(1);
    many_turns(-1);
    puts("Unwrap: wraps, 50+ turns each way, tie, rejection, alias, zero, overflow PASS");
    return 0;
}
