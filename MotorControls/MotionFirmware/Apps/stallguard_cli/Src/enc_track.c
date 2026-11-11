/**
 * @file    enc_track.c
 * @brief   Multi-turn tracking of a 12-bit absolute angle.
 */
#include "enc_track.h"

void enc_track_reset(enc_track_t *t)
{
    t->counts = 0;
    t->last_raw = 0U;
    t->primed = false;
}

int32_t enc_track_delta(uint16_t from_raw, uint16_t to_raw)
{
    int32_t d = (int32_t)(to_raw & 0x0FFFU) - (int32_t)(from_raw & 0x0FFFU);

    if (d >= (ENC_COUNTS_PER_REV / 2)) {
        d -= ENC_COUNTS_PER_REV;
    } else if (d < -(ENC_COUNTS_PER_REV / 2)) {
        d += ENC_COUNTS_PER_REV;
    }
    return d;
}

int32_t enc_track_update(enc_track_t *t, uint16_t raw12)
{
    raw12 &= 0x0FFFU;
    if (!t->primed) {
        t->counts = (int32_t)raw12;
        t->primed = true;
    } else {
        t->counts += enc_track_delta(t->last_raw, raw12);
    }
    t->last_raw = raw12;
    return t->counts;
}
