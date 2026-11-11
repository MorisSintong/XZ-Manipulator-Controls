/**
 * @file    stall_detect.c
 * @brief   StallGuard4 end-stop detector.
 */
#include "stall_detect.h"

void stall_det_reset(stall_det_t *d, const stall_cfg_t *cfg)
{
    d->cfg = *cfg;
    if (d->cfg.baseline_samples == 0U) {
        d->cfg.baseline_samples = 1U;
    }
    if (d->cfg.confirm == 0U) {
        d->cfg.confirm = 1U;
    }
    if (d->cfg.ema_shift > 8U) {
        d->cfg.ema_shift = 8U;
    }
    if (!(d->cfg.drop_ratio > 0.0f) || (d->cfg.drop_ratio >= 1.0f)) {
        d->cfg.drop_ratio = 0.5f;
    }
    d->acc_sum = 0U;
    d->acc_n = 0U;
    d->baseline = 0.0f;
    d->baseline_valid = false;
    d->hits = 0U;
    d->candidate = STALL_NONE;
    d->last_sg = 0U;
    d->min_sg = UINT16_MAX;
}

stall_source_t stall_det_update(stall_det_t *d, int32_t since_cruise_ms,
                                uint16_t sg4, bool hw_flag)
{
    stall_source_t cand = STALL_NONE;

    d->last_sg = sg4;
    if (since_cruise_ms < (int32_t)d->cfg.blank_ms) {
        d->hits = 0U;
        d->candidate = STALL_NONE;
        return STALL_NONE;
    }

    if (hw_flag) {
        cand = STALL_HW_FLAG;
    } else if (sg4 <= d->cfg.abs_threshold) {
        cand = STALL_SG_ABS;
    } else if (d->baseline_valid &&
               ((float)sg4 < d->baseline * d->cfg.drop_ratio)) {
        cand = STALL_SG_DROP;
    }

    if (cand == STALL_NONE) {
        if (sg4 < d->min_sg) {
            d->min_sg = sg4;
        }
        if (d->baseline_valid) {
            d->baseline += ((float)sg4 - d->baseline) /
                           (float)(1UL << d->cfg.ema_shift);
        } else {
            d->acc_sum += sg4;
            d->acc_n++;
            if (d->acc_n >= d->cfg.baseline_samples) {
                const float avg = (float)d->acc_sum / (float)d->acc_n;
                if (avg >= (float)d->cfg.min_baseline) {
                    d->baseline = avg;
                    d->baseline_valid = true;
                } else {
                    d->acc_sum = 0U;
                    d->acc_n = 0U;
                }
            }
        }
    }

    if (!d->baseline_valid && (d->cfg.no_baseline_ms != 0U) &&
        (since_cruise_ms >= (int32_t)d->cfg.blank_ms + (int32_t)d->cfg.no_baseline_ms)) {
        d->candidate = STALL_NO_BASELINE;
        return STALL_NO_BASELINE;
    }

    if (cand != STALL_NONE) {
        if (d->hits < UINT8_MAX) {
            d->hits++;
        }
        d->candidate = cand;
        if (d->hits >= d->cfg.confirm) {
            return cand;
        }
    } else {
        d->hits = 0U;
        d->candidate = STALL_NONE;
    }
    return STALL_NONE;
}

uint8_t stall_det_hw_threshold(const stall_det_t *d)
{
    float thr;

    if (!d->baseline_valid) {
        return 0U;
    }
    thr = d->baseline * d->cfg.drop_ratio * 0.5f + 0.5f;
    if (thr < 1.0f) {
        thr = 1.0f;
    }
    if (thr > 255.0f) {
        thr = 255.0f;
    }
    return (uint8_t)thr;
}

const char *stall_source_name(stall_source_t s)
{
    switch (s) {
    case STALL_HW_FLAG:     return "sg4-flag";
    case STALL_SG_ABS:      return "sg4-abs";
    case STALL_SG_DROP:     return "sg4-drop";
    case STALL_NO_BASELINE: return "sg4-low";
    case STALL_ENCODER:     return "encoder";
    case STALL_NONE:
    default:                return "none";
    }
}
