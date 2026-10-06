#include "encoder_sampler.h"
#include <string.h>

void encoder_sampler_init(encoder_sampler_t *s, const motion_config_t *c, encoder_io_t io)
{
    memset(s, 0, sizeof(*s)); s->config = c; s->io = io;
    for (unsigned i = 0U; i < 2U; ++i) {
        s->axis[i].health_age_ms = UINT16_MAX;
    }
}

void encoder_sampler_config(encoder_sample_t *s, bool ok, uint16_t conf)
{
    if (!ok) { s->faults |= PH_I2C; s->valid = false; return; }
    s->conf = conf; s->conf_seen = true;
    if ((conf & ENCODER_CONF_MASK) != ENCODER_CONF_EXPECTED) {
        s->conf_fault = true; s->valid = false;
    }
}

void encoder_sampler_health(encoder_sample_t *s, bool ok, uint8_t status,
                            uint8_t agc, uint32_t now)
{
    if (!ok) { s->faults |= PH_I2C; s->valid = false; return; }
    s->status = status; s->agc = agc; s->health_us = now; s->health_seen = true;
    if ((status & 0x38U) != 0x20U) { s->faults |= PH_MAGNET; s->valid = false; }
}

bool encoder_sampler_accept(encoder_sample_t *s, uint16_t raw, uint32_t now,
                            uint32_t max_rate)
{
    const uint32_t age = s->health_seen ? (now - s->health_us) / 1000U : UINT32_MAX;
    s->health_age_ms = age > UINT16_MAX ? UINT16_MAX : (uint16_t)age;
    if (!s->conf_seen || s->conf_fault) { s->valid = false; return false; }
    if (raw > 4095U || !s->health_seen || age > 25U ||
        (s->status & 0x38U) != 0x20U) {
        s->valid = false;
        s->faults |= raw > 4095U ? PH_I2C : PH_MAGNET;
        return false;
    }
    if (s->seeded) {
        const uint32_t gap = now - s->timestamp_us;
        if (gap > s->max_gap_us) { s->max_gap_us = gap; }
        if (gap > s->status_gap_us) { s->status_gap_us = gap; }
        if (gap > 10000U || (uint64_t)max_rate * gap >= 800000000ULL) {
            s->faults |= PH_OVERRUN; s->valid = false;
        }
    }
    if (s->faults != 0U) { return false; }
    if (!s->seeded) {
        enc_unwrap_init(&s->unwrap, raw);
        s->seeded = true;
    } else {
        enc_unwrap_t candidate = s->unwrap;
        (void)enc_unwrap_update(&candidate, raw);
        if (!candidate.initialized || candidate.suspect_alias || candidate.overflow) {
            s->faults |= PH_OVERRUN; s->valid = false; return false;
        }
        s->unwrap = candidate;
    }
    s->raw = raw; s->timestamp_us = now; s->valid = true;
    return true;
}

bool encoder_sampler_fresh(const encoder_sample_t *s, uint32_t now)
{
    return s->valid && s->conf_seen && !s->conf_fault && s->faults == 0U &&
           now - s->timestamp_us <= 10000U &&
           s->health_seen && now - s->health_us <= 25000U;
}

void encoder_sampler_poll(encoder_sampler_t *s, bool endpoint)
{
    for (uint8_t i = 0U; i < 2U; ++i) {
        encoder_sample_t *a = &s->axis[i];
        uint32_t now = s->io.now_us(s->io.context);
        if (endpoint || !a->health_seen || now - a->last_health_us >= 20000U) {
            uint8_t status = 0U, agc = 0U;
            uint16_t conf = 0U;
            a->last_health_us = now;
            const bool ok = s->io.health(s->io.context, i, &status, &agc);
            const bool conf_ok = s->io.conf(s->io.context, i, &conf);
            now = s->io.now_us(s->io.context);
            encoder_sampler_config(a, conf_ok, conf);
            encoder_sampler_health(a, ok, status, agc, now);
        }
        if (endpoint || !a->seeded || now - a->last_poll_us >= 2000U) {
            uint16_t raw;
            a->last_poll_us = now;
            const bool ok = s->io.raw(s->io.context, i, &raw);
            now = s->io.now_us(s->io.context);
            if (!ok) { a->faults |= PH_I2C; a->valid = false; }
            else { (void)encoder_sampler_accept(a, raw, now, s->config->axis[i].max_rate); }
        }
        if (a->seeded && now - a->timestamp_us > 10000U) {
            a->faults |= PH_OVERRUN; a->valid = false;
        }
    }
}

bool encoder_sampler_reseed(encoder_sampler_t *s, uint8_t i)
{
    uint16_t raw;
    uint16_t conf = 0U;
    uint8_t status, agc;
    encoder_sample_t *a = &s->axis[i];
    const bool health_ok = s->io.health(s->io.context, i, &status, &agc);
    const bool conf_ok = s->io.conf(s->io.context, i, &conf);
    encoder_sampler_config(a, conf_ok, conf);
    if (!health_ok || !conf_ok || (conf & ENCODER_CONF_MASK) != ENCODER_CONF_EXPECTED ||
        (status & 0x38U) != 0x20U || !s->io.raw(s->io.context, i, &raw) || raw > 4095U) {
        a->valid = false;
        return false;
    }
    const uint32_t now = s->io.now_us(s->io.context);
    memset(a, 0, sizeof(*a));
    encoder_sampler_config(a, true, conf);
    encoder_sampler_health(a, true, status, agc, now);
    return encoder_sampler_accept(a, raw, now, s->config->axis[i].max_rate);
}
