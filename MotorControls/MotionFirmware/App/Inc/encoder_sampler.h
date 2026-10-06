#ifndef ENCODER_SAMPLER_H
#define ENCODER_SAMPLER_H
#include "encoder_unwrap.h"
#include "motion_config.h"
#include "diag_record.h"
#define ENCODER_CONF_MASK 0x3FFFU
#define ENCODER_CONF_EXPECTED 0x0300U
typedef struct {
    void *context;
    bool (*raw)(void *, uint8_t, uint16_t *);
    bool (*health)(void *, uint8_t, uint8_t *, uint8_t *);
    uint32_t (*now_us)(void *);
    uint32_t (*now_ms)(void *);
    bool (*conf)(void *, uint8_t, uint16_t *);
} encoder_io_t;
typedef struct {
    enc_unwrap_t unwrap;
    uint16_t raw, conf, health_age_ms;
    uint8_t status, agc;
    uint32_t timestamp_us, health_us, last_poll_us, last_health_us, max_gap_us, status_gap_us;
    uint16_t faults;
    bool seeded, health_seen, valid, conf_seen, conf_fault;
    int32_t home_offset;
} encoder_sample_t;
typedef struct {
    encoder_sample_t axis[2];
    encoder_io_t io;
    const motion_config_t *config;
} encoder_sampler_t;
void encoder_sampler_init(encoder_sampler_t *, const motion_config_t *, encoder_io_t);
void encoder_sampler_poll(encoder_sampler_t *, bool endpoint_health);
bool encoder_sampler_accept(encoder_sample_t *, uint16_t, uint32_t, uint32_t);
void encoder_sampler_health(encoder_sample_t *, bool, uint8_t, uint8_t, uint32_t);
void encoder_sampler_config(encoder_sample_t *, bool, uint16_t readback);
bool encoder_sampler_reseed(encoder_sampler_t *, uint8_t);
bool encoder_sampler_fresh(const encoder_sample_t *, uint32_t);
#endif
