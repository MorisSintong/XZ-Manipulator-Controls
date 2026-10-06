#ifndef DRIVER_EVIDENCE_H
#define DRIVER_EVIDENCE_H
#include "motion_config.h"
typedef struct {
    uint32_t status, sg_ind, tstep;
    uint16_t mscnt, sg;
    uint8_t gstat, spi_status, mode;
    bool valid, fault;
} driver_sample_t;
typedef struct {
    void *context;
    bool (*read)(void *, uint8_t, driver_sample_t *);
} driver_io_t;
typedef struct {
    driver_sample_t axis[2];
    driver_io_t io;
} driver_evidence_t;
void driver_evidence_init(driver_evidence_t *, driver_io_t);
bool driver_evidence_poll(driver_evidence_t *, uint8_t);
#endif
