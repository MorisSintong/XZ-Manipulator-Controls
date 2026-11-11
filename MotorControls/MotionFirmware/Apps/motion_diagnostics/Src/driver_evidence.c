#include "driver_evidence.h"
#include <string.h>

void driver_evidence_init(driver_evidence_t *d, driver_io_t io)
{
    memset(d, 0, sizeof(*d)); d->io = io;
}
bool driver_evidence_poll(driver_evidence_t *d, uint8_t axis)
{
    driver_sample_t sample = {0};
    const bool ok = d->io.read(d->io.context, axis, &sample);
    sample.valid = ok;
    /* Short, overtemperature and supply/reset faults invalidate coordinates.
     * Open-load bits are not trusted outside their specified operating regime. */
    sample.fault = sample.fault || !ok || sample.gstat != 0U ||
        (sample.status & 0x1E003000U) != 0U || sample.mode != 4U;
    d->axis[axis] = sample;
    return ok && !sample.fault;
}
