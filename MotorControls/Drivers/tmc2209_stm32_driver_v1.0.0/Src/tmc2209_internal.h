#ifndef TMC2209_INTERNAL_H
#define TMC2209_INTERNAL_H
#include "tmc2209.h"
#include <stddef.h>

typedef struct {
    uint8_t address;
    bool readable;
    uint32_t write_mask;
    uint16_t mirror_bit;
    size_t mirror_offset;
    uint8_t mirror_size;
} tmc2209_reg_desc_t;

bool tmc2209_unit_ready(const tmc2209_unit_t *unit);
tmc2209_status_t tmc2209_check_faults(tmc2209_unit_t *unit);
const tmc2209_reg_desc_t *tmc2209_register(uint8_t reg);
tmc2209_status_t tmc2209_validate_value(const tmc2209_reg_desc_t *desc,
                                      uint32_t data);
void tmc2209_store_desired(tmc2209_unit_t *unit, const tmc2209_reg_desc_t *desc,
                          uint32_t value);
uint32_t tmc2209_load_desired(const tmc2209_unit_t *unit,
                            const tmc2209_reg_desc_t *desc);
#endif
