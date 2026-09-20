#ifndef TMC2240_MODEL_DEVICE_H
#define TMC2240_MODEL_DEVICE_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint8_t address;
    uint32_t readable;
    uint32_t writable;
    uint32_t reset;
} SpecRegister;
extern const SpecRegister model_spec[];
extern const size_t model_spec_count;
const SpecRegister *model_register(uint8_t address);

typedef struct {
    uint32_t reg[128];
    uint32_t pipeline;
    uint8_t status_pipeline;
    uint32_t writes;
    uint32_t activations;
    bool ignore_write;
    uint32_t persistent_faults;
} ModelDevice;
extern ModelDevice model_devices[4];
void model_reset_devices(void);
void model_reset_device(unsigned id);
void model_spi(unsigned id, const uint8_t tx[5], uint8_t rx[5]);
void model_write(unsigned id, uint8_t address, uint32_t data);
void model_n_event(unsigned id);
uint8_t model_crc(const uint8_t *data, size_t length);
uint32_t model_unpack(const uint8_t *bytes);
void model_pack(uint8_t *bytes, uint32_t value);
#endif
