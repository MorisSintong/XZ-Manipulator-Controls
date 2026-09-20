/* Independent Rev.2 device model. This file deliberately includes no driver
 * headers: register constants, access rules, CRC, and wire order are oracles.
 */
#include "model_device.h"
#include <string.h>

const SpecRegister model_spec[] = {
    {0x00,0x0001F7FEU,0x0001F7FEU,0x8U}, {0x01,0x1FU,0x1FU,0x1DU},
    {0x02,0xFFU,0U,0U}, {0x03,0xFFFU,0xFFFU,0U},
    {0x04,0xFF07FF7FU,0x1000U,0x40001000U},
    {0x0A,0x33U,0x33U,0U}, {0x0B,0xFFU,0xFFU,0U},
    {0x10,0x0F0F1F1FU,0x0F0F1F1FU,0x04011F08U},
    {0x11,0xFFU,0xFFU,10U}, {0x12,0xFFFFFU,0U,0U},
    {0x13,0xFFFFFU,0xFFFFFU,0U}, {0x14,0xFFFFFU,0xFFFFFU,0U},
    {0x15,0xFFFFFU,0xFFFFFU,0U}, {0x2D,0x01FF01FFU,0x01FF01FFU,0U},
    {0x38,0x5FFU,0x5FFU,0U}, {0x39,0xFFFFFFFFU,0xFFFFFFFFU,0U},
    {0x3A,0xFFFFFFFFU,0xFFFFFFFFU,0x10000U}, {0x3B,1U,1U,0U},
    {0x3C,0xFFFFFFFFU,0U,0U}, {0x50,0x1FFF1FFFU,0U,0U},
    {0x51,0x1FFFU,0U,0U}, {0x52,0x1FFF1FFFU,0x1FFF1FFFU,0x0B920F25U},
    {0x60,0xFFFFFFFFU,0xFFFFFFFFU,0xAAAAB554U},
    {0x61,0xFFFFFFFFU,0xFFFFFFFFU,0x4A9554AAU},
    {0x62,0xFFFFFFFFU,0xFFFFFFFFU,0x24492929U},
    {0x63,0xFFFFFFFFU,0xFFFFFFFFU,0x10104222U},
    {0x64,0xFFFFFFFFU,0xFFFFFFFFU,0xFBFFFFFFU},
    {0x65,0xFFFFFFFFU,0xFFFFFFFFU,0xB5BB777DU},
    {0x66,0xFFFFFFFFU,0xFFFFFFFFU,0x49295556U},
    {0x67,0xFFFFFFFFU,0xFFFFFFFFU,0x00404222U},
    {0x68,0xFFFFFFFFU,0xFFFFFFFFU,0xFFFF8056U},
    {0x69,0xFFFF00FFU,0xFFFF00FFU,0x00F70000U},
    {0x6A,0x3FFU,0U,0U}, {0x6B,0x01FF01FFU,0U,0U},
    {0x6C,0xFFFDDFFFU,0xFFFDDFFFU,0x10410150U},
    {0x6D,0x017FEF6FU,0x017FEF6FU,0U},
    {0x6F,0xFF1FF3FFU,0U,0U}, {0x70,0xFFFFFFFFU,0xFFFFFFFFU,0xC40C001DU},
    {0x71,0x01FF03FFU,0U,0U}, {0x72,0x00FF00FFU,0U,0U},
    {0x74,0x3FFU,0x3FFU,0x200U}, {0x75,0x3FFU,0U,0U},
    {0x76,0xFFFFFFFFU,0U,0U}
};
const size_t model_spec_count = sizeof(model_spec) / sizeof(model_spec[0]);
ModelDevice model_devices[4];

const SpecRegister *model_register(uint8_t address)
{
    for (size_t i = 0U; i < model_spec_count; ++i) {
        if (model_spec[i].address == address) {
            return &model_spec[i];
        }
    }
    return NULL;
}

void model_reset_device(unsigned id)
{
    ModelDevice *device = &model_devices[id];
    memset(device, 0, sizeof(*device));
    for (size_t i = 0U; i < model_spec_count; ++i) {
        device->reg[model_spec[i].address] = model_spec[i].reset;
    }
    device->reg[0x6F] = 0x80000000U;
    device->status_pipeline = 9U;
}

void model_reset_devices(void)
{
    for (unsigned i = 0U; i < 4U; ++i) {
        model_reset_device(i);
    }
}

uint32_t model_unpack(const uint8_t *bytes)
{
    uint32_t result = 0U;
    for (size_t i = 0U; i < 4U; ++i) {
        result = result * 256U + bytes[i];
    }
    return result;
}

void model_pack(uint8_t *bytes, uint32_t value)
{
    for (size_t i = 4U; i != 0U; --i) {
        bytes[i - 1U] = (uint8_t)(value % 256U);
        value /= 256U;
    }
}

void model_write(unsigned id, uint8_t address, uint32_t data)
{
    ModelDevice *device = &model_devices[id];
    const SpecRegister *reg = model_register(address);
    if ((reg == NULL) || device->ignore_write) {
        return;
    }
    ++device->writes;
    if ((address == 0x01U) || (address == 0x3BU)) {
        device->reg[address] &= ~(data & reg->writable);
        if (address == 0x01U) {
            device->reg[address] |= device->persistent_faults;
        }
    } else {
        device->reg[address] = (device->reg[address] & ~reg->writable) |
                               (data & reg->writable);
    }
    if ((address == 0x6CU) && ((data & 15U) != 0U)) {
        ++device->activations;
    }
}

void model_spi(unsigned id, const uint8_t tx[5], uint8_t rx[5])
{
    ModelDevice *device = &model_devices[id];
    uint8_t address = tx[0] & 127U;
    rx[0] = device->status_pipeline;
    model_pack(&rx[1], device->pipeline);
    if ((tx[0] & 128U) != 0U) {
        uint32_t data = model_unpack(&tx[1]);
        model_write(id, address, data);
        device->pipeline = data; /* Write response is an echo, not verification. */
    } else {
        device->pipeline = device->reg[address];
    }
    device->status_pipeline = (uint8_t)((device->reg[1] & 3U) |
        ((device->reg[0x6F] >> 22U) & 4U) | ((device->reg[0x6F] >> 28U) & 8U));
}

void model_n_event(unsigned id)
{
    ModelDevice *device = &model_devices[id];
    uint32_t mode = device->reg[0x38];
    if ((mode & 0x30U) != 0U) {
        device->reg[0x3C] = device->reg[0x39];
        device->reg[0x3B] |= 1U;
        if ((mode & 0x100U) != 0U) {
            device->reg[0x39] = 0U;
        }
        device->reg[0x38] &= ~0x20U;
    }
}

uint8_t model_crc(const uint8_t *data, size_t length)
{
    unsigned crc = 0U;
    for (size_t byte = 0U; byte < length; ++byte) {
        unsigned value = data[byte];
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            unsigned feedback = ((crc >> 7U) ^ value) & 1U;
            crc = (crc << 1U) & 255U;
            if (feedback != 0U) {
                crc ^= 7U;
            }
            value >>= 1U;
        }
    }
    return (uint8_t)crc;
}
