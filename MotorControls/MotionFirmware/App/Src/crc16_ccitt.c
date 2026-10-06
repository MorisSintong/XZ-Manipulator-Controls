#include "crc16_ccitt.h"

uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte)
{
    crc = (uint16_t)(crc ^ (uint16_t)((uint16_t)byte << 8U));
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        uint32_t next = (uint32_t)crc << 1U;
        if ((crc & UINT16_C(0x8000)) != 0U) {
            next ^= UINT32_C(0x1021);
        }
        crc = (uint16_t)next;
    }
    return crc;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = UINT16_C(0xFFFF);
    for (size_t i = 0U; i < len; ++i) {
        crc = crc16_ccitt_update(crc, data[i]);
    }
    return crc;
}
