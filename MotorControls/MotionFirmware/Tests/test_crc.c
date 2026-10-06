#include "crc16_ccitt.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    const uint8_t standard[] = "123456789";
    const uint8_t payload[] = {
        0x01U, 0x02U, 0x01U, 0x01U, 0xE8U, 0x03U,
        0xC4U, 0x09U, 0x08U, 0x07U, 0xF0U, 0xFFU
    };
    uint8_t range[256];
    uint8_t zeros[12] = {0};
    uint8_t ones[12];
    uint16_t stream = 0xFFFFU;
    assert(crc16_ccitt(NULL, 0U) == 0xFFFFU);
    assert(crc16_ccitt(standard, 9U) == 0x29B1U);
    assert(crc16_ccitt(payload, sizeof(payload)) == 0x08B7U);
    for (size_t i = 0U; i < sizeof(range); ++i) {
        range[i] = (uint8_t)i;
        stream = crc16_ccitt_update(stream, range[i]);
        assert(stream == crc16_ccitt(range, i + 1U));
    }
    for (size_t i = 0U; i < sizeof(ones); ++i) {
        ones[i] = 0xFFU;
    }
    assert(crc16_ccitt(range, sizeof(range)) == 0x3FBDU);
    assert(stream == 0x3FBDU);
    assert(crc16_ccitt(zeros, sizeof(zeros)) == 0x84F9U);
    assert(crc16_ccitt(ones, sizeof(ones)) == 0x47D8U);
    puts("CRC: six Python reference vectors and 256 streaming prefixes PASS");
    return 0;
}
