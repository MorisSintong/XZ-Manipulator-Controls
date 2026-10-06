#ifndef APP_CRC16_CCITT_H
#define APP_CRC16_CCITT_H

#include <stddef.h>
#include <stdint.h>

/** CCITT-FALSE, initial value 0xFFFF; data may be NULL only when len is zero. */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);
/** Extend a CCITT-FALSE checksum by one byte. */
uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte);

#endif
