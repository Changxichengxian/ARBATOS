#ifndef TEST_ZEPHYR_CRC_H
#define TEST_ZEPHYR_CRC_H

#include <stddef.h>
#include <stdint.h>
uint32_t crc32_ieee(const uint8_t *data, size_t length);

#endif
