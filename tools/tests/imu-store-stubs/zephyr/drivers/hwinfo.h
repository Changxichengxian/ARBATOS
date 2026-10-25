#ifndef TEST_ZEPHYR_HWINFO_H
#define TEST_ZEPHYR_HWINFO_H

#include <stddef.h>
#include <stdint.h>
int hwinfo_get_device_id(uint8_t *buffer, size_t length);

#endif
