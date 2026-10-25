#ifndef TEST_ZEPHYR_FLASH_H
#define TEST_ZEPHYR_FLASH_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
struct flash_area { unsigned marker; };
int flash_area_read(const struct flash_area *area, off_t offset, void *data, size_t length);
int flash_area_write(const struct flash_area *area, off_t offset, const void *data, size_t length);
int flash_area_erase(const struct flash_area *area, off_t offset, size_t length);
size_t flash_area_align(const struct flash_area *area);

#endif
