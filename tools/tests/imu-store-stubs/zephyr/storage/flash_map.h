#ifndef TEST_ZEPHYR_FLASH_MAP_H
#define TEST_ZEPHYR_FLASH_MAP_H

#include <stdint.h>
#include <zephyr/drivers/flash.h>
#define PARTITION_ID(name) 1
#define PARTITION_SIZE(name) (0x40000u)
#define PARTITION_OFFSET(name) (0xc0000u)
int flash_area_open(uint8_t id, const struct flash_area **area);
void flash_area_close(const struct flash_area *area);

#endif
