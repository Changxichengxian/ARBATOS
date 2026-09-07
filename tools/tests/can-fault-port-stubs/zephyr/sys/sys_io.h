#ifndef TEST_SYS_IO_H
#define TEST_SYS_IO_H

#include <stdint.h>

uint32_t sys_read32(uintptr_t addr);
void sys_write32(uint32_t value, uintptr_t addr);

#endif
