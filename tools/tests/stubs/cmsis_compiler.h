#ifndef HOST_TEST_CMSIS_COMPILER_H
#define HOST_TEST_CMSIS_COMPILER_H

#include <stdint.h>

static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __set_PRIMASK(uint32_t value) { (void)value; }

#endif
