#ifndef TEST_ZEPHYR_FATAL_H
#define TEST_ZEPHYR_FATAL_H
#include <stdint.h>
struct arch_esf_basic { uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr; };
struct arch_esf { struct arch_esf_basic basic; };
enum { K_ERR_CPU_EXCEPTION = 1, K_ERR_STACK_CHK_FAIL = 2, K_ERR_SPURIOUS_IRQ = 3,
       K_ERR_KERNEL_PANIC = 4, K_ERR_ARCH_START = 16 };
#endif
