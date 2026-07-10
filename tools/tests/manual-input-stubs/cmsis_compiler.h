#ifndef TEST_CMSIS_COMPILER_H
#define TEST_CMSIS_COMPILER_H

#ifndef __DMB
#define __DMB() __asm__ __volatile__("" ::: "memory")
#endif

#endif
