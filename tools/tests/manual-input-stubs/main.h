#ifndef MANUAL_INPUT_TEST_MAIN_H
#define MANUAL_INPUT_TEST_MAIN_H

#include <stdint.h>

uint32_t __get_IPSR(void);
void ManualInputTestDmb(void);

#define __DMB() ManualInputTestDmb()

#endif
