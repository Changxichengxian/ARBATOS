#ifndef MANUAL_INPUT_TEST_DETECT_TASK_H
#define MANUAL_INPUT_TEST_DETECT_TASK_H

#include <stdint.h>

#define DBUS_TOE 0u

void DetectHook(uint8_t toe);
uint8_t toe_is_error(uint8_t toe);

#endif
