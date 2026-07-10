#ifndef MANUAL_INPUT_TEST_WATCH_H
#define MANUAL_INPUT_TEST_WATCH_H

#include <stdint.h>

#define WATCH_TASK_ELRS 0u

void WatchTaskWait(uint8_t taskId);
void WatchTaskBeat(uint8_t taskId);
void WatchTaskError(uint8_t taskId);
void WatchTaskTimeout(uint8_t taskId);

#endif
