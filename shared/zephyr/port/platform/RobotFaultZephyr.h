/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ROBOT_FAULT_ZEPHYR_H
#define ROBOT_FAULT_ZEPHYR_H
#include <stdint.h>

uint8_t RobotFaultZephyrBootLocked(void);
void RobotFaultZephyrObserveTime(uint32_t tickMs);
__attribute__((noreturn)) void RobotFaultZephyrReset(uint32_t reason, uint32_t arg0, uint32_t arg1);

#endif
