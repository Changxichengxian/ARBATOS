/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_LIFECYCLE_H
#define ROBOT_LIFECYCLE_H

#include <stdint.h>

typedef enum
{
    ROBOT_LIFECYCLE_BOOT = 0u,
    ROBOT_LIFECYCLE_SAFE,
    ROBOT_LIFECYCLE_ACTIVE,
    ROBOT_LIFECYCLE_FAULT,
} robot_lifecycle_state_e;

typedef enum
{
    ROBOT_LIFECYCLE_REASON_BOOT = 0u,
    ROBOT_LIFECYCLE_REASON_NONE,
    ROBOT_LIFECYCLE_REASON_MANUAL_OFFLINE,
    ROBOT_LIFECYCLE_REASON_MANUAL_SAFE_SWITCH,
    ROBOT_LIFECYCLE_REASON_FAULT_LATCHED,
    ROBOT_LIFECYCLE_REASON_FATAL_FAULT,
} robot_lifecycle_reason_e;

typedef struct
{
    robot_lifecycle_state_e state;
    robot_lifecycle_state_e prev_state;
    robot_lifecycle_reason_e reason;
    uint32_t enter_tick;
    uint32_t transition_count;
    uint8_t output_allowed;
    uint8_t manual_online;
    uint8_t manual_safe;
    uint8_t fault_latched;
} robot_lifecycle_snapshot_t;

void robot_lifecycle_init(void);
void robot_lifecycle_update(void);
robot_lifecycle_state_e robot_lifecycle_current(void);
uint8_t robot_lifecycle_output_allowed(void);
uint8_t robot_lifecycle_get_snapshot(robot_lifecycle_snapshot_t *out);
void robot_lifecycle_enter_fault(robot_lifecycle_reason_e reason);
void robot_lifecycle_clear_fault(void);
uint8_t robot_lifecycle_fault_latched(void);
const char *robot_lifecycle_name(robot_lifecycle_state_e state);

#endif
