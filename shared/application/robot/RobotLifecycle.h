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
} RobotLifecycleState;

typedef enum
{
    ROBOT_LIFECYCLE_REASON_BOOT = 0u,
    ROBOT_LIFECYCLE_REASON_NONE,
    ROBOT_LIFECYCLE_REASON_MANUAL_OFFLINE,
    ROBOT_LIFECYCLE_REASON_MANUAL_SAFE_SWITCH,
    ROBOT_LIFECYCLE_REASON_FAULT_LATCHED,
    ROBOT_LIFECYCLE_REASON_FATAL_FAULT,
    ROBOT_LIFECYCLE_REASON_STARTUP_SAFE_REQUIRED,
} RobotLifecycleReason;

typedef struct
{
    RobotLifecycleState state;
    RobotLifecycleState prev_state;
    RobotLifecycleReason reason;
    uint32_t enter_tick;
    uint32_t transition_count;
    uint8_t output_allowed;
    uint8_t manual_online;
    uint8_t manual_safe;
    uint8_t fault_latched;
    uint8_t startup_safe_seen;
} RobotLifecycleSnapshot;

void RobotLifecycleInit(void);
void RobotLifecycleUpdate(void);
RobotLifecycleState RobotLifecycleCurrent(void);
uint8_t RobotLifecycleOutputAllowed(void);
uint8_t RobotLifecycleGetSnapshot(RobotLifecycleSnapshot *out);
void RobotLifecycleEnterFault(RobotLifecycleReason reason);
void RobotLifecycleClearFault(void);
uint8_t RobotLifecycleFaultLatched(void);
const char *RobotLifecycleName(RobotLifecycleState state);

#endif
