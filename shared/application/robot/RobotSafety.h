/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_SAFETY_H
#define ROBOT_SAFETY_H

#include <stdint.h>

#include "RobotLifecycle.h"

static inline uint8_t RobotSafetyManualDisconnected(void)
{
    RobotLifecycleSnapshot snapshot;

    (void)RobotLifecycleGetSnapshot(&snapshot);
    return (uint8_t)((snapshot.manual_online == 0u) ? 1u : 0u);
}

static inline uint8_t RobotSafetyManualSafeActive(void)
{
    RobotLifecycleSnapshot snapshot;

    (void)RobotLifecycleGetSnapshot(&snapshot);
    return snapshot.manual_safe;
}

static inline uint8_t RobotSafetyOutputLocked(void)
{
    return (uint8_t)((RobotLifecycleOutputAllowed() == 0u) ? 1u : 0u);
}

#endif
