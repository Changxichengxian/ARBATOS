/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_SAFETY_H
#define ROBOT_SAFETY_H

#include <stdint.h>

#include "robot_lifecycle.h"

static inline uint8_t robot_safety_manual_disconnected(void)
{
    robot_lifecycle_snapshot_t snapshot;

    (void)robot_lifecycle_get_snapshot(&snapshot);
    return (uint8_t)((snapshot.manual_online == 0u) ? 1u : 0u);
}

static inline uint8_t robot_safety_manual_safe_active(void)
{
    robot_lifecycle_snapshot_t snapshot;

    (void)robot_lifecycle_get_snapshot(&snapshot);
    return snapshot.manual_safe;
}

static inline uint8_t robot_safety_output_locked(void)
{
    return (uint8_t)((robot_lifecycle_output_allowed() == 0u) ? 1u : 0u);
}

#endif
