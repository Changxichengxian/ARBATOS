/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include <stdint.h>

#include "RobotConfig.h"

#define ARM_MOTOR_COUNT MOTOR_ARM_JOINT_COUNT

typedef enum
{
    ARM_MOTOR_DRIVER_J0 = 0u,
    ARM_MOTOR_DRIVER_CAN_MIT,
} ArmMotorDriver;

typedef struct
{
    const char *name;
    uint8_t driver; // ArmMotorDriver
    uint8_t fallback_bus;
    int8_t direction;
    uint16_t key_mask;
    fp32 key_speed_rad_s;
} ArmMotorEntry;

extern const ArmMotorEntry g_arm_motor_table[ARM_MOTOR_COUNT];
