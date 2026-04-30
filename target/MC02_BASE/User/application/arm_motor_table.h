/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include <stdint.h>

#include "config.h"

#define ARM_MOTOR_COUNT 6u

typedef enum
{
    ARM_MOTOR_DRIVER_J0 = 0u,
    ARM_MOTOR_DRIVER_CAN_MIT,
} arm_motor_driver_e;

typedef struct
{
    const char *name;
    uint8_t driver; // arm_motor_driver_e
    motor_node_param_t node;
    uint8_t bus; // fallback CAN bus; node.can_bus wins when set
    int8_t direction;
    uint16_t key_mask;
    fp32 key_speed_rad_s;
} arm_motor_entry_t;

extern const arm_motor_entry_t g_arm_motor_table[ARM_MOTOR_COUNT];
