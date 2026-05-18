/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "arm_motor_table.h"

#include "manual_input.h"

/*
 * This table only describes arm-side metadata: joint names, key binding,
 * direction and the default bus. The actual motor node for J0..J5 lives in
 * g_config.motor.arm[0..5].
 */
const arm_motor_entry_t g_arm_motor_table[ARM_MOTOR_COUNT] =
{
    {
        .name = "arm_j0_base",
        .driver = ARM_MOTOR_DRIVER_J0,
        .fallback_bus = 1u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_G,
        .key_speed_rad_s = 0.0f,
    },
    {
        .name = "arm_j1_base",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_Z,
        .key_speed_rad_s = 1.2f,
    },
    {
        .name = "arm_j2_shoulder",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_X,
        .key_speed_rad_s = 1.4f,
    },
    {
        .name = "arm_j3_elbow",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_C,
        .key_speed_rad_s = 1.8f,
    },
    {
        .name = "arm_j4_wrist_pitch",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_V,
        .key_speed_rad_s = 2.2f,
    },
    {
        .name = "arm_j5_wrist_roll",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_B,
        .key_speed_rad_s = 2.5f,
    },
};
