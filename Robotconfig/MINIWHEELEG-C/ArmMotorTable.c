/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ArmMotorTable.h"

#include "ManualInput.h"

/*
 * This table only describes arm-side metadata: joint names, key binding,
 * direction and the default bus. The actual motor node for J0..J5 lives in
 * g_config.motor.arm[0..5].
 */
const ArmMotorEntry g_arm_motor_table[ARM_MOTOR_COUNT] =
{
    {
        .name = "ArmJ0Base",
        .driver = ARM_MOTOR_DRIVER_J0,
        .fallback_bus = 1u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_G,
        .key_speed_rad_s = 0.0f,
    },
    {
        .name = "ArmJ1Base",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_Z,
        .key_speed_rad_s = 1.2f,
    },
    {
        .name = "ArmJ2Shoulder",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_X,
        .key_speed_rad_s = 1.4f,
    },
    {
        .name = "ArmJ3Elbow",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_C,
        .key_speed_rad_s = 1.8f,
    },
    {
        .name = "ArmJ4WristPitch",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_V,
        .key_speed_rad_s = 2.2f,
    },
    {
        .name = "ArmJ5WristRoll",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .fallback_bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_B,
        .key_speed_rad_s = 2.5f,
    },
};
