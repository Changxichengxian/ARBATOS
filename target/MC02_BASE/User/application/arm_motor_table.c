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
 * 这张表只描述“机械臂哪一轴接了什么执行器”。
 * 上层 arm_task 只遍历表，不直接关心底层到底是 CAN MIT、CAN current 还是 RS485。
 *
 * 当前约定:
 * - G/Z/X/C/V/B: 依次控制 J0~J5
 * - J0 走一个动态底层:
 *   - arm.j0_unitree.enable=0: 复用 CAN1 yaw current 通道
 *   - arm.j0_unitree.enable=1: 切到 Unitree RS485 驱动
 * - J1~J5 继续走 DM MIT
 */
const arm_motor_entry_t g_arm_motor_table[ARM_MOTOR_COUNT] =
{
    {
        .name = "arm_j0_base",
        .driver = ARM_MOTOR_DRIVER_J0,
        .node = {
            .model = MOTOR_MODEL_UNITREE_GO_M8010_6,
            .can_id = 1u,
            .protocol = (uint8_t)MOTOR_PROTOCOL_UNITREE_RS485,
            .control_mode = (uint8_t)MOTOR_CONTROL_MODE_MIT,
            .master_id = 0u,
            .transport = (uint8_t)MOTOR_TRANSPORT_RS485,
            .rs485_port = 0u,
            .baudrate = 4000000u,
            .rx_timeout_ms = 30u,
        },
        .bus = 1u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_G,
        .key_speed_rad_s = 0.0f,
    },
    {
        .name = "arm_j1_base",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .node = {
            .model = MOTOR_MODEL_DM_J8009_2EC_V10,
            .can_id = 1u,
            .can_bus = 2u,
            .protocol = (uint8_t)MOTOR_PROTOCOL_INHERIT,
            .control_mode = (uint8_t)MOTOR_CONTROL_MODE_INHERIT,
            .master_id = 0u,
            .transport = (uint8_t)MOTOR_TRANSPORT_CAN,
        },
        .bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_Z,
        .key_speed_rad_s = 1.2f,
    },
    {
        .name = "arm_j2_shoulder",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .node = {
            .model = MOTOR_MODEL_DM_J8006_2EC_V10,
            .can_id = 2u,
            .can_bus = 2u,
            .protocol = (uint8_t)MOTOR_PROTOCOL_INHERIT,
            .control_mode = (uint8_t)MOTOR_CONTROL_MODE_INHERIT,
            .master_id = 0u,
            .transport = (uint8_t)MOTOR_TRANSPORT_CAN,
        },
        .bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_X,
        .key_speed_rad_s = 1.4f,
    },
    {
        .name = "arm_j3_elbow",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .node = {
            .model = MOTOR_MODEL_DM_J4310_2EC_V11,
            .can_id = 3u,
            .can_bus = 2u,
            .protocol = (uint8_t)MOTOR_PROTOCOL_INHERIT,
            .control_mode = (uint8_t)MOTOR_CONTROL_MODE_INHERIT,
            .master_id = 0u,
            .transport = (uint8_t)MOTOR_TRANSPORT_CAN,
        },
        .bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_C,
        .key_speed_rad_s = 1.8f,
    },
    {
        .name = "arm_j4_wrist_pitch",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .node = {
            .model = MOTOR_MODEL_DM_J4310_2EC_V12,
            .can_id = 4u,
            .can_bus = 2u,
            .protocol = (uint8_t)MOTOR_PROTOCOL_INHERIT,
            .control_mode = (uint8_t)MOTOR_CONTROL_MODE_INHERIT,
            .master_id = 0u,
            .transport = (uint8_t)MOTOR_TRANSPORT_CAN,
        },
        .bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_V,
        .key_speed_rad_s = 2.2f,
    },
    {
        .name = "arm_j5_wrist_roll",
        .driver = ARM_MOTOR_DRIVER_CAN_MIT,
        .node = {
            .model = MOTOR_MODEL_DM_J4310_2EC_V12,
            .can_id = 5u,
            .can_bus = 2u,
            .protocol = (uint8_t)MOTOR_PROTOCOL_INHERIT,
            .control_mode = (uint8_t)MOTOR_CONTROL_MODE_INHERIT,
            .master_id = 0u,
            .transport = (uint8_t)MOTOR_TRANSPORT_CAN,
        },
        .bus = 2u,
        .direction = 1,
        .key_mask = KEY_PRESSED_OFFSET_B,
        .key_speed_rad_s = 2.5f,
    },
};
