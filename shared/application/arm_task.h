/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "struct_typedef.h"

typedef struct
{
    uint8_t online;
    uint8_t rx_dlc;
    uint16_t rx_id;
    uint32_t rx_count;
    uint32_t last_rx_tick;
    fp32 position;
    fp32 velocity;
    fp32 torque;
} arm_motor_feedback_t;

typedef struct
{
    uint8_t enabled;
    uint8_t rs485_port;
    uint8_t motor_id;
    uint8_t online;
    uint8_t last_mode;
    uint8_t motor_error;
    int8_t motor_temp;
    uint8_t last_tx_status;
    uint32_t tx_count;
    uint32_t tx_fail_count;
    uint32_t rx_frame_count;
    uint32_t rx_crc_fail_count;
    uint32_t rx_parse_error_count;
    uint32_t last_rx_tick_ms;
    fp32 cmd_output_speed_rad_s;
    fp32 cmd_output_kd;
    fp32 torque_nm;
    fp32 joint_speed_rad_s;
    fp32 joint_position_rad;
} arm_j0_unitree_state_t;

// Runtime tuning knobs shared by arm motor drivers.
extern volatile uint8_t g_arm_deadman_hold_ctrl;
extern volatile fp32 g_arm_key_speed_scale;
extern volatile fp32 g_arm_key_kd;
extern volatile int16_t g_arm_j0_current;

// Single arm task entry and public runtime query interface.
void arm_task(void const *argument);
const arm_motor_feedback_t *arm_get_feedback(uint8_t index);
const arm_j0_unitree_state_t *arm_j0_unitree_get_state(void);
