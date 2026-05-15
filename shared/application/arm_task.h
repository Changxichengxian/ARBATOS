/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "arm_msg.h"

// Runtime tuning knobs shared by arm motor drivers.
extern volatile uint8_t g_arm_deadman_hold_ctrl;
extern volatile fp32 g_arm_key_speed_scale;
extern volatile fp32 g_arm_key_kd;
extern volatile int16_t g_arm_j0_current;

// Single arm task entry and public runtime query API.
void arm_task(void const *argument);
const arm_motor_feedback_t *arm_get_feedback(uint8_t index);
const arm_j0_unitree_state_t *arm_j0_unitree_get_state(void);
