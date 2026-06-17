/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "ArmMsg.h"

// Runtime tuning knobs shared by arm motor drivers.
extern volatile uint8_t g_arm_deadman_hold_ctrl;
extern volatile fp32 g_arm_key_speed_scale;
extern volatile fp32 g_arm_key_kd;
extern volatile int16_t g_arm_j0_current;

// Single arm task entry and public runtime query API.
void ArmTask(void const *argument);
const ArmMotorFeedback *ArmGetFeedback(uint8_t index);
const ArmJ0UnitreeState *ArmJ0UnitreeGetState(void);
