/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef ACTUATOR_CMD_H
#define ACTUATOR_CMD_H

#include "struct_typedef.h"

// Shared actuator set currents (written by control tasks, sent by CAN TX task).
// NOTE: index is 0~3; out-of-range indices are ignored / return 0.
void actuator_cmd_set_chassis_current_can1(uint8_t index, int16_t current);
int16_t actuator_cmd_get_chassis_current_can1(uint8_t index);

void actuator_cmd_set_yaw_current_can1(int16_t current);
int16_t actuator_cmd_get_yaw_current_can1(void);

void actuator_cmd_set_yaw_upper_current_can1(int16_t current);
int16_t actuator_cmd_get_yaw_upper_current_can1(void);

void actuator_cmd_set_pitch_current_can1(int16_t current);
int16_t actuator_cmd_get_pitch_current_can1(void);

void actuator_cmd_set_trigger_current_can1(int16_t current);
int16_t actuator_cmd_get_trigger_current_can1(void);

// CAN2 friction motors (0x201~0x204 on CAN2 0x200 group).
void actuator_cmd_set_friction_current_can2(uint8_t index, int16_t current);
int16_t actuator_cmd_get_friction_current_can2(uint8_t index);

#endif
