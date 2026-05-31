/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Chen Xuan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef HOST_TUNE_BRIDGE_H
#define HOST_TUNE_BRIDGE_H

#include "chassis_control_task.h"
#include "config.h"
#include "gimbal_control_task.h"

void shoot_tune_apply_fric_speed_pid(void);
void shoot_tune_apply_trigger_pid(void);

const gimbal_motor_t *get_yaw_motor_point(void);
const gimbal_motor_t *get_pitch_motor_point(void);
void gimbal_tune_get_yaw_speed_pid(pid_param_t *out);
void gimbal_tune_get_yaw_angle_pid(pid_param_t *out);
void gimbal_tune_set_yaw_speed_pid(const pid_param_t *pid, bool_t clear_state);
void gimbal_tune_set_yaw_angle_pid(const pid_param_t *pid, bool_t clear_state);
void gimbal_tune_clear_yaw_pid(void);
void gimbal_tune_get_pitch_speed_pid(pid_param_t *out);
void gimbal_tune_get_pitch_angle_pid(pid_param_t *out);
void gimbal_tune_set_pitch_speed_pid(const pid_param_t *pid, bool_t clear_state);
void gimbal_tune_set_pitch_angle_pid(const pid_param_t *pid, bool_t clear_state);
void gimbal_tune_clear_pitch_pid(void);

const chassis_move_t *get_chassis_move_point(void);
void chassis_tune_get_follow_pid(pid_param_t *out);
void chassis_tune_set_follow_pid(const pid_param_t *pid, bool_t clear_state);
void chassis_tune_clear_follow_pid(void);
void chassis_tune_get_motor_speed_pid(pid_param_t *out);
void chassis_tune_set_motor_speed_pid(const pid_param_t *pid, bool_t clear_state);

#endif
