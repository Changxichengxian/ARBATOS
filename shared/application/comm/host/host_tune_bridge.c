/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "host_tune_bridge.h"

#include <string.h>
#include "arm_math.h"
#include "gimbal_behaviour.h"

__weak volatile gimbal_behaviour_e gimbal_behaviour_watch = GIMBAL_ZERO_FORCE;

__weak void shoot_tune_apply_fric_speed_pid(void)
{
}

__weak void shoot_tune_apply_trigger_pid(void)
{
}

__weak const gimbal_motor_t *get_yaw_motor_point(void)
{
    return NULL;
}

__weak const gimbal_motor_t *get_pitch_motor_point(void)
{
    return NULL;
}

__weak void gimbal_tune_get_yaw_speed_pid(pid_param_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void gimbal_tune_get_yaw_angle_pid(pid_param_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void gimbal_tune_set_yaw_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void gimbal_tune_set_yaw_angle_pid(const pid_param_t *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void gimbal_tune_clear_yaw_pid(void)
{
}

__weak void gimbal_tune_get_pitch_speed_pid(pid_param_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void gimbal_tune_get_pitch_angle_pid(pid_param_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void gimbal_tune_set_pitch_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void gimbal_tune_set_pitch_angle_pid(const pid_param_t *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void gimbal_tune_clear_pitch_pid(void)
{
}

__weak const chassis_move_t *get_chassis_move_point(void)
{
    return NULL;
}

__weak void chassis_tune_get_follow_pid(pid_param_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void chassis_tune_set_follow_pid(const pid_param_t *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void chassis_tune_clear_follow_pid(void)
{
}

__weak void chassis_tune_get_motor_speed_pid(pid_param_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void chassis_tune_set_motor_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}
