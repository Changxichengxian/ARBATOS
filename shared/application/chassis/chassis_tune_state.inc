/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by chassis_control_task.c.
 */

const chassis_move_t *get_chassis_move_point(void)
{
    return &chassis_move;
}

void chassis_tune_get_follow_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }
    out->kp = chassis_move.chassis_angle_pid.Kp;
    out->ki = chassis_move.chassis_angle_pid.Ki;
    out->kd = chassis_move.chassis_angle_pid.Kd;
    out->max_out = chassis_move.chassis_angle_pid.max_out;
    out->max_iout = chassis_move.chassis_angle_pid.max_iout;
}

void chassis_tune_set_follow_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    chassis_move.chassis_angle_pid.Kp = pid->kp;
    chassis_move.chassis_angle_pid.Ki = pid->ki;
    chassis_move.chassis_angle_pid.Kd = pid->kd;
    chassis_move.chassis_angle_pid.max_out = pid->max_out;
    chassis_move.chassis_angle_pid.max_iout = pid->max_iout;

    if (clear_state)
    {
        PID_clear(&chassis_move.chassis_angle_pid);
    }
}

void chassis_tune_clear_follow_pid(void)
{
    PID_clear(&chassis_move.chassis_angle_pid);
}

void chassis_tune_get_motor_speed_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    const pid_type_def *pid = &chassis_move.motor_speed_pid[0];
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
}

void chassis_tune_set_motor_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        pid_type_def *dst = &chassis_move.motor_speed_pid[i];
        dst->Kp = pid->kp;
        dst->Ki = pid->ki;
        dst->Kd = pid->kd;
        dst->max_out = pid->max_out;
        dst->max_iout = pid->max_iout;
        if (clear_state)
        {
            PID_clear(dst);
        }
    }
    taskEXIT_CRITICAL();
}

static void chassis_write_state(const chassis_move_t *control)
{
    if (control == NULL)
    {
        return;
    }

    chassis_state_t state = {0};
    state.valid = 1u;
    state.mode = (uint8_t)control->chassis_mode;
    state.last_mode = (uint8_t)control->last_chassis_mode;
    state.vx = control->vx;
    state.vy = control->vy;
    state.wz = control->wz;
    state.vx_set = control->vx_set;
    state.vy_set = control->vy_set;
    state.wz_set = control->wz_set;
    state.chassis_yaw_offset = control->chassis_yaw_offset;
    state.chassis_yaw_offset_set = control->chassis_yaw_offset_set;
    state.chassis_yaw_set = control->chassis_yaw_set;
    state.chassis_yaw = control->chassis_yaw;
    state.chassis_pitch = control->chassis_pitch;
    state.chassis_roll = control->chassis_roll;
    state.angle_pid = control->chassis_angle_pid;

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT && i < CHASSIS_STATE_MOTOR_COUNT; i++)
    {
        chassis_fill_motor_measure_state(&state.motor[i].measure, control->motor_chassis[i].chassis_motor_measure);
        state.motor[i].accel = control->motor_chassis[i].accel;
        state.motor[i].speed = control->motor_chassis[i].speed;
        state.motor[i].speed_set = control->motor_chassis[i].speed_set;
        state.motor[i].give_current = control->motor_chassis[i].give_current;
    }

    (void)chassis_state_write(&state);
}
