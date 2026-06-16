/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by gimbal_control_task.c.
 */

/**
  * @brief          "gimbal_control" variable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" variable point
  * @retval         none
  */
void gimbal_tune_get_yaw_speed_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const pid_type_def *pid = &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_get_yaw_angle_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const gimbal_PID_t *pid = &gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_yaw_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid_type_def *dst = &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid;
    dst->Kp = pid->kp;
    dst->Ki = pid->ki;
    dst->Kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_yaw_angle_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    gimbal_PID_t *dst = &gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid;
    dst->kp = pid->kp;
    dst->ki = pid->ki;
    dst->kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        gimbal_PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_clear_yaw_pid(void)
{
    taskENTER_CRITICAL();
    gimbal_PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid);
    PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
    taskEXIT_CRITICAL();
}

void gimbal_tune_get_pitch_speed_pid(pid_param_t *out)
{
    const pid_type_def *pid;

    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid = &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_get_pitch_angle_pid(pid_param_t *out)
{
    const gimbal_PID_t *pid;

    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid = &gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_pitch_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    pid_type_def *dst;
    fp32 max_out;
    fp32 max_iout;
    fp32 limit;

    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    dst = &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid;
    dst->Kp = pid->kp;
    dst->Ki = pid->ki;
    dst->Kd = pid->kd;
    max_out = fabsf(pid->max_out);
    max_iout = fabsf(pid->max_iout);
    limit = fabsf(PITCH_CURRENT_LIMIT);
    if (limit > 0.0f)
    {
        if (max_out > limit)
        {
            max_out = limit;
        }
        if (max_iout > limit)
        {
            max_iout = limit;
        }
    }
    dst->max_out = max_out;
    dst->max_iout = max_iout;
    if (clear_state)
    {
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_pitch_angle_pid(const pid_param_t *pid, bool_t clear_state)
{
    gimbal_PID_t *dst;

    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    dst = &gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid;
    dst->kp = pid->kp;
    dst->ki = pid->ki;
    dst->kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        gimbal_PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_clear_pitch_pid(void)
{
    taskENTER_CRITICAL();
    gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid);
    PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
    taskEXIT_CRITICAL();
}
