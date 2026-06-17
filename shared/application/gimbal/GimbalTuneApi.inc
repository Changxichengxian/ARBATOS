/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by GimbalControlTask.c.
 */

/**
  * @brief          "GimbalControl" variable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     GimbalInit: "GimbalControl" variable point
  * @retval         none
  */
void GimbalTuneGetYawSpeedPid(PidParam *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const PidTypeDef *pid = &g_gimbal.GimbalYawMotor.GimbalMotorGyroPid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void GimbalTuneGetYawAnglePid(PidParam *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const GimbalPid *pid = &g_gimbal.GimbalYawMotor.GimbalMotorAnglePid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void GimbalTuneSetYawSpeedPid(const PidParam *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    PidTypeDef *dst = &g_gimbal.GimbalYawMotor.GimbalMotorGyroPid;
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

void GimbalTuneSetYawAnglePid(const PidParam *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    GimbalPid *dst = &g_gimbal.GimbalYawMotor.GimbalMotorAnglePid;
    dst->kp = pid->kp;
    dst->ki = pid->ki;
    dst->kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        GimbalPidClear(dst);
    }
    taskEXIT_CRITICAL();
}

void GimbalTuneClearYawPid(void)
{
    taskENTER_CRITICAL();
    GimbalPidClear(&g_gimbal.GimbalYawMotor.GimbalMotorAnglePid);
    PID_clear(&g_gimbal.GimbalYawMotor.GimbalMotorGyroPid);
    taskEXIT_CRITICAL();
}

void GimbalTuneGetPitchSpeedPid(PidParam *out)
{
    const PidTypeDef *pid;

    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid = &g_gimbal.GimbalPitchMotor.GimbalMotorGyroPid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void GimbalTuneGetPitchAnglePid(PidParam *out)
{
    const GimbalPid *pid;

    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid = &g_gimbal.GimbalPitchMotor.GimbalMotorAnglePid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void GimbalTuneSetPitchSpeedPid(const PidParam *pid, bool_t clear_state)
{
    PidTypeDef *dst;
    fp32 max_out;
    fp32 max_iout;
    fp32 limit;

    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    dst = &g_gimbal.GimbalPitchMotor.GimbalMotorGyroPid;
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

void GimbalTuneSetPitchAnglePid(const PidParam *pid, bool_t clear_state)
{
    GimbalPid *dst;

    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    dst = &g_gimbal.GimbalPitchMotor.GimbalMotorAnglePid;
    dst->kp = pid->kp;
    dst->ki = pid->ki;
    dst->kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        GimbalPidClear(dst);
    }
    taskEXIT_CRITICAL();
}

void GimbalTuneClearPitchPid(void)
{
    taskENTER_CRITICAL();
    GimbalPidClear(&g_gimbal.GimbalPitchMotor.GimbalMotorAnglePid);
    PID_clear(&g_gimbal.GimbalPitchMotor.GimbalMotorGyroPid);
    taskEXIT_CRITICAL();
}
