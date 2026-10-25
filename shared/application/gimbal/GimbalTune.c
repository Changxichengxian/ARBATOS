/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * 云台运行期 PID 调参接口。
 */

#include "GimbalTaskPrivate.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include <math.h>

/**
  * @brief          "GimbalControl" variable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     GimbalInit: "GimbalControl" variable point
  * @retval         none
  */
void GimbalTuneGetYawSpeedPidFor(const GimbalControl *control, PidParam *out)
{
    if (control == NULL || out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const PidTypeDef *pid = &control->GimbalYawMotor.GimbalMotorGyroPid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void GimbalTuneGetYawAnglePidFor(const GimbalControl *control, PidParam *out)
{
    if (control == NULL || out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const GimbalPid *pid = &control->GimbalYawMotor.GimbalMotorAnglePid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void GimbalTuneSetYawSpeedPidFor(GimbalControl *control, const PidParam *pid, bool_t clear_state)
{
    if (control == NULL || pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    PidTypeDef *dst = &control->GimbalYawMotor.GimbalMotorGyroPid;
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

void GimbalTuneSetYawAnglePidFor(GimbalControl *control, const PidParam *pid, bool_t clear_state)
{
    if (control == NULL || pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    GimbalPid *dst = &control->GimbalYawMotor.GimbalMotorAnglePid;
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

void GimbalTuneClearYawPidFor(GimbalControl *control)
{
    if (control == NULL)
    {
        return;
    }
    taskENTER_CRITICAL();
    GimbalPidClear(&control->GimbalYawMotor.GimbalMotorAnglePid);
    PID_clear(&control->GimbalYawMotor.GimbalMotorGyroPid);
    taskEXIT_CRITICAL();
}

void GimbalTuneGetPitchSpeedPidFor(const GimbalControl *control, PidParam *out)
{
    const PidTypeDef *pid;

    if (control == NULL || out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid = &control->GimbalPitchMotor.GimbalMotorGyroPid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void GimbalTuneGetPitchAnglePidFor(const GimbalControl *control, PidParam *out)
{
    const GimbalPid *pid;

    if (control == NULL || out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid = &control->GimbalPitchMotor.GimbalMotorAnglePid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void GimbalTuneSetPitchSpeedPidFor(GimbalControl *control, const PidParam *pid, bool_t clear_state)
{
    PidTypeDef *dst;
    fp32 max_out;
    fp32 max_iout;
    fp32 limit;

    if (control == NULL || pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    dst = &control->GimbalPitchMotor.GimbalMotorGyroPid;
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

void GimbalTuneSetPitchAnglePidFor(GimbalControl *control, const PidParam *pid, bool_t clear_state)
{
    GimbalPid *dst;

    if (control == NULL || pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    dst = &control->GimbalPitchMotor.GimbalMotorAnglePid;
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

void GimbalTuneClearPitchPidFor(GimbalControl *control)
{
    if (control == NULL)
    {
        return;
    }
    taskENTER_CRITICAL();
    GimbalPidClear(&control->GimbalPitchMotor.GimbalMotorAnglePid);
    PID_clear(&control->GimbalPitchMotor.GimbalMotorGyroPid);
    taskEXIT_CRITICAL();
}

#endif
