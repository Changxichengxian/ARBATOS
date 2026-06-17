/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by ChassisControlTask.c.
 */

const ChassisMove *get_chassis_move_point(void)
{
    return &g_chassis;
}

void ChassisTuneGetFollowPid(PidParam *out)
{
    if (out == NULL)
    {
        return;
    }
    out->kp = g_chassis.ChassisAnglePid.Kp;
    out->ki = g_chassis.ChassisAnglePid.Ki;
    out->kd = g_chassis.ChassisAnglePid.Kd;
    out->max_out = g_chassis.ChassisAnglePid.max_out;
    out->max_iout = g_chassis.ChassisAnglePid.max_iout;
}

void ChassisTuneSetFollowPid(const PidParam *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    g_chassis.ChassisAnglePid.Kp = pid->kp;
    g_chassis.ChassisAnglePid.Ki = pid->ki;
    g_chassis.ChassisAnglePid.Kd = pid->kd;
    g_chassis.ChassisAnglePid.max_out = pid->max_out;
    g_chassis.ChassisAnglePid.max_iout = pid->max_iout;

    if (clear_state)
    {
        PID_clear(&g_chassis.ChassisAnglePid);
    }
}

void ChassisTuneClearFollowPid(void)
{
    PID_clear(&g_chassis.ChassisAnglePid);
}

void ChassisTuneGetMotorSpeedPid(PidParam *out)
{
    if (out == NULL)
    {
        return;
    }

    const PidTypeDef *pid = &g_chassis.motor_speed_pid[0];
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
}

void ChassisTuneSetMotorSpeedPid(const PidParam *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        PidTypeDef *dst = &g_chassis.motor_speed_pid[i];
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

static void ChassisWriteState(const ChassisMove *control)
{
    if (control == NULL)
    {
        return;
    }

    ChassisState state = {0};
    state.valid = 1u;
    state.mode = (uint8_t)control->mode;
    state.last_mode = (uint8_t)control->last_mode;
    state.vx = control->vx;
    state.vy = control->vy;
    state.wz = control->wz;
    state.vx_set = control->vx_set;
    state.vy_set = control->vy_set;
    state.wz_set = control->wz_set;
    state.ChassisYawOffset = control->ChassisYawOffset;
    state.ChassisYawOffsetSet = control->ChassisYawOffsetSet;
    state.ChassisYawSet = control->ChassisYawSet;
    state.ChassisYaw = control->ChassisYaw;
    state.ChassisPitch = control->ChassisPitch;
    state.ChassisRoll = control->ChassisRoll;
    state.angle_pid = control->ChassisAnglePid;

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT && i < CHASSIS_STATE_MOTOR_COUNT; i++)
    {
        ChassisFillMotorMeasureState(&state.motor[i].measure, control->motor_chassis[i].ChassisMotorMeasure);
        state.motor[i].accel = control->motor_chassis[i].accel;
        state.motor[i].speed = control->motor_chassis[i].speed;
        state.motor[i].speed_set = control->motor_chassis[i].speed_set;
        state.motor[i].give_current = control->motor_chassis[i].give_current;
    }

    (void)ChassisStateWrite(&state);
}
