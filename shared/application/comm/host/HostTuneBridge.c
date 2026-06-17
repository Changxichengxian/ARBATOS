/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "HostTuneBridge.h"

#include <string.h>
#include "arm_math.h"
#include "GimbalBehaviour.h"

__weak volatile GimbalBehaviour GimbalBehaviourWatch = GIMBAL_ZERO_FORCE;

__weak void ShootTuneApplyFricSpeedPid(void)
{
}

__weak void ShootTuneApplyTriggerPid(void)
{
}

__weak const GimbalMotor *get_yaw_motor_point(void)
{
    return NULL;
}

__weak const GimbalMotor *get_pitch_motor_point(void)
{
    return NULL;
}

__weak void GimbalTuneGetYawSpeedPid(PidParam *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void GimbalTuneGetYawAnglePid(PidParam *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void GimbalTuneSetYawSpeedPid(const PidParam *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void GimbalTuneSetYawAnglePid(const PidParam *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void GimbalTuneClearYawPid(void)
{
}

__weak void GimbalTuneGetPitchSpeedPid(PidParam *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void GimbalTuneGetPitchAnglePid(PidParam *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void GimbalTuneSetPitchSpeedPid(const PidParam *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void GimbalTuneSetPitchAnglePid(const PidParam *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void GimbalTuneClearPitchPid(void)
{
}

__weak const ChassisMove *get_chassis_move_point(void)
{
    return NULL;
}

__weak void ChassisTuneGetFollowPid(PidParam *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void ChassisTuneSetFollowPid(const PidParam *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}

__weak void ChassisTuneClearFollowPid(void)
{
}

__weak void ChassisTuneGetMotorSpeedPid(PidParam *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

__weak void ChassisTuneSetMotorSpeedPid(const PidParam *pid, bool_t clear_state)
{
    (void)pid;
    (void)clear_state;
}
