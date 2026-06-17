/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef HOST_TUNE_BRIDGE_H
#define HOST_TUNE_BRIDGE_H

#include "ChassisControlTask.h"
#include "config.h"
#include "GimbalControlTask.h"

void ShootTuneApplyFricSpeedPid(void);
void ShootTuneApplyTriggerPid(void);

const GimbalMotor *get_yaw_motor_point(void);
const GimbalMotor *get_pitch_motor_point(void);
void GimbalTuneGetYawSpeedPid(PidParam *out);
void GimbalTuneGetYawAnglePid(PidParam *out);
void GimbalTuneSetYawSpeedPid(const PidParam *pid, bool_t clear_state);
void GimbalTuneSetYawAnglePid(const PidParam *pid, bool_t clear_state);
void GimbalTuneClearYawPid(void);
void GimbalTuneGetPitchSpeedPid(PidParam *out);
void GimbalTuneGetPitchAnglePid(PidParam *out);
void GimbalTuneSetPitchSpeedPid(const PidParam *pid, bool_t clear_state);
void GimbalTuneSetPitchAnglePid(const PidParam *pid, bool_t clear_state);
void GimbalTuneClearPitchPid(void);

const ChassisMove *get_chassis_move_point(void);
void ChassisTuneGetFollowPid(PidParam *out);
void ChassisTuneSetFollowPid(const PidParam *pid, bool_t clear_state);
void ChassisTuneClearFollowPid(void);
void ChassisTuneGetMotorSpeedPid(PidParam *out);
void ChassisTuneSetMotorSpeedPid(const PidParam *pid, bool_t clear_state);

#endif
