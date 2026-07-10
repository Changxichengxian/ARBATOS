/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include "Types.h"
#include "GimbalControlTask.h"

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_ANGLE,
    GIMBAL_MOTIONLESS,
    GIMBAL_PITCH_CALI,
} GimbalBehaviour;

extern volatile GimbalBehaviour GimbalBehaviourWatch;

extern void GimbalBehaviourModeSet(GimbalControl *GimbalModeSet);
extern void GimbalBehaviourControlSet(fp32 *add_yaw, fp32 *add_pitch,
                                         GimbalControl *GimbalControlSet);
extern bool_t GimbalCmdToChassisStop(void);
extern bool_t GimbalCmdToShootStop(void);

// One-button turnaround helpers shared with chassis.
extern bool_t GimbalTurnaroundIsActive(void);
extern void GimbalBehaviourInputGateBlock(void);
extern fp32 GimbalTurnaroundChassisFollowOffsetRad(void);
extern bool_t GimbalTurnaroundGetFrameYawRelative(fp32 *out_yaw_relative);

#endif
