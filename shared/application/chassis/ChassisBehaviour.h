/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */



#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "Types.h"
#include "ChassisControlTask.h"

typedef enum
{
    CHASSIS_ZERO_FORCE = 0,                // no current output
    CHASSIS_NO_MOVE,                      // keep chassis stopped
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,   // follow gimbal yaw
    CHASSIS_SWING,                        // swing around gimbal heading
    CHASSIS_GYRO_SPIN,                    // constant spin
    CHASSIS_GYRO_SPIN_VAR,                // variable-speed spin
    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  // follow chassis yaw
    CHASSIS_NO_FOLLOW_YAW,                // open-loop yaw rate
    CHASSIS_OPEN,                         // raw current mode
} ChassisBehaviour;

#define CHASSIS_OPEN_RC_SCALE 10

typedef struct
{
    uint32_t count;
    uint32_t last_ms;
    int16_t vx_cmps;
    int16_t vy_cmps;
    int16_t wz_mradps;
} ChassisAlgorithmDebug;

extern void ChassisBehaviourModeSet(ChassisMove *ChassisMoveMode);
extern void ChassisBehaviourInputGateBlock(void);
extern void ChassisBehaviourControlSet(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set,
                                          ChassisMove *ChassisMoveRcToVector);
extern void ChassisAlgorithmDebugRead(ChassisAlgorithmDebug *out);

#endif
