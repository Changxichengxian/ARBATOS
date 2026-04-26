/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */



#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "struct_typedef.h"
#include "chassis_control_task.h"

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
} chassis_behaviour_e;

#define CHASSIS_OPEN_RC_SCALE 10

extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set,
                                          chassis_move_t *chassis_move_rc_to_vector);

#endif
