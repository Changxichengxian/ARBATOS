/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include "types.h"
#include "gimbal_control_task.h"

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_ANGLE,
    GIMBAL_MOTIONLESS,
    GIMBAL_PITCH_CALI,
} gimbal_behaviour_e;

extern volatile gimbal_behaviour_e gimbal_behaviour_watch;

extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch,
                                         gimbal_control_t *gimbal_control_set);
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern bool_t gimbal_cmd_to_shoot_stop(void);

// One-button turnaround helpers shared with chassis.
extern bool_t gimbal_turnaround_is_active(void);
extern fp32 gimbal_turnaround_chassis_follow_offset_rad(void);
extern bool_t gimbal_turnaround_get_frame_yaw_relative(fp32 *out_yaw_relative);

#endif
