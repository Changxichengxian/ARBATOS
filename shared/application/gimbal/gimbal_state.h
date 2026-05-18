/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef GIMBAL_STATE_H
#define GIMBAL_STATE_H

#include "robot_msg.h"
#include "state_store.h"
#include "gimbal_pid.h"
#include "pid.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t valid;
    uint8_t motor_mode;
    uint8_t last_motor_mode;
    uint16_t offset_ecd;
    fp32 max_angle;
    fp32 min_angle;
    motor_measure_state_t measure;
    gimbal_PID_t angle_pid;
    pid_type_def gyro_pid;
    fp32 angle;
    fp32 angle_set;
    fp32 motor_gyro;
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
} gimbal_motor_state_t;

typedef struct
{
    uint8_t valid;
    uint8_t behaviour;
    uint8_t chassis_stop;
    uint8_t shoot_stop;
    uint8_t turnaround_active;
    uint8_t turnaround_frame_valid;
    fp32 turnaround_frame_yaw_relative;
    fp32 turnaround_follow_offset_rad;
    gimbal_motor_state_t yaw;
    gimbal_motor_state_t pitch;
} gimbal_state_t;

typedef char gimbal_state_fits_store[(sizeof(gimbal_state_t) <= STATE_STORE_MAX_BYTES) ? 1 : -1];

static inline uint8_t gimbal_state_write(const gimbal_state_t *state)
{
    return state_store_write(STATE_GIMBAL, state, (uint16_t)sizeof(*state));
}

static inline uint8_t gimbal_state_read(gimbal_state_t *out)
{
    return state_store_read(STATE_GIMBAL, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
