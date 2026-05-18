/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CHASSIS_STATE_H
#define CHASSIS_STATE_H

#include "robot_msg.h"
#include "state_store.h"
#include "pid.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_STATE_MOTOR_COUNT 4u

typedef enum
{
    CHASSIS_STATE_MODE_FOLLOW_GIMBAL_YAW = 0,
    CHASSIS_STATE_MODE_FOLLOW_CHASSIS_YAW,
    CHASSIS_STATE_MODE_NO_FOLLOW_YAW,
    CHASSIS_STATE_MODE_RAW,
} chassis_state_mode_e;

typedef struct
{
    motor_measure_state_t measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} chassis_motor_state_t;

typedef struct
{
    uint8_t valid;
    uint8_t mode;
    uint8_t last_mode;
    fp32 vx;
    fp32 vy;
    fp32 wz;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 chassis_yaw_offset;
    fp32 chassis_yaw_offset_set;
    fp32 chassis_yaw_set;
    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;
    pid_type_def angle_pid;
    chassis_motor_state_t motor[CHASSIS_STATE_MOTOR_COUNT];
} chassis_state_t;

typedef char chassis_state_fits_store[(sizeof(chassis_state_t) <= STATE_STORE_MAX_BYTES) ? 1 : -1];

static inline uint8_t chassis_state_write(const chassis_state_t *state)
{
    return state_store_write(STATE_CHASSIS, state, (uint16_t)sizeof(*state));
}

static inline uint8_t chassis_state_read(chassis_state_t *out)
{
    return state_store_read(STATE_CHASSIS, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
