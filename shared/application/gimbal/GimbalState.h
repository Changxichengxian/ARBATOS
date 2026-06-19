/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef GIMBAL_STATE_H
#define GIMBAL_STATE_H

#include "RobotMsg.h"
#include "StateStore.h"
#include "GimbalPid.h"
#include "Pid.h"
#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_STATE_OFFLINE_DBUS      (1u << 0)
#define GIMBAL_STATE_OFFLINE_YAW       (1u << 1)
#define GIMBAL_STATE_OFFLINE_YAW_UPPER (1u << 2)
#define GIMBAL_STATE_OFFLINE_PITCH     (1u << 3)
#define GIMBAL_STATE_OFFLINE_IMU       (1u << 4)

typedef struct
{
    uint8_t valid;
    uint8_t motor_mode;
    uint8_t last_motor_mode;
    uint16_t offset_ecd;
    fp32 max_angle;
    fp32 min_angle;
    motor_measure_state_t measure;
    GimbalPid angle_pid;
    PidTypeDef gyro_pid;
    fp32 angle;
    fp32 angle_set;
    fp32 motor_gyro;
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
} GimbalMotorState;

typedef struct
{
    uint8_t valid;
    uint8_t behaviour;
    uint8_t ChassisStop;
    uint8_t ShootStop;
    uint8_t online;
    uint8_t controllable;
    uint8_t fire_allowed;
    uint8_t manual_online;
    uint8_t imu_online;
    uint8_t yaw_required;
    uint8_t yaw_online;
    uint8_t yaw_upper_required;
    uint8_t yaw_upper_online;
    uint8_t pitch_required;
    uint8_t pitch_online;
    uint8_t reserved0;
    uint16_t offline_mask;
    uint16_t required_offline_mask;
    uint8_t turnaround_active;
    uint8_t turnaround_frame_valid;
    fp32 turnaround_frame_yaw_relative;
    fp32 turnaround_follow_offset_rad;
    GimbalMotorState yaw;
    GimbalMotorState pitch;
} GimbalState;

typedef char GimbalStateFitsStore[(sizeof(GimbalState) <= STATE_STORE_MAX_BYTES) ? 1 : -1];

static inline uint8_t GimbalStateWrite(const GimbalState *state)
{
    return StateStoreWrite(STATE_GIMBAL, state, (uint16_t)sizeof(*state));
}

static inline uint8_t GimbalStateRead(GimbalState *out)
{
    return StateStoreRead(STATE_GIMBAL, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
