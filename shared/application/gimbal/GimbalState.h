/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef GIMBAL_STATE_H
#define GIMBAL_STATE_H

#include <stddef.h>

#include "RobotMsg.h"
#include "StateStore.h"
#include "GimbalPid.h"
#include "Pid.h"
#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_STATE_OFFLINE_MANUAL    (1u << 0)
#define GIMBAL_STATE_OFFLINE_DBUS      GIMBAL_STATE_OFFLINE_MANUAL
#define GIMBAL_STATE_OFFLINE_YAW       (1u << 1)
#define GIMBAL_STATE_OFFLINE_YAW_UPPER (1u << 2)
#define GIMBAL_STATE_OFFLINE_PITCH     (1u << 3)
#define GIMBAL_STATE_OFFLINE_IMU       (1u << 4)
#define GIMBAL_STATE_AGE_UNKNOWN       0xFFFFFFFFu
#define GIMBAL_STATE_FRESH_TIMEOUT_MS  20u

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
    uint8_t feedback_mode;
    uint8_t encoder_fallback_mask;
    uint8_t feedback_degraded_mask;
    uint8_t feedback_block_mask;
    uint8_t feedback_recovery_pending;
    uint8_t feedback_transition_zero_mask;
    uint8_t yaw_required;
    uint8_t yaw_online;
    uint8_t yaw_upper_required;
    uint8_t yaw_upper_online;
    uint8_t pitch_required;
    uint8_t pitch_online;
    uint8_t follow_available;
    uint8_t yaw_control_is_upper;
    uint8_t fault_configured_mask;
    uint8_t fault_active_mask;
    uint8_t fault_blocking_mask;
    uint8_t fault_recovery_mask;
    uint8_t fault_inhibit_mask;
    uint8_t fault_hold_zero_mask;
    uint8_t fault_imu_required_mask;
    uint8_t fault_recovery_input_safe;
    uint16_t offline_mask;
    uint16_t required_offline_mask;
    uint16_t yaw_reason_mask;
    uint16_t yaw_upper_reason_mask;
    uint16_t pitch_reason_mask;
    uint16_t reserved0;
    uint32_t yaw_feedback_age_ms;
    uint32_t yaw_upper_feedback_age_ms;
    uint32_t pitch_feedback_age_ms;
    uint32_t imu_age_ms;
    uint32_t fault_inhibit_fail_count;
    uint32_t fault_release_fail_count;
    uint8_t turnaround_active;
    uint8_t turnaround_frame_valid;
    fp32 turnaround_frame_yaw_relative;
    fp32 turnaround_follow_offset_rad;
    GimbalMotorState yaw;
    GimbalMotorState pitch;
} GimbalState;

typedef char GimbalStateFitsStore[(sizeof(GimbalState) <= STATE_STORE_GIMBAL_BYTES) ? 1 : -1];

static inline uint8_t GimbalStateWrite(const GimbalState *state)
{
    return StateStoreWrite(STATE_GIMBAL, state, (uint16_t)sizeof(*state));
}

static inline uint8_t GimbalStateRead(GimbalState *out)
{
    return StateStoreRead(STATE_GIMBAL, out, (uint16_t)sizeof(*out));
}

static inline uint8_t GimbalStateReadFresh(GimbalState *out, uint32_t max_age_ms)
{
    state_info_t info = {0};

    if (out == NULL ||
        StateStoreReadSnapshot(STATE_GIMBAL,
                               out,
                               (uint16_t)sizeof(*out),
                               &info) == 0u)
    {
        return 0u;
    }
    return (info.age_ms <= max_age_ms) ? 1u : 0u;
}

#ifdef __cplusplus
}
#endif

#endif
