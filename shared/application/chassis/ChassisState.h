/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CHASSIS_STATE_H
#define CHASSIS_STATE_H

#include "RobotMsg.h"
#include "StateStore.h"
#include "Pid.h"
#include "Types.h"

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
} ChassisStateMode;

typedef struct
{
    motor_measure_state_t measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} ChassisMotorState;

typedef struct
{
    uint8_t valid;
    uint8_t mode;
    uint8_t last_mode;
    uint8_t fault_configured_mask;
    uint8_t fault_active_mask;
    uint8_t fault_blocking_mask;
    uint8_t fault_recovery_mask;
    uint8_t fault_inhibit_mask;
    uint8_t fault_hold_zero_mask;
    uint8_t fault_recovery_input_safe;
    uint8_t reserved0;
    uint16_t motor_reason_mask[CHASSIS_STATE_MOTOR_COUNT];
    uint32_t motor_feedback_age_ms[CHASSIS_STATE_MOTOR_COUNT];
    uint32_t fault_inhibit_fail_count;
    uint32_t fault_release_fail_count;
    fp32 vx;
    fp32 vy;
    fp32 wz;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 ChassisYawOffset;
    fp32 ChassisYawOffsetSet;
    fp32 ChassisYawSet;
    fp32 ChassisYaw;
    fp32 ChassisPitch;
    fp32 ChassisRoll;
    PidTypeDef angle_pid;
    ChassisMotorState motor[CHASSIS_STATE_MOTOR_COUNT];
} ChassisState;

typedef char ChassisStateFitsStore[(sizeof(ChassisState) <= STATE_STORE_CHASSIS_BYTES) ? 1 : -1];

static inline uint8_t ChassisStateWrite(const ChassisState *state)
{
    return StateStoreWrite(STATE_CHASSIS, state, (uint16_t)sizeof(*state));
}

static inline uint8_t ChassisStateRead(ChassisState *out)
{
    return StateStoreRead(STATE_CHASSIS, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
