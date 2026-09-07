/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "RobotTargetConfig.h"
#define ROBOT_PROFILE_GIMBAL_ENCODER_FALLBACK_MASK 0x07u

#define MOTOR_ARM_JOINT_COUNT 6u
#define CHASSIS_USE_IMU_YAW_FEEDBACK 1u
#define CHASSIS_GIMBAL_YAW_RELATIVE_TURN 1u
#define GIMBAL_DUAL_YAW_IMU_TURN 0u
#define GIMBAL_PITCH_MIDDLE_ECD 3600u
#define GIMBAL_PITCH_HOLD_CURRENT 6000.0f
#define CHASSIS_STOP_ON_GIMBAL_STATE 0u

#define WATCH_ENABLE_RUNTIME_COPY 0
#define WATCH_ENABLE_COMM_COPY 0
#define WATCH_ENABLE_DIAG_COPY 0

#include "RobotConfigTypes.h"
